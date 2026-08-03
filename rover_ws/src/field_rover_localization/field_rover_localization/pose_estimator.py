"""Pure Python complementary-style planar localization estimator."""

# This is a deliberately understandable fusion of wheel odometry, IMU yaw
# rate, and low-rate GPS position corrections. It is NOT an Extended Kalman
# Filter: there is no state covariance propagation through a process model,
# no Kalman gain computed from measurement/process noise, and the output
# covariance below is a fixed nominal description, not a probabilistically
# propagated one. A future milestone may replace this with a real EKF.
#
# Design:
#   - Wheel odometry provides forward speed and a baseline turn rate, and
#     drives the prediction timing (dt comes from wheel message timestamps).
#   - IMU angular velocity, when fresh and valid, is blended with the wheel
#     turn rate by a configurable weight to form the yaw rate used for
#     prediction; a stale or missing IMU falls back to wheel-only turning.
#   - GPS, converted from latitude/longitude into local east/north metres by
#     field_rover_localization.geographic_conversion, nudges the position
#     estimate toward each new fix by a configurable fraction (a simple
#     proportional correction), gated against implausibly large jumps. GPS
#     never touches yaw, because the Day 7 GPS simulation has no heading.

from dataclasses import dataclass
import math

from field_rover_localization.geographic_conversion import (
    validate_geographic_reference,
)


@dataclass(frozen=True)
class LocalizationConfig:
    """Store the estimator's fusion weights, gates, and static covariance."""

    imu_yaw_rate_weight: float
    imu_gyro_bias_correction: float
    imu_timeout_s: float
    gps_position_gain: float
    max_gps_innovation_m: float
    max_dt: float
    reference_latitude_deg: float
    reference_longitude_deg: float
    frame_id: str
    child_frame_id: str
    position_variance: float
    yaw_variance: float
    linear_velocity_variance: float
    angular_velocity_variance: float

    def __post_init__(self):
        """Reject non-finite, out-of-range, or otherwise unusable settings."""
        if (
            not math.isfinite(self.imu_yaw_rate_weight)
            or not (0.0 <= self.imu_yaw_rate_weight <= 1.0)
        ):
            raise ValueError(
                'imu_yaw_rate_weight must be finite and within [0.0, 1.0].'
            )

        if not math.isfinite(self.imu_gyro_bias_correction):
            raise ValueError('imu_gyro_bias_correction must be finite.')

        if not math.isfinite(self.imu_timeout_s) or self.imu_timeout_s <= 0.0:
            raise ValueError('imu_timeout_s must be positive and finite.')

        if (
            not math.isfinite(self.gps_position_gain)
            or not (0.0 <= self.gps_position_gain <= 1.0)
        ):
            raise ValueError(
                'gps_position_gain must be finite and within [0.0, 1.0].'
            )

        if (
            not math.isfinite(self.max_gps_innovation_m)
            or self.max_gps_innovation_m <= 0.0
        ):
            raise ValueError('max_gps_innovation_m must be positive and finite.')

        if not math.isfinite(self.max_dt) or self.max_dt <= 0.0:
            raise ValueError('max_dt must be positive and finite.')

        validate_geographic_reference(
            self.reference_latitude_deg, self.reference_longitude_deg,
        )

        if not self.frame_id:
            raise ValueError('frame_id must be non-empty.')
        if not self.child_frame_id:
            raise ValueError('child_frame_id must be non-empty.')

        for name, value in (
            ('position_variance', self.position_variance),
            ('yaw_variance', self.yaw_variance),
            ('linear_velocity_variance', self.linear_velocity_variance),
            ('angular_velocity_variance', self.angular_velocity_variance),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be non-negative and finite.')


DEFAULT_LOCALIZATION_CONFIG = LocalizationConfig(
    imu_yaw_rate_weight=0.70,
    imu_gyro_bias_correction=0.01,
    imu_timeout_s=0.20,
    gps_position_gain=0.20,
    max_gps_innovation_m=5.0,
    max_dt=0.50,
    reference_latitude_deg=43.2609,
    reference_longitude_deg=-79.9192,
    frame_id='map',
    child_frame_id='base_link',
    position_variance=0.50,
    yaw_variance=0.05,
    linear_velocity_variance=0.10,
    angular_velocity_variance=0.02,
)


@dataclass(frozen=True)
class LocalizationState:
    """Represent the fused planar pose estimate and its output velocities."""

    x: float
    y: float
    yaw: float
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


@dataclass(frozen=True)
class GpsCorrectionResult:
    """Represent the outcome of one attempted GPS position correction."""

    state: LocalizationState
    applied: bool
    innovation_distance: float


def normalize_yaw(yaw: float) -> float:
    """Normalize an angle into the interval [-pi, pi)."""
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(yaw_a: float, yaw_b: float) -> float:
    """Return the shortest signed angular difference yaw_a - yaw_b."""
    return normalize_yaw(yaw_a - yaw_b)


def is_imu_measurement_usable(
    imu_stamp_seconds: float | None,
    wheel_stamp_seconds: float,
    config: LocalizationConfig,
) -> bool:
    """Return whether the latest IMU sample is fresh enough to blend in."""
    if imu_stamp_seconds is None:
        return False

    age_seconds = wheel_stamp_seconds - imu_stamp_seconds
    if not math.isfinite(age_seconds):
        return False

    # A negative age means the IMU stamp is somehow ahead of the wheel
    # stamp (clock skew, out-of-order delivery); treat it as untrustworthy
    # rather than guessing which timestamp is wrong.
    if age_seconds < 0.0:
        return False

    return age_seconds <= config.imu_timeout_s


def calculate_fused_yaw_rate(
    wheel_angular_velocity: float,
    imu_angular_velocity_z: float | None,
    imu_is_usable: bool,
    config: LocalizationConfig,
) -> float:
    """Blend wheel and bias-corrected IMU yaw rate, or fall back to wheel."""
    if (
        not imu_is_usable
        or imu_angular_velocity_z is None
        or not math.isfinite(imu_angular_velocity_z)
    ):
        return wheel_angular_velocity

    imu_corrected = imu_angular_velocity_z - config.imu_gyro_bias_correction
    return (
        config.imu_yaw_rate_weight * imu_corrected
        + (1.0 - config.imu_yaw_rate_weight) * wheel_angular_velocity
    )


def predict_state(
    state: LocalizationState,
    wheel_linear_velocity: float,
    wheel_angular_velocity: float,
    imu_angular_velocity_z: float | None,
    imu_is_usable: bool,
    dt: float,
    config: LocalizationConfig,
) -> LocalizationState:
    """Predict one time step of pose from wheel speed and fused yaw rate."""
    # An invalid or excessive dt cannot be safely integrated (no fabricated
    # time step), so the pose is retained and the reported velocities drop
    # to zero for that tick — the same convention wheel_odometry.py uses.
    if not math.isfinite(dt) or dt <= 0.0 or dt > config.max_dt:
        return LocalizationState(
            x=state.x, y=state.y, yaw=state.yaw,
            linear_velocity=0.0, angular_velocity=0.0,
        )

    omega_fused = calculate_fused_yaw_rate(
        wheel_angular_velocity, imu_angular_velocity_z, imu_is_usable, config,
    )

    delta_yaw = omega_fused * dt
    heading_mid = state.yaw + delta_yaw / 2.0
    delta_distance = wheel_linear_velocity * dt

    return LocalizationState(
        x=state.x + delta_distance * math.cos(heading_mid),
        y=state.y + delta_distance * math.sin(heading_mid),
        yaw=normalize_yaw(state.yaw + delta_yaw),
        linear_velocity=wheel_linear_velocity,
        angular_velocity=omega_fused,
    )


def is_gps_measurement_new(
    gps_stamp_seconds: float,
    last_applied_gps_stamp_seconds: float | None,
) -> bool:
    """Return whether a GPS fix is newer than the last one that was applied."""
    # A duplicate, out-of-order, or non-finite stamp is rejected here,
    # before any innovation gating — the same fix (or an older one arriving
    # late) must never correct position twice.
    if last_applied_gps_stamp_seconds is None:
        return True

    if not math.isfinite(gps_stamp_seconds):
        return False

    return gps_stamp_seconds > last_applied_gps_stamp_seconds


def apply_gps_correction(
    state: LocalizationState,
    gps_east_m: float,
    gps_north_m: float,
    config: LocalizationConfig,
) -> GpsCorrectionResult:
    """Nudge position toward a GPS fix by a gated, partial correction."""
    # Yaw and velocity are never touched here — the Day 7 GPS simulation
    # has no heading, and a position fix says nothing about current speed.
    if not (math.isfinite(gps_east_m) and math.isfinite(gps_north_m)):
        return GpsCorrectionResult(
            state=state, applied=False, innovation_distance=math.nan,
        )

    innovation_x = gps_east_m - state.x
    innovation_y = gps_north_m - state.y
    innovation_distance = math.hypot(innovation_x, innovation_y)

    if innovation_distance > config.max_gps_innovation_m:
        return GpsCorrectionResult(
            state=state, applied=False, innovation_distance=innovation_distance,
        )

    corrected_state = LocalizationState(
        x=state.x + config.gps_position_gain * innovation_x,
        y=state.y + config.gps_position_gain * innovation_y,
        yaw=state.yaw,
        linear_velocity=state.linear_velocity,
        angular_velocity=state.angular_velocity,
    )
    return GpsCorrectionResult(
        state=corrected_state, applied=True, innovation_distance=innovation_distance,
    )


# --- Static output covariance ------------------------------------------
#
# Day 8 does not propagate covariance through a filter. These helpers
# populate a fixed nominal 6x6 (row-major, x/y/z/roll/pitch/yaw) diagonal
# that describes a static confidence level only — it does not shrink after
# a GPS correction and does not grow during dead reckoning. A future EKF
# milestone would replace this with real propagated covariance.

UNMODELLED_POSE_VARIANCE = 1e6
UNMODELLED_TWIST_VARIANCE = 1e6


def build_pose_covariance(config: LocalizationConfig) -> list[float]:
    """Return a flat 36-entry pose covariance (row-major 6x6: x,y,z,r,p,yaw)."""
    covariance = [0.0] * 36
    covariance[0] = config.position_variance      # x
    covariance[7] = config.position_variance      # y
    covariance[14] = UNMODELLED_POSE_VARIANCE     # z (not simulated)
    covariance[21] = UNMODELLED_POSE_VARIANCE     # roll (not simulated)
    covariance[28] = UNMODELLED_POSE_VARIANCE     # pitch (not simulated)
    covariance[35] = config.yaw_variance          # yaw
    return covariance


def build_twist_covariance(config: LocalizationConfig) -> list[float]:
    """Return a flat 36-entry twist covariance (row-major 6x6)."""
    covariance = [0.0] * 36
    covariance[0] = config.linear_velocity_variance    # linear x
    covariance[7] = UNMODELLED_TWIST_VARIANCE          # linear y
    covariance[14] = UNMODELLED_TWIST_VARIANCE         # linear z
    covariance[21] = UNMODELLED_TWIST_VARIANCE         # angular x (roll rate)
    covariance[28] = UNMODELLED_TWIST_VARIANCE         # angular y (pitch rate)
    covariance[35] = config.angular_velocity_variance  # angular z (yaw rate)
    return covariance
