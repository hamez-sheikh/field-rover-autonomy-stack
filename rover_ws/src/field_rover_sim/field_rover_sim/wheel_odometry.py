"""Pure Python differential-drive wheel-odometry model."""

from dataclasses import dataclass
import math

from field_rover_sim.rover_dynamics import normalize_yaw


MINIMUM_WHEEL_SCALE = 0.5
MAXIMUM_WHEEL_SCALE = 1.5


@dataclass(frozen=True)
class WheelOdometryConfig:
    """Store the wheel track width and per-wheel calibration scale factors."""

    wheel_track_width: float
    left_wheel_scale: float
    right_wheel_scale: float

    def __post_init__(self):
        """Reject a non-positive track width or an out-of-range wheel scale."""
        if (
            not math.isfinite(self.wheel_track_width)
            or self.wheel_track_width <= 0.0
        ):
            raise ValueError('wheel_track_width must be positive and finite.')

        for name, scale in (
            ('left_wheel_scale', self.left_wheel_scale),
            ('right_wheel_scale', self.right_wheel_scale),
        ):
            if not math.isfinite(scale):
                raise ValueError(f'{name} must be finite.')
            if not (MINIMUM_WHEEL_SCALE <= scale <= MAXIMUM_WHEEL_SCALE):
                raise ValueError(
                    f'{name} must be between {MINIMUM_WHEEL_SCALE} and '
                    f'{MAXIMUM_WHEEL_SCALE}.'
                )


DEFAULT_WHEEL_ODOMETRY_CONFIG = WheelOdometryConfig(
    wheel_track_width=0.6,
    left_wheel_scale=1.01,
    right_wheel_scale=0.99,
)


@dataclass
class WheelOdometryState:
    """Represent the dead-reckoned pose and derived velocity estimate."""

    x: float
    y: float
    yaw: float
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


def calculate_wheel_velocities(
    linear_velocity: float,
    angular_velocity: float,
    config: WheelOdometryConfig,
) -> tuple[float, float]:
    """Convert rover linear/angular velocity into ideal left/right wheel speeds."""
    half_track_width = config.wheel_track_width / 2.0
    left_wheel_velocity = linear_velocity - angular_velocity * half_track_width
    right_wheel_velocity = linear_velocity + angular_velocity * half_track_width
    return left_wheel_velocity, right_wheel_velocity


def calculate_measured_wheel_increments(
    linear_velocity: float,
    angular_velocity: float,
    config: WheelOdometryConfig,
    dt: float,
) -> tuple[float, float]:
    """Return calibration-scaled left/right wheel-distance increments for dt."""
    left_wheel_velocity, right_wheel_velocity = calculate_wheel_velocities(
        linear_velocity,
        angular_velocity,
        config,
    )

    delta_left_true = left_wheel_velocity * dt
    delta_right_true = right_wheel_velocity * dt

    delta_left_measured = delta_left_true * config.left_wheel_scale
    delta_right_measured = delta_right_true * config.right_wheel_scale
    return delta_left_measured, delta_right_measured


def integrate_wheel_odometry(
    state: WheelOdometryState,
    linear_velocity: float,
    angular_velocity: float,
    config: WheelOdometryConfig,
    dt: float,
) -> WheelOdometryState:
    """Dead-reckon one time step of pose from simulated wheel travel."""
    # The calibration scale is applied before distance/heading are
    # reconstructed, so a wheel mismatch produces a wrong heading estimate,
    # and that wrong heading is what later travel integrates against —
    # this is the mechanism that produces cumulative drift.
    if dt <= 0.0:
        return WheelOdometryState(
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            linear_velocity=0.0,
            angular_velocity=0.0,
        )

    delta_left_measured, delta_right_measured = calculate_measured_wheel_increments(
        linear_velocity,
        angular_velocity,
        config,
        dt,
    )

    delta_distance = (delta_right_measured + delta_left_measured) / 2.0
    delta_yaw = (
        delta_right_measured - delta_left_measured
    ) / config.wheel_track_width

    heading_mid = state.yaw + delta_yaw / 2.0

    return WheelOdometryState(
        x=state.x + delta_distance * math.cos(heading_mid),
        y=state.y + delta_distance * math.sin(heading_mid),
        yaw=normalize_yaw(state.yaw + delta_yaw),
        linear_velocity=delta_distance / dt,
        angular_velocity=delta_yaw / dt,
    )
