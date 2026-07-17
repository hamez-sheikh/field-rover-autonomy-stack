"""Pure Python planar IMU model deriving gyroscope/accelerometer readings."""

# This is a planar simplification, not a full 6-DOF physical IMU:
#
# - Only yaw rate (rotation about the vertical axis) is modelled; roll and
#   pitch rate are always zero.
# - Only forward (body x) and lateral (body y) acceleration are modelled;
#   vertical (body z) acceleration is always zero.
# - Gravity is excluded from the accelerometer model. A real accelerometer
#   at rest reads approximately 9.81 m/s^2 upward from gravity; this
#   simulated sensor instead reports the horizontal motion acceleration
#   only, so a stationary rover reads approximately zero (plus configured
#   bias/noise).
# - No orientation estimate is produced here; this module only ever
#   measures angular velocity and linear acceleration.

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class ImuConfig:
    """Store configured IMU bias, noise, seed, and time-step limits."""

    gyro_bias_z: float
    accel_bias_x: float
    accel_bias_y: float
    gyro_noise_stddev: float
    accel_noise_stddev: float
    random_seed: int
    max_dt: float

    def __post_init__(self):
        """Reject non-finite bias, invalid noise, a bad seed, or a bad max_dt."""
        for name, value in (
            ('gyro_bias_z', self.gyro_bias_z),
            ('accel_bias_x', self.accel_bias_x),
            ('accel_bias_y', self.accel_bias_y),
        ):
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite.')

        for name, value in (
            ('gyro_noise_stddev', self.gyro_noise_stddev),
            ('accel_noise_stddev', self.accel_noise_stddev),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be non-negative and finite.')

        if isinstance(self.random_seed, bool) or not isinstance(
            self.random_seed, int
        ):
            raise ValueError('random_seed must be an integer.')

        if not math.isfinite(self.max_dt) or self.max_dt <= 0.0:
            raise ValueError('max_dt must be positive and finite.')


DEFAULT_IMU_CONFIG = ImuConfig(
    gyro_bias_z=0.01,
    accel_bias_x=0.03,
    accel_bias_y=-0.02,
    gyro_noise_stddev=0.005,
    accel_noise_stddev=0.02,
    random_seed=42,
    max_dt=0.5,
)


@dataclass(frozen=True)
class ImuState:
    """Track the previous world-frame velocity used for differentiation."""

    previous_velocity_world_x: float
    previous_velocity_world_y: float


@dataclass(frozen=True)
class ImuMeasurement:
    """Represent one simulated IMU sample: measured yaw rate and accel."""

    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float


def create_rng(config: ImuConfig) -> random.Random:
    """Build a dedicated, seeded random generator (not the global instance)."""
    return random.Random(config.random_seed)


def calculate_world_velocity(
    forward_speed: float,
    yaw: float,
) -> tuple[float, float]:
    """Convert body-forward speed and heading into world-frame velocity."""
    return forward_speed * math.cos(yaw), forward_speed * math.sin(yaw)


def calculate_world_acceleration(
    velocity_world_x: float,
    velocity_world_y: float,
    previous_velocity_world_x: float,
    previous_velocity_world_y: float,
    dt: float,
) -> tuple[float, float]:
    """Numerically differentiate world-frame velocity into acceleration."""
    return (
        (velocity_world_x - previous_velocity_world_x) / dt,
        (velocity_world_y - previous_velocity_world_y) / dt,
    )


def rotate_world_to_body(
    acceleration_world_x: float,
    acceleration_world_y: float,
    yaw: float,
) -> tuple[float, float]:
    """Rotate world-frame acceleration into the rover body frame."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    acceleration_body_x = cos_yaw * acceleration_world_x + sin_yaw * acceleration_world_y
    acceleration_body_y = -sin_yaw * acceleration_world_x + cos_yaw * acceleration_world_y
    return acceleration_body_x, acceleration_body_y


def calculate_gyro_variance(config: ImuConfig) -> float:
    """Return the gyroscope measurement-noise variance (stddev squared)."""
    return config.gyro_noise_stddev ** 2


def calculate_accel_variance(config: ImuConfig) -> float:
    """Return the accelerometer measurement-noise variance (stddev squared)."""
    return config.accel_noise_stddev ** 2


def generate_imu_measurement(
    previous_state: ImuState | None,
    yaw: float,
    forward_speed: float,
    yaw_rate: float,
    dt: float,
    config: ImuConfig,
    rng: random.Random,
) -> tuple[ImuMeasurement, ImuState]:
    """Derive one IMU sample from ground truth and advance the diff state."""
    # The gyroscope reading is instantaneous (it needs no history), so the
    # ideal yaw rate is always the true yaw_rate passed in.
    #
    # The accelerometer reading requires differentiating world-frame
    # velocity, which needs a previous sample. On the first call
    # (previous_state is None) and on an invalid time step (dt <= 0 or
    # dt > config.max_dt), the model reports zero ideal acceleration rather
    # than fabricating a spike from an unreliable or undefined time
    # difference. In both cases the velocity baseline is reset to the
    # current sample so the next valid step differentiates against a
    # fresh, trustworthy reference.
    velocity_world_x, velocity_world_y = calculate_world_velocity(
        forward_speed, yaw,
    )

    if previous_state is None or dt <= 0.0 or dt > config.max_dt:
        ideal_acceleration_body_x = 0.0
        ideal_acceleration_body_y = 0.0
    else:
        acceleration_world_x, acceleration_world_y = calculate_world_acceleration(
            velocity_world_x,
            velocity_world_y,
            previous_state.previous_velocity_world_x,
            previous_state.previous_velocity_world_y,
            dt,
        )
        ideal_acceleration_body_x, ideal_acceleration_body_y = rotate_world_to_body(
            acceleration_world_x, acceleration_world_y, yaw,
        )

    measured_angular_velocity_z = (
        yaw_rate + config.gyro_bias_z + rng.gauss(0.0, config.gyro_noise_stddev)
    )
    measured_acceleration_x = (
        ideal_acceleration_body_x
        + config.accel_bias_x
        + rng.gauss(0.0, config.accel_noise_stddev)
    )
    measured_acceleration_y = (
        ideal_acceleration_body_y
        + config.accel_bias_y
        + rng.gauss(0.0, config.accel_noise_stddev)
    )

    measurement = ImuMeasurement(
        angular_velocity_z=measured_angular_velocity_z,
        linear_acceleration_x=measured_acceleration_x,
        linear_acceleration_y=measured_acceleration_y,
    )
    new_state = ImuState(
        previous_velocity_world_x=velocity_world_x,
        previous_velocity_world_y=velocity_world_y,
    )
    return measurement, new_state
