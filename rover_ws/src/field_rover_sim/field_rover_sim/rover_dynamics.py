"""Pure Python motion mathematics for the simulated field rover."""

from dataclasses import dataclass
import math

from field_rover_sim.world_model import (
    is_pose_in_collision,
    Pose2D,
    WorldModel,
)


@dataclass
class RoverState:
    """Represent the rover pose and current velocities."""

    x: float
    y: float
    yaw: float
    linear_speed: float = 0.0
    angular_speed: float = 0.0


@dataclass(frozen=True)
class MotionLimits:
    """Store the configured rover speed and acceleration limits."""

    max_forward_speed: float
    max_reverse_speed: float
    max_turn_rate: float
    max_linear_acceleration: float
    max_linear_deceleration: float
    max_angular_acceleration: float

    def __post_init__(self):
        """Reject motion limits that are non-positive or non-finite."""
        configured_limits = {
            'max_forward_speed': self.max_forward_speed,
            'max_reverse_speed': self.max_reverse_speed,
            'max_turn_rate': self.max_turn_rate,
            'max_linear_acceleration': self.max_linear_acceleration,
            'max_linear_deceleration': self.max_linear_deceleration,
            'max_angular_acceleration': self.max_angular_acceleration,
        }

        for name, value in configured_limits.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite.')


DEFAULT_MOTION_LIMITS = MotionLimits(
    max_forward_speed=1.0,
    max_reverse_speed=0.5,
    max_turn_rate=1.0,
    max_linear_acceleration=0.5,
    max_linear_deceleration=0.8,
    max_angular_acceleration=1.5,
)


def normalize_yaw(yaw: float) -> float:
    """Normalize an angle into the interval [-pi, pi)."""
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def approach_target(
    current_value: float,
    target_value: float,
    maximum_change: float,
) -> float:
    """Move a value toward a target without exceeding a maximum change."""
    if maximum_change < 0.0:
        raise ValueError('maximum_change cannot be negative.')

    if current_value < target_value:
        return min(current_value + maximum_change, target_value)

    if current_value > target_value:
        return max(current_value - maximum_change, target_value)

    return current_value


def clamp_requested_linear_speed(
    requested_speed: float,
    limits: MotionLimits,
) -> float:
    """Clamp a requested linear speed to forward and reverse limits."""
    return max(
        -limits.max_reverse_speed,
        min(requested_speed, limits.max_forward_speed),
    )


def clamp_requested_angular_speed(
    requested_speed: float,
    limits: MotionLimits,
) -> float:
    """Clamp a requested angular speed to the symmetric turn-rate limit."""
    return max(
        -limits.max_turn_rate,
        min(requested_speed, limits.max_turn_rate),
    )


def calculate_limited_linear_speed(
    current_speed: float,
    requested_speed: float,
    limits: MotionLimits,
    dt: float,
) -> float:
    """Move linear speed toward its clamped request using the proper limit."""
    if dt <= 0.0:
        return current_speed

    target_speed = clamp_requested_linear_speed(requested_speed, limits)

    if current_speed * target_speed < 0.0:
        return approach_target(
            current_speed,
            0.0,
            limits.max_linear_deceleration * dt,
        )

    if target_speed == 0.0 or abs(target_speed) < abs(current_speed):
        rate_limit = limits.max_linear_deceleration
    else:
        rate_limit = limits.max_linear_acceleration

    return approach_target(
        current_speed,
        target_speed,
        rate_limit * dt,
    )


def calculate_limited_angular_speed(
    current_speed: float,
    requested_speed: float,
    limits: MotionLimits,
    dt: float,
) -> float:
    """Move angular speed toward its clamped request at a limited rate."""
    if dt <= 0.0:
        return current_speed

    target_speed = clamp_requested_angular_speed(requested_speed, limits)

    return approach_target(
        current_speed,
        target_speed,
        limits.max_angular_acceleration * dt,
    )


def integrate_pose(
    state: RoverState,
    linear_speed: float,
    angular_speed: float,
    dt: float,
) -> RoverState:
    """Integrate rover pose over one time step using unicycle kinematics."""
    if dt <= 0.0:
        return RoverState(
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            linear_speed=state.linear_speed,
            angular_speed=state.angular_speed,
        )

    return RoverState(
        x=state.x + linear_speed * math.cos(state.yaw) * dt,
        y=state.y + linear_speed * math.sin(state.yaw) * dt,
        yaw=normalize_yaw(state.yaw + angular_speed * dt),
        linear_speed=linear_speed,
        angular_speed=angular_speed,
    )


def update_rover_state(
    state: RoverState,
    requested_linear_speed: float,
    requested_angular_speed: float,
    limits: MotionLimits,
    world: WorldModel,
    dt: float,
) -> tuple[RoverState, bool]:
    """Apply limits, integrate motion, and reject colliding translation."""
    if dt <= 0.0:
        unchanged_state = RoverState(
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            linear_speed=state.linear_speed,
            angular_speed=state.angular_speed,
        )
        return unchanged_state, False

    limited_linear_speed = calculate_limited_linear_speed(
        state.linear_speed,
        requested_linear_speed,
        limits,
        dt,
    )
    limited_angular_speed = calculate_limited_angular_speed(
        state.angular_speed,
        requested_angular_speed,
        limits,
        dt,
    )

    candidate_state = integrate_pose(
        state,
        limited_linear_speed,
        limited_angular_speed,
        dt,
    )
    candidate_pose = Pose2D(
        x=candidate_state.x,
        y=candidate_state.y,
        yaw=candidate_state.yaw,
    )

    if not is_pose_in_collision(candidate_pose, world):
        return candidate_state, False

    rotation_only_state = RoverState(
        x=state.x,
        y=state.y,
        yaw=normalize_yaw(state.yaw + limited_angular_speed * dt),
        linear_speed=0.0,
        angular_speed=limited_angular_speed,
    )
    rotation_only_pose = Pose2D(
        x=rotation_only_state.x,
        y=rotation_only_state.y,
        yaw=rotation_only_state.yaw,
    )

    if not is_pose_in_collision(rotation_only_pose, world):
        return rotation_only_state, True

    stopped_state = RoverState(
        x=state.x,
        y=state.y,
        yaw=state.yaw,
        linear_speed=0.0,
        angular_speed=0.0,
    )
    return stopped_state, True
