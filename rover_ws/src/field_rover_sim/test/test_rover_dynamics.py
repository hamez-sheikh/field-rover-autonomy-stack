"""Unit tests for the field rover motion mathematics."""

import math

from field_rover_sim.rover_dynamics import (
    calculate_limited_angular_speed,
    calculate_limited_linear_speed,
    clamp_requested_angular_speed,
    clamp_requested_linear_speed,
    DEFAULT_MOTION_LIMITS,
    integrate_pose,
    normalize_yaw,
    RoverState,
    update_rover_state,
)

from field_rover_sim.world_model import DEFAULT_WORLD

import pytest


def test_zero_velocity_leaves_pose_unchanged():
    """Confirm zero velocity does not move or rotate the rover."""
    state = RoverState(x=2.0, y=2.0, yaw=0.4)

    result = integrate_pose(state, 0.0, 0.0, 1.0)

    assert result.x == pytest.approx(2.0)
    assert result.y == pytest.approx(2.0)
    assert result.yaw == pytest.approx(0.4)


def test_forward_motion_at_zero_yaw_increases_x():
    """Confirm forward motion along zero yaw increases x only."""
    state = RoverState(x=2.0, y=2.0, yaw=0.0)

    result = integrate_pose(state, 0.5, 0.0, 2.0)

    assert result.x == pytest.approx(3.0)
    assert result.y == pytest.approx(2.0)


def test_forward_motion_at_half_pi_increases_y():
    """Confirm forward motion at pi over two increases y only."""
    state = RoverState(x=2.0, y=2.0, yaw=math.pi / 2.0)

    result = integrate_pose(state, 0.5, 0.0, 2.0)

    assert result.x == pytest.approx(2.0)
    assert result.y == pytest.approx(3.0)


def test_negative_linear_speed_moves_backward():
    """Confirm negative linear speed moves opposite the heading."""
    state = RoverState(x=2.0, y=2.0, yaw=0.0)

    result = integrate_pose(state, -0.5, 0.0, 1.0)

    assert result.x == pytest.approx(1.5)
    assert result.y == pytest.approx(2.0)


def test_positive_angular_speed_increases_yaw():
    """Confirm positive angular speed rotates counter-clockwise."""
    state = RoverState(x=2.0, y=2.0, yaw=0.0)

    result = integrate_pose(state, 0.0, 0.5, 1.0)

    assert result.yaw == pytest.approx(0.5)


def test_negative_angular_speed_decreases_yaw():
    """Confirm negative angular speed rotates clockwise."""
    state = RoverState(x=2.0, y=2.0, yaw=0.0)

    result = integrate_pose(state, 0.0, -0.5, 1.0)

    assert result.yaw == pytest.approx(-0.5)


def test_yaw_above_pi_wraps_to_negative_side():
    """Confirm angles greater than pi wrap into the required interval."""
    result = normalize_yaw(math.pi + 0.25)

    assert result == pytest.approx(-math.pi + 0.25)


def test_yaw_below_negative_pi_wraps_to_positive_side():
    """Confirm angles below negative pi wrap into the required interval."""
    result = normalize_yaw(-math.pi - 0.25)

    assert result == pytest.approx(math.pi - 0.25)


def test_linear_acceleration_is_limited():
    """Confirm forward speed increases only by acceleration times dt."""
    result = calculate_limited_linear_speed(
        current_speed=0.0,
        requested_speed=1.0,
        limits=DEFAULT_MOTION_LIMITS,
        dt=1.0,
    )

    assert result == pytest.approx(0.5)


def test_linear_deceleration_is_limited():
    """Confirm stopping uses the configured linear deceleration."""
    result = calculate_limited_linear_speed(
        current_speed=1.0,
        requested_speed=0.0,
        limits=DEFAULT_MOTION_LIMITS,
        dt=0.5,
    )

    assert result == pytest.approx(0.6)


def test_direction_reversal_passes_through_zero():
    """Confirm reversing decelerates to zero before moving backward."""
    stopped_speed = calculate_limited_linear_speed(
        current_speed=0.4,
        requested_speed=-0.5,
        limits=DEFAULT_MOTION_LIMITS,
        dt=1.0,
    )
    reversing_speed = calculate_limited_linear_speed(
        current_speed=stopped_speed,
        requested_speed=-0.5,
        limits=DEFAULT_MOTION_LIMITS,
        dt=0.2,
    )

    assert stopped_speed == pytest.approx(0.0)
    assert reversing_speed == pytest.approx(-0.1)


def test_forward_request_is_clamped():
    """Confirm excessive forward requests use the forward speed limit."""
    result = clamp_requested_linear_speed(5.0, DEFAULT_MOTION_LIMITS)

    assert result == pytest.approx(1.0)


def test_reverse_request_is_clamped():
    """Confirm excessive reverse requests use the reverse speed limit."""
    result = clamp_requested_linear_speed(-5.0, DEFAULT_MOTION_LIMITS)

    assert result == pytest.approx(-0.5)


@pytest.mark.parametrize(
    ('requested_speed', 'expected_speed'),
    [
        (5.0, 1.0),
        (-5.0, -1.0),
    ],
)
def test_turn_rate_request_is_clamped(requested_speed, expected_speed):
    """Confirm left and right requests use the turn-rate limit."""
    result = clamp_requested_angular_speed(
        requested_speed,
        DEFAULT_MOTION_LIMITS,
    )

    assert result == pytest.approx(expected_speed)


def test_angular_acceleration_is_limited():
    """Confirm turn rate changes only by angular acceleration times dt."""
    result = calculate_limited_angular_speed(
        current_speed=0.0,
        requested_speed=1.0,
        limits=DEFAULT_MOTION_LIMITS,
        dt=0.2,
    )

    assert result == pytest.approx(0.3)


@pytest.mark.parametrize('dt', [0.0, -0.1])
def test_non_positive_dt_preserves_motion_state(dt):
    """Confirm a non-positive time step cannot change the state."""
    state = RoverState(
        x=2.0,
        y=2.0,
        yaw=0.4,
        linear_speed=0.2,
        angular_speed=0.1,
    )

    limited_speed = calculate_limited_linear_speed(
        current_speed=state.linear_speed,
        requested_speed=1.0,
        limits=DEFAULT_MOTION_LIMITS,
        dt=dt,
    )
    integrated_state = integrate_pose(
        state,
        state.linear_speed,
        state.angular_speed,
        dt,
    )

    assert limited_speed == pytest.approx(0.2)
    assert integrated_state == state


def test_collision_free_motion_is_accepted():
    """Confirm a valid candidate motion becomes the new rover state."""
    state = RoverState(
        x=2.0,
        y=2.0,
        yaw=0.0,
        linear_speed=0.5,
    )

    result, collision_occurred = update_rover_state(
        state=state,
        requested_linear_speed=0.5,
        requested_angular_speed=0.0,
        limits=DEFAULT_MOTION_LIMITS,
        world=DEFAULT_WORLD,
        dt=1.0,
    )

    assert not collision_occurred
    assert result.x == pytest.approx(2.5)
    assert result.y == pytest.approx(2.0)
    assert result.linear_speed == pytest.approx(0.5)


def test_motion_into_obstacle_is_rejected():
    """Confirm the rover cannot enter a circular obstacle."""
    state = RoverState(
        x=4.5,
        y=4.0,
        yaw=0.0,
        linear_speed=1.0,
    )

    result, collision_occurred = update_rover_state(
        state=state,
        requested_linear_speed=1.0,
        requested_angular_speed=0.0,
        limits=DEFAULT_MOTION_LIMITS,
        world=DEFAULT_WORLD,
        dt=0.2,
    )

    assert collision_occurred
    assert result.x == pytest.approx(4.5)
    assert result.y == pytest.approx(4.0)


def test_motion_into_wall_is_rejected():
    """Confirm the rover cannot cross a world boundary."""
    state = RoverState(
        x=0.41,
        y=2.0,
        yaw=math.pi,
        linear_speed=0.5,
    )

    result, collision_occurred = update_rover_state(
        state=state,
        requested_linear_speed=0.5,
        requested_angular_speed=0.0,
        limits=DEFAULT_MOTION_LIMITS,
        world=DEFAULT_WORLD,
        dt=0.1,
    )

    assert collision_occurred
    assert result.x == pytest.approx(0.41)
    assert result.y == pytest.approx(2.0)


def test_collision_rejection_stops_represented_velocity():
    """Confirm blocked motion preserves pose and stops both velocities."""
    state = RoverState(
        x=4.5,
        y=4.0,
        yaw=0.0,
        linear_speed=1.0,
        angular_speed=0.0,
    )

    result, collision_occurred = update_rover_state(
        state=state,
        requested_linear_speed=1.0,
        requested_angular_speed=0.0,
        limits=DEFAULT_MOTION_LIMITS,
        world=DEFAULT_WORLD,
        dt=0.2,
    )

    assert collision_occurred
    assert result.x == pytest.approx(state.x)
    assert result.y == pytest.approx(state.y)
    assert result.yaw == pytest.approx(state.yaw)
    assert result.linear_speed == pytest.approx(0.0)
    assert result.angular_speed == pytest.approx(0.0)


def test_in_place_rotation_is_allowed():
    """Confirm a circular rover can rotate without changing position."""
    state = RoverState(x=4.5, y=4.0, yaw=0.0)

    result, collision_occurred = update_rover_state(
        state=state,
        requested_linear_speed=0.0,
        requested_angular_speed=1.0,
        limits=DEFAULT_MOTION_LIMITS,
        world=DEFAULT_WORLD,
        dt=0.2,
    )

    assert not collision_occurred
    assert result.x == pytest.approx(4.5)
    assert result.y == pytest.approx(4.0)
    assert result.yaw == pytest.approx(0.06)
    assert result.angular_speed == pytest.approx(0.3)


def test_blocked_translation_can_continue_rotating():
    """Confirm collision stops translation but permits safe rotation."""
    state = RoverState(
        x=4.5,
        y=4.0,
        yaw=0.0,
        linear_speed=1.0,
        angular_speed=0.0,
    )

    result, collision_occurred = update_rover_state(
        state=state,
        requested_linear_speed=1.0,
        requested_angular_speed=1.0,
        limits=DEFAULT_MOTION_LIMITS,
        world=DEFAULT_WORLD,
        dt=0.2,
    )

    assert collision_occurred
    assert result.x == pytest.approx(4.5)
    assert result.y == pytest.approx(4.0)
    assert result.yaw == pytest.approx(0.06)
    assert result.linear_speed == pytest.approx(0.0)
    assert result.angular_speed == pytest.approx(0.3)
