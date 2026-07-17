"""Unit tests for the differential-drive wheel-odometry model."""

import math

from field_rover_sim.wheel_odometry import (
    calculate_measured_wheel_increments,
    calculate_wheel_velocities,
    DEFAULT_WHEEL_ODOMETRY_CONFIG,
    integrate_wheel_odometry,
    WheelOdometryConfig,
    WheelOdometryState,
)

import pytest


PERFECT_CONFIG = WheelOdometryConfig(
    wheel_track_width=0.6,
    left_wheel_scale=1.0,
    right_wheel_scale=1.0,
)


def test_zero_track_width_is_rejected():
    """Confirm a zero track width fails configuration validation."""
    with pytest.raises(ValueError):
        WheelOdometryConfig(
            wheel_track_width=0.0,
            left_wheel_scale=1.0,
            right_wheel_scale=1.0,
        )


def test_negative_track_width_is_rejected():
    """Confirm a negative track width fails configuration validation."""
    with pytest.raises(ValueError):
        WheelOdometryConfig(
            wheel_track_width=-0.6,
            left_wheel_scale=1.0,
            right_wheel_scale=1.0,
        )


def test_out_of_range_wheel_scale_is_rejected():
    """Confirm a wheel scale far outside the reasonable range is rejected."""
    with pytest.raises(ValueError):
        WheelOdometryConfig(
            wheel_track_width=0.6,
            left_wheel_scale=5.0,
            right_wheel_scale=1.0,
        )


def test_straight_forward_motion_produces_equal_wheel_velocities():
    """Confirm zero angular velocity gives equal left and right wheel speeds."""
    left, right = calculate_wheel_velocities(1.0, 0.0, PERFECT_CONFIG)

    assert left == pytest.approx(1.0)
    assert right == pytest.approx(1.0)


def test_straight_reverse_motion_produces_equal_negative_wheel_velocities():
    """Confirm reverse straight motion gives equal negative wheel speeds."""
    left, right = calculate_wheel_velocities(-1.0, 0.0, PERFECT_CONFIG)

    assert left == pytest.approx(-1.0)
    assert right == pytest.approx(-1.0)


def test_positive_angular_velocity_makes_right_wheel_faster():
    """Confirm a positive (counter-clockwise) turn speeds up the right wheel."""
    left, right = calculate_wheel_velocities(0.5, 0.5, PERFECT_CONFIG)

    assert right > left


def test_negative_angular_velocity_makes_left_wheel_faster():
    """Confirm a negative (clockwise) turn speeds up the left wheel."""
    left, right = calculate_wheel_velocities(0.5, -0.5, PERFECT_CONFIG)

    assert left > right


def test_in_place_left_turn_produces_opposite_wheel_velocities():
    """Confirm an in-place left turn drives the wheels in opposite directions."""
    left, right = calculate_wheel_velocities(0.0, 1.0, PERFECT_CONFIG)

    assert left == pytest.approx(-0.3)
    assert right == pytest.approx(0.3)


def test_in_place_right_turn_produces_opposite_wheel_velocities():
    """Confirm an in-place right turn drives the wheels in opposite directions."""
    left, right = calculate_wheel_velocities(0.0, -1.0, PERFECT_CONFIG)

    assert left == pytest.approx(0.3)
    assert right == pytest.approx(-0.3)


def test_zero_velocity_leaves_pose_unchanged():
    """Confirm zero commanded velocity does not move the estimated pose."""
    state = WheelOdometryState(x=2.0, y=2.0, yaw=0.4)

    result = integrate_wheel_odometry(state, 0.0, 0.0, PERFECT_CONFIG, dt=1.0)

    assert result.x == pytest.approx(2.0)
    assert result.y == pytest.approx(2.0)
    assert result.yaw == pytest.approx(0.4)
    assert result.linear_velocity == pytest.approx(0.0)
    assert result.angular_velocity == pytest.approx(0.0)


def test_perfect_calibration_straight_motion_increases_x_at_zero_yaw():
    """Confirm ideal straight travel at yaw zero increases x only."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    result = integrate_wheel_odometry(state, 1.0, 0.0, PERFECT_CONFIG, dt=2.0)

    assert result.x == pytest.approx(2.0)
    assert result.y == pytest.approx(0.0, abs=1e-9)


def test_perfect_calibration_at_half_pi_yaw_increases_y():
    """Confirm ideal straight travel at yaw pi/2 increases y only."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=math.pi / 2.0)

    result = integrate_wheel_odometry(state, 1.0, 0.0, PERFECT_CONFIG, dt=2.0)

    assert result.x == pytest.approx(0.0, abs=1e-9)
    assert result.y == pytest.approx(2.0)


def test_reverse_motion_moves_opposite_the_heading():
    """Confirm reverse motion at yaw zero decreases x."""
    state = WheelOdometryState(x=2.0, y=0.0, yaw=0.0)

    result = integrate_wheel_odometry(state, -1.0, 0.0, PERFECT_CONFIG, dt=1.0)

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(0.0, abs=1e-9)


def test_curved_motion_uses_midpoint_integration():
    """Confirm curved motion matches the documented midpoint integration rule."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)
    linear_velocity = 1.0
    angular_velocity = 0.5
    dt = 1.0

    result = integrate_wheel_odometry(
        state, linear_velocity, angular_velocity, PERFECT_CONFIG, dt,
    )

    delta_left, delta_right = calculate_measured_wheel_increments(
        linear_velocity, angular_velocity, PERFECT_CONFIG, dt,
    )
    delta_distance = (delta_right + delta_left) / 2.0
    delta_yaw = (
        delta_right - delta_left
    ) / PERFECT_CONFIG.wheel_track_width
    heading_mid = delta_yaw / 2.0

    assert result.x == pytest.approx(delta_distance * math.cos(heading_mid))
    assert result.y == pytest.approx(delta_distance * math.sin(heading_mid))
    assert result.yaw == pytest.approx(delta_yaw)


def test_positive_turn_increases_yaw():
    """Confirm positive angular velocity increases the estimated yaw."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    result = integrate_wheel_odometry(state, 0.0, 0.5, PERFECT_CONFIG, dt=1.0)

    assert result.yaw > 0.0


def test_negative_turn_decreases_yaw():
    """Confirm negative angular velocity decreases the estimated yaw."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    result = integrate_wheel_odometry(state, 0.0, -0.5, PERFECT_CONFIG, dt=1.0)

    assert result.yaw < 0.0


def test_yaw_wraps_above_pi():
    """Confirm the estimated yaw wraps once it exceeds pi."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=math.pi - 0.1)

    result = integrate_wheel_odometry(state, 0.0, 1.0, PERFECT_CONFIG, dt=0.3)

    assert -math.pi <= result.yaw < math.pi
    assert result.yaw == pytest.approx(math.pi - 0.1 + 0.3 - 2.0 * math.pi)


def test_yaw_wraps_below_negative_pi():
    """Confirm the estimated yaw wraps once it falls below negative pi."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=-math.pi + 0.1)

    result = integrate_wheel_odometry(state, 0.0, -1.0, PERFECT_CONFIG, dt=0.3)

    assert -math.pi <= result.yaw < math.pi
    assert result.yaw == pytest.approx(-math.pi + 0.1 - 0.3 + 2.0 * math.pi)


def test_equal_wheel_scales_preserve_straight_line_heading():
    """Confirm matched calibration keeps yaw at zero during straight travel."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    for _ in range(20):
        state = integrate_wheel_odometry(state, 1.0, 0.0, PERFECT_CONFIG, dt=0.1)

    assert state.yaw == pytest.approx(0.0, abs=1e-9)


def test_mismatched_wheel_scales_create_heading_drift():
    """Confirm mismatched calibration drifts yaw away from zero over time."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    for _ in range(20):
        state = integrate_wheel_odometry(
            state, 1.0, 0.0, DEFAULT_WHEEL_ODOMETRY_CONFIG, dt=0.1,
        )

    assert state.yaw != pytest.approx(0.0, abs=1e-9)


def test_larger_left_scale_drifts_yaw_negative():
    """Confirm a left wheel reporting more travel drifts yaw clockwise."""
    config = WheelOdometryConfig(
        wheel_track_width=0.6,
        left_wheel_scale=1.05,
        right_wheel_scale=0.95,
    )
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    for _ in range(10):
        state = integrate_wheel_odometry(state, 1.0, 0.0, config, dt=0.1)

    assert state.yaw < 0.0


def test_larger_right_scale_drifts_yaw_positive():
    """Confirm a right wheel reporting more travel drifts yaw counter-clockwise."""
    config = WheelOdometryConfig(
        wheel_track_width=0.6,
        left_wheel_scale=0.95,
        right_wheel_scale=1.05,
    )
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    for _ in range(10):
        state = integrate_wheel_odometry(state, 1.0, 0.0, config, dt=0.1)

    assert state.yaw > 0.0


def test_calibration_scale_affects_measured_wheel_distances():
    """Confirm the calibration scale multiplies the true wheel increments."""
    delta_left, delta_right = calculate_measured_wheel_increments(
        1.0, 0.0, DEFAULT_WHEEL_ODOMETRY_CONFIG, dt=1.0,
    )

    assert delta_left == pytest.approx(DEFAULT_WHEEL_ODOMETRY_CONFIG.left_wheel_scale)
    assert delta_right == pytest.approx(
        DEFAULT_WHEEL_ODOMETRY_CONFIG.right_wheel_scale
    )


@pytest.mark.parametrize('dt', [0.0, -0.1])
def test_non_positive_dt_is_handled_safely(dt):
    """Confirm a non-positive time step cannot move the pose or fabricate speed."""
    state = WheelOdometryState(x=1.0, y=1.0, yaw=0.2, linear_velocity=0.3)

    result = integrate_wheel_odometry(
        state, 1.0, 0.5, DEFAULT_WHEEL_ODOMETRY_CONFIG, dt,
    )

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(1.0)
    assert result.yaw == pytest.approx(0.2)
    assert result.linear_velocity == pytest.approx(0.0)
    assert result.angular_velocity == pytest.approx(0.0)


def test_repeated_updates_accumulate_position():
    """Confirm repeated straight-travel updates keep advancing x."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)
    previous_x = state.x

    for _ in range(5):
        state = integrate_wheel_odometry(state, 1.0, 0.0, PERFECT_CONFIG, dt=0.5)
        assert state.x > previous_x
        previous_x = state.x


def test_repeated_updates_accumulate_heading_drift():
    """Confirm mismatched calibration drift grows in magnitude over time."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)
    previous_abs_yaw = 0.0

    for _ in range(10):
        state = integrate_wheel_odometry(
            state, 1.0, 0.0, DEFAULT_WHEEL_ODOMETRY_CONFIG, dt=0.1,
        )
        assert abs(state.yaw) >= previous_abs_yaw
        previous_abs_yaw = abs(state.yaw)

    assert previous_abs_yaw > 0.0


def test_perfect_calibration_matches_analytical_straight_motion():
    """Confirm perfect calibration matches simple analytical straight motion."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    for _ in range(10):
        state = integrate_wheel_odometry(state, 2.0, 0.0, PERFECT_CONFIG, dt=0.1)

    assert state.x == pytest.approx(2.0)
    assert state.y == pytest.approx(0.0, abs=1e-9)
    assert state.yaw == pytest.approx(0.0, abs=1e-9)


def test_wheel_derived_linear_velocity_matches_commanded_speed():
    """Confirm perfect calibration reconstructs the commanded linear speed."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    result = integrate_wheel_odometry(state, 0.75, 0.0, PERFECT_CONFIG, dt=0.2)

    assert result.linear_velocity == pytest.approx(0.75)


def test_wheel_derived_angular_velocity_matches_commanded_turn_rate():
    """Confirm perfect calibration reconstructs the commanded turn rate."""
    state = WheelOdometryState(x=0.0, y=0.0, yaw=0.0)

    result = integrate_wheel_odometry(state, 0.0, 0.4, PERFECT_CONFIG, dt=0.2)

    assert result.angular_velocity == pytest.approx(0.4)


def test_initial_state_is_not_mutated():
    """Confirm integrating returns a new state without mutating the input."""
    state = WheelOdometryState(x=1.0, y=1.0, yaw=0.0)

    integrate_wheel_odometry(state, 1.0, 0.0, PERFECT_CONFIG, dt=1.0)

    assert state.x == pytest.approx(1.0)
    assert state.y == pytest.approx(1.0)
    assert state.yaw == pytest.approx(0.0)


def test_units_remain_consistent_between_velocity_and_distance():
    """Confirm metres-per-second inputs over dt seconds yield metre increments."""
    delta_left, delta_right = calculate_measured_wheel_increments(
        2.0, 0.0, PERFECT_CONFIG, dt=3.0,
    )

    assert delta_left == pytest.approx(6.0)
    assert delta_right == pytest.approx(6.0)
