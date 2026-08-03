"""Unit tests for the pure Python complementary-style pose estimator."""

import math

from field_rover_localization.pose_estimator import (
    angle_difference,
    apply_gps_correction,
    build_pose_covariance,
    build_twist_covariance,
    calculate_fused_yaw_rate,
    DEFAULT_LOCALIZATION_CONFIG,
    is_gps_measurement_new,
    is_imu_measurement_usable,
    LocalizationConfig,
    LocalizationState,
    normalize_yaw,
    predict_state,
    UNMODELLED_POSE_VARIANCE,
    UNMODELLED_TWIST_VARIANCE,
)

import pytest


def _valid_kwargs(**overrides):
    kwargs = {
        'imu_yaw_rate_weight': 0.70,
        'imu_gyro_bias_correction': 0.01,
        'imu_timeout_s': 0.20,
        'gps_position_gain': 0.20,
        'max_gps_innovation_m': 5.0,
        'max_dt': 0.50,
        'reference_latitude_deg': 43.2609,
        'reference_longitude_deg': -79.9192,
        'frame_id': 'map',
        'child_frame_id': 'base_link',
        'position_variance': 0.50,
        'yaw_variance': 0.05,
        'linear_velocity_variance': 0.10,
        'angular_velocity_variance': 0.02,
    }
    kwargs.update(overrides)
    return kwargs


# --- Angle utilities ------------------------------------------------------


def test_normalize_yaw_wraps_above_pi():
    """Confirm yaw normalization wraps a value above pi."""
    assert normalize_yaw(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_normalize_yaw_wraps_below_negative_pi():
    """Confirm yaw normalization wraps a value below negative pi."""
    assert normalize_yaw(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_normalize_yaw_leaves_in_range_value_unchanged():
    """Confirm an already-valid yaw is left unchanged by normalization."""
    assert normalize_yaw(0.3) == pytest.approx(0.3)


def test_angle_difference_returns_shortest_positive_delta():
    """Confirm angle_difference returns a small positive delta directly."""
    assert angle_difference(0.5, 0.2) == pytest.approx(0.3)


def test_angle_difference_returns_shortest_negative_delta():
    """Confirm angle_difference returns a small negative delta directly."""
    assert angle_difference(0.2, 0.5) == pytest.approx(-0.3)


def test_angle_difference_wraps_across_the_pi_boundary():
    """Confirm angle_difference takes the short way across +/-pi."""
    difference = angle_difference(-math.pi + 0.1, math.pi - 0.1)
    assert difference == pytest.approx(0.2)


# --- Configuration validation ----------------------------------------------


def test_default_configuration_matches_documented_recommended_values():
    """Confirm the shipped default configuration matches the recommended values."""
    config = DEFAULT_LOCALIZATION_CONFIG
    assert config.imu_yaw_rate_weight == pytest.approx(0.70)
    assert config.imu_gyro_bias_correction == pytest.approx(0.01)
    assert config.imu_timeout_s == pytest.approx(0.20)
    assert config.gps_position_gain == pytest.approx(0.20)
    assert config.max_gps_innovation_m == pytest.approx(5.0)
    assert config.max_dt == pytest.approx(0.50)
    assert config.reference_latitude_deg == pytest.approx(43.2609)
    assert config.reference_longitude_deg == pytest.approx(-79.9192)
    assert config.frame_id == 'map'
    assert config.child_frame_id == 'base_link'
    assert config.position_variance == pytest.approx(0.50)
    assert config.yaw_variance == pytest.approx(0.05)
    assert config.linear_velocity_variance == pytest.approx(0.10)
    assert config.angular_velocity_variance == pytest.approx(0.02)


def test_valid_configuration_is_accepted():
    """Confirm a fully valid configuration passes validation."""
    LocalizationConfig(**_valid_kwargs())


def test_imu_weight_below_zero_is_rejected():
    """Confirm an IMU yaw-rate weight below zero fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(imu_yaw_rate_weight=-0.01))


def test_imu_weight_above_one_is_rejected():
    """Confirm an IMU yaw-rate weight above one fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(imu_yaw_rate_weight=1.01))


def test_gps_gain_below_zero_is_rejected():
    """Confirm a GPS position gain below zero fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(gps_position_gain=-0.01))


def test_gps_gain_above_one_is_rejected():
    """Confirm a GPS position gain above one fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(gps_position_gain=1.01))


def test_zero_imu_timeout_is_rejected():
    """Confirm a zero IMU timeout fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(imu_timeout_s=0.0))


def test_negative_imu_timeout_is_rejected():
    """Confirm a negative IMU timeout fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(imu_timeout_s=-0.1))


def test_zero_max_dt_is_rejected():
    """Confirm a zero maximum dt fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(max_dt=0.0))


def test_negative_max_dt_is_rejected():
    """Confirm a negative maximum dt fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(max_dt=-0.5))


def test_zero_max_gps_innovation_is_rejected():
    """Confirm a zero GPS innovation gate fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(max_gps_innovation_m=0.0))


def test_negative_max_gps_innovation_is_rejected():
    """Confirm a negative GPS innovation gate fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(max_gps_innovation_m=-5.0))


def test_non_finite_imu_gyro_bias_correction_is_rejected():
    """Confirm a non-finite gyro bias correction fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(imu_gyro_bias_correction=math.nan))


def test_invalid_reference_latitude_is_rejected():
    """Confirm an out-of-range reference latitude fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(reference_latitude_deg=95.0))


def test_invalid_reference_longitude_is_rejected():
    """Confirm an out-of-range reference longitude fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(reference_longitude_deg=185.0))


def test_pole_unsafe_reference_latitude_is_rejected():
    """Confirm a reference latitude too close to a pole fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(reference_latitude_deg=89.999))


def test_empty_frame_id_is_rejected():
    """Confirm an empty frame_id fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(frame_id=''))


def test_empty_child_frame_id_is_rejected():
    """Confirm an empty child_frame_id fails validation."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(child_frame_id=''))


@pytest.mark.parametrize('variance_name', [
    'position_variance', 'yaw_variance',
    'linear_velocity_variance', 'angular_velocity_variance',
])
def test_negative_variance_is_rejected(variance_name):
    """Confirm each output variance rejects a negative value."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(**{variance_name: -0.1}))


@pytest.mark.parametrize('variance_name', [
    'position_variance', 'yaw_variance',
    'linear_velocity_variance', 'angular_velocity_variance',
])
def test_non_finite_variance_is_rejected(variance_name):
    """Confirm each output variance rejects a non-finite value."""
    with pytest.raises(ValueError):
        LocalizationConfig(**_valid_kwargs(**{variance_name: math.nan}))


# --- Initialization ---------------------------------------------------------


def test_state_constructed_from_a_wheel_pose_stores_it_exactly():
    """Confirm a state seeded from a wheel pose stores x/y/yaw unchanged."""
    state = LocalizationState(x=1.5, y=-2.5, yaw=0.3)

    assert state.x == pytest.approx(1.5)
    assert state.y == pytest.approx(-2.5)
    assert state.yaw == pytest.approx(0.3)


def test_state_seeded_from_a_pose_reports_zero_velocity():
    """Confirm seeding a state from a pose alone does not fabricate velocity."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    assert state.linear_velocity == pytest.approx(0.0)
    assert state.angular_velocity == pytest.approx(0.0)


# --- Prediction --------------------------------------------------------------


def test_stationary_pose_remains_unchanged():
    """Confirm zero linear and angular velocity leaves the pose unchanged."""
    state = LocalizationState(x=2.0, y=2.0, yaw=0.4)

    result = predict_state(
        state, wheel_linear_velocity=0.0, wheel_angular_velocity=0.0,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.3,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(2.0)
    assert result.y == pytest.approx(2.0)
    assert result.yaw == pytest.approx(0.4)


def test_forward_motion_at_zero_yaw_increases_x_only():
    """Confirm straight travel at yaw zero increases x, not y."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    result = predict_state(
        state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.0,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.4,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(0.4)
    assert result.y == pytest.approx(0.0, abs=1e-9)


def test_forward_motion_at_half_pi_yaw_increases_y_only():
    """Confirm straight travel at yaw pi/2 increases y, not x."""
    state = LocalizationState(x=0.0, y=0.0, yaw=math.pi / 2.0)

    result = predict_state(
        state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.0,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.4,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(0.0, abs=1e-9)
    assert result.y == pytest.approx(0.4)


def test_reverse_motion_decreases_x_at_zero_yaw():
    """Confirm reverse travel at yaw zero decreases x."""
    state = LocalizationState(x=2.0, y=0.0, yaw=0.0)

    result = predict_state(
        state, wheel_linear_velocity=-1.0, wheel_angular_velocity=0.0,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.4,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(1.6)
    assert result.y == pytest.approx(0.0, abs=1e-9)


def test_positive_turn_increases_yaw():
    """Confirm positive angular velocity increases yaw."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    result = predict_state(
        state, wheel_linear_velocity=0.0, wheel_angular_velocity=0.5,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.3,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.yaw > 0.0


def test_negative_turn_decreases_yaw():
    """Confirm negative angular velocity decreases yaw."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    result = predict_state(
        state, wheel_linear_velocity=0.0, wheel_angular_velocity=-0.5,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.3,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.yaw < 0.0


def test_yaw_wraps_above_pi():
    """Confirm predicted yaw wraps once it exceeds pi."""
    state = LocalizationState(x=0.0, y=0.0, yaw=math.pi - 0.1)

    result = predict_state(
        state, wheel_linear_velocity=0.0, wheel_angular_velocity=1.0,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.3,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert -math.pi <= result.yaw < math.pi
    assert result.yaw == pytest.approx(math.pi - 0.1 + 0.3 - 2.0 * math.pi)


def test_yaw_wraps_below_negative_pi():
    """Confirm predicted yaw wraps once it falls below negative pi."""
    state = LocalizationState(x=0.0, y=0.0, yaw=-math.pi + 0.1)

    result = predict_state(
        state, wheel_linear_velocity=0.0, wheel_angular_velocity=-1.0,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.3,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert -math.pi <= result.yaw < math.pi
    assert result.yaw == pytest.approx(-math.pi + 0.1 - 0.3 + 2.0 * math.pi)


def test_curved_motion_matches_the_documented_midpoint_formula():
    """Confirm curved motion matches the documented midpoint-integration formula."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)
    wheel_linear_velocity = 1.0
    wheel_angular_velocity = 0.5
    dt = 0.4

    result = predict_state(
        state, wheel_linear_velocity, wheel_angular_velocity,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=dt,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    delta_yaw = wheel_angular_velocity * dt
    heading_mid = state.yaw + delta_yaw / 2.0
    delta_distance = wheel_linear_velocity * dt

    assert result.x == pytest.approx(delta_distance * math.cos(heading_mid))
    assert result.y == pytest.approx(delta_distance * math.sin(heading_mid))
    assert result.yaw == pytest.approx(delta_yaw)


def test_repeated_updates_accumulate_position():
    """Confirm repeated straight-travel updates keep advancing x."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)
    previous_x = state.x

    for _ in range(5):
        state = predict_state(
            state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.0,
            imu_angular_velocity_z=None, imu_is_usable=False, dt=0.5,
            config=DEFAULT_LOCALIZATION_CONFIG,
        )
        assert state.x > previous_x
        previous_x = state.x


@pytest.mark.parametrize('dt', [0.0, -0.1])
def test_non_positive_dt_does_not_integrate(dt):
    """Confirm a non-positive dt cannot move the pose or fabricate velocity."""
    state = LocalizationState(x=1.0, y=1.0, yaw=0.2)

    result = predict_state(
        state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.5,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=dt,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(1.0)
    assert result.yaw == pytest.approx(0.2)
    assert result.linear_velocity == pytest.approx(0.0)
    assert result.angular_velocity == pytest.approx(0.0)


def test_excessive_dt_does_not_integrate():
    """Confirm a dt beyond max_dt cannot move the pose or fabricate velocity."""
    state = LocalizationState(x=1.0, y=1.0, yaw=0.2)

    result = predict_state(
        state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.5,
        imu_angular_velocity_z=None, imu_is_usable=False,
        dt=DEFAULT_LOCALIZATION_CONFIG.max_dt + 0.01,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(1.0)
    assert result.linear_velocity == pytest.approx(0.0)
    assert result.angular_velocity == pytest.approx(0.0)


# --- IMU staleness and fusion ------------------------------------------------


def test_imu_usable_when_fresh():
    """Confirm a zero-age IMU sample is usable."""
    assert is_imu_measurement_usable(10.0, 10.0, DEFAULT_LOCALIZATION_CONFIG)


def test_imu_exactly_at_timeout_boundary_is_usable():
    """Confirm an IMU sample exactly at the timeout boundary is still usable."""
    wheel_stamp = 10.0 + DEFAULT_LOCALIZATION_CONFIG.imu_timeout_s
    assert is_imu_measurement_usable(
        10.0, wheel_stamp, DEFAULT_LOCALIZATION_CONFIG,
    )


def test_imu_just_past_timeout_boundary_is_not_usable():
    """Confirm an IMU sample just past the timeout boundary is stale."""
    wheel_stamp = 10.0 + DEFAULT_LOCALIZATION_CONFIG.imu_timeout_s + 0.001
    assert not is_imu_measurement_usable(
        10.0, wheel_stamp, DEFAULT_LOCALIZATION_CONFIG,
    )


def test_missing_imu_stamp_is_not_usable():
    """Confirm a missing (None) IMU stamp is never usable."""
    assert not is_imu_measurement_usable(
        None, 10.0, DEFAULT_LOCALIZATION_CONFIG,
    )


def test_future_dated_imu_is_not_usable():
    """Confirm an IMU stamp ahead of the wheel stamp is not trusted."""
    assert not is_imu_measurement_usable(
        10.5, 10.0, DEFAULT_LOCALIZATION_CONFIG,
    )


def test_zero_weight_uses_wheel_rate_only():
    """Confirm a zero IMU weight ignores the IMU rate entirely."""
    config = LocalizationConfig(**_valid_kwargs(imu_yaw_rate_weight=0.0))

    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=0.3, imu_angular_velocity_z=5.0,
        imu_is_usable=True, config=config,
    )

    assert fused == pytest.approx(0.3)


def test_weight_one_uses_corrected_imu_rate_only():
    """Confirm a weight of one uses the bias-corrected IMU rate exactly."""
    config = LocalizationConfig(**_valid_kwargs(
        imu_yaw_rate_weight=1.0, imu_gyro_bias_correction=0.02,
    ))

    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=0.3, imu_angular_velocity_z=0.5,
        imu_is_usable=True, config=config,
    )

    assert fused == pytest.approx(0.48)


def test_intermediate_weight_blends_wheel_and_imu_rates():
    """Confirm an intermediate weight linearly blends both rates."""
    config = LocalizationConfig(**_valid_kwargs(
        imu_yaw_rate_weight=0.7, imu_gyro_bias_correction=0.0,
    ))

    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=1.0, imu_angular_velocity_z=0.0,
        imu_is_usable=True, config=config,
    )

    assert fused == pytest.approx(0.3)


def test_gyro_bias_is_subtracted_before_blending():
    """Confirm the configured gyro bias is subtracted from the raw IMU rate."""
    config = LocalizationConfig(**_valid_kwargs(
        imu_yaw_rate_weight=1.0, imu_gyro_bias_correction=0.1,
    ))

    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=0.0, imu_angular_velocity_z=0.1,
        imu_is_usable=True, config=config,
    )

    assert fused == pytest.approx(0.0, abs=1e-9)


def test_missing_imu_value_falls_back_to_wheel_rate():
    """Confirm a None IMU value falls back to the wheel rate even if usable."""
    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=0.4, imu_angular_velocity_z=None,
        imu_is_usable=True, config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert fused == pytest.approx(0.4)


def test_stale_imu_falls_back_to_wheel_rate():
    """Confirm imu_is_usable=False falls back to the wheel rate."""
    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=0.4, imu_angular_velocity_z=5.0,
        imu_is_usable=False, config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert fused == pytest.approx(0.4)


def test_invalid_imu_value_falls_back_to_wheel_rate():
    """Confirm a non-finite IMU value falls back to the wheel rate."""
    fused = calculate_fused_yaw_rate(
        wheel_angular_velocity=0.4, imu_angular_velocity_z=math.nan,
        imu_is_usable=True, config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert fused == pytest.approx(0.4)


def test_predict_state_uses_fused_rate_when_imu_is_usable():
    """Confirm predict_state's yaw change reflects the fused, not wheel-only, rate."""
    config = LocalizationConfig(**_valid_kwargs(
        imu_yaw_rate_weight=1.0, imu_gyro_bias_correction=0.0,
    ))
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    result = predict_state(
        state, wheel_linear_velocity=0.0, wheel_angular_velocity=0.0,
        imu_angular_velocity_z=1.0, imu_is_usable=True, dt=0.4,
        config=config,
    )

    assert result.yaw == pytest.approx(0.4)
    assert result.angular_velocity == pytest.approx(1.0)


# --- GPS correction -----------------------------------------------------------


def test_gps_gain_zero_does_not_move_position():
    """Confirm a GPS gain of zero leaves position unchanged despite an accepted fix."""
    config = LocalizationConfig(**_valid_kwargs(gps_position_gain=0.0))
    state = LocalizationState(x=1.0, y=2.0, yaw=0.0)

    result = apply_gps_correction(state, 3.0, 3.0, config)

    assert result.applied is True
    assert result.state.x == pytest.approx(1.0)
    assert result.state.y == pytest.approx(2.0)


def test_gps_gain_one_moves_position_fully_to_the_fix():
    """Confirm a GPS gain of one moves position all the way to the fix."""
    config = LocalizationConfig(**_valid_kwargs(gps_position_gain=1.0))
    state = LocalizationState(x=0.0, y=0.0, yaw=0.7)

    result = apply_gps_correction(state, 2.0, -1.0, config)

    assert result.state.x == pytest.approx(2.0)
    assert result.state.y == pytest.approx(-1.0)


def test_gps_intermediate_gain_partially_moves_toward_the_fix():
    """Confirm the default intermediate gain applies a partial correction."""
    result = apply_gps_correction(
        LocalizationState(x=0.0, y=0.0, yaw=0.0), 2.0, 0.0,
        DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.state.x == pytest.approx(0.4)
    assert result.state.y == pytest.approx(0.0)


def test_gps_correction_never_changes_yaw():
    """Confirm applying a GPS correction never alters yaw."""
    state = LocalizationState(x=0.0, y=0.0, yaw=1.2)

    result = apply_gps_correction(
        state, 1.0, 1.0, DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.state.yaw == pytest.approx(1.2)


def test_gps_correction_never_changes_velocity():
    """Confirm applying a GPS correction never alters reported velocity."""
    state = LocalizationState(
        x=0.0, y=0.0, yaw=0.0, linear_velocity=1.5, angular_velocity=0.2,
    )

    result = apply_gps_correction(
        state, 1.0, 1.0, DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.state.linear_velocity == pytest.approx(1.5)
    assert result.state.angular_velocity == pytest.approx(0.2)


def test_gps_innovation_within_gate_is_accepted():
    """Confirm an innovation comfortably inside the gate is accepted."""
    result = apply_gps_correction(
        LocalizationState(x=0.0, y=0.0, yaw=0.0), 1.0, 1.0,
        DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.applied is True


def test_gps_innovation_exactly_at_the_gate_is_accepted():
    """Confirm an innovation exactly at the configured gate is accepted."""
    config = DEFAULT_LOCALIZATION_CONFIG
    result = apply_gps_correction(
        LocalizationState(x=0.0, y=0.0, yaw=0.0),
        config.max_gps_innovation_m, 0.0, config,
    )

    assert result.applied is True
    assert result.innovation_distance == pytest.approx(
        config.max_gps_innovation_m,
    )


def test_gps_innovation_beyond_the_gate_is_rejected():
    """Confirm an innovation beyond the configured gate is rejected outright."""
    config = DEFAULT_LOCALIZATION_CONFIG
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    result = apply_gps_correction(
        state, config.max_gps_innovation_m + 0.01, 0.0, config,
    )

    assert result.applied is False
    assert result.state.x == pytest.approx(0.0)
    assert result.state.y == pytest.approx(0.0)


def test_non_finite_gps_coordinates_are_rejected():
    """Confirm a non-finite GPS coordinate is rejected without side effects."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    result = apply_gps_correction(
        state, math.nan, 0.0, DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.applied is False
    assert result.state.x == pytest.approx(0.0)
    assert math.isnan(result.innovation_distance)


def test_repeated_corrections_approach_the_gps_fix_gradually():
    """Confirm repeated corrections shrink the remaining GPS distance gradually."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)
    gps_east_m, gps_north_m = 3.0, 3.0

    previous_distance = math.hypot(gps_east_m, gps_north_m)
    for _ in range(20):
        result = apply_gps_correction(
            state, gps_east_m, gps_north_m, DEFAULT_LOCALIZATION_CONFIG,
        )
        state = result.state
        distance = math.hypot(gps_east_m - state.x, gps_north_m - state.y)
        assert distance < previous_distance
        previous_distance = distance

    assert previous_distance < 0.1


def test_prediction_continues_without_any_gps_correction():
    """Confirm dead reckoning keeps advancing without any GPS correction call."""
    state = LocalizationState(x=0.0, y=0.0, yaw=0.0)

    for _ in range(50):
        state = predict_state(
            state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.0,
            imu_angular_velocity_z=None, imu_is_usable=False, dt=0.1,
            config=DEFAULT_LOCALIZATION_CONFIG,
        )

    assert state.x == pytest.approx(5.0)
    assert math.isfinite(state.y)


# --- Ordering: GPS duplicate/out-of-order rejection --------------------------


def test_gps_measurement_is_new_when_there_is_no_previous_stamp():
    """Confirm the first-ever GPS fix is always treated as new."""
    assert is_gps_measurement_new(10.0, None)


def test_duplicate_gps_stamp_is_not_new():
    """Confirm a GPS fix repeating the last-applied stamp is not new."""
    assert not is_gps_measurement_new(10.0, 10.0)


def test_older_gps_stamp_is_not_new():
    """Confirm a GPS fix older than the last-applied stamp is not new."""
    assert not is_gps_measurement_new(9.0, 10.0)


def test_newer_gps_stamp_is_new():
    """Confirm a GPS fix newer than the last-applied stamp is new."""
    assert is_gps_measurement_new(10.5, 10.0)


def test_non_finite_gps_stamp_is_not_new():
    """Confirm a non-finite GPS stamp is never treated as new."""
    assert not is_gps_measurement_new(math.nan, 10.0)


# --- Ordering: wheel timestamp handling ---------------------------------


def test_duplicate_wheel_timestamp_does_not_integrate():
    """Confirm a repeated wheel timestamp (dt=0) produces no motion."""
    state = LocalizationState(x=1.0, y=1.0, yaw=0.3)

    result = predict_state(
        state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.5,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=0.0,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(1.0)
    assert result.yaw == pytest.approx(0.3)


def test_out_of_order_wheel_timestamp_does_not_create_a_jump():
    """Confirm a wheel message older than the last one (negative dt) causes no jump."""
    state = LocalizationState(x=2.0, y=-1.0, yaw=0.1)

    result = predict_state(
        state, wheel_linear_velocity=1.0, wheel_angular_velocity=0.5,
        imu_angular_velocity_z=None, imu_is_usable=False, dt=-0.05,
        config=DEFAULT_LOCALIZATION_CONFIG,
    )

    assert result.x == pytest.approx(2.0)
    assert result.y == pytest.approx(-1.0)
    assert result.yaw == pytest.approx(0.1)


# --- Covariance ---------------------------------------------------------


def test_pose_covariance_has_36_entries():
    """Confirm the pose covariance is a flat 6x6 (36-entry) matrix."""
    assert len(build_pose_covariance(DEFAULT_LOCALIZATION_CONFIG)) == 36


def test_twist_covariance_has_36_entries():
    """Confirm the twist covariance is a flat 6x6 (36-entry) matrix."""
    assert len(build_twist_covariance(DEFAULT_LOCALIZATION_CONFIG)) == 36


def test_pose_covariance_diagonal_positions_match_config():
    """Confirm x/y/yaw diagonal entries match the configured variances."""
    covariance = build_pose_covariance(DEFAULT_LOCALIZATION_CONFIG)

    assert covariance[0] == pytest.approx(
        DEFAULT_LOCALIZATION_CONFIG.position_variance,
    )
    assert covariance[7] == pytest.approx(
        DEFAULT_LOCALIZATION_CONFIG.position_variance,
    )
    assert covariance[35] == pytest.approx(
        DEFAULT_LOCALIZATION_CONFIG.yaw_variance,
    )


def test_twist_covariance_diagonal_positions_match_config():
    """Confirm linear-x/angular-z diagonal entries match configured variances."""
    covariance = build_twist_covariance(DEFAULT_LOCALIZATION_CONFIG)

    assert covariance[0] == pytest.approx(
        DEFAULT_LOCALIZATION_CONFIG.linear_velocity_variance,
    )
    assert covariance[35] == pytest.approx(
        DEFAULT_LOCALIZATION_CONFIG.angular_velocity_variance,
    )


def test_pose_covariance_uses_large_uncertainty_for_unmodelled_axes():
    """Confirm z/roll/pitch use the large unmodelled-axis variance."""
    covariance = build_pose_covariance(DEFAULT_LOCALIZATION_CONFIG)

    assert covariance[14] == pytest.approx(UNMODELLED_POSE_VARIANCE)
    assert covariance[21] == pytest.approx(UNMODELLED_POSE_VARIANCE)
    assert covariance[28] == pytest.approx(UNMODELLED_POSE_VARIANCE)


def test_twist_covariance_uses_large_uncertainty_for_unmodelled_axes():
    """Confirm linear-y/z and angular-x/y use the large unmodelled variance."""
    covariance = build_twist_covariance(DEFAULT_LOCALIZATION_CONFIG)

    for index in (7, 14, 21, 28):
        assert covariance[index] == pytest.approx(UNMODELLED_TWIST_VARIANCE)


def test_pose_covariance_is_not_all_zero():
    """Confirm the pose covariance is not left as an all-zero placeholder."""
    covariance = build_pose_covariance(DEFAULT_LOCALIZATION_CONFIG)
    assert any(value != 0.0 for value in covariance)


def test_twist_covariance_is_not_all_zero():
    """Confirm the twist covariance is not left as an all-zero placeholder."""
    covariance = build_twist_covariance(DEFAULT_LOCALIZATION_CONFIG)
    assert any(value != 0.0 for value in covariance)


def test_covariance_values_are_finite_and_non_negative():
    """Confirm every covariance entry is finite and non-negative."""
    for covariance in (
        build_pose_covariance(DEFAULT_LOCALIZATION_CONFIG),
        build_twist_covariance(DEFAULT_LOCALIZATION_CONFIG),
    ):
        for value in covariance:
            assert math.isfinite(value)
            assert value >= 0.0
