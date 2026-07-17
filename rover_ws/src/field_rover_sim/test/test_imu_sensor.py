"""Unit tests for the pure Python planar IMU model."""

import math

from field_rover_sim.imu_sensor import (
    calculate_accel_variance,
    calculate_gyro_variance,
    calculate_world_acceleration,
    calculate_world_velocity,
    create_rng,
    DEFAULT_IMU_CONFIG,
    generate_imu_measurement,
    ImuConfig,
    ImuState,
    rotate_world_to_body,
)

import pytest


ZERO_CONFIG = ImuConfig(
    gyro_bias_z=0.0,
    accel_bias_x=0.0,
    accel_bias_y=0.0,
    gyro_noise_stddev=0.0,
    accel_noise_stddev=0.0,
    random_seed=1,
    max_dt=0.5,
)

BIASED_ZERO_NOISE_CONFIG = ImuConfig(
    gyro_bias_z=0.01,
    accel_bias_x=0.03,
    accel_bias_y=-0.02,
    gyro_noise_stddev=0.0,
    accel_noise_stddev=0.0,
    random_seed=1,
    max_dt=0.5,
)


# --- Configuration validation ---------------------------------------------


def test_negative_gyro_noise_stddev_is_rejected():
    """Confirm a negative gyroscope noise standard deviation is rejected."""
    with pytest.raises(ValueError):
        ImuConfig(
            gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
            gyro_noise_stddev=-0.1, accel_noise_stddev=0.0,
            random_seed=1, max_dt=0.5,
        )


def test_negative_accel_noise_stddev_is_rejected():
    """Confirm a negative accelerometer noise standard deviation is rejected."""
    with pytest.raises(ValueError):
        ImuConfig(
            gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
            gyro_noise_stddev=0.0, accel_noise_stddev=-0.1,
            random_seed=1, max_dt=0.5,
        )


@pytest.mark.parametrize('field', ['gyro_bias_z', 'accel_bias_x', 'accel_bias_y'])
def test_non_finite_bias_is_rejected(field):
    """Confirm NaN or infinite bias values fail configuration validation."""
    kwargs = {
        'gyro_bias_z': 0.0, 'accel_bias_x': 0.0, 'accel_bias_y': 0.0,
        'gyro_noise_stddev': 0.0, 'accel_noise_stddev': 0.0,
        'random_seed': 1, 'max_dt': 0.5,
    }
    kwargs[field] = math.nan
    with pytest.raises(ValueError):
        ImuConfig(**kwargs)


def test_non_positive_max_dt_is_rejected():
    """Confirm a non-positive max_dt fails configuration validation."""
    with pytest.raises(ValueError):
        ImuConfig(
            gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
            gyro_noise_stddev=0.0, accel_noise_stddev=0.0,
            random_seed=1, max_dt=0.0,
        )


def test_non_integer_seed_is_rejected():
    """Confirm a non-integer random seed fails configuration validation."""
    with pytest.raises(ValueError):
        ImuConfig(
            gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
            gyro_noise_stddev=0.0, accel_noise_stddev=0.0,
            random_seed=1.5, max_dt=0.5,
        )


def test_valid_configuration_is_accepted():
    """Confirm the documented default configuration passes validation."""
    assert DEFAULT_IMU_CONFIG.gyro_bias_z == pytest.approx(0.01)
    assert DEFAULT_IMU_CONFIG.accel_bias_x == pytest.approx(0.03)
    assert DEFAULT_IMU_CONFIG.accel_bias_y == pytest.approx(-0.02)
    assert DEFAULT_IMU_CONFIG.random_seed == 42
    assert DEFAULT_IMU_CONFIG.max_dt == pytest.approx(0.5)


# --- Deterministic and ideal behaviour -------------------------------------


def test_first_sample_produces_no_false_acceleration_spike():
    """Confirm the first sample reports zero ideal acceleration."""
    rng = create_rng(ZERO_CONFIG)

    measurement, state = generate_imu_measurement(
        previous_state=None, yaw=0.0, forward_speed=5.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x == pytest.approx(0.0)
    assert measurement.linear_acceleration_y == pytest.approx(0.0)
    assert isinstance(state, ImuState)


def test_stationary_zero_bias_zero_noise_gives_zero_angular_velocity():
    """Confirm a stationary rover with no bias/noise reports zero yaw rate."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.angular_velocity_z == pytest.approx(0.0)


def test_stationary_zero_bias_zero_noise_gives_zero_acceleration():
    """Confirm a stationary rover with no bias/noise reports zero accel."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x == pytest.approx(0.0)
    assert measurement.linear_acceleration_y == pytest.approx(0.0)


def test_constant_speed_after_initialization_gives_near_zero_acceleration():
    """Confirm steady straight-line speed produces near-zero acceleration."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=1.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=1.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x == pytest.approx(0.0, abs=1e-9)
    assert measurement.linear_acceleration_y == pytest.approx(0.0, abs=1e-9)


def test_increasing_forward_speed_gives_positive_body_x_acceleration():
    """Confirm accelerating forward produces positive body x acceleration."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=1.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=2.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x > 0.0
    assert measurement.linear_acceleration_x == pytest.approx(10.0)


def test_decreasing_forward_speed_gives_negative_body_x_acceleration():
    """Confirm decelerating forward produces negative body x acceleration."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=2.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=1.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x < 0.0
    assert measurement.linear_acceleration_x == pytest.approx(-10.0)


def test_reverse_acceleration_has_correct_sign():
    """Confirm speeding up in reverse produces negative body x acceleration."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=-1.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x < 0.0
    assert measurement.linear_acceleration_x == pytest.approx(-10.0)


def test_positive_yaw_rate_gives_positive_ideal_angular_velocity():
    """Confirm a positive commanded yaw rate yields positive angular_velocity.z."""
    rng = create_rng(ZERO_CONFIG)
    measurement, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.5,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.angular_velocity_z == pytest.approx(0.5)


def test_negative_yaw_rate_gives_negative_ideal_angular_velocity():
    """Confirm a negative commanded yaw rate yields negative angular_velocity.z."""
    rng = create_rng(ZERO_CONFIG)
    measurement, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=-0.5,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.angular_velocity_z == pytest.approx(-0.5)


def test_constant_speed_turning_produces_lateral_body_acceleration():
    """Confirm constant-speed turning yields nonzero lateral acceleration."""
    rng = create_rng(ZERO_CONFIG)
    forward_speed = 1.0
    yaw_rate = 0.5
    dt = 0.01
    yaw = 0.0

    _, state = generate_imu_measurement(
        None, yaw=yaw, forward_speed=forward_speed, yaw_rate=yaw_rate,
        dt=dt, config=ZERO_CONFIG, rng=rng,
    )
    yaw += yaw_rate * dt
    measurement, _ = generate_imu_measurement(
        state, yaw=yaw, forward_speed=forward_speed, yaw_rate=yaw_rate,
        dt=dt, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_y != pytest.approx(0.0, abs=1e-6)


def test_lateral_acceleration_sign_matches_v_omega_convention():
    """Confirm lateral acceleration approximates v * omega with matching sign."""
    rng = create_rng(ZERO_CONFIG)
    forward_speed = 2.0
    yaw_rate = 0.3
    dt = 0.001
    yaw = 0.0

    _, state = generate_imu_measurement(
        None, yaw=yaw, forward_speed=forward_speed, yaw_rate=yaw_rate,
        dt=dt, config=ZERO_CONFIG, rng=rng,
    )
    yaw += yaw_rate * dt
    measurement, _ = generate_imu_measurement(
        state, yaw=yaw, forward_speed=forward_speed, yaw_rate=yaw_rate,
        dt=dt, config=ZERO_CONFIG, rng=rng,
    )

    expected_lateral = forward_speed * yaw_rate
    assert measurement.linear_acceleration_y == pytest.approx(
        expected_lateral, rel=1e-2,
    )


def test_world_to_body_rotation_identity_at_yaw_zero():
    """Confirm world-to-body rotation is the identity when yaw is zero."""
    body_x, body_y = rotate_world_to_body(3.0, -2.0, yaw=0.0)

    assert body_x == pytest.approx(3.0)
    assert body_y == pytest.approx(-2.0)


def test_world_to_body_rotation_at_half_pi_yaw():
    """Confirm world-to-body rotation swaps axes correctly at yaw = pi/2."""
    body_x, body_y = rotate_world_to_body(0.0, 1.0, yaw=math.pi / 2.0)

    assert body_x == pytest.approx(1.0)
    assert body_y == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('dt', [0.0, -0.1])
def test_non_positive_dt_is_handled_safely(dt):
    """Confirm a non-positive dt does not crash and reports zero ideal accel."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=1.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )
    measurement, _ = generate_imu_measurement(
        state, yaw=0.0, forward_speed=5.0, yaw_rate=0.0,
        dt=dt, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x == pytest.approx(0.0)
    assert measurement.linear_acceleration_y == pytest.approx(0.0)
    assert math.isfinite(measurement.angular_velocity_z)


def test_excessive_dt_resets_baseline_without_a_spike():
    """Confirm a dt beyond max_dt reports zero ideal accel, not a spike."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )
    measurement, next_state = generate_imu_measurement(
        state, yaw=0.0, forward_speed=5.0, yaw_rate=0.0,
        dt=10.0, config=ZERO_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x == pytest.approx(0.0)

    # The baseline should reset to the current sample, so the next valid
    # step differentiates cleanly instead of using the stale velocity.
    followup, _ = generate_imu_measurement(
        next_state, yaw=0.0, forward_speed=5.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )
    assert followup.linear_acceleration_x == pytest.approx(0.0, abs=1e-9)


# --- Bias behaviour ---------------------------------------------------------


def test_gyro_bias_shifts_angular_velocity_by_configured_amount():
    """Confirm gyro bias shifts angular_velocity.z by exactly the bias value."""
    rng = create_rng(BIASED_ZERO_NOISE_CONFIG)
    measurement, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=BIASED_ZERO_NOISE_CONFIG, rng=rng,
    )

    assert measurement.angular_velocity_z == pytest.approx(
        BIASED_ZERO_NOISE_CONFIG.gyro_bias_z,
    )


def test_accel_x_bias_shifts_body_x_acceleration():
    """Confirm accelerometer x bias shifts body x acceleration when stationary."""
    rng = create_rng(BIASED_ZERO_NOISE_CONFIG)
    measurement, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=BIASED_ZERO_NOISE_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_x == pytest.approx(
        BIASED_ZERO_NOISE_CONFIG.accel_bias_x,
    )


def test_accel_y_bias_shifts_body_y_acceleration():
    """Confirm accelerometer y bias shifts body y acceleration when stationary."""
    rng = create_rng(BIASED_ZERO_NOISE_CONFIG)
    measurement, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=BIASED_ZERO_NOISE_CONFIG, rng=rng,
    )

    assert measurement.linear_acceleration_y == pytest.approx(
        BIASED_ZERO_NOISE_CONFIG.accel_bias_y,
    )


def test_bias_remains_constant_across_repeated_stationary_samples():
    """Confirm zero-noise bias output does not vary across repeated samples."""
    rng = create_rng(BIASED_ZERO_NOISE_CONFIG)
    state = None
    readings = []

    for _ in range(5):
        measurement, state = generate_imu_measurement(
            state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
            dt=0.05, config=BIASED_ZERO_NOISE_CONFIG, rng=rng,
        )
        readings.append(measurement)

    for measurement in readings:
        assert measurement.angular_velocity_z == pytest.approx(
            BIASED_ZERO_NOISE_CONFIG.gyro_bias_z,
        )
        assert measurement.linear_acceleration_x == pytest.approx(
            BIASED_ZERO_NOISE_CONFIG.accel_bias_x,
        )
        assert measurement.linear_acceleration_y == pytest.approx(
            BIASED_ZERO_NOISE_CONFIG.accel_bias_y,
        )


def test_zero_noise_with_nonzero_bias_is_deterministic():
    """Confirm zero noise with bias gives identical output across two runs."""
    first_rng = create_rng(BIASED_ZERO_NOISE_CONFIG)
    second_rng = create_rng(BIASED_ZERO_NOISE_CONFIG)

    first, _ = generate_imu_measurement(
        None, yaw=0.1, forward_speed=0.5, yaw_rate=0.2,
        dt=0.05, config=BIASED_ZERO_NOISE_CONFIG, rng=first_rng,
    )
    second, _ = generate_imu_measurement(
        None, yaw=0.1, forward_speed=0.5, yaw_rate=0.2,
        dt=0.05, config=BIASED_ZERO_NOISE_CONFIG, rng=second_rng,
    )

    assert first == second


# --- Noise behaviour ---------------------------------------------------------


def test_zero_stddev_produces_no_random_variation():
    """Confirm zero standard deviation gives identical repeated readings."""
    rng = create_rng(ZERO_CONFIG)
    state = None
    readings = []

    for _ in range(10):
        measurement, state = generate_imu_measurement(
            state, yaw=0.0, forward_speed=1.0, yaw_rate=0.2,
            dt=0.05, config=ZERO_CONFIG, rng=rng,
        )
        readings.append(measurement)

    assert all(r == readings[0] for r in readings[1:])


def test_same_seed_gives_identical_gyro_sequences():
    """Confirm the same seed produces an identical gyroscope noise sequence."""
    noisy_config = ImuConfig(
        gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.01, accel_noise_stddev=0.0,
        random_seed=7, max_dt=0.5,
    )

    def collect_sequence():
        rng = create_rng(noisy_config)
        state = None
        values = []
        for _ in range(10):
            measurement, state = generate_imu_measurement(
                state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
                dt=0.05, config=noisy_config, rng=rng,
            )
            values.append(measurement.angular_velocity_z)
        return values

    assert collect_sequence() == collect_sequence()


def test_same_seed_gives_identical_accel_sequences():
    """Confirm the same seed produces an identical accelerometer noise sequence."""
    noisy_config = ImuConfig(
        gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.0, accel_noise_stddev=0.02,
        random_seed=11, max_dt=0.5,
    )

    def collect_sequence():
        rng = create_rng(noisy_config)
        state = None
        values = []
        for _ in range(10):
            measurement, state = generate_imu_measurement(
                state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
                dt=0.05, config=noisy_config, rng=rng,
            )
            values.append(measurement.linear_acceleration_x)
        return values

    assert collect_sequence() == collect_sequence()


def test_different_seeds_produce_different_sequences():
    """Confirm two different seeds produce a different noise sequence."""
    def collect_sequence(seed):
        config = ImuConfig(
            gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
            gyro_noise_stddev=0.02, accel_noise_stddev=0.0,
            random_seed=seed, max_dt=0.5,
        )
        rng = create_rng(config)
        state = None
        values = []
        for _ in range(10):
            measurement, state = generate_imu_measurement(
                state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
                dt=0.05, config=config, rng=rng,
            )
            values.append(measurement.angular_velocity_z)
        return values

    assert collect_sequence(1) != collect_sequence(2)


def test_noise_samples_vary_around_the_biased_ideal_value():
    """Confirm noisy stationary samples are not all identical."""
    config = ImuConfig(
        gyro_bias_z=0.01, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.01, accel_noise_stddev=0.0,
        random_seed=3, max_dt=0.5,
    )
    rng = create_rng(config)
    state = None
    values = []
    for _ in range(20):
        measurement, state = generate_imu_measurement(
            state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
            dt=0.05, config=config, rng=rng,
        )
        values.append(measurement.angular_velocity_z)

    assert len(set(values)) > 1


def test_large_sample_mean_is_near_biased_ideal_value():
    """Confirm a large deterministic sample's mean approaches bias + ideal."""
    config = ImuConfig(
        gyro_bias_z=0.01, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.01, accel_noise_stddev=0.0,
        random_seed=99, max_dt=0.5,
    )
    rng = create_rng(config)
    state = None
    values = []
    for _ in range(2000):
        measurement, state = generate_imu_measurement(
            state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
            dt=0.05, config=config, rng=rng,
        )
        values.append(measurement.angular_velocity_z)

    mean = sum(values) / len(values)
    assert mean == pytest.approx(config.gyro_bias_z, abs=0.005)


def test_noise_level_affects_output_spread():
    """Confirm a larger configured stddev produces a larger sample spread."""
    def sample_spread(stddev, seed):
        config = ImuConfig(
            gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
            gyro_noise_stddev=stddev, accel_noise_stddev=0.0,
            random_seed=seed, max_dt=0.5,
        )
        rng = create_rng(config)
        state = None
        values = []
        for _ in range(500):
            measurement, state = generate_imu_measurement(
                state, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
                dt=0.05, config=config, rng=rng,
            )
            values.append(measurement.angular_velocity_z)
        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        return math.sqrt(variance)

    small_spread = sample_spread(0.005, 21)
    large_spread = sample_spread(0.05, 22)

    assert large_spread > small_spread


# --- State behaviour ---------------------------------------------------------


def test_previous_sample_state_updates_correctly():
    """Confirm the returned state stores the current world-frame velocity."""
    rng = create_rng(ZERO_CONFIG)
    _, state = generate_imu_measurement(
        None, yaw=0.0, forward_speed=3.0, yaw_rate=0.0,
        dt=0.05, config=ZERO_CONFIG, rng=rng,
    )

    assert state.previous_velocity_world_x == pytest.approx(3.0)
    assert state.previous_velocity_world_y == pytest.approx(0.0, abs=1e-9)


def test_input_state_is_not_mutated():
    """Confirm the previous state object passed in is left unchanged."""
    rng = create_rng(ZERO_CONFIG)
    original_state = ImuState(
        previous_velocity_world_x=1.0, previous_velocity_world_y=0.0,
    )

    generate_imu_measurement(
        original_state, yaw=0.0, forward_speed=5.0, yaw_rate=0.0,
        dt=0.1, config=ZERO_CONFIG, rng=rng,
    )

    assert original_state.previous_velocity_world_x == pytest.approx(1.0)
    assert original_state.previous_velocity_world_y == pytest.approx(0.0)


def test_repeated_samples_evolve_deterministically_with_fixed_seed():
    """Confirm a fixed seed reproduces the exact same evolving state sequence."""
    config = ImuConfig(
        gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.01, accel_noise_stddev=0.01,
        random_seed=5, max_dt=0.5,
    )

    def run():
        rng = create_rng(config)
        state = None
        results = []
        for i in range(5):
            measurement, state = generate_imu_measurement(
                state, yaw=0.0, forward_speed=float(i), yaw_rate=0.1,
                dt=0.05, config=config, rng=rng,
            )
            results.append(measurement)
        return results

    assert run() == run()


def test_reset_via_create_rng_reproduces_identical_sequence():
    """Confirm re-creating the RNG from config acts as a reproducible reset."""
    config = ImuConfig(
        gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.02, accel_noise_stddev=0.0,
        random_seed=17, max_dt=0.5,
    )
    first_rng = create_rng(config)
    second_rng = create_rng(config)

    first, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=config, rng=first_rng,
    )
    second, _ = generate_imu_measurement(
        None, yaw=0.0, forward_speed=0.0, yaw_rate=0.0,
        dt=0.05, config=config, rng=second_rng,
    )

    assert first == second


def test_units_remain_consistent_for_world_velocity():
    """Confirm forward speed (m/s) and yaw (rad) yield metre-per-second velocity."""
    velocity_x, velocity_y = calculate_world_velocity(2.0, math.pi / 2.0)

    assert velocity_x == pytest.approx(0.0, abs=1e-9)
    assert velocity_y == pytest.approx(2.0)


def test_units_remain_consistent_for_world_acceleration():
    """Confirm a 1 m/s velocity change over 0.5 s yields 2 m/s^2 acceleration."""
    accel_x, accel_y = calculate_world_acceleration(1.0, 0.0, 0.0, 0.0, 0.5)

    assert accel_x == pytest.approx(2.0)
    assert accel_y == pytest.approx(0.0)


# --- Covariance helpers -------------------------------------------------------


def test_gyro_covariance_uses_variance_not_stddev():
    """Confirm the gyro covariance helper squares the standard deviation."""
    config = ImuConfig(
        gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.02, accel_noise_stddev=0.0,
        random_seed=1, max_dt=0.5,
    )

    assert calculate_gyro_variance(config) == pytest.approx(0.0004)


def test_accel_covariance_uses_variance_not_stddev():
    """Confirm the accel covariance helper squares the standard deviation."""
    config = ImuConfig(
        gyro_bias_z=0.0, accel_bias_x=0.0, accel_bias_y=0.0,
        gyro_noise_stddev=0.0, accel_noise_stddev=0.05,
        random_seed=1, max_dt=0.5,
    )

    assert calculate_accel_variance(config) == pytest.approx(0.0025)
