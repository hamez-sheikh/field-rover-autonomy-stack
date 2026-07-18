"""Unit tests for the pure Python simulated-GPS model."""

import math

from field_rover_sim.gps_sensor import (
    calculate_horizontal_variance,
    create_dropout_rng,
    create_noise_rng,
    decide_dropout,
    DEFAULT_GPS_CONFIG,
    EARTH_RADIUS_M,
    generate_gps_measurement,
    GpsConfig,
    GpsMeasurement,
    local_to_geographic,
    normalize_longitude_deg,
)

import pytest


ZERO_ERROR_CONFIG = GpsConfig(
    publish_rate_hz=2.0,
    reference_latitude_deg=43.2609,
    reference_longitude_deg=-79.9192,
    reference_altitude_m=0.0,
    position_bias_east_m=0.0,
    position_bias_north_m=0.0,
    position_noise_stddev_m=0.0,
    dropout_probability=0.0,
    random_seed=1,
    frame_id='gps_link',
)

BIASED_ZERO_NOISE_CONFIG = GpsConfig(
    publish_rate_hz=2.0,
    reference_latitude_deg=43.2609,
    reference_longitude_deg=-79.9192,
    reference_altitude_m=0.0,
    position_bias_east_m=0.40,
    position_bias_north_m=-0.25,
    position_noise_stddev_m=0.0,
    dropout_probability=0.0,
    random_seed=1,
    frame_id='gps_link',
)


def _valid_kwargs(**overrides):
    kwargs = {
        'publish_rate_hz': 2.0,
        'reference_latitude_deg': 43.2609,
        'reference_longitude_deg': -79.9192,
        'reference_altitude_m': 0.0,
        'position_bias_east_m': 0.0,
        'position_bias_north_m': 0.0,
        'position_noise_stddev_m': 0.0,
        'dropout_probability': 0.0,
        'random_seed': 1,
        'frame_id': 'gps_link',
    }
    kwargs.update(overrides)
    return kwargs


# --- Configuration validation ------------------------------------------------


def test_valid_configuration_is_accepted():
    """Confirm the documented default configuration passes validation."""
    assert DEFAULT_GPS_CONFIG.publish_rate_hz == pytest.approx(2.0)
    assert DEFAULT_GPS_CONFIG.reference_latitude_deg == pytest.approx(43.2609)
    assert DEFAULT_GPS_CONFIG.reference_longitude_deg == pytest.approx(-79.9192)
    assert DEFAULT_GPS_CONFIG.random_seed == 42
    assert DEFAULT_GPS_CONFIG.frame_id == 'gps_link'


def test_zero_publish_rate_is_rejected():
    """Confirm a zero publishing rate fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(publish_rate_hz=0.0))


def test_negative_publish_rate_is_rejected():
    """Confirm a negative publishing rate fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(publish_rate_hz=-1.0))


def test_latitude_below_minus_90_is_rejected():
    """Confirm a reference latitude below -90 fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(reference_latitude_deg=-90.1))


def test_latitude_above_90_is_rejected():
    """Confirm a reference latitude above 90 fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(reference_latitude_deg=90.1))


def test_latitude_too_close_to_pole_is_rejected():
    """Confirm a reference latitude within the pole-safety margin is rejected."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(reference_latitude_deg=89.999))


def test_longitude_below_minus_180_is_rejected():
    """Confirm a reference longitude below -180 fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(reference_longitude_deg=-180.1))


def test_longitude_above_180_is_rejected():
    """Confirm a reference longitude above 180 fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(reference_longitude_deg=180.1))


def test_non_finite_reference_altitude_is_rejected():
    """Confirm a non-finite reference altitude fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(reference_altitude_m=math.nan))


def test_non_finite_east_bias_is_rejected():
    """Confirm a non-finite east bias fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(position_bias_east_m=math.inf))


def test_non_finite_north_bias_is_rejected():
    """Confirm a non-finite north bias fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(position_bias_north_m=math.nan))


def test_negative_noise_stddev_is_rejected():
    """Confirm a negative noise standard deviation fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(position_noise_stddev_m=-0.1))


def test_dropout_probability_below_zero_is_rejected():
    """Confirm a dropout probability below zero fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(dropout_probability=-0.01))


def test_dropout_probability_above_one_is_rejected():
    """Confirm a dropout probability above one fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(dropout_probability=1.01))


def test_empty_frame_id_is_rejected():
    """Confirm an empty frame ID fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(frame_id=''))


def test_non_integer_seed_is_rejected():
    """Confirm a non-integer random seed fails validation."""
    with pytest.raises(ValueError):
        GpsConfig(**_valid_kwargs(random_seed=1.5))


# --- Geographic conversion ----------------------------------------------------


def test_local_origin_converts_exactly_to_reference():
    """Confirm (0, 0) local position converts exactly to the reference point."""
    latitude_deg, longitude_deg = local_to_geographic(
        0.0, 0.0, ZERO_ERROR_CONFIG,
    )

    assert latitude_deg == pytest.approx(
        ZERO_ERROR_CONFIG.reference_latitude_deg,
    )
    assert longitude_deg == pytest.approx(
        ZERO_ERROR_CONFIG.reference_longitude_deg,
    )


def test_positive_east_displacement_increases_longitude():
    """Confirm moving east increases longitude at this positive-longitude test."""
    _, longitude_at_origin = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    _, longitude_east = local_to_geographic(5.0, 0.0, ZERO_ERROR_CONFIG)

    assert longitude_east > longitude_at_origin


def test_negative_east_displacement_decreases_longitude():
    """Confirm moving west decreases longitude."""
    _, longitude_at_origin = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    _, longitude_west = local_to_geographic(-5.0, 0.0, ZERO_ERROR_CONFIG)

    assert longitude_west < longitude_at_origin


def test_positive_north_displacement_increases_latitude():
    """Confirm moving north increases latitude."""
    latitude_at_origin, _ = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    latitude_north, _ = local_to_geographic(0.0, 5.0, ZERO_ERROR_CONFIG)

    assert latitude_north > latitude_at_origin


def test_negative_north_displacement_decreases_latitude():
    """Confirm moving south decreases latitude."""
    latitude_at_origin, _ = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    latitude_south, _ = local_to_geographic(0.0, -5.0, ZERO_ERROR_CONFIG)

    assert latitude_south < latitude_at_origin


def test_east_motion_does_not_materially_change_latitude():
    """Confirm pure east displacement leaves latitude effectively unchanged."""
    latitude_at_origin, _ = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    latitude_east, _ = local_to_geographic(10.0, 0.0, ZERO_ERROR_CONFIG)

    assert latitude_east == pytest.approx(latitude_at_origin, abs=1e-9)


def test_north_motion_does_not_materially_change_longitude():
    """Confirm pure north displacement leaves longitude effectively unchanged."""
    _, longitude_at_origin = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    _, longitude_north = local_to_geographic(0.0, 10.0, ZERO_ERROR_CONFIG)

    assert longitude_north == pytest.approx(longitude_at_origin, abs=1e-9)


def test_one_metre_north_matches_expected_latitude_change():
    """Confirm a known 1 m north displacement matches the expected delta."""
    latitude_at_origin, _ = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    latitude_north, _ = local_to_geographic(0.0, 1.0, ZERO_ERROR_CONFIG)

    expected_delta_deg = math.degrees(1.0 / EARTH_RADIUS_M)
    assert (latitude_north - latitude_at_origin) == pytest.approx(
        expected_delta_deg,
    )


def test_one_metre_east_matches_expected_longitude_change():
    """Confirm a known 1 m east displacement matches the expected delta."""
    _, longitude_at_origin = local_to_geographic(0.0, 0.0, ZERO_ERROR_CONFIG)
    _, longitude_east = local_to_geographic(1.0, 0.0, ZERO_ERROR_CONFIG)

    reference_latitude_rad = math.radians(
        ZERO_ERROR_CONFIG.reference_latitude_deg,
    )
    expected_delta_deg = math.degrees(
        1.0 / (EARTH_RADIUS_M * math.cos(reference_latitude_rad))
    )
    assert (longitude_east - longitude_at_origin) == pytest.approx(
        expected_delta_deg,
    )


def test_conversion_remains_finite_across_simulated_world():
    """Confirm conversion stays finite across the full 20 m x 15 m world."""
    for east_m in (-10.0, 0.0, 10.0, 20.0):
        for north_m in (-7.5, 0.0, 7.5, 15.0):
            latitude_deg, longitude_deg = local_to_geographic(
                east_m, north_m, ZERO_ERROR_CONFIG,
            )
            assert math.isfinite(latitude_deg)
            assert math.isfinite(longitude_deg)


def test_longitude_normalization_wraps_above_180():
    """Confirm longitude normalization wraps a value above 180 correctly."""
    assert normalize_longitude_deg(190.0) == pytest.approx(-170.0)


def test_longitude_normalization_wraps_below_minus_180():
    """Confirm longitude normalization wraps a value below -180 correctly."""
    assert normalize_longitude_deg(-190.0) == pytest.approx(170.0)


def test_longitude_normalization_leaves_in_range_value_unchanged():
    """Confirm an already-valid longitude is left unchanged by normalization."""
    assert normalize_longitude_deg(-79.9192) == pytest.approx(-79.9192)


def test_reference_altitude_is_preserved_in_measurement():
    """Confirm the generated measurement altitude matches the reference."""
    rng = create_noise_rng(ZERO_ERROR_CONFIG)
    measurement = generate_gps_measurement(0.0, 0.0, ZERO_ERROR_CONFIG, rng)

    assert measurement.altitude_m == pytest.approx(
        ZERO_ERROR_CONFIG.reference_altitude_m,
    )


# --- Bias behaviour ------------------------------------------------------------


def test_east_bias_shifts_measured_position_east():
    """Confirm east bias increases longitude relative to a zero-bias fix."""
    zero_bias_rng = create_noise_rng(ZERO_ERROR_CONFIG)
    biased_rng = create_noise_rng(BIASED_ZERO_NOISE_CONFIG)

    zero_bias = generate_gps_measurement(0.0, 0.0, ZERO_ERROR_CONFIG, zero_bias_rng)
    biased = generate_gps_measurement(
        0.0, 0.0, BIASED_ZERO_NOISE_CONFIG, biased_rng,
    )

    assert biased.longitude_deg > zero_bias.longitude_deg


def test_north_bias_shifts_measured_position_south_when_negative():
    """Confirm a negative north bias decreases latitude relative to zero bias."""
    zero_bias_rng = create_noise_rng(ZERO_ERROR_CONFIG)
    biased_rng = create_noise_rng(BIASED_ZERO_NOISE_CONFIG)

    zero_bias = generate_gps_measurement(0.0, 0.0, ZERO_ERROR_CONFIG, zero_bias_rng)
    biased = generate_gps_measurement(
        0.0, 0.0, BIASED_ZERO_NOISE_CONFIG, biased_rng,
    )

    assert biased.latitude_deg < zero_bias.latitude_deg


def test_negative_east_bias_shifts_longitude_west():
    """Confirm a negative east bias decreases longitude."""
    negative_bias_config = GpsConfig(
        **{
            **_valid_kwargs(),
            'position_bias_east_m': -1.0,
        }
    )
    zero_rng = create_noise_rng(ZERO_ERROR_CONFIG)
    negative_rng = create_noise_rng(negative_bias_config)

    zero_bias = generate_gps_measurement(0.0, 0.0, ZERO_ERROR_CONFIG, zero_rng)
    negative_bias = generate_gps_measurement(
        0.0, 0.0, negative_bias_config, negative_rng,
    )

    assert negative_bias.longitude_deg < zero_bias.longitude_deg


def test_negative_north_bias_shifts_latitude_south():
    """Confirm a negative north bias decreases latitude."""
    negative_bias_config = GpsConfig(
        **{
            **_valid_kwargs(),
            'position_bias_north_m': -1.0,
        }
    )
    zero_rng = create_noise_rng(ZERO_ERROR_CONFIG)
    negative_rng = create_noise_rng(negative_bias_config)

    zero_bias = generate_gps_measurement(0.0, 0.0, ZERO_ERROR_CONFIG, zero_rng)
    negative_bias = generate_gps_measurement(
        0.0, 0.0, negative_bias_config, negative_rng,
    )

    assert negative_bias.latitude_deg < zero_bias.latitude_deg


def test_zero_noise_with_nonzero_bias_is_deterministic():
    """Confirm zero noise with bias gives identical output across two runs."""
    first_rng = create_noise_rng(BIASED_ZERO_NOISE_CONFIG)
    second_rng = create_noise_rng(BIASED_ZERO_NOISE_CONFIG)

    first = generate_gps_measurement(
        1.0, 2.0, BIASED_ZERO_NOISE_CONFIG, first_rng,
    )
    second = generate_gps_measurement(
        1.0, 2.0, BIASED_ZERO_NOISE_CONFIG, second_rng,
    )

    assert first == second


def test_bias_remains_constant_across_repeated_samples():
    """Confirm zero-noise bias output does not vary across repeated samples."""
    rng = create_noise_rng(BIASED_ZERO_NOISE_CONFIG)
    readings = [
        generate_gps_measurement(3.0, 4.0, BIASED_ZERO_NOISE_CONFIG, rng)
        for _ in range(5)
    ]

    for reading in readings:
        assert reading == readings[0]


# --- Noise behaviour -------------------------------------------------------


def test_zero_stddev_produces_no_random_variation():
    """Confirm zero standard deviation gives identical repeated readings."""
    rng = create_noise_rng(ZERO_ERROR_CONFIG)
    readings = [
        generate_gps_measurement(2.0, 3.0, ZERO_ERROR_CONFIG, rng)
        for _ in range(10)
    ]

    assert all(r == readings[0] for r in readings[1:])


def test_same_seed_gives_identical_east_noise_sequence():
    """Confirm the same seed produces an identical east (longitude) sequence."""
    noisy_config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 1.0, 'random_seed': 7},
    )

    def collect_sequence():
        rng = create_noise_rng(noisy_config)
        return [
            generate_gps_measurement(0.0, 0.0, noisy_config, rng).longitude_deg
            for _ in range(10)
        ]

    assert collect_sequence() == collect_sequence()


def test_same_seed_gives_identical_north_noise_sequence():
    """Confirm the same seed produces an identical north (latitude) sequence."""
    noisy_config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 1.0, 'random_seed': 11},
    )

    def collect_sequence():
        rng = create_noise_rng(noisy_config)
        return [
            generate_gps_measurement(0.0, 0.0, noisy_config, rng).latitude_deg
            for _ in range(10)
        ]

    assert collect_sequence() == collect_sequence()


def test_different_seeds_produce_different_measurement_sequences():
    """Confirm two different seeds produce a different noise sequence."""
    def collect_sequence(seed):
        config = GpsConfig(
            **{**_valid_kwargs(), 'position_noise_stddev_m': 1.0, 'random_seed': seed},
        )
        rng = create_noise_rng(config)
        return [
            generate_gps_measurement(0.0, 0.0, config, rng).latitude_deg
            for _ in range(10)
        ]

    assert collect_sequence(1) != collect_sequence(2)


def test_noise_samples_vary_around_biased_position():
    """Confirm noisy samples are not all identical."""
    config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 1.0, 'random_seed': 3},
    )
    rng = create_noise_rng(config)
    values = [
        generate_gps_measurement(0.0, 0.0, config, rng).latitude_deg
        for _ in range(20)
    ]

    assert len(set(values)) > 1


def test_large_sample_mean_is_near_biased_position():
    """Confirm a large deterministic sample's mean approaches the biased fix."""
    config = GpsConfig(
        **{
            **_valid_kwargs(),
            'position_bias_east_m': 0.4,
            'position_noise_stddev_m': 1.0,
            'random_seed': 99,
        },
    )
    rng = create_noise_rng(config)

    east_offsets = []
    for _ in range(3000):
        measurement = generate_gps_measurement(0.0, 0.0, config, rng)
        reference_latitude_rad = math.radians(config.reference_latitude_deg)
        delta_lon_rad = math.radians(
            measurement.longitude_deg - config.reference_longitude_deg
        )
        east_offsets.append(
            delta_lon_rad * EARTH_RADIUS_M * math.cos(reference_latitude_rad)
        )

    mean_east = sum(east_offsets) / len(east_offsets)
    assert mean_east == pytest.approx(config.position_bias_east_m, abs=0.1)


def test_larger_noise_stddev_produces_greater_sample_spread():
    """Confirm a larger configured stddev produces a larger sample spread."""
    def sample_spread(stddev, seed):
        config = GpsConfig(
            **{
                **_valid_kwargs(),
                'position_noise_stddev_m': stddev,
                'random_seed': seed,
            },
        )
        rng = create_noise_rng(config)
        values = [
            generate_gps_measurement(0.0, 0.0, config, rng).latitude_deg
            for _ in range(500)
        ]
        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        return math.sqrt(variance)

    small_spread = sample_spread(0.5, 21)
    large_spread = sample_spread(5.0, 22)

    assert large_spread > small_spread


# --- Dropout behaviour -------------------------------------------------------


def test_dropout_probability_zero_never_drops():
    """Confirm a dropout probability of zero never reports a dropout."""
    config = GpsConfig(**{**_valid_kwargs(), 'dropout_probability': 0.0})
    rng = create_dropout_rng(config)

    assert not any(decide_dropout(config, rng) for _ in range(500))


def test_dropout_probability_one_always_drops():
    """Confirm a dropout probability of one always reports a dropout."""
    config = GpsConfig(**{**_valid_kwargs(), 'dropout_probability': 1.0})
    rng = create_dropout_rng(config)

    assert all(decide_dropout(config, rng) for _ in range(500))


def test_same_seed_gives_identical_dropout_sequence():
    """Confirm the same seed reproduces the same dropout sequence."""
    config = GpsConfig(
        **{**_valid_kwargs(), 'dropout_probability': 0.3, 'random_seed': 5},
    )

    def collect_sequence():
        rng = create_dropout_rng(config)
        return [decide_dropout(config, rng) for _ in range(50)]

    assert collect_sequence() == collect_sequence()


def test_different_seeds_can_produce_different_dropout_sequences():
    """Confirm different seeds can produce a different dropout sequence."""
    def collect_sequence(seed):
        config = GpsConfig(
            **{**_valid_kwargs(), 'dropout_probability': 0.3, 'random_seed': seed},
        )
        rng = create_dropout_rng(config)
        return [decide_dropout(config, rng) for _ in range(50)]

    assert collect_sequence(1) != collect_sequence(2)


def test_default_dropout_probability_produces_kept_and_dropped_samples():
    """Confirm the default dropout probability yields both outcomes over time."""
    rng = create_dropout_rng(DEFAULT_GPS_CONFIG)
    outcomes = {decide_dropout(DEFAULT_GPS_CONFIG, rng) for _ in range(200)}

    assert outcomes == {True, False}


def test_dropout_decisions_are_deterministic_regardless_of_noise_config():
    """Confirm dropout RNG stream is unaffected by the noise configuration."""
    # Separate RNG streams for noise and dropout mean the dropout sequence
    # should not change even if a different noise standard deviation is used
    # for the same seed.
    low_noise_config = GpsConfig(
        **{**_valid_kwargs(), 'dropout_probability': 0.3, 'position_noise_stddev_m': 0.1},
    )
    high_noise_config = GpsConfig(
        **{**_valid_kwargs(), 'dropout_probability': 0.3, 'position_noise_stddev_m': 5.0},
    )

    low_rng = create_dropout_rng(low_noise_config)
    high_rng = create_dropout_rng(high_noise_config)

    low_sequence = [decide_dropout(low_noise_config, low_rng) for _ in range(30)]
    high_sequence = [decide_dropout(high_noise_config, high_rng) for _ in range(30)]

    assert low_sequence == high_sequence


# --- Covariance behaviour -----------------------------------------------------


def test_horizontal_variance_uses_variance_not_stddev():
    """Confirm the covariance helper squares the standard deviation."""
    config = GpsConfig(**{**_valid_kwargs(), 'position_noise_stddev_m': 2.0})

    assert calculate_horizontal_variance(config) == pytest.approx(4.0)


def test_horizontal_variance_is_zero_when_noise_stddev_is_zero():
    """Confirm zero noise standard deviation gives zero covariance."""
    assert calculate_horizontal_variance(ZERO_ERROR_CONFIG) == pytest.approx(0.0)


def test_bias_does_not_inflate_noise_covariance():
    """Confirm a nonzero bias with zero noise still yields zero covariance."""
    assert calculate_horizontal_variance(
        BIASED_ZERO_NOISE_CONFIG,
    ) == pytest.approx(0.0)


# --- State and input behaviour ------------------------------------------------


def test_repeated_generation_with_fixed_seed_is_deterministic():
    """Confirm repeated runs from a fresh seeded RNG produce identical output."""
    config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 0.5, 'random_seed': 17},
    )

    def run():
        rng = create_noise_rng(config)
        return [
            generate_gps_measurement(float(i), float(i), config, rng)
            for i in range(5)
        ]

    assert run() == run()


def test_all_generated_coordinates_remain_finite():
    """Confirm generated latitude/longitude/altitude values are always finite."""
    config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 2.0, 'random_seed': 4},
    )
    rng = create_noise_rng(config)

    for _ in range(200):
        measurement = generate_gps_measurement(0.0, 0.0, config, rng)
        assert math.isfinite(measurement.latitude_deg)
        assert math.isfinite(measurement.longitude_deg)
        assert math.isfinite(measurement.altitude_m)


def test_generated_latitude_stays_within_valid_world_bounds():
    """Confirm latitude stays within a tight bound for the 20 m x 15 m world."""
    config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 3.0, 'random_seed': 6},
    )
    rng = create_noise_rng(config)

    for _ in range(200):
        measurement = generate_gps_measurement(10.0, 7.5, config, rng)
        assert abs(
            measurement.latitude_deg - config.reference_latitude_deg
        ) < 0.01


def test_generated_longitude_stays_within_valid_world_bounds():
    """Confirm longitude stays within a tight bound for the 20 m x 15 m world."""
    config = GpsConfig(
        **{**_valid_kwargs(), 'position_noise_stddev_m': 3.0, 'random_seed': 8},
    )
    rng = create_noise_rng(config)

    for _ in range(200):
        measurement = generate_gps_measurement(10.0, 7.5, config, rng)
        assert abs(
            measurement.longitude_deg - config.reference_longitude_deg
        ) < 0.01


def test_measurement_equality_supports_dataclass_comparison():
    """Confirm two identically-valued GpsMeasurement instances compare equal."""
    first = GpsMeasurement(latitude_deg=1.0, longitude_deg=2.0, altitude_m=0.0)
    second = GpsMeasurement(latitude_deg=1.0, longitude_deg=2.0, altitude_m=0.0)

    assert first == second
