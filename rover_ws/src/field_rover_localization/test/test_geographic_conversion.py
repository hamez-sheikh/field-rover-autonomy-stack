"""Unit tests for the pure Python geographic-to-local conversion module."""

import math

from field_rover_localization.geographic_conversion import (
    EARTH_RADIUS_M,
    geographic_to_local,
    normalize_longitude_deg,
    normalized_longitude_difference,
    validate_geographic_reference,
)

import pytest


REFERENCE_LATITUDE_DEG = 43.2609
REFERENCE_LONGITUDE_DEG = -79.9192


def _forward_local_to_geographic(
    east_m, north_m, reference_latitude_deg, reference_longitude_deg,
):
    """Mirror the Day 7 GPS simulator's forward local-to-geographic formula."""
    # A small, test-only reimplementation (not an import from
    # field_rover_sim) used solely to check that geographic_to_local is the
    # correct inverse of that forward conversion.
    reference_latitude_rad = math.radians(reference_latitude_deg)
    delta_latitude_rad = north_m / EARTH_RADIUS_M
    delta_longitude_rad = east_m / (
        EARTH_RADIUS_M * math.cos(reference_latitude_rad)
    )
    latitude_deg = reference_latitude_deg + math.degrees(delta_latitude_rad)
    longitude_deg = normalize_longitude_deg(
        reference_longitude_deg + math.degrees(delta_longitude_rad)
    )
    return latitude_deg, longitude_deg


# --- Reference-point conversion ----------------------------------------


def test_reference_point_converts_to_local_origin():
    """Confirm the reference lat/lon itself converts to (0, 0)."""
    east_m, north_m = geographic_to_local(
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert east_m == pytest.approx(0.0, abs=1e-9)
    assert north_m == pytest.approx(0.0, abs=1e-9)


def test_higher_longitude_gives_positive_east():
    """Confirm a fix east of the reference gives positive east metres."""
    east_m, _ = geographic_to_local(
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG + 0.001,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert east_m > 0.0


def test_higher_latitude_gives_positive_north():
    """Confirm a fix north of the reference gives positive north metres."""
    _, north_m = geographic_to_local(
        REFERENCE_LATITUDE_DEG + 0.001, REFERENCE_LONGITUDE_DEG,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert north_m > 0.0


def test_lower_longitude_gives_negative_east():
    """Confirm a fix west of the reference gives negative east metres."""
    east_m, _ = geographic_to_local(
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG - 0.001,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert east_m < 0.0


def test_lower_latitude_gives_negative_north():
    """Confirm a fix south of the reference gives negative north metres."""
    _, north_m = geographic_to_local(
        REFERENCE_LATITUDE_DEG - 0.001, REFERENCE_LONGITUDE_DEG,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert north_m < 0.0


def test_one_metre_north_round_trips_through_the_forward_conversion():
    """Confirm a forward-placed 1 m north point converts back to 1 m north."""
    latitude_deg, longitude_deg = _forward_local_to_geographic(
        0.0, 1.0, REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    east_m, north_m = geographic_to_local(
        latitude_deg, longitude_deg,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert north_m == pytest.approx(1.0)
    assert east_m == pytest.approx(0.0, abs=1e-9)


def test_one_metre_east_round_trips_through_the_forward_conversion():
    """Confirm a forward-placed 1 m east point converts back to 1 m east."""
    latitude_deg, longitude_deg = _forward_local_to_geographic(
        1.0, 0.0, REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    east_m, north_m = geographic_to_local(
        latitude_deg, longitude_deg,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert east_m == pytest.approx(1.0)
    assert north_m == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('east_m,north_m', [
    (0.0, 0.0), (5.0, 0.0), (-5.0, 0.0), (0.0, 5.0), (0.0, -5.0),
    (10.0, 7.5), (-10.0, -7.5), (20.0, 15.0),
])
def test_forward_then_inverse_conversion_returns_the_original_point(
    east_m, north_m,
):
    """Confirm the Day 7 forward conversion inverts back to the original point."""
    latitude_deg, longitude_deg = _forward_local_to_geographic(
        east_m, north_m, REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    recovered_east_m, recovered_north_m = geographic_to_local(
        latitude_deg, longitude_deg,
        REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG,
    )
    assert recovered_east_m == pytest.approx(east_m, abs=1e-6)
    assert recovered_north_m == pytest.approx(north_m, abs=1e-6)


# --- Longitude normalization and wraparound -----------------------------


def test_normalize_longitude_wraps_above_180():
    """Confirm longitude normalization wraps a value above 180 correctly."""
    assert normalize_longitude_deg(190.0) == pytest.approx(-170.0)


def test_normalize_longitude_wraps_below_minus_180():
    """Confirm longitude normalization wraps a value below -180 correctly."""
    assert normalize_longitude_deg(-190.0) == pytest.approx(170.0)


def test_normalize_longitude_leaves_in_range_value_unchanged():
    """Confirm an already-valid longitude is left unchanged by normalization."""
    assert normalize_longitude_deg(-79.9192) == pytest.approx(-79.9192)


def test_normalized_longitude_difference_wraps_across_the_antimeridian():
    """Confirm the shortest signed difference is used across +/-180."""
    difference = normalized_longitude_difference(-179.0, 179.0)
    assert difference == pytest.approx(2.0)


def test_normalized_longitude_difference_matches_subtraction_when_no_wrap():
    """Confirm a normal difference with no wraparound matches subtraction."""
    difference = normalized_longitude_difference(10.0, 5.0)
    assert difference == pytest.approx(5.0)


def test_geographic_to_local_handles_longitude_wraparound_at_antimeridian():
    """Confirm a fix across the antimeridian gives a small, finite offset."""
    east_m, north_m = geographic_to_local(0.0, -179.999, 0.0, 179.999)

    assert math.isfinite(east_m)
    assert math.isfinite(north_m)
    assert abs(east_m) < 1000.0


# --- Reference validation ------------------------------------------------


def test_valid_reference_is_accepted():
    """Confirm the documented default GPS reference passes validation."""
    validate_geographic_reference(REFERENCE_LATITUDE_DEG, REFERENCE_LONGITUDE_DEG)


def test_latitude_below_minus_90_is_rejected():
    """Confirm a reference latitude below -90 fails validation."""
    with pytest.raises(ValueError):
        validate_geographic_reference(-90.1, 0.0)


def test_latitude_above_90_is_rejected():
    """Confirm a reference latitude above 90 fails validation."""
    with pytest.raises(ValueError):
        validate_geographic_reference(90.1, 0.0)


def test_latitude_too_close_to_north_pole_is_rejected():
    """Confirm a reference latitude within the pole-safety margin is rejected."""
    with pytest.raises(ValueError):
        validate_geographic_reference(89.999, 0.0)


def test_latitude_too_close_to_south_pole_is_rejected():
    """Confirm a southern reference latitude within the pole margin is rejected."""
    with pytest.raises(ValueError):
        validate_geographic_reference(-89.999, 0.0)


def test_longitude_below_minus_180_is_rejected():
    """Confirm a reference longitude below -180 fails validation."""
    with pytest.raises(ValueError):
        validate_geographic_reference(0.0, -180.1)


def test_longitude_above_180_is_rejected():
    """Confirm a reference longitude above 180 fails validation."""
    with pytest.raises(ValueError):
        validate_geographic_reference(0.0, 180.1)


def test_non_finite_latitude_is_rejected():
    """Confirm a non-finite reference latitude fails validation."""
    with pytest.raises(ValueError):
        validate_geographic_reference(math.nan, 0.0)


def test_non_finite_longitude_is_rejected():
    """Confirm a non-finite reference longitude fails validation."""
    with pytest.raises(ValueError):
        validate_geographic_reference(0.0, math.inf)
