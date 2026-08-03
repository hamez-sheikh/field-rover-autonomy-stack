"""Pure Python geographic-to-local conversion for GPS fixes."""

# This is the inverse of the local-plane approximation used by the Day 7
# GPS simulator (field_rover_sim/gps_sensor.py): small-distance, spherical
# Earth, accurate for a world a few tens of metres across, not general
# geodesy. It is reimplemented here (not imported from field_rover_sim) so
# that field_rover_localization stays a standalone consumer of the standard
# sensor_msgs/msg/NavSatFix message rather than depending on simulator
# internals.

import math


EARTH_RADIUS_M = 6378137.0

# Same pole-safety threshold as the Day 7 GPS simulator: a reference this
# close to a pole makes the cos(latitude) division numerically unstable.
MINIMUM_COS_REFERENCE_LATITUDE = 1e-3


def validate_geographic_reference(
    reference_latitude_deg: float,
    reference_longitude_deg: float,
) -> None:
    """Reject a non-finite, out-of-range, or pole-unsafe geographic reference."""
    if (
        not math.isfinite(reference_latitude_deg)
        or not (-90.0 <= reference_latitude_deg <= 90.0)
    ):
        raise ValueError(
            'reference_latitude_deg must be finite and within [-90, 90].'
        )

    reference_latitude_rad = math.radians(reference_latitude_deg)
    if math.cos(reference_latitude_rad) < MINIMUM_COS_REFERENCE_LATITUDE:
        raise ValueError(
            'reference_latitude_deg is too close to a pole for a stable '
            'local-plane conversion.'
        )

    if (
        not math.isfinite(reference_longitude_deg)
        or not (-180.0 <= reference_longitude_deg <= 180.0)
    ):
        raise ValueError(
            'reference_longitude_deg must be finite and within [-180, 180].'
        )


def normalize_longitude_deg(longitude_deg: float) -> float:
    """Wrap a longitude value into the [-180, 180) range."""
    return ((longitude_deg + 180.0) % 360.0) - 180.0


def normalized_longitude_difference(
    longitude_deg: float,
    reference_longitude_deg: float,
) -> float:
    """Return longitude_deg minus reference_longitude_deg, wrapped correctly."""
    return normalize_longitude_deg(longitude_deg - reference_longitude_deg)


def geographic_to_local(
    latitude_deg: float,
    longitude_deg: float,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
) -> tuple[float, float]:
    """Convert a latitude/longitude fix into local east/north metres."""
    reference_latitude_rad = math.radians(reference_latitude_deg)

    delta_latitude_rad = math.radians(latitude_deg - reference_latitude_deg)
    delta_longitude_rad = math.radians(
        normalized_longitude_difference(longitude_deg, reference_longitude_deg)
    )

    north_m = delta_latitude_rad * EARTH_RADIUS_M
    east_m = (
        delta_longitude_rad * EARTH_RADIUS_M * math.cos(reference_latitude_rad)
    )
    return east_m, north_m
