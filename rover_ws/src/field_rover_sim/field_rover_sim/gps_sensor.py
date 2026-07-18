"""Pure Python simulated-GPS model: local-plane to geographic conversion."""

# This is a local tangent-plane approximation for a small (~20 m x 15 m)
# simulated world, not a general-purpose geodesy library:
#
# - World x is treated as east metres and world y as north metres, both
#   measured from a configurable geographic reference point.
# - Latitude and longitude are derived with a small-distance spherical-Earth
#   approximation (equirectangular-style), which is accurate for a world
#   this size but is not suitable for long-distance navigation.
# - Altitude is always the configured constant reference altitude; vertical
#   rover motion is not simulated.
# - GPS is an independent, imperfect measurement source. This module never
#   corrects, fuses, or is corrected by wheel odometry or the IMU.

from dataclasses import dataclass
import math
import random


EARTH_RADIUS_M = 6378137.0

# Longitude conversion divides by cos(reference_latitude); a reference this
# close to a pole would make that division numerically unstable, so
# configuration validation rejects it below this cosine threshold.
MINIMUM_COS_REFERENCE_LATITUDE = 1e-3


@dataclass(frozen=True)
class GpsConfig:
    """Store the geographic reference, error model, and update settings."""

    publish_rate_hz: float
    reference_latitude_deg: float
    reference_longitude_deg: float
    reference_altitude_m: float
    position_bias_east_m: float
    position_bias_north_m: float
    position_noise_stddev_m: float
    dropout_probability: float
    random_seed: int
    frame_id: str

    def __post_init__(self):
        """Reject non-finite, out-of-range, or otherwise unusable settings."""
        if (
            not math.isfinite(self.publish_rate_hz)
            or self.publish_rate_hz <= 0.0
        ):
            raise ValueError('publish_rate_hz must be positive and finite.')

        if (
            not math.isfinite(self.reference_latitude_deg)
            or not (-90.0 <= self.reference_latitude_deg <= 90.0)
        ):
            raise ValueError(
                'reference_latitude_deg must be finite and within '
                '[-90, 90].'
            )

        reference_latitude_rad = math.radians(self.reference_latitude_deg)
        if math.cos(reference_latitude_rad) < MINIMUM_COS_REFERENCE_LATITUDE:
            raise ValueError(
                'reference_latitude_deg is too close to a pole for a '
                'stable local-plane longitude conversion.'
            )

        if (
            not math.isfinite(self.reference_longitude_deg)
            or not (-180.0 <= self.reference_longitude_deg <= 180.0)
        ):
            raise ValueError(
                'reference_longitude_deg must be finite and within '
                '[-180, 180].'
            )

        if not math.isfinite(self.reference_altitude_m):
            raise ValueError('reference_altitude_m must be finite.')

        for name, value in (
            ('position_bias_east_m', self.position_bias_east_m),
            ('position_bias_north_m', self.position_bias_north_m),
        ):
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite.')

        if (
            not math.isfinite(self.position_noise_stddev_m)
            or self.position_noise_stddev_m < 0.0
        ):
            raise ValueError(
                'position_noise_stddev_m must be non-negative and finite.'
            )

        if (
            not math.isfinite(self.dropout_probability)
            or not (0.0 <= self.dropout_probability <= 1.0)
        ):
            raise ValueError(
                'dropout_probability must be finite and within [0.0, 1.0].'
            )

        if isinstance(self.random_seed, bool) or not isinstance(
            self.random_seed, int
        ):
            raise ValueError('random_seed must be an integer.')

        if not self.frame_id:
            raise ValueError('frame_id must be non-empty.')


DEFAULT_GPS_CONFIG = GpsConfig(
    publish_rate_hz=2.0,
    reference_latitude_deg=43.2609,
    reference_longitude_deg=-79.9192,
    reference_altitude_m=0.0,
    position_bias_east_m=0.40,
    position_bias_north_m=-0.25,
    position_noise_stddev_m=1.0,
    dropout_probability=0.10,
    random_seed=42,
    frame_id='gps_link',
)


@dataclass(frozen=True)
class GpsMeasurement:
    """Represent one simulated GPS fix: geographic position, in degrees/m."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float


def create_noise_rng(config: GpsConfig) -> random.Random:
    """Build a dedicated, seeded RNG for position noise (not the global one)."""
    return random.Random(config.random_seed)


def create_dropout_rng(config: GpsConfig) -> random.Random:
    """Build a dedicated, seeded RNG for dropout decisions."""
    # A distinct stream (seeded from random_seed + 1) keeps the dropout
    # sequence stable even if the number of noise samples drawn per update
    # ever changes.
    return random.Random(config.random_seed + 1)


def normalize_longitude_deg(longitude_deg: float) -> float:
    """Wrap a longitude value into the [-180, 180) range."""
    return ((longitude_deg + 180.0) % 360.0) - 180.0


def local_to_geographic(
    east_m: float,
    north_m: float,
    config: GpsConfig,
) -> tuple[float, float]:
    """Convert a local east/north offset (m) into latitude/longitude (deg)."""
    reference_latitude_rad = math.radians(config.reference_latitude_deg)

    delta_latitude_rad = north_m / EARTH_RADIUS_M
    delta_longitude_rad = east_m / (
        EARTH_RADIUS_M * math.cos(reference_latitude_rad)
    )

    latitude_deg = config.reference_latitude_deg + math.degrees(
        delta_latitude_rad
    )
    longitude_deg = normalize_longitude_deg(
        config.reference_longitude_deg + math.degrees(delta_longitude_rad)
    )
    return latitude_deg, longitude_deg


def calculate_horizontal_variance(config: GpsConfig) -> float:
    """Return the horizontal position-noise variance (stddev squared)."""
    return config.position_noise_stddev_m ** 2


def decide_dropout(config: GpsConfig, dropout_rng: random.Random) -> bool:
    """Make one Bernoulli dropout decision for a scheduled GPS update."""
    return dropout_rng.random() < config.dropout_probability


def generate_gps_measurement(
    east_true_m: float,
    north_true_m: float,
    config: GpsConfig,
    noise_rng: random.Random,
) -> GpsMeasurement:
    """Apply bias/noise to a true local position and convert it to a fix."""
    # Error is applied in local metre coordinates (where bias and noise are
    # physically interpretable and easy to configure) before converting to
    # latitude/longitude, which have different metre-per-degree scales.
    east_biased_m = east_true_m + config.position_bias_east_m
    north_biased_m = north_true_m + config.position_bias_north_m

    east_measured_m = east_biased_m + noise_rng.gauss(
        0.0, config.position_noise_stddev_m,
    )
    north_measured_m = north_biased_m + noise_rng.gauss(
        0.0, config.position_noise_stddev_m,
    )

    latitude_deg, longitude_deg = local_to_geographic(
        east_measured_m, north_measured_m, config,
    )

    return GpsMeasurement(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=config.reference_altitude_m,
    )
