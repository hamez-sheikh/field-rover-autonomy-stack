"""Pure Python bounded-evidence 2D occupancy-grid mapping model."""

from dataclasses import dataclass
import math


# ROS 2 nav_msgs/OccupancyGrid cell values.
UNKNOWN_OCCUPANCY = -1
FREE_OCCUPANCY = 0
OCCUPIED_OCCUPANCY = 100

# Tolerance for comparing a measured range against min_range/max_range.
RANGE_VALUE_TOLERANCE = 1e-6

# A world coordinate within this distance of the maximum x or y boundary is
# nudged inward by this amount before conversion, so a geometric hit that
# lands exactly on the world edge resolves to the last valid cell instead
# of being rejected as out-of-map.
BOUNDARY_EPSILON = 1e-9


@dataclass(frozen=True)
class OccupancyGridConfig:
    """Store grid geometry, evidence update rules, and node-facing timing."""

    world_width_m: float
    world_height_m: float
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    free_evidence_delta: int
    occupied_evidence_delta: int
    minimum_evidence: int
    maximum_evidence: int
    free_threshold: int
    occupied_threshold: int
    frame_id: str
    map_update_rate_hz: float
    localization_timeout_s: float
    range_timeout_s: float

    def __post_init__(self):
        """Reject a configuration that cannot produce a valid, stable map."""
        if not math.isfinite(self.world_width_m) or self.world_width_m <= 0.0:
            raise ValueError('world_width_m must be positive and finite.')
        if not math.isfinite(self.world_height_m) or self.world_height_m <= 0.0:
            raise ValueError('world_height_m must be positive and finite.')
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError('resolution_m must be positive and finite.')

        width_ratio = self.world_width_m / self.resolution_m
        if (
            not math.isclose(width_ratio, round(width_ratio), abs_tol=1e-6)
            or self.width_cells <= 0
        ):
            raise ValueError(
                'world_width_m must be a positive whole multiple of resolution_m.'
            )

        height_ratio = self.world_height_m / self.resolution_m
        if (
            not math.isclose(height_ratio, round(height_ratio), abs_tol=1e-6)
            or self.height_cells <= 0
        ):
            raise ValueError(
                'world_height_m must be a positive whole multiple of resolution_m.'
            )

        if self.minimum_evidence >= self.maximum_evidence:
            raise ValueError('minimum_evidence must be less than maximum_evidence.')
        if self.free_evidence_delta >= 0:
            raise ValueError('free_evidence_delta must be negative.')
        if self.occupied_evidence_delta <= 0:
            raise ValueError('occupied_evidence_delta must be positive.')
        if self.free_threshold >= self.occupied_threshold:
            raise ValueError('free_threshold must be less than occupied_threshold.')
        if not (self.minimum_evidence <= self.free_threshold <= self.maximum_evidence):
            raise ValueError('free_threshold must fall within the evidence bounds.')
        if not (
            self.minimum_evidence <= self.occupied_threshold <= self.maximum_evidence
        ):
            raise ValueError('occupied_threshold must fall within the evidence bounds.')
        if not self.frame_id:
            raise ValueError('frame_id must be non-empty.')
        if (
            not math.isfinite(self.map_update_rate_hz)
            or self.map_update_rate_hz <= 0.0
        ):
            raise ValueError('map_update_rate_hz must be positive and finite.')
        if (
            not math.isfinite(self.localization_timeout_s)
            or self.localization_timeout_s <= 0.0
        ):
            raise ValueError('localization_timeout_s must be positive and finite.')
        if not math.isfinite(self.range_timeout_s) or self.range_timeout_s <= 0.0:
            raise ValueError('range_timeout_s must be positive and finite.')

    @property
    def width_cells(self) -> int:
        """Return the grid width in cells, rounded from world_width_m."""
        return round(self.world_width_m / self.resolution_m)

    @property
    def height_cells(self) -> int:
        """Return the grid height in cells, rounded from world_height_m."""
        return round(self.world_height_m / self.resolution_m)

    @property
    def total_cells(self) -> int:
        """Return the total number of cells in the grid."""
        return self.width_cells * self.height_cells


DEFAULT_OCCUPANCY_GRID_CONFIG = OccupancyGridConfig(
    world_width_m=20.0,
    world_height_m=15.0,
    resolution_m=0.25,
    origin_x_m=0.0,
    origin_y_m=0.0,
    free_evidence_delta=-1,
    occupied_evidence_delta=3,
    minimum_evidence=-5,
    maximum_evidence=5,
    free_threshold=-1,
    occupied_threshold=1,
    frame_id='map',
    map_update_rate_hz=5.0,
    localization_timeout_s=0.5,
    range_timeout_s=0.5,
)


class OccupancyGridState:
    """Hold the persistent bounded-evidence grid for one occupancy map."""

    # Internal evidence is an integer per cell: 0 means unknown, negative
    # means evidence of free space, and positive means evidence of an
    # obstacle. This is a bounded evidence accumulator, not a log-odds
    # Bayesian filter — repeated observations simply push a cell's evidence
    # up or down within [minimum_evidence, maximum_evidence].

    def __init__(self, config: OccupancyGridConfig):
        """Allocate an all-unknown (zero-evidence) grid for the given config."""
        self.config = config
        self.evidence = [0] * config.total_cells


def reset_grid_state(state: OccupancyGridState) -> None:
    """Clear all accumulated evidence back to unknown, in place."""
    state.evidence = [0] * state.config.total_cells


def world_to_grid_unbounded(
    config: OccupancyGridConfig, world_x: float, world_y: float,
) -> tuple[int, int]:
    """Convert a world position into grid indices without bounds checking."""
    grid_x = math.floor((world_x - config.origin_x_m) / config.resolution_m)
    grid_y = math.floor((world_y - config.origin_y_m) / config.resolution_m)
    return int(grid_x), int(grid_y)


def world_to_grid(
    config: OccupancyGridConfig, world_x: float, world_y: float,
) -> tuple[int, int] | None:
    """Convert a world (x, y) position into in-map (grid_x, grid_y) indices."""
    # Returns None when the position falls outside the mapped world. A
    # position that lands exactly on the maximum x or y boundary is nudged
    # a tiny epsilon inward first, so it resolves to the last valid cell
    # instead of being rejected — a geometric ray endpoint computed from
    # trigonometry routinely lands within nanometres of an exact boundary.
    max_x = config.origin_x_m + config.world_width_m
    max_y = config.origin_y_m + config.world_height_m

    adjusted_x = world_x
    if math.isclose(world_x, max_x, rel_tol=0.0, abs_tol=BOUNDARY_EPSILON):
        adjusted_x = max_x - BOUNDARY_EPSILON

    adjusted_y = world_y
    if math.isclose(world_y, max_y, rel_tol=0.0, abs_tol=BOUNDARY_EPSILON):
        adjusted_y = max_y - BOUNDARY_EPSILON

    if not (config.origin_x_m <= adjusted_x < max_x):
        return None
    if not (config.origin_y_m <= adjusted_y < max_y):
        return None

    return world_to_grid_unbounded(config, adjusted_x, adjusted_y)


def grid_to_world_center(
    config: OccupancyGridConfig, grid_x: int, grid_y: int,
) -> tuple[float, float]:
    """Return the world (x, y) coordinate at the centre of one grid cell."""
    world_x = config.origin_x_m + (grid_x + 0.5) * config.resolution_m
    world_y = config.origin_y_m + (grid_y + 0.5) * config.resolution_m
    return world_x, world_y


def grid_index(config: OccupancyGridConfig, grid_x: int, grid_y: int) -> int:
    """Return the row-major flat index for one grid cell."""
    return grid_y * config.width_cells + grid_x


def is_cell_in_bounds(config: OccupancyGridConfig, grid_x: int, grid_y: int) -> bool:
    """Return whether a grid cell index falls within the mapped grid."""
    return 0 <= grid_x < config.width_cells and 0 <= grid_y < config.height_cells


def calculate_beam_world_angle(rover_yaw: float, beam_relative_angle: float) -> float:
    """Rotate a beam's body-relative angle into the world map frame."""
    return rover_yaw + beam_relative_angle


def calculate_beam_endpoint(
    rover_x: float, rover_y: float, beam_world_angle: float, measured_range: float,
) -> tuple[float, float]:
    """Return the world (x, y) position a beam's measured range reaches."""
    endpoint_x = rover_x + measured_range * math.cos(beam_world_angle)
    endpoint_y = rover_y + measured_range * math.sin(beam_world_angle)
    return endpoint_x, endpoint_y


def bresenham_line(
    start_x: int, start_y: int, end_x: int, end_y: int,
) -> list[tuple[int, int]]:
    """Return an ordered, duplicate-free list of integer cells from start to end."""
    # A standard symmetric integer Bresenham line, inclusive of both the
    # start and end cell. Handles horizontal, vertical, diagonal, steep, and
    # reversed-direction lines, including the degenerate case where start
    # and end are the same cell.
    cells = []

    delta_x = abs(end_x - start_x)
    delta_y = -abs(end_y - start_y)
    step_x = 1 if start_x < end_x else -1
    step_y = 1 if start_y < end_y else -1
    error = delta_x + delta_y

    x, y = start_x, start_y
    while True:
        cells.append((x, y))
        if x == end_x and y == end_y:
            break
        doubled_error = 2 * error
        if doubled_error >= delta_y:
            error += delta_y
            x += step_x
        if doubled_error <= delta_x:
            error += delta_x
            y += step_y

    return cells


def trace_ray_cells(
    config: OccupancyGridConfig,
    start_cell: tuple[int, int],
    end_cell: tuple[int, int],
) -> list[tuple[int, int]]:
    """Return the in-bounds cells along the straight line from start to end."""
    raw_cells = bresenham_line(start_cell[0], start_cell[1], end_cell[0], end_cell[1])
    return [cell for cell in raw_cells if is_cell_in_bounds(config, *cell)]


def clamp_evidence(config: OccupancyGridConfig, value: int) -> int:
    """Clamp an evidence value into the configured [minimum, maximum] bounds."""
    return max(config.minimum_evidence, min(config.maximum_evidence, value))


def apply_free_evidence(state: OccupancyGridState, grid_x: int, grid_y: int) -> None:
    """Push one cell's evidence toward free by the configured delta, in place."""
    index = grid_index(state.config, grid_x, grid_y)
    state.evidence[index] = clamp_evidence(
        state.config, state.evidence[index] + state.config.free_evidence_delta,
    )


def apply_occupied_evidence(state: OccupancyGridState, grid_x: int, grid_y: int) -> None:
    """Push one cell's evidence toward occupied by the configured delta, in place."""
    index = grid_index(state.config, grid_x, grid_y)
    state.evidence[index] = clamp_evidence(
        state.config, state.evidence[index] + state.config.occupied_evidence_delta,
    )


def is_range_reading_valid(
    measured_range: float, min_range: float, max_range: float,
) -> bool:
    """Return whether a measured range is a usable, in-range number."""
    if measured_range is None:
        return False
    if math.isnan(measured_range) or math.isinf(measured_range):
        return False
    if measured_range < min_range - RANGE_VALUE_TOLERANCE:
        return False
    if measured_range > max_range + RANGE_VALUE_TOLERANCE:
        return False
    return True


def is_no_hit_reading(measured_range: float, max_range: float) -> bool:
    """Return whether a range reading means no obstacle was detected."""
    return math.isclose(
        measured_range, max_range, rel_tol=0.0, abs_tol=RANGE_VALUE_TOLERANCE,
    )


def update_grid_with_beam(
    state: OccupancyGridState,
    rover_x: float,
    rover_y: float,
    rover_yaw: float,
    beam_relative_angle: float,
    measured_range: float,
    min_range: float,
    max_range: float,
) -> None:
    """Apply one beam's range reading to the persistent evidence grid, in place."""
    # Cells between the rover and the measured endpoint are marked free. A
    # reading below max_range marks its endpoint occupied; a reading at
    # max_range means no obstacle was detected, so only free space is
    # updated. The rover's own cell is always marked free, never occupied,
    # even when a very short reading places the endpoint in the rover's own
    # cell. An invalid reading, or a rover position outside the mapped
    # world, leaves the grid unchanged.
    if not is_range_reading_valid(measured_range, min_range, max_range):
        return

    config = state.config
    rover_cell = world_to_grid(config, rover_x, rover_y)
    if rover_cell is None:
        return

    beam_world_angle = calculate_beam_world_angle(rover_yaw, beam_relative_angle)
    endpoint_x, endpoint_y = calculate_beam_endpoint(
        rover_x, rover_y, beam_world_angle, measured_range,
    )
    is_hit = not is_no_hit_reading(measured_range, max_range)
    endpoint_cell = world_to_grid(config, endpoint_x, endpoint_y)

    if endpoint_cell is None:
        # The measured endpoint falls outside the mapped world. Trace
        # toward its raw (unclamped) grid coordinates anyway so traversal
        # still follows the correct direction; trace_ray_cells drops every
        # cell past the map edge, so only the in-map portion is marked
        # free and no occupied cell is ever created from an out-of-map hit.
        raw_endpoint_cell = world_to_grid_unbounded(config, endpoint_x, endpoint_y)
        for cell in trace_ray_cells(config, rover_cell, raw_endpoint_cell):
            apply_free_evidence(state, *cell)
        return

    # Both endpoints are confirmed in-map, and the map is a rectangle (a
    # convex region), so the straight line between them cannot leave the
    # grid — the last traced cell is always the endpoint itself.
    path_cells = trace_ray_cells(config, rover_cell, endpoint_cell)
    free_cells = path_cells[:-1]

    for cell in free_cells:
        apply_free_evidence(state, *cell)

    if is_hit and endpoint_cell != rover_cell:
        apply_occupied_evidence(state, *endpoint_cell)
    else:
        apply_free_evidence(state, *endpoint_cell)


def encode_occupancy(config: OccupancyGridConfig, evidence_value: int) -> int:
    """Encode one cell's internal evidence into a ROS OccupancyGrid value."""
    if evidence_value <= config.free_threshold:
        return FREE_OCCUPANCY
    if evidence_value >= config.occupied_threshold:
        return OCCUPIED_OCCUPANCY
    return UNKNOWN_OCCUPANCY


def build_occupancy_grid_data(state: OccupancyGridState) -> list[int]:
    """Encode the full evidence grid into row-major ROS OccupancyGrid data."""
    return [encode_occupancy(state.config, value) for value in state.evidence]


def is_measurement_fresh(
    measurement_stamp_seconds: float | None,
    now_seconds: float,
    timeout_s: float,
) -> bool:
    """Return whether a stamped measurement is fresh enough to use right now."""
    # A missing measurement, a non-finite age, or a measurement stamped in
    # the future relative to now (clock skew or out-of-order delivery) is
    # treated as untrustworthy rather than guessing which clock is wrong.
    if measurement_stamp_seconds is None:
        return False

    age_seconds = now_seconds - measurement_stamp_seconds
    if not math.isfinite(age_seconds):
        return False
    if age_seconds < 0.0:
        return False

    return age_seconds <= timeout_s


def is_new_range_sample(
    sample_stamp_seconds: float, last_processed_stamp_seconds: float | None,
) -> bool:
    """Return whether a range sample's stamp has not already been processed."""
    # A duplicate or older-or-equal stamp is rejected here, before any
    # freshness check, so the same sample (or a late-arriving older one)
    # never updates the map twice.
    if last_processed_stamp_seconds is None:
        return True
    if not math.isfinite(sample_stamp_seconds):
        return False

    return sample_stamp_seconds > last_processed_stamp_seconds
