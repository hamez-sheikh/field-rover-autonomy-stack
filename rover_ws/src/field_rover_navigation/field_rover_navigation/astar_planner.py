"""Pure Python eight- or four-connected A* planner over an occupancy grid."""

from dataclasses import dataclass, field
import heapq
import itertools
import math

from field_rover_navigation.occupancy_grid import (
    grid_index,
    grid_to_world_center,
    is_cell_in_bounds,
    UNKNOWN_OCCUPANCY,
    world_to_grid,
)


# Cell classifications returned by classify_cell.
CELL_FREE = 'free'
CELL_OCCUPIED = 'occupied'
CELL_UNKNOWN = 'unknown'

# Grid neighbor offsets, in a fixed order so search and neighbor generation
# are deterministic regardless of dict/set iteration order.
ORTHOGONAL_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))
DIAGONAL_STEPS = ((1, 1), (1, -1), (-1, 1), (-1, -1))

ORTHOGONAL_COST = 1.0
DIAGONAL_COST = math.sqrt(2.0)

# Tolerance used when comparing accumulated path costs, so floating-point
# rounding never causes a strictly-better path to be rejected or re-expanded.
COST_EPSILON = 1e-9


@dataclass(frozen=True)
class GridGeometry:
    """Store the size, resolution, and origin of one occupancy grid snapshot."""

    width_cells: int
    height_cells: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float

    def __post_init__(self):
        """Reject grid geometry that cannot host a search or coordinate mapping."""
        if self.width_cells <= 0 or self.height_cells <= 0:
            raise ValueError('width_cells and height_cells must be positive.')
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError('resolution_m must be positive and finite.')
        if not math.isfinite(self.origin_x_m) or not math.isfinite(self.origin_y_m):
            raise ValueError('origin_x_m and origin_y_m must be finite.')

    @property
    def world_width_m(self) -> float:
        """Return the grid width in metres, matching OccupancyGridConfig's field."""
        return self.width_cells * self.resolution_m

    @property
    def world_height_m(self) -> float:
        """Return the grid height in metres, matching OccupancyGridConfig's field."""
        return self.height_cells * self.resolution_m

    @property
    def total_cells(self) -> int:
        """Return the total number of cells in the grid."""
        return self.width_cells * self.height_cells


@dataclass(frozen=True)
class AStarConfig:
    """Store planner behavior: connectivity, occupancy rules, and search limits."""

    allow_diagonal: bool = True
    prevent_corner_cutting: bool = True
    allow_unknown: bool = False
    occupied_threshold: int = 50
    max_expansions: int = 100_000
    unknown_traversal_cost: float = 5.0

    def __post_init__(self):
        """Reject a configuration that cannot produce a safe, admissible search."""
        if not (1 <= self.occupied_threshold <= 100):
            raise ValueError('occupied_threshold must be between 1 and 100 inclusive.')
        if self.max_expansions <= 0:
            raise ValueError('max_expansions must be a positive integer.')
        if not math.isfinite(self.unknown_traversal_cost) or self.unknown_traversal_cost < 0.0:
            raise ValueError('unknown_traversal_cost must be non-negative and finite.')


DEFAULT_ASTAR_CONFIG = AStarConfig()


@dataclass(frozen=True)
class AStarResult:
    """Report the outcome of one planning attempt."""

    success: bool
    grid_path: tuple = field(default_factory=tuple)
    cost: float = 0.0
    expanded_nodes: int = 0
    failure_reason: str | None = None


def classify_cell(value: int, config: AStarConfig) -> str:
    """Classify a raw OccupancyGrid cell value as free, occupied, or unknown."""
    if value == UNKNOWN_OCCUPANCY:
        return CELL_UNKNOWN
    if value >= config.occupied_threshold:
        return CELL_OCCUPIED
    return CELL_FREE


def read_cell_value(geometry: GridGeometry, data: list[int], cell: tuple[int, int]) -> int:
    """Return the raw occupancy value stored at one in-bounds grid cell."""
    return data[grid_index(geometry, cell[0], cell[1])]


def is_cell_enterable(
    geometry: GridGeometry, data: list[int], config: AStarConfig, cell: tuple[int, int],
) -> tuple[bool, str | None]:
    """Return whether a cell can be entered, and its occupancy classification."""
    if not is_cell_in_bounds(geometry, cell[0], cell[1]):
        return False, None
    kind = classify_cell(read_cell_value(geometry, data, cell), config)
    if kind == CELL_OCCUPIED:
        return False, kind
    if kind == CELL_UNKNOWN and not config.allow_unknown:
        return False, kind
    return True, kind


def generate_neighbors(
    geometry: GridGeometry, data: list[int], config: AStarConfig, cell: tuple[int, int],
) -> list[tuple[tuple[int, int], float]]:
    """Return each enterable neighbor of one cell with its movement cost."""
    x, y = cell
    neighbors = []

    for dx, dy in ORTHOGONAL_STEPS:
        candidate = (x + dx, y + dy)
        enterable, kind = is_cell_enterable(geometry, data, config, candidate)
        if not enterable:
            continue
        cost = ORTHOGONAL_COST
        if kind == CELL_UNKNOWN:
            cost += config.unknown_traversal_cost
        neighbors.append((candidate, cost))

    if config.allow_diagonal:
        for dx, dy in DIAGONAL_STEPS:
            candidate = (x + dx, y + dy)
            enterable, kind = is_cell_enterable(geometry, data, config, candidate)
            if not enterable:
                continue
            if config.prevent_corner_cutting:
                # A diagonal step is only allowed when both orthogonal cells
                # it passes between are also enterable, so the path never
                # squeezes through the corner between two blocked cells.
                side_a_ok, _ = is_cell_enterable(geometry, data, config, (x + dx, y))
                side_b_ok, _ = is_cell_enterable(geometry, data, config, (x, y + dy))
                if not (side_a_ok and side_b_ok):
                    continue
            cost = DIAGONAL_COST
            if kind == CELL_UNKNOWN:
                cost += config.unknown_traversal_cost
            neighbors.append((candidate, cost))

    return neighbors


def manhattan_distance(dx: float, dy: float) -> float:
    """Return the Manhattan distance for two non-negative axis deltas."""
    return dx + dy


def octile_distance(dx: float, dy: float) -> float:
    """Return the octile distance for two non-negative axis deltas."""
    return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


def heuristic_cost(
    cell: tuple[int, int], goal: tuple[int, int], config: AStarConfig,
) -> float:
    """Return an admissible heuristic estimate matching the configured connectivity."""
    dx = abs(goal[0] - cell[0])
    dy = abs(goal[1] - cell[1])
    if config.allow_diagonal:
        return octile_distance(dx, dy)
    return manhattan_distance(dx, dy)


def reconstruct_path(
    parents: dict[tuple[int, int], tuple[int, int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    """Trace parent links from goal back to start, then return start-to-goal order."""
    path = [goal]
    current = goal
    while current != start:
        current = parents[current]
        path.append(current)
    path.reverse()
    return path


def run_astar_search(
    geometry: GridGeometry,
    data: list[int],
    config: AStarConfig,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> AStarResult:
    """Run A* between two already-validated, distinct grid cells."""
    # heapq entries are (priority, tie_break, cost_so_far, cell). The
    # monotonically increasing tie_break makes ordering among equal
    # priorities deterministic, independent of Python's dict/set hashing.
    tie_break = itertools.count()
    open_heap = [(heuristic_cost(start, goal, config), next(tie_break), 0.0, start)]
    best_cost = {start: 0.0}
    parents: dict[tuple[int, int], tuple[int, int]] = {}
    closed = set()
    expanded = 0

    while open_heap:
        if expanded >= config.max_expansions:
            return AStarResult(
                success=False, expanded_nodes=expanded, failure_reason='max_expansions_reached',
            )

        _, _, cost_so_far, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if cost_so_far > best_cost.get(current, math.inf) + COST_EPSILON:
            continue
        closed.add(current)
        expanded += 1

        if current == goal:
            path = reconstruct_path(parents, start, goal)
            return AStarResult(
                success=True, grid_path=tuple(path), cost=cost_so_far, expanded_nodes=expanded,
            )

        for neighbor, step_cost in generate_neighbors(geometry, data, config, current):
            if neighbor in closed:
                continue
            tentative_cost = cost_so_far + step_cost
            if tentative_cost < best_cost.get(neighbor, math.inf) - COST_EPSILON:
                best_cost[neighbor] = tentative_cost
                parents[neighbor] = current
                priority = tentative_cost + heuristic_cost(neighbor, goal, config)
                heapq.heappush(open_heap, (priority, next(tie_break), tentative_cost, neighbor))

    return AStarResult(success=False, expanded_nodes=expanded, failure_reason='no_path')


def plan_grid_path(
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    data: list[int],
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
    config: AStarConfig = DEFAULT_ASTAR_CONFIG,
) -> AStarResult:
    """Plan a collision-free grid path between two world points on one map."""
    start_x, start_y = start_world
    goal_x, goal_y = goal_world
    if not all(math.isfinite(value) for value in (start_x, start_y, goal_x, goal_y)):
        return AStarResult(success=False, failure_reason='invalid_coordinates')

    try:
        geometry = GridGeometry(width_cells, height_cells, resolution_m, origin_x_m, origin_y_m)
    except ValueError:
        return AStarResult(success=False, failure_reason='invalid_map_metadata')

    if len(data) != geometry.total_cells:
        return AStarResult(success=False, failure_reason='invalid_map_data_length')

    start_cell = world_to_grid(geometry, start_x, start_y)
    if start_cell is None:
        return AStarResult(success=False, failure_reason='start_out_of_bounds')
    goal_cell = world_to_grid(geometry, goal_x, goal_y)
    if goal_cell is None:
        return AStarResult(success=False, failure_reason='goal_out_of_bounds')

    start_kind = classify_cell(read_cell_value(geometry, data, start_cell), config)
    if start_kind == CELL_OCCUPIED:
        return AStarResult(success=False, failure_reason='start_occupied')
    if start_kind == CELL_UNKNOWN and not config.allow_unknown:
        return AStarResult(success=False, failure_reason='start_unknown')

    goal_kind = classify_cell(read_cell_value(geometry, data, goal_cell), config)
    if goal_kind == CELL_OCCUPIED:
        return AStarResult(success=False, failure_reason='goal_occupied')
    if goal_kind == CELL_UNKNOWN and not config.allow_unknown:
        return AStarResult(success=False, failure_reason='goal_unknown')

    if start_cell == goal_cell:
        return AStarResult(success=True, grid_path=(start_cell,), cost=0.0, expanded_nodes=0)

    return run_astar_search(geometry, data, config, start_cell, goal_cell)


def world_path_from_grid_path(
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    grid_path: tuple,
) -> list[tuple[float, float]]:
    """Convert a sequence of grid cells into world (x, y) cell-centre points."""
    geometry = GridGeometry(width_cells, height_cells, resolution_m, origin_x_m, origin_y_m)
    return [grid_to_world_center(geometry, grid_x, grid_y) for grid_x, grid_y in grid_path]


def yaw_between_points(x1: float, y1: float, x2: float, y2: float) -> float:
    """Return the planar heading from one world point toward the next."""
    return math.atan2(y2 - y1, x2 - x1)


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    """Convert a planar yaw angle into the (z, w) components of a quaternion."""
    half_yaw = yaw / 2.0
    return math.sin(half_yaw), math.cos(half_yaw)


def compute_path_yaws(
    world_points: list[tuple[float, float]], goal_yaw: float | None = None,
) -> list[float]:
    """Return one heading per world point, facing each point toward the next."""
    if not world_points:
        return []
    if len(world_points) == 1:
        if goal_yaw is not None and math.isfinite(goal_yaw):
            return [goal_yaw]
        return [0.0]

    yaws = [
        yaw_between_points(x1, y1, x2, y2)
        for (x1, y1), (x2, y2) in zip(world_points, world_points[1:])
    ]
    # The final pose has no next point to face, so it keeps the heading of
    # the segment that arrives at it.
    yaws.append(yaws[-1])
    return yaws
