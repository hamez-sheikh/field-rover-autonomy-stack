"""Unit tests for the pure Python eight/four-connected A* planner."""

import math

from field_rover_navigation.astar_planner import (
    AStarConfig,
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_UNKNOWN,
    classify_cell,
    compute_path_yaws,
    DEFAULT_ASTAR_CONFIG,
    generate_neighbors,
    GridGeometry,
    heuristic_cost,
    is_cell_enterable,
    manhattan_distance,
    octile_distance,
    ORTHOGONAL_STEPS,
    plan_grid_path,
    world_path_from_grid_path,
    yaw_between_points,
    yaw_to_quaternion_zw,
)
from field_rover_navigation.occupancy_grid import grid_to_world_center, world_to_grid

import pytest


SQRT2 = math.sqrt(2.0)

_SYMBOL_TO_VALUE = {'.': 0, '#': 100, '?': -1}


def make_grid(rows: list[str]) -> tuple[int, int, list[int]]:
    """Build (width, height, data) from equal-length row strings."""
    # '.' is free (0), '#' is occupied (100), '?' is unknown (-1). rows[0]
    # is grid row y=0, matching the row-major layout used by grid_index.
    height = len(rows)
    width = len(rows[0])
    data = []
    for row in rows:
        data.extend(_SYMBOL_TO_VALUE[symbol] for symbol in row)
    return width, height, data


def assert_no_corner_cutting(
    width, height, resolution_m, origin_x_m, origin_y_m, data, config, path,
):
    """Confirm no consecutive diagonal step in path cuts a blocked corner."""
    geometry = GridGeometry(width, height, resolution_m, origin_x_m, origin_y_m)
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        dx, dy = x2 - x1, y2 - y1
        if abs(dx) == 1 and abs(dy) == 1:
            side_a_ok, _ = is_cell_enterable(geometry, data, config, (x1 + dx, y1))
            side_b_ok, _ = is_cell_enterable(geometry, data, config, (x1, y1 + dy))
            assert side_a_ok and side_b_ok, f'corner cut between {(x1, y1)} and {(x2, y2)}'


def assert_valid_path(
    width, height, resolution_m, origin_x_m, origin_y_m, data, config, result, start, goal,
):
    """Confirm a successful result is a well-formed, traversable, connected path."""
    assert result.success
    assert len(result.grid_path) > 0
    assert result.grid_path[0] == start
    assert result.grid_path[-1] == goal

    total_cost = 0.0
    for index, cell in enumerate(result.grid_path):
        x, y = cell
        assert 0 <= x < width and 0 <= y < height
        value = data[y * width + x]
        kind = classify_cell(value, config)
        assert kind != CELL_OCCUPIED
        assert kind != CELL_UNKNOWN or config.allow_unknown

        if index > 0:
            prev_x, prev_y = result.grid_path[index - 1]
            dx, dy = x - prev_x, y - prev_y
            assert (abs(dx), abs(dy)) in {(1, 0), (0, 1), (1, 1)}
            step_cost = SQRT2 if abs(dx) == 1 and abs(dy) == 1 else 1.0
            if kind == CELL_UNKNOWN:
                step_cost += config.unknown_traversal_cost
            total_cost += step_cost

    assert result.cost == pytest.approx(total_cost)
    assert_no_corner_cutting(
        width, height, resolution_m, origin_x_m, origin_y_m, data, config, result.grid_path,
    )


def plan(width, height, data, start_world, goal_world, config=DEFAULT_ASTAR_CONFIG):
    """Call plan_grid_path with resolution=1.0 and origin=(0, 0) (test helper)."""
    return plan_grid_path(width, height, 1.0, 0.0, 0.0, data, start_world, goal_world, config)


OPEN_5X5 = make_grid(['.....'] * 5)
WALL_5X5 = make_grid(['.....', '.....', '.###.', '.....', '.....'])
BARRIER_5X5 = make_grid(['.....', '.....', '#####', '.....', '.....'])
CORRIDOR_5X5 = make_grid(['#.###', '#.###', '#...#', '###.#', '###.#'])
GATE_5X3 = make_grid(['.....', '##?##', '.....'])
CORNER_4X4 = make_grid(['....', '..#.', '.#..', '....'])
CORNER_MICRO_2X2 = make_grid(['.#', '#.'])


# --- Configuration -------------------------------------------------------


def test_default_configuration_matches_recommended_values():
    """Confirm the shipped default configuration matches recommended values."""
    config = DEFAULT_ASTAR_CONFIG
    assert config.allow_diagonal is True
    assert config.prevent_corner_cutting is True
    assert config.allow_unknown is False
    assert config.occupied_threshold == 50
    assert config.max_expansions == 100_000
    assert config.unknown_traversal_cost == pytest.approx(5.0)


def test_valid_configuration_is_accepted():
    """Confirm a fully valid non-default configuration passes validation."""
    AStarConfig(
        allow_diagonal=False, prevent_corner_cutting=False, allow_unknown=True,
        occupied_threshold=1, max_expansions=10, unknown_traversal_cost=0.0,
    )


def test_occupied_threshold_below_range_is_rejected():
    """Confirm an occupied_threshold below 1 fails validation."""
    with pytest.raises(ValueError):
        AStarConfig(occupied_threshold=0)


def test_occupied_threshold_above_range_is_rejected():
    """Confirm an occupied_threshold above 100 fails validation."""
    with pytest.raises(ValueError):
        AStarConfig(occupied_threshold=101)


def test_non_positive_max_expansions_is_rejected():
    """Confirm a zero or negative max_expansions fails validation."""
    with pytest.raises(ValueError):
        AStarConfig(max_expansions=0)
    with pytest.raises(ValueError):
        AStarConfig(max_expansions=-5)


def test_negative_unknown_traversal_cost_is_rejected():
    """Confirm a negative unknown_traversal_cost fails validation."""
    with pytest.raises(ValueError):
        AStarConfig(unknown_traversal_cost=-1.0)


def test_non_finite_unknown_traversal_cost_is_rejected():
    """Confirm an infinite or NaN unknown_traversal_cost fails validation."""
    with pytest.raises(ValueError):
        AStarConfig(unknown_traversal_cost=math.inf)
    with pytest.raises(ValueError):
        AStarConfig(unknown_traversal_cost=math.nan)


def test_multiple_invalid_fields_together_are_rejected():
    """Confirm a configuration invalid in more than one field still fails."""
    with pytest.raises(ValueError):
        AStarConfig(occupied_threshold=500, max_expansions=-1)


# --- Occupancy interpretation ---------------------------------------------


def test_free_cell_is_traversable():
    """Confirm a value of 0 classifies as free."""
    assert classify_cell(0, DEFAULT_ASTAR_CONFIG) == CELL_FREE


def test_occupied_cell_is_blocked():
    """Confirm a value of 100 classifies as occupied."""
    assert classify_cell(100, DEFAULT_ASTAR_CONFIG) == CELL_OCCUPIED


def test_unknown_cell_blocked_by_default():
    """Confirm a value of -1 classifies as unknown and is not enterable by default."""
    assert classify_cell(-1, DEFAULT_ASTAR_CONFIG) == CELL_UNKNOWN
    geometry = GridGeometry(3, 3, 1.0, 0.0, 0.0)
    data = [0, 0, 0, 0, -1, 0, 0, 0, 0]
    enterable, kind = is_cell_enterable(geometry, data, DEFAULT_ASTAR_CONFIG, (1, 1))
    assert enterable is False
    assert kind == CELL_UNKNOWN


def test_unknown_cell_traversable_only_when_enabled():
    """Confirm an unknown cell becomes enterable only when allow_unknown is True."""
    geometry = GridGeometry(3, 3, 1.0, 0.0, 0.0)
    data = [0, 0, 0, 0, -1, 0, 0, 0, 0]
    config = AStarConfig(allow_unknown=True)
    enterable, kind = is_cell_enterable(geometry, data, config, (1, 1))
    assert enterable is True
    assert kind == CELL_UNKNOWN


def test_occupied_threshold_boundary_behavior():
    """Confirm the occupied_threshold boundary is inclusive on the occupied side."""
    config = AStarConfig(occupied_threshold=50)
    assert classify_cell(49, config) == CELL_FREE
    assert classify_cell(50, config) == CELL_OCCUPIED


def test_intermediate_occupancy_value_below_threshold_is_free():
    """Confirm a non-binary evidence-derived value below threshold is still free."""
    assert classify_cell(37, DEFAULT_ASTAR_CONFIG) == CELL_FREE


# --- Neighbors -------------------------------------------------------------


def test_four_connected_center_cell_neighbors():
    """Confirm a four-connected center cell returns exactly its 4 orthogonal neighbors."""
    width, height, data = OPEN_5X5
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    config = AStarConfig(allow_diagonal=False)
    neighbors = generate_neighbors(geometry, data, config, (2, 2))
    cells = {cell for cell, _ in neighbors}
    assert cells == {(3, 2), (1, 2), (2, 3), (2, 1)}
    assert all(cost == pytest.approx(1.0) for _, cost in neighbors)


def orthogonal_neighbor_cells(cell):
    """Return the set of orthogonal neighbor cells of one cell (test helper)."""
    x, y = cell
    return {(x + dx, y + dy) for dx, dy in ORTHOGONAL_STEPS}


def test_eight_connected_center_cell_neighbors():
    """Confirm an eight-connected center cell returns 4 orthogonal + 4 diagonal neighbors."""
    width, height, data = OPEN_5X5
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    neighbors = generate_neighbors(geometry, data, DEFAULT_ASTAR_CONFIG, (2, 2))
    costs_by_cell = dict(neighbors)
    assert set(costs_by_cell) == {
        (3, 2), (1, 2), (2, 3), (2, 1), (3, 3), (3, 1), (1, 3), (1, 1),
    }
    orthogonal_cells = orthogonal_neighbor_cells((2, 2))
    for cell, cost in costs_by_cell.items():
        expected = 1.0 if cell in orthogonal_cells else SQRT2
        assert cost == pytest.approx(expected)


def test_edge_cell_neighbors_exclude_out_of_bounds():
    """Confirm a left-edge cell has no neighbor with a negative x index."""
    width, height, data = OPEN_5X5
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    neighbors = generate_neighbors(geometry, data, DEFAULT_ASTAR_CONFIG, (0, 2))
    cells = {cell for cell, _ in neighbors}
    assert all(x >= 0 for x, _ in cells)
    assert len(cells) == 5


def test_corner_cell_neighbors_exclude_out_of_bounds():
    """Confirm a grid-corner cell only returns its in-bounds neighbors."""
    width, height, data = OPEN_5X5
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    neighbors = generate_neighbors(geometry, data, DEFAULT_ASTAR_CONFIG, (0, 0))
    cells = {cell for cell, _ in neighbors}
    assert cells == {(1, 0), (0, 1), (1, 1)}


def test_occupied_neighbor_is_excluded():
    """Confirm an occupied neighbor cell never appears in the neighbor list."""
    width, height, data = make_grid(['...', '.#.', '...'])
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    config = AStarConfig(allow_diagonal=False)
    neighbors = generate_neighbors(geometry, data, config, (1, 0))
    cells = {cell for cell, _ in neighbors}
    assert (1, 1) not in cells


def test_unknown_neighbor_excluded_by_default_and_included_when_enabled():
    """Confirm an unknown neighbor is excluded by default and included with a cost when enabled."""
    width, height, data = make_grid(['...', '.?.', '...'])
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    default_neighbors = generate_neighbors(geometry, data, DEFAULT_ASTAR_CONFIG, (1, 0))
    assert (1, 1) not in {cell for cell, _ in default_neighbors}

    allow_config = AStarConfig(allow_unknown=True, unknown_traversal_cost=2.5)
    allowed_neighbors = generate_neighbors(geometry, data, allow_config, (1, 0))
    allowed_by_cell = dict(allowed_neighbors)
    assert (1, 1) in allowed_by_cell
    assert allowed_by_cell[(1, 1)] == pytest.approx(1.0 + 2.5)


def test_diagonal_neighbors_included_only_when_enabled():
    """Confirm diagonal neighbors appear only when allow_diagonal is True."""
    width, height, data = OPEN_5X5
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)

    no_diagonal = generate_neighbors(geometry, data, AStarConfig(allow_diagonal=False), (2, 2))
    assert (3, 3) not in {cell for cell, _ in no_diagonal}

    with_diagonal = generate_neighbors(geometry, data, AStarConfig(allow_diagonal=True), (2, 2))
    diagonal_by_cell = dict(with_diagonal)
    assert (3, 3) in diagonal_by_cell
    assert diagonal_by_cell[(3, 3)] == pytest.approx(SQRT2)


def test_corner_cutting_prevented_by_default():
    """Confirm a diagonal move between two touching blocked cells is rejected."""
    width, height, data = CORNER_MICRO_2X2
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    neighbors = generate_neighbors(geometry, data, DEFAULT_ASTAR_CONFIG, (0, 0))
    assert neighbors == []


def test_corner_cutting_allowed_when_disabled():
    """Confirm disabling prevent_corner_cutting allows the same diagonal move."""
    width, height, data = CORNER_MICRO_2X2
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    config = AStarConfig(prevent_corner_cutting=False)
    neighbors = generate_neighbors(geometry, data, config, (0, 0))
    assert neighbors == [((1, 1), pytest.approx(SQRT2))]


def test_no_duplicate_neighbor_cells():
    """Confirm no neighbor cell is returned more than once."""
    width, height, data = OPEN_5X5
    geometry = GridGeometry(width, height, 1.0, 0.0, 0.0)
    neighbors = generate_neighbors(geometry, data, DEFAULT_ASTAR_CONFIG, (2, 2))
    cells = [cell for cell, _ in neighbors]
    assert len(cells) == len(set(cells))


# --- Heuristics --------------------------------------------------------


def test_heuristic_zero_distance():
    """Confirm the heuristic is zero when the cell and goal coincide."""
    assert heuristic_cost((2, 2), (2, 2), DEFAULT_ASTAR_CONFIG) == pytest.approx(0.0)


def test_manhattan_distance_matches_four_connected_heuristic():
    """Confirm the four-connected heuristic equals Manhattan distance."""
    config = AStarConfig(allow_diagonal=False)
    assert heuristic_cost((0, 0), (3, 4), config) == pytest.approx(7.0)
    assert manhattan_distance(3, 4) == pytest.approx(7.0)


def test_octile_distance_matches_eight_connected_heuristic():
    """Confirm the eight-connected heuristic equals octile distance."""
    config = AStarConfig(allow_diagonal=True)
    expected = 4.0 + 3.0 * (SQRT2 - 1.0)
    assert heuristic_cost((0, 0), (3, 4), config) == pytest.approx(expected)
    assert octile_distance(3, 4) == pytest.approx(expected)


def test_heuristic_is_symmetric():
    """Confirm the heuristic is the same in either direction, for both connectivities."""
    for config in (AStarConfig(allow_diagonal=False), AStarConfig(allow_diagonal=True)):
        assert heuristic_cost((0, 0), (3, 4), config) == pytest.approx(
            heuristic_cost((3, 4), (0, 0), config)
        )


def test_heuristic_is_never_negative():
    """Confirm the heuristic never returns a negative value."""
    for config in (AStarConfig(allow_diagonal=False), AStarConfig(allow_diagonal=True)):
        assert heuristic_cost((5, 1), (0, 6), config) >= 0.0


def test_heuristic_does_not_exceed_known_open_grid_path_cost():
    """Confirm the heuristic never overestimates a real path's cost in an open grid."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (0.5, 0.5), (4.5, 4.5), DEFAULT_ASTAR_CONFIG)
    assert result.success
    assert heuristic_cost((0, 0), (4, 4), DEFAULT_ASTAR_CONFIG) <= result.cost + 1e-9


# --- Planning ------------------------------------------------------------


def test_straight_horizontal_path():
    """Confirm a straight horizontal path in an open grid is optimal and exact."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (0.5, 0.5), (4.5, 0.5), DEFAULT_ASTAR_CONFIG)
    assert result.success
    assert result.grid_path == ((0, 0), (1, 0), (2, 0), (3, 0), (4, 0))
    assert result.cost == pytest.approx(4.0)


def test_straight_vertical_path():
    """Confirm a straight vertical path in an open grid is optimal and exact."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (0.5, 0.5), (0.5, 4.5), DEFAULT_ASTAR_CONFIG)
    assert result.success
    assert result.grid_path == ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4))
    assert result.cost == pytest.approx(4.0)


def test_diagonal_path():
    """Confirm a pure diagonal path in an open grid is optimal and exact."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (0.5, 0.5), (4.5, 4.5), DEFAULT_ASTAR_CONFIG)
    assert result.success
    assert result.grid_path == ((0, 0), (1, 1), (2, 2), (3, 3), (4, 4))
    assert result.cost == pytest.approx(4.0 * SQRT2)
    orthogonal_steps = sum(
        1 for (x1, y1), (x2, y2) in zip(result.grid_path, result.grid_path[1:])
        if x1 == x2 or y1 == y2
    )
    diagonal_steps = len(result.grid_path) - 1 - orthogonal_steps
    assert orthogonal_steps == 0
    assert diagonal_steps == 4


def test_path_around_one_obstacle():
    """Confirm the planner detours around a wall instead of crossing it."""
    width, height, data = WALL_5X5
    config = DEFAULT_ASTAR_CONFIG
    result = plan(width, height, data, (2.5, 0.5), (2.5, 4.5), config)
    assert_valid_path(width, height, 1.0, 0.0, 0.0, data, config, result, (2, 0), (2, 4))
    direct_distance = 4.0
    assert result.cost > direct_distance


def test_path_through_narrow_valid_corridor():
    """Confirm the planner finds the unique route through a one-cell-wide corridor."""
    width, height, data = CORRIDOR_5X5
    result = plan(width, height, data, (1.5, 0.5), (3.5, 4.5), DEFAULT_ASTAR_CONFIG)
    assert result.success
    assert result.grid_path == ((1, 0), (1, 1), (1, 2), (2, 2), (3, 2), (3, 3), (3, 4))
    assert result.cost == pytest.approx(6.0)


def test_no_path_through_fully_blocked_barrier():
    """Confirm planning fails cleanly when no route exists around a full wall."""
    width, height, data = BARRIER_5X5
    result = plan(width, height, data, (2.5, 0.5), (2.5, 4.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.grid_path == ()
    assert result.cost == pytest.approx(0.0)
    assert result.failure_reason == 'no_path'


def test_unknown_region_blocked_by_default():
    """Confirm the sole connecting cell being unknown fails planning by default."""
    width, height, data = GATE_5X3
    result = plan(width, height, data, (2.5, 0.5), (2.5, 2.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'no_path'


def test_unknown_region_allowed_when_configured():
    """Confirm the same gate succeeds, at extra cost, once allow_unknown is enabled."""
    width, height, data = GATE_5X3
    config = AStarConfig(allow_unknown=True, unknown_traversal_cost=5.0)
    result = plan(width, height, data, (2.5, 0.5), (2.5, 2.5), config)
    assert result.success
    assert result.grid_path == ((2, 0), (2, 1), (2, 2))
    assert result.cost == pytest.approx(7.0)


def test_start_equals_goal():
    """Confirm planning inside the rover's current cell succeeds trivially."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (2.2, 2.7), (2.9, 2.1), DEFAULT_ASTAR_CONFIG)
    assert result.success
    assert result.grid_path == ((2, 2),)
    assert result.cost == pytest.approx(0.0)
    assert result.expanded_nodes == 0


def test_start_outside_map_fails():
    """Confirm a start position outside the map fails with a clear reason."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (-1.0, 2.5), (2.5, 2.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'start_out_of_bounds'


def test_goal_outside_map_fails():
    """Confirm a goal position outside the map fails with a clear reason."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (2.5, 2.5), (10.0, 2.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'goal_out_of_bounds'


def test_start_occupied_fails():
    """Confirm a start position inside an occupied cell fails cleanly."""
    width, height, data = WALL_5X5
    result = plan(width, height, data, (2.5, 2.5), (2.5, 0.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'start_occupied'


def test_goal_occupied_fails():
    """Confirm a goal position inside an occupied cell fails cleanly."""
    width, height, data = WALL_5X5
    result = plan(width, height, data, (2.5, 0.5), (2.5, 2.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'goal_occupied'


def test_start_unknown_fails():
    """Confirm a start position inside an unknown cell fails by default."""
    width, height, data = GATE_5X3
    result = plan(width, height, data, (2.5, 1.5), (0.5, 0.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'start_unknown'


def test_goal_unknown_fails():
    """Confirm a goal position inside an unknown cell fails by default."""
    width, height, data = GATE_5X3
    result = plan(width, height, data, (0.5, 0.5), (2.5, 1.5), DEFAULT_ASTAR_CONFIG)
    assert result.success is False
    assert result.failure_reason == 'goal_unknown'


def test_invalid_data_length_fails():
    """Confirm a map data length mismatch fails cleanly instead of crashing."""
    width, height, data = OPEN_5X5
    truncated_data = data[:-1]
    result = plan_grid_path(
        width, height, 1.0, 0.0, 0.0, truncated_data, (0.5, 0.5), (4.5, 4.5), DEFAULT_ASTAR_CONFIG,
    )
    assert result.success is False
    assert result.failure_reason == 'invalid_map_data_length'


def test_planning_is_deterministic_across_repeated_calls():
    """Confirm identical map/start/goal/config inputs always produce the same result."""
    width, height, data = WALL_5X5
    results = [
        plan(width, height, data, (2.5, 0.5), (2.5, 4.5), DEFAULT_ASTAR_CONFIG)
        for _ in range(5)
    ]
    first = results[0]
    for other in results[1:]:
        assert other.success == first.success
        assert other.grid_path == first.grid_path
        assert other.cost == pytest.approx(first.cost)


def test_maximum_expansion_limit_fails_cleanly():
    """Confirm a very low max_expansions limit stops the search without a crash."""
    width, height, data = OPEN_5X5
    config = AStarConfig(max_expansions=1)
    result = plan(width, height, data, (0.5, 0.5), (4.5, 4.5), config)
    assert result.success is False
    assert result.failure_reason == 'max_expansions_reached'
    assert result.grid_path == ()


# --- Path validity / corner-cut prevention --------------------------------


def test_corner_cut_avoidance_over_a_full_plan():
    """Confirm a full plan around a diagonal obstacle pair never cuts the corner."""
    width, height, data = CORNER_4X4
    config = DEFAULT_ASTAR_CONFIG
    result = plan(width, height, data, (0.5, 0.5), (3.5, 3.5), config)
    assert_valid_path(width, height, 1.0, 0.0, 0.0, data, config, result, (0, 0), (3, 3))
    consecutive_pairs = set(zip(result.grid_path, result.grid_path[1:]))
    assert ((1, 1), (2, 2)) not in consecutive_pairs
    assert ((2, 2), (1, 1)) not in consecutive_pairs


def test_reconstruction_order_is_start_to_goal():
    """Confirm the returned path is ordered from start to goal, not reversed."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (0.5, 0.5), (4.5, 0.5), DEFAULT_ASTAR_CONFIG)
    assert result.grid_path[0] == (0, 0)
    assert result.grid_path[-1] == (4, 0)


# --- World conversion ------------------------------------------------------


def test_cell_centre_conversion_with_zero_origin():
    """Confirm grid cells convert to their world cell-centre coordinates."""
    width, height, _ = OPEN_5X5
    world_points = world_path_from_grid_path(width, height, 1.0, 0.0, 0.0, ((0, 0), (4, 4)))
    assert world_points[0] == pytest.approx((0.5, 0.5))
    assert world_points[1] == pytest.approx((4.5, 4.5))


def test_cell_centre_conversion_with_non_zero_origin():
    """Confirm a non-zero map origin is honored, not assumed to be zero."""
    width, height, _ = OPEN_5X5
    world_points = world_path_from_grid_path(width, height, 1.0, 10.0, 20.0, ((0, 0),))
    assert world_points[0] == pytest.approx((10.5, 20.5))


def test_cell_centre_conversion_with_different_resolution():
    """Confirm a non-unit resolution is applied correctly to cell centres."""
    world_points = world_path_from_grid_path(80, 60, 0.25, 0.0, 0.0, ((2, 3),))
    assert world_points[0] == pytest.approx((0.625, 0.875))


def test_world_conversion_round_trips_through_day9_helpers():
    """Confirm planner geometry round-trips through the Day 9 conversion helpers."""
    geometry = GridGeometry(80, 60, 0.25, 1.0, -2.0)
    original_world = (6.4, 3.1)
    cell = world_to_grid(geometry, *original_world)
    assert cell is not None
    recovered_world = grid_to_world_center(geometry, *cell)
    assert recovered_world[0] == pytest.approx(original_world[0], abs=geometry.resolution_m)
    assert recovered_world[1] == pytest.approx(original_world[1], abs=geometry.resolution_m)


def test_path_world_points_remain_inside_map():
    """Confirm every world point of a planned path stays within the map bounds."""
    width, height, data = OPEN_5X5
    result = plan(width, height, data, (0.5, 0.5), (4.5, 4.5), DEFAULT_ASTAR_CONFIG)
    world_points = world_path_from_grid_path(width, height, 1.0, 0.0, 0.0, result.grid_path)
    for world_x, world_y in world_points:
        assert 0.0 <= world_x <= 5.0
        assert 0.0 <= world_y <= 5.0


# --- ROS-independent orientation helpers ------------------------------------


def test_yaw_for_positive_x_segment_is_zero():
    """Confirm a purely +x segment yields yaw 0."""
    assert yaw_between_points(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0)


def test_yaw_for_positive_y_segment_is_half_pi():
    """Confirm a purely +y segment yields yaw pi/2."""
    assert yaw_between_points(0.0, 0.0, 0.0, 1.0) == pytest.approx(math.pi / 2.0)


def test_yaw_for_negative_x_segment_is_pi():
    """Confirm a purely -x segment yields yaw pi."""
    assert yaw_between_points(0.0, 0.0, -1.0, 0.0) == pytest.approx(math.pi)


def test_yaw_for_diagonal_segment_is_quarter_pi():
    """Confirm a 45-degree segment yields yaw pi/4."""
    assert yaw_between_points(0.0, 0.0, 1.0, 1.0) == pytest.approx(math.pi / 4.0)


def test_yaw_to_quaternion_is_normalized():
    """Confirm the (z, w) pair from yaw_to_quaternion_zw is always unit-length."""
    for yaw in (0.0, math.pi / 4.0, math.pi / 2.0, math.pi, -math.pi / 3.0):
        z, w = yaw_to_quaternion_zw(yaw)
        assert z * z + w * w == pytest.approx(1.0)


def test_compute_path_yaws_final_pose_repeats_previous_segment():
    """Confirm the final pose keeps the heading of the segment that reaches it."""
    yaws = compute_path_yaws([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert yaws[0] == pytest.approx(0.0)
    assert yaws[1] == pytest.approx(math.pi / 2.0)
    assert yaws[2] == pytest.approx(math.pi / 2.0)


def test_compute_path_yaws_single_point_uses_goal_yaw_when_valid():
    """Confirm a single-cell path uses the requested goal yaw when it is finite."""
    assert compute_path_yaws([(1.0, 1.0)], goal_yaw=1.23) == [pytest.approx(1.23)]


def test_compute_path_yaws_single_point_falls_back_to_identity():
    """Confirm a single-cell path falls back to identity when goal_yaw is missing or invalid."""
    assert compute_path_yaws([(1.0, 1.0)], goal_yaw=None) == [0.0]
    assert compute_path_yaws([(1.0, 1.0)], goal_yaw=math.nan) == [0.0]


def test_compute_path_yaws_empty_path_returns_empty_list():
    """Confirm an empty world path produces an empty yaw list."""
    assert compute_path_yaws([]) == []
