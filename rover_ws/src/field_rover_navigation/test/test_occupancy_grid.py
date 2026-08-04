"""Unit tests for the pure Python bounded-evidence occupancy-grid model."""

import math

from field_rover_navigation.occupancy_grid import (
    apply_free_evidence,
    apply_occupied_evidence,
    bresenham_line,
    build_occupancy_grid_data,
    calculate_beam_endpoint,
    calculate_beam_world_angle,
    clamp_evidence,
    DEFAULT_OCCUPANCY_GRID_CONFIG,
    encode_occupancy,
    FREE_OCCUPANCY,
    grid_index,
    grid_to_world_center,
    is_cell_in_bounds,
    is_measurement_fresh,
    is_new_range_sample,
    is_no_hit_reading,
    is_range_reading_valid,
    OccupancyGridConfig,
    OccupancyGridState,
    OCCUPIED_OCCUPANCY,
    reset_grid_state,
    trace_ray_cells,
    UNKNOWN_OCCUPANCY,
    update_grid_with_beam,
    world_to_grid,
)

import pytest


def _valid_kwargs(**overrides):
    kwargs = {
        'world_width_m': 20.0,
        'world_height_m': 15.0,
        'resolution_m': 0.25,
        'origin_x_m': 0.0,
        'origin_y_m': 0.0,
        'free_evidence_delta': -1,
        'occupied_evidence_delta': 3,
        'minimum_evidence': -5,
        'maximum_evidence': 5,
        'free_threshold': -1,
        'occupied_threshold': 1,
        'frame_id': 'map',
        'map_update_rate_hz': 5.0,
        'localization_timeout_s': 0.5,
        'range_timeout_s': 0.5,
    }
    kwargs.update(overrides)
    return kwargs


SMALL_CONFIG = OccupancyGridConfig(**_valid_kwargs(
    world_width_m=4.0, world_height_m=4.0, resolution_m=1.0,
))


# --- Configuration -----------------------------------------------------


def test_default_configuration_matches_documented_recommended_values():
    """Confirm the shipped default configuration matches recommended values."""
    config = DEFAULT_OCCUPANCY_GRID_CONFIG
    assert config.world_width_m == pytest.approx(20.0)
    assert config.world_height_m == pytest.approx(15.0)
    assert config.resolution_m == pytest.approx(0.25)
    assert config.free_evidence_delta == -1
    assert config.occupied_evidence_delta == 3
    assert config.minimum_evidence == -5
    assert config.maximum_evidence == 5
    assert config.free_threshold == -1
    assert config.occupied_threshold == 1
    assert config.width_cells == 80
    assert config.height_cells == 60
    assert config.total_cells == 4800


def test_valid_configuration_is_accepted():
    """Confirm a fully valid configuration passes validation."""
    OccupancyGridConfig(**_valid_kwargs())


def test_non_positive_width_is_rejected():
    """Confirm a non-positive world width fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(world_width_m=0.0))


def test_infinite_width_is_rejected():
    """Confirm a non-finite world width fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(world_width_m=math.inf))


def test_non_positive_height_is_rejected():
    """Confirm a non-positive world height fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(world_height_m=-1.0))


def test_non_positive_resolution_is_rejected():
    """Confirm a non-positive resolution fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(resolution_m=0.0))


def test_width_not_a_whole_multiple_of_resolution_is_rejected():
    """Confirm a width that does not divide evenly by resolution is rejected."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(world_width_m=20.1, resolution_m=0.25))


def test_height_not_a_whole_multiple_of_resolution_is_rejected():
    """Confirm a height that does not divide evenly by resolution is rejected."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(world_height_m=15.1, resolution_m=0.25))


def test_minimum_evidence_not_below_maximum_is_rejected():
    """Confirm minimum_evidence at or above maximum_evidence fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(minimum_evidence=5, maximum_evidence=5))


def test_non_negative_free_evidence_delta_is_rejected():
    """Confirm a free_evidence_delta that is not negative fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(free_evidence_delta=0))


def test_non_positive_occupied_evidence_delta_is_rejected():
    """Confirm an occupied_evidence_delta that is not positive fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(occupied_evidence_delta=0))


def test_free_threshold_not_below_occupied_threshold_is_rejected():
    """Confirm free_threshold at or above occupied_threshold fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(free_threshold=1, occupied_threshold=1))


def test_free_threshold_outside_evidence_bounds_is_rejected():
    """Confirm a free_threshold outside [minimum, maximum] evidence is rejected."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(free_threshold=-10))


def test_occupied_threshold_outside_evidence_bounds_is_rejected():
    """Confirm an occupied_threshold outside [minimum, maximum] evidence is rejected."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(occupied_threshold=10))


def test_empty_frame_id_is_rejected():
    """Confirm an empty frame_id fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(frame_id=''))


def test_non_positive_map_update_rate_is_rejected():
    """Confirm a non-positive map_update_rate_hz fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(map_update_rate_hz=0.0))


def test_non_positive_localization_timeout_is_rejected():
    """Confirm a non-positive localization_timeout_s fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(localization_timeout_s=0.0))


def test_non_positive_range_timeout_is_rejected():
    """Confirm a non-positive range_timeout_s fails validation."""
    with pytest.raises(ValueError):
        OccupancyGridConfig(**_valid_kwargs(range_timeout_s=-0.1))


# --- Dimensions and indexing --------------------------------------------


def test_width_and_height_cells_are_correct():
    """Confirm width_cells and height_cells match world size / resolution."""
    assert SMALL_CONFIG.width_cells == 4
    assert SMALL_CONFIG.height_cells == 4


def test_total_cells_is_width_times_height():
    """Confirm total_cells equals width_cells multiplied by height_cells."""
    assert SMALL_CONFIG.total_cells == 16


def test_grid_index_matches_row_major_formula():
    """Confirm grid_index follows flat_index = grid_y * width + grid_x."""
    assert grid_index(SMALL_CONFIG, grid_x=2, grid_y=1) == 1 * 4 + 2


def test_world_to_grid_lower_left_origin_maps_to_cell_zero_zero():
    """Confirm the world origin maps to grid cell (0, 0)."""
    assert world_to_grid(SMALL_CONFIG, 0.0, 0.0) == (0, 0)


def test_world_to_grid_interior_conversion():
    """Confirm an interior world position converts to the expected cell."""
    assert world_to_grid(SMALL_CONFIG, 2.5, 1.5) == (2, 1)


def test_world_to_grid_upper_right_interior_cell():
    """Confirm a position just inside the top-right corner maps correctly."""
    assert world_to_grid(SMALL_CONFIG, 3.9, 3.9) == (3, 3)


def test_world_to_grid_negative_coordinates_are_out_of_bounds():
    """Confirm a negative world coordinate is reported as out of bounds."""
    assert world_to_grid(SMALL_CONFIG, -0.1, 1.0) is None
    assert world_to_grid(SMALL_CONFIG, 1.0, -0.1) is None


def test_world_to_grid_exact_maximum_boundary_does_not_crash():
    """Confirm a position exactly on the max boundary resolves to the last cell."""
    assert world_to_grid(SMALL_CONFIG, 4.0, 4.0) == (3, 3)


def test_world_to_grid_beyond_maximum_boundary_is_out_of_bounds():
    """Confirm a position clearly beyond the max boundary is out of bounds."""
    assert world_to_grid(SMALL_CONFIG, 4.5, 1.0) is None
    assert world_to_grid(SMALL_CONFIG, 1.0, 4.5) is None


def test_world_to_grid_near_boundary_float_noise_resolves_to_last_cell():
    """Confirm a boundary value with tiny float noise still resolves cleanly."""
    just_inside = 4.0 - 1e-12
    assert world_to_grid(SMALL_CONFIG, just_inside, just_inside) == (3, 3)


def test_grid_to_world_center_returns_cell_midpoint():
    """Confirm grid_to_world_center returns the geometric centre of a cell."""
    world_x, world_y = grid_to_world_center(SMALL_CONFIG, 0, 0)
    assert world_x == pytest.approx(0.5)
    assert world_y == pytest.approx(0.5)


def test_is_cell_in_bounds_accepts_interior_and_edge_cells():
    """Confirm is_cell_in_bounds accepts every valid cell index."""
    assert is_cell_in_bounds(SMALL_CONFIG, 0, 0)
    assert is_cell_in_bounds(SMALL_CONFIG, 3, 3)


def test_is_cell_in_bounds_rejects_negative_and_overflowing_indices():
    """Confirm is_cell_in_bounds rejects indices outside the grid."""
    assert not is_cell_in_bounds(SMALL_CONFIG, -1, 0)
    assert not is_cell_in_bounds(SMALL_CONFIG, 0, -1)
    assert not is_cell_in_bounds(SMALL_CONFIG, 4, 0)
    assert not is_cell_in_bounds(SMALL_CONFIG, 0, 4)


# --- Ray traversal -------------------------------------------------------


def test_bresenham_line_same_start_and_end_returns_single_cell():
    """Confirm a degenerate line with equal start and end returns one cell."""
    assert bresenham_line(2, 2, 2, 2) == [(2, 2)]


def test_bresenham_line_horizontal_forward():
    """Confirm a forward horizontal line visits every cell in order."""
    assert bresenham_line(0, 0, 3, 0) == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_bresenham_line_horizontal_reverse():
    """Confirm a reverse horizontal line visits every cell in order."""
    assert bresenham_line(3, 0, 0, 0) == [(3, 0), (2, 0), (1, 0), (0, 0)]


def test_bresenham_line_vertical_forward():
    """Confirm a forward vertical line visits every cell in order."""
    assert bresenham_line(0, 0, 0, 3) == [(0, 0), (0, 1), (0, 2), (0, 3)]


def test_bresenham_line_vertical_reverse():
    """Confirm a reverse vertical line visits every cell in order."""
    assert bresenham_line(0, 3, 0, 0) == [(0, 3), (0, 2), (0, 1), (0, 0)]


def test_bresenham_line_diagonal():
    """Confirm a 45-degree diagonal line visits the expected cells."""
    assert bresenham_line(0, 0, 3, 3) == [(0, 0), (1, 1), (2, 2), (3, 3)]


def test_bresenham_line_steep_line():
    """Confirm a steep (mostly-vertical) line stays connected without gaps."""
    cells = bresenham_line(0, 0, 1, 4)
    assert cells[0] == (0, 0)
    assert cells[-1] == (1, 4)
    assert len(cells) == 5


def test_bresenham_line_has_no_duplicate_cells():
    """Confirm no cell is visited twice along a line."""
    cells = bresenham_line(0, 0, 5, 3)
    assert len(cells) == len(set(cells))


def test_bresenham_line_preserves_stable_start_to_end_order():
    """Confirm the returned cells are ordered from start to end."""
    cells = bresenham_line(0, 0, 5, 3)
    assert cells[0] == (0, 0)
    assert cells[-1] == (5, 3)


def test_trace_ray_cells_partially_out_of_bounds_line_is_truncated():
    """Confirm a line that exits the grid keeps only the in-bounds portion."""
    cells = trace_ray_cells(SMALL_CONFIG, (2, 2), (6, 2))
    assert cells == [(2, 2), (3, 2)]


def test_trace_ray_cells_completely_out_of_bounds_line_is_handled_safely():
    """Confirm a line entirely outside the grid returns no cells, without error."""
    cells = trace_ray_cells(SMALL_CONFIG, (10, 10), (12, 12))
    assert cells == []


# --- Evidence updates ------------------------------------------------------


def test_new_grid_state_starts_at_zero_evidence_everywhere():
    """Confirm a freshly created grid state has all-zero (unknown) evidence."""
    state = OccupancyGridState(SMALL_CONFIG)
    assert state.evidence == [0] * SMALL_CONFIG.total_cells


def test_clamp_evidence_passes_through_in_range_values():
    """Confirm clamp_evidence leaves an in-range value unchanged."""
    assert clamp_evidence(SMALL_CONFIG, 2) == 2


def test_clamp_evidence_clamps_at_minimum():
    """Confirm clamp_evidence clamps a too-low value at minimum_evidence."""
    assert clamp_evidence(SMALL_CONFIG, -100) == SMALL_CONFIG.minimum_evidence


def test_clamp_evidence_clamps_at_maximum():
    """Confirm clamp_evidence clamps a too-high value at maximum_evidence."""
    assert clamp_evidence(SMALL_CONFIG, 100) == SMALL_CONFIG.maximum_evidence


def test_apply_free_evidence_decreases_evidence():
    """Confirm a free-evidence update decreases a cell's evidence."""
    state = OccupancyGridState(SMALL_CONFIG)
    apply_free_evidence(state, 1, 1)
    assert state.evidence[grid_index(SMALL_CONFIG, 1, 1)] == -1


def test_apply_occupied_evidence_increases_evidence():
    """Confirm an occupied-evidence update increases a cell's evidence."""
    state = OccupancyGridState(SMALL_CONFIG)
    apply_occupied_evidence(state, 1, 1)
    assert state.evidence[grid_index(SMALL_CONFIG, 1, 1)] == 3


def test_repeated_free_updates_clamp_at_minimum_evidence():
    """Confirm many free updates in a row clamp instead of running away."""
    state = OccupancyGridState(SMALL_CONFIG)
    for _ in range(20):
        apply_free_evidence(state, 0, 0)
    assert state.evidence[grid_index(SMALL_CONFIG, 0, 0)] == SMALL_CONFIG.minimum_evidence


def test_repeated_occupied_updates_clamp_at_maximum_evidence():
    """Confirm many occupied updates in a row clamp instead of running away."""
    state = OccupancyGridState(SMALL_CONFIG)
    for _ in range(20):
        apply_occupied_evidence(state, 0, 0)
    assert state.evidence[grid_index(SMALL_CONFIG, 0, 0)] == SMALL_CONFIG.maximum_evidence


def test_repeated_free_observations_overcome_weak_occupied_evidence():
    """Confirm enough free observations can flip a weakly-occupied cell to free."""
    state = OccupancyGridState(SMALL_CONFIG)
    apply_occupied_evidence(state, 0, 0)  # evidence = 3
    apply_free_evidence(state, 0, 0)  # evidence = 2
    apply_free_evidence(state, 0, 0)  # evidence = 1
    apply_free_evidence(state, 0, 0)  # evidence = 0
    apply_free_evidence(state, 0, 0)  # evidence = -1
    assert state.evidence[grid_index(SMALL_CONFIG, 0, 0)] < 0


def test_repeated_occupied_observations_overcome_weak_free_evidence():
    """Confirm enough occupied observations can flip a weakly-free cell to occupied."""
    state = OccupancyGridState(SMALL_CONFIG)
    apply_free_evidence(state, 0, 0)  # evidence = -1
    apply_occupied_evidence(state, 0, 0)  # evidence = 2
    assert state.evidence[grid_index(SMALL_CONFIG, 0, 0)] > 0


def test_reset_grid_state_clears_all_evidence():
    """Confirm reset_grid_state returns every cell to zero evidence."""
    state = OccupancyGridState(SMALL_CONFIG)
    apply_occupied_evidence(state, 0, 0)
    apply_free_evidence(state, 1, 1)
    reset_grid_state(state)
    assert state.evidence == [0] * SMALL_CONFIG.total_cells


def test_update_grid_with_beam_never_marks_rover_cell_occupied():
    """Confirm a very short hit never marks the rover's own cell occupied."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=2.0, rover_y=2.0, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=0.1,
        min_range=0.1, max_range=8.0,
    )
    rover_index = grid_index(SMALL_CONFIG, *world_to_grid(SMALL_CONFIG, 2.0, 2.0))
    assert state.evidence[rover_index] <= 0


def test_update_grid_with_beam_marks_intermediate_cells_free():
    """Confirm a hit marks the cells between rover and endpoint as free."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=0.5, rover_y=0.5, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=2.5,
        min_range=0.1, max_range=8.0,
    )
    intermediate_index = grid_index(SMALL_CONFIG, 1, 0)
    assert state.evidence[intermediate_index] < 0


def test_update_grid_with_beam_marks_hit_endpoint_occupied():
    """Confirm a hit below max_range marks the endpoint cell occupied."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=0.5, rover_y=0.5, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=2.5,
        min_range=0.1, max_range=8.0,
    )
    endpoint_index = grid_index(SMALL_CONFIG, 3, 0)
    assert state.evidence[endpoint_index] > 0


def test_update_grid_with_beam_no_hit_endpoint_is_not_occupied():
    """Confirm a max-range (no-hit) reading never marks an endpoint occupied."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=0.5, rover_y=0.5, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=8.0,
        min_range=0.1, max_range=8.0,
    )
    assert all(value <= 0 for value in state.evidence)


# --- Occupancy encoding ------------------------------------------------


def test_unknown_evidence_encodes_to_negative_one():
    """Confirm zero (unknown) evidence encodes to -1."""
    assert encode_occupancy(SMALL_CONFIG, 0) == UNKNOWN_OCCUPANCY


def test_free_evidence_encodes_to_zero():
    """Confirm evidence at or below free_threshold encodes to 0."""
    assert encode_occupancy(SMALL_CONFIG, SMALL_CONFIG.free_threshold) == FREE_OCCUPANCY


def test_occupied_evidence_encodes_to_one_hundred():
    """Confirm evidence at or above occupied_threshold encodes to 100."""
    assert (
        encode_occupancy(SMALL_CONFIG, SMALL_CONFIG.occupied_threshold)
        == OCCUPIED_OCCUPANCY
    )


def test_ambiguous_evidence_between_thresholds_remains_unknown():
    """Confirm evidence strictly between the thresholds stays unknown."""
    ambiguous_config = OccupancyGridConfig(**_valid_kwargs(
        free_threshold=-3, occupied_threshold=3,
    ))
    assert encode_occupancy(ambiguous_config, 0) == UNKNOWN_OCCUPANCY


def test_build_occupancy_grid_data_length_matches_total_cells():
    """Confirm the encoded data array has one entry per grid cell."""
    state = OccupancyGridState(SMALL_CONFIG)
    data = build_occupancy_grid_data(state)
    assert len(data) == SMALL_CONFIG.total_cells


def test_build_occupancy_grid_data_only_contains_valid_values():
    """Confirm every encoded value is one of -1, 0, or 100."""
    state = OccupancyGridState(SMALL_CONFIG)
    apply_occupied_evidence(state, 0, 0)
    apply_free_evidence(state, 1, 1)
    data = build_occupancy_grid_data(state)
    assert set(data) <= {UNKNOWN_OCCUPANCY, FREE_OCCUPANCY, OCCUPIED_OCCUPANCY}


# --- Beam geometry -------------------------------------------------------


def test_front_beam_at_zero_yaw_points_positive_x():
    """Confirm a zero-angle beam at zero yaw points along positive x."""
    world_angle = calculate_beam_world_angle(rover_yaw=0.0, beam_relative_angle=0.0)
    endpoint_x, endpoint_y = calculate_beam_endpoint(0.0, 0.0, world_angle, 1.0)
    assert endpoint_x == pytest.approx(1.0)
    assert endpoint_y == pytest.approx(0.0, abs=1e-9)


def test_positive_relative_angle_points_counter_clockwise():
    """Confirm a positive relative angle rotates the beam toward +y."""
    world_angle = calculate_beam_world_angle(
        rover_yaw=0.0, beam_relative_angle=math.pi / 2.0,
    )
    endpoint_x, endpoint_y = calculate_beam_endpoint(0.0, 0.0, world_angle, 1.0)
    assert endpoint_x == pytest.approx(0.0, abs=1e-9)
    assert endpoint_y == pytest.approx(1.0)


def test_rover_yaw_rotates_beam_direction():
    """Confirm a quarter-turn rover yaw carries a zero-angle beam to face +y."""
    world_angle = calculate_beam_world_angle(
        rover_yaw=math.pi / 2.0, beam_relative_angle=0.0,
    )
    endpoint_x, endpoint_y = calculate_beam_endpoint(0.0, 0.0, world_angle, 1.0)
    assert endpoint_x == pytest.approx(0.0, abs=1e-9)
    assert endpoint_y == pytest.approx(1.0)


def test_endpoint_for_known_pose_and_range_is_correct():
    """Confirm the endpoint calculation matches simple right-triangle geometry."""
    world_angle = calculate_beam_world_angle(rover_yaw=0.0, beam_relative_angle=0.0)
    endpoint_x, endpoint_y = calculate_beam_endpoint(2.0, 3.0, world_angle, 4.0)
    assert endpoint_x == pytest.approx(6.0)
    assert endpoint_y == pytest.approx(3.0)


def test_valid_hit_updates_expected_free_and_occupied_cells():
    """Confirm a full beam update marks both free and occupied cells correctly."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=0.5, rover_y=0.5, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=2.5,
        min_range=0.1, max_range=8.0,
    )
    rover_index = grid_index(SMALL_CONFIG, 0, 0)
    endpoint_index = grid_index(SMALL_CONFIG, 3, 0)
    assert state.evidence[rover_index] < 0
    assert state.evidence[endpoint_index] > 0


def test_maximum_range_reading_creates_only_free_space():
    """Confirm a max-range reading never introduces occupied evidence anywhere."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=0.5, rover_y=0.5, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=SMALL_CONFIG.width_cells * SMALL_CONFIG.resolution_m,
        min_range=0.1, max_range=SMALL_CONFIG.width_cells * SMALL_CONFIG.resolution_m,
    )
    assert all(value <= 0 for value in state.evidence)


def test_out_of_map_endpoint_does_not_crash():
    """Confirm a beam whose endpoint lies outside the map updates safely."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=1.0, rover_y=2.0, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=5.0,
        min_range=0.1, max_range=8.0,
    )
    # The endpoint (6.0, 2.0) lies outside the 4x4 world, so only the
    # in-map portion of the traced ray (cells (1,2), (2,2), (3,2)) is
    # marked free, and no occupied cell is created anywhere.
    for grid_x in (1, 2, 3):
        assert state.evidence[grid_index(SMALL_CONFIG, grid_x, 2)] < 0
    assert all(-5 <= value <= 5 for value in state.evidence)
    assert all(value <= 0 for value in state.evidence)


def test_exact_wall_endpoint_follows_boundary_convention():
    """Confirm a hit exactly at the world's max edge lands in the last cell."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=0.5, rover_y=2.0, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=3.5,
        min_range=0.1, max_range=8.0,
    )
    endpoint_index = grid_index(SMALL_CONFIG, 3, 2)
    assert state.evidence[endpoint_index] > 0


def test_minimum_range_endpoint_in_rover_cell_is_safe():
    """Confirm a minimum-range hit landing in the rover's own cell stays free."""
    state = OccupancyGridState(SMALL_CONFIG)
    update_grid_with_beam(
        state,
        rover_x=2.1, rover_y=2.1, rover_yaw=0.0,
        beam_relative_angle=0.0,
        measured_range=0.1,
        min_range=0.1, max_range=8.0,
    )
    rover_index = grid_index(SMALL_CONFIG, 2, 2)
    assert state.evidence[rover_index] <= 0


def test_invalid_range_values_do_not_change_the_map():
    """Confirm NaN, infinite, and out-of-range readings leave the grid untouched."""
    state = OccupancyGridState(SMALL_CONFIG)
    for bad_range in (math.nan, math.inf, -math.inf, -1.0, 100.0):
        update_grid_with_beam(
            state,
            rover_x=2.0, rover_y=2.0, rover_yaw=0.0,
            beam_relative_angle=0.0,
            measured_range=bad_range,
            min_range=0.1, max_range=8.0,
        )
    assert state.evidence == [0] * SMALL_CONFIG.total_cells


def test_is_range_reading_valid_accepts_in_range_values():
    """Confirm a plain in-range reading is accepted."""
    assert is_range_reading_valid(2.0, min_range=0.1, max_range=8.0)


def test_is_range_reading_valid_rejects_nan():
    """Confirm a NaN reading is rejected."""
    assert not is_range_reading_valid(math.nan, min_range=0.1, max_range=8.0)


def test_is_range_reading_valid_rejects_infinity():
    """Confirm an infinite reading is rejected."""
    assert not is_range_reading_valid(math.inf, min_range=0.1, max_range=8.0)


def test_is_range_reading_valid_rejects_below_minimum():
    """Confirm a reading well below min_range is rejected."""
    assert not is_range_reading_valid(0.0, min_range=0.1, max_range=8.0)


def test_is_range_reading_valid_rejects_above_maximum():
    """Confirm a reading well above max_range is rejected."""
    assert not is_range_reading_valid(9.0, min_range=0.1, max_range=8.0)


def test_is_no_hit_reading_true_at_max_range():
    """Confirm a reading equal to max_range is treated as no detection."""
    assert is_no_hit_reading(8.0, max_range=8.0)


def test_is_no_hit_reading_false_below_max_range():
    """Confirm a reading below max_range is treated as a real detection."""
    assert not is_no_hit_reading(7.5, max_range=8.0)


# --- Duplicate and stale handling helpers -------------------------------


def test_is_measurement_fresh_true_for_recent_stamp():
    """Confirm a recently stamped measurement is reported fresh."""
    assert is_measurement_fresh(10.0, now_seconds=10.2, timeout_s=0.5)


def test_is_measurement_fresh_false_for_stale_stamp():
    """Confirm an old stamp beyond the timeout is reported stale."""
    assert not is_measurement_fresh(10.0, now_seconds=11.0, timeout_s=0.5)


def test_is_measurement_fresh_false_when_missing():
    """Confirm a missing (None) measurement is never fresh."""
    assert not is_measurement_fresh(None, now_seconds=10.0, timeout_s=0.5)


def test_is_measurement_fresh_false_for_future_stamp():
    """Confirm a stamp ahead of now (clock skew) is treated as untrustworthy."""
    assert not is_measurement_fresh(11.0, now_seconds=10.0, timeout_s=0.5)


def test_is_new_range_sample_true_for_first_sample():
    """Confirm the first-ever sample for a beam counts as new."""
    assert is_new_range_sample(5.0, last_processed_stamp_seconds=None)


def test_is_new_range_sample_false_for_duplicate_timestamp():
    """Confirm a sample with the same stamp as the last processed one is rejected."""
    assert not is_new_range_sample(5.0, last_processed_stamp_seconds=5.0)


def test_is_new_range_sample_false_for_older_timestamp():
    """Confirm a sample older than the last processed one is rejected."""
    assert not is_new_range_sample(4.0, last_processed_stamp_seconds=5.0)


def test_is_new_range_sample_true_for_newer_timestamp():
    """Confirm a sample newer than the last processed one is accepted."""
    assert is_new_range_sample(6.0, last_processed_stamp_seconds=5.0)


def test_is_new_range_sample_handles_zero_timestamps():
    """Confirm a zero timestamp is treated as new only the first time."""
    assert is_new_range_sample(0.0, last_processed_stamp_seconds=None)
    assert not is_new_range_sample(0.0, last_processed_stamp_seconds=0.0)


def test_one_beam_can_update_independently_of_others():
    """Confirm one beam's freshness/newness does not depend on another beam's."""
    front_last_processed = 5.0
    left_last_processed = None
    assert not is_new_range_sample(5.0, front_last_processed)
    assert is_new_range_sample(5.0, left_last_processed)
