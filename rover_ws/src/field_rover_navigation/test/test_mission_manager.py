"""Unit tests for the pure Python multi-waypoint mission-sequencing state machine."""

import math

from field_rover_navigation.mission_manager import (
    active_waypoint,
    advance_mission,
    build_waypoints,
    distance_to_active_waypoint,
    initialize_mission_state,
    is_localization_usable,
    is_mission_complete,
    is_waypoint_reached,
    MISSION_ACTIVE,
    MISSION_COMPLETE,
    MISSION_IDLE,
    MissionConfig,
    MissionState,
    Waypoint,
)
import pytest


def make_config(waypoint_x=(4.0, 7.0, 7.0), waypoint_y=(2.0, 2.0, 5.0), **overrides):
    """Build a MissionConfig from the sample three-waypoint mission, with overrides."""
    return MissionConfig(waypoint_x=waypoint_x, waypoint_y=waypoint_y, **overrides)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def test_valid_configuration_is_accepted():
    """Confirm a well-formed multi-waypoint configuration builds without error."""
    config = make_config()
    assert config.waypoints == (Waypoint(4.0, 2.0), Waypoint(7.0, 2.0), Waypoint(7.0, 5.0))


def test_non_positive_mission_rate_is_rejected():
    """Confirm a zero or negative mission_rate_hz is rejected."""
    with pytest.raises(ValueError):
        make_config(mission_rate_hz=0.0)
    with pytest.raises(ValueError):
        make_config(mission_rate_hz=-1.0)


def test_non_finite_mission_rate_is_rejected():
    """Confirm a NaN or infinite mission_rate_hz is rejected."""
    with pytest.raises(ValueError):
        make_config(mission_rate_hz=math.nan)
    with pytest.raises(ValueError):
        make_config(mission_rate_hz=math.inf)


def test_non_positive_waypoint_tolerance_is_rejected():
    """Confirm a zero or negative waypoint_tolerance_m is rejected."""
    with pytest.raises(ValueError):
        make_config(waypoint_tolerance_m=0.0)
    with pytest.raises(ValueError):
        make_config(waypoint_tolerance_m=-0.25)


def test_non_positive_localization_timeout_is_rejected():
    """Confirm a zero or negative localization_timeout_s is rejected."""
    with pytest.raises(ValueError):
        make_config(localization_timeout_s=0.0)
    with pytest.raises(ValueError):
        make_config(localization_timeout_s=-0.5)


def test_empty_map_frame_is_rejected():
    """Confirm an empty map_frame string is rejected."""
    with pytest.raises(ValueError):
        make_config(map_frame='')


def test_mismatched_waypoint_arrays_are_rejected():
    """Confirm waypoint_x and waypoint_y of different lengths are rejected."""
    with pytest.raises(ValueError):
        make_config(waypoint_x=(1.0, 2.0), waypoint_y=(1.0,))


def test_non_finite_waypoint_is_rejected():
    """Confirm a NaN or infinite waypoint coordinate is rejected."""
    with pytest.raises(ValueError):
        make_config(waypoint_x=(1.0, math.nan), waypoint_y=(1.0, 2.0))
    with pytest.raises(ValueError):
        make_config(waypoint_x=(1.0, 2.0), waypoint_y=(1.0, math.inf))


def test_empty_waypoint_list_is_a_valid_configuration():
    """Confirm an empty mission (equal, zero-length arrays) is valid, not an error."""
    config = make_config(waypoint_x=(), waypoint_y=())
    assert config.waypoints == ()


def test_build_waypoints_matches_config_validation():
    """Confirm build_waypoints applies the same length and finiteness rules directly."""
    with pytest.raises(ValueError):
        build_waypoints((1.0,), (1.0, 2.0))
    assert build_waypoints((1.0, 2.0), (3.0, 4.0)) == (Waypoint(1.0, 3.0), Waypoint(2.0, 4.0))


# ---------------------------------------------------------------------------
# Mission initialization
# ---------------------------------------------------------------------------

def test_empty_mission_remains_idle():
    """Confirm an empty mission initializes as idle, not active or complete."""
    state = initialize_mission_state(make_config(waypoint_x=(), waypoint_y=()))
    assert state.status == MISSION_IDLE
    assert state.index == 0
    assert state.waypoints == ()


def test_single_waypoint_mission_becomes_active():
    """Confirm a one-waypoint mission initializes as active at index zero."""
    state = initialize_mission_state(make_config(waypoint_x=(5.0,), waypoint_y=(1.0,)))
    assert state.status == MISSION_ACTIVE
    assert state.index == 0
    assert len(state.waypoints) == 1


def test_multi_waypoint_mission_becomes_active():
    """Confirm a multi-waypoint mission initializes as active with all waypoints stored."""
    state = initialize_mission_state(make_config())
    assert state.status == MISSION_ACTIVE
    assert len(state.waypoints) == 3


def test_initial_index_is_zero():
    """Confirm the initial active index is always zero for a non-empty mission."""
    state = initialize_mission_state(make_config())
    assert state.index == 0


def test_fresh_mission_is_not_marked_complete():
    """Confirm a freshly initialized non-empty mission is never immediately complete."""
    state = initialize_mission_state(make_config())
    assert not is_mission_complete(state)


def test_auto_start_false_keeps_a_non_empty_mission_idle():
    """Confirm auto_start=False keeps even a non-empty mission from starting."""
    state = initialize_mission_state(make_config(auto_start=False))
    assert state.status == MISSION_IDLE
    assert len(state.waypoints) == 3


# ---------------------------------------------------------------------------
# Active waypoint
# ---------------------------------------------------------------------------

def test_active_waypoint_returns_first_waypoint():
    """Confirm the active waypoint at index zero is the first waypoint."""
    state = initialize_mission_state(make_config())
    assert active_waypoint(state) == Waypoint(4.0, 2.0)


def test_active_waypoint_returns_middle_waypoint():
    """Confirm the active waypoint at a middle index is the correct waypoint."""
    state = MissionState(waypoints=make_config().waypoints, index=1, status=MISSION_ACTIVE)
    assert active_waypoint(state) == Waypoint(7.0, 2.0)


def test_active_waypoint_returns_final_waypoint():
    """Confirm the active waypoint at the final index is the last waypoint."""
    state = MissionState(waypoints=make_config().waypoints, index=2, status=MISSION_ACTIVE)
    assert active_waypoint(state) == Waypoint(7.0, 5.0)


def test_complete_mission_has_no_active_waypoint():
    """Confirm a completed mission reports no active waypoint."""
    state = MissionState(waypoints=make_config().waypoints, index=2, status=MISSION_COMPLETE)
    assert active_waypoint(state) is None


def test_invalid_index_is_handled_safely():
    """Confirm an out-of-range index returns None instead of raising."""
    state = MissionState(waypoints=make_config().waypoints, index=5, status=MISSION_ACTIVE)
    assert active_waypoint(state) is None


# ---------------------------------------------------------------------------
# Distance and completion
# ---------------------------------------------------------------------------

def test_distance_outside_tolerance_is_not_reached():
    """Confirm a distance greater than the tolerance is not reached."""
    assert not is_waypoint_reached(distance_m=1.0, tolerance_m=0.25)


def test_distance_inside_tolerance_is_reached():
    """Confirm a distance smaller than the tolerance is reached."""
    assert is_waypoint_reached(distance_m=0.1, tolerance_m=0.25)


def test_distance_exact_tolerance_boundary_is_reached():
    """Confirm a distance exactly equal to the tolerance counts as reached (inclusive)."""
    assert is_waypoint_reached(distance_m=0.25, tolerance_m=0.25)


def test_distance_to_active_waypoint_matches_euclidean_formula():
    """Confirm distance_to_active_waypoint computes plain Euclidean distance."""
    state = initialize_mission_state(make_config(waypoint_x=(3.0,), waypoint_y=(4.0,)))
    assert distance_to_active_waypoint(state, rover_x=0.0, rover_y=0.0) == pytest.approx(5.0)


def test_non_finite_rover_position_is_rejected():
    """Confirm a non-finite rover position yields no distance."""
    state = initialize_mission_state(make_config())
    assert distance_to_active_waypoint(state, rover_x=math.nan, rover_y=2.0) is None
    assert distance_to_active_waypoint(state, rover_x=4.0, rover_y=math.inf) is None


def test_distance_with_no_active_waypoint_is_none():
    """Confirm distance lookup on an idle or complete mission returns None."""
    idle_state = initialize_mission_state(make_config(waypoint_x=(), waypoint_y=()))
    assert distance_to_active_waypoint(idle_state, rover_x=0.0, rover_y=0.0) is None


# ---------------------------------------------------------------------------
# Progression
# ---------------------------------------------------------------------------

def test_first_waypoint_completion_advances_to_second():
    """Confirm completing waypoint 0 advances the active index to 1."""
    state = initialize_mission_state(make_config())
    next_state = advance_mission(state)
    assert next_state.index == 1
    assert next_state.status == MISSION_ACTIVE


def test_middle_waypoint_completion_advances_once():
    """Confirm completing a middle waypoint advances the index by exactly one."""
    state = MissionState(waypoints=make_config().waypoints, index=1, status=MISSION_ACTIVE)
    next_state = advance_mission(state)
    assert next_state.index == 2
    assert next_state.status == MISSION_ACTIVE


def test_final_waypoint_completion_marks_mission_complete():
    """Confirm completing the final waypoint marks the mission complete, index unchanged."""
    state = MissionState(waypoints=make_config().waypoints, index=2, status=MISSION_ACTIVE)
    next_state = advance_mission(state)
    assert next_state.status == MISSION_COMPLETE
    assert next_state.index == 2


def test_index_never_decreases_across_repeated_advancement():
    """Confirm advancing repeatedly, including past completion, never decreases the index."""
    state = initialize_mission_state(make_config())
    indices = [state.index]
    for _ in range(6):
        state = advance_mission(state)
        indices.append(state.index)
    assert indices == sorted(indices)


def test_advance_moves_at_most_one_index():
    """Confirm a single advance_mission call never skips more than one waypoint."""
    state = initialize_mission_state(make_config())
    next_state = advance_mission(state)
    assert next_state.index == state.index + 1


def test_repeated_evaluation_outside_tolerance_does_not_advance():
    """Confirm repeatedly checking a too-far distance never signals completion."""
    state = initialize_mission_state(make_config())
    for _ in range(5):
        distance = distance_to_active_waypoint(state, rover_x=0.0, rover_y=0.0)
        assert not is_waypoint_reached(distance, tolerance_m=0.25)
    assert state.index == 0
    assert state.status == MISSION_ACTIVE


def test_repeated_evaluation_after_completion_does_not_advance():
    """Confirm advance_mission on an already-complete state is a no-op."""
    complete_state = MissionState(
        waypoints=make_config().waypoints, index=2, status=MISSION_COMPLETE,
    )
    for _ in range(3):
        complete_state = advance_mission(complete_state)
        assert complete_state.status == MISSION_COMPLETE
        assert complete_state.index == 2


# ---------------------------------------------------------------------------
# Closely spaced waypoints
# ---------------------------------------------------------------------------

def test_rover_within_tolerance_of_multiple_waypoints_advances_only_one():
    """Confirm being within tolerance of several waypoints still advances just one."""
    config = make_config(waypoint_x=(1.0, 1.05, 1.10), waypoint_y=(1.0, 1.0, 1.0))
    state = initialize_mission_state(config)

    # The rover sits within tolerance of all three closely spaced waypoints.
    distance = distance_to_active_waypoint(state, rover_x=1.02, rover_y=1.0)
    assert is_waypoint_reached(distance, config.waypoint_tolerance_m)

    next_state = advance_mission(state)
    assert next_state.index == 1
    assert next_state.status == MISSION_ACTIVE


def test_next_update_can_advance_again_if_still_within_tolerance():
    """Confirm a second, separate update can advance again when still in tolerance."""
    config = make_config(waypoint_x=(1.0, 1.05, 1.10), waypoint_y=(1.0, 1.0, 1.0))
    state = initialize_mission_state(config)

    state = advance_mission(state)
    assert state.index == 1

    distance = distance_to_active_waypoint(state, rover_x=1.02, rover_y=1.0)
    assert is_waypoint_reached(distance, config.waypoint_tolerance_m)
    state = advance_mission(state)
    assert state.index == 2


# ---------------------------------------------------------------------------
# Stale-input helpers (pure)
# ---------------------------------------------------------------------------

def test_is_localization_usable_true_for_fresh_localization():
    """Confirm a recently stamped, finite localization sample is usable."""
    assert is_localization_usable(1.0, 2.0, 10.0, now_seconds=10.1, timeout_s=0.5)


def test_is_localization_usable_false_for_stale_localization():
    """Confirm a localization sample older than the timeout is not usable."""
    assert not is_localization_usable(1.0, 2.0, 10.0, now_seconds=10.6, timeout_s=0.5)


def test_is_localization_usable_false_when_missing():
    """Confirm a never-received localization sample is not usable."""
    assert not is_localization_usable(None, None, None, now_seconds=10.0, timeout_s=0.5)


def test_is_localization_usable_false_for_future_timestamp():
    """Confirm a localization sample stamped after 'now' is not usable."""
    assert not is_localization_usable(1.0, 2.0, 10.5, now_seconds=10.0, timeout_s=0.5)


def test_is_localization_usable_true_at_exact_timeout_boundary():
    """Confirm an age exactly equal to the timeout still counts as usable (inclusive)."""
    assert is_localization_usable(1.0, 2.0, 10.0, now_seconds=10.5, timeout_s=0.5)


def test_is_localization_usable_false_for_non_finite_coordinates():
    """Confirm a non-finite localization position is never usable, even if fresh."""
    assert not is_localization_usable(math.nan, 2.0, 10.0, now_seconds=10.1, timeout_s=0.5)
    assert not is_localization_usable(1.0, math.inf, 10.0, now_seconds=10.1, timeout_s=0.5)


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------

def test_advance_mission_is_deterministic():
    """Confirm advancing two equal mission states produces two equal results."""
    state_a = initialize_mission_state(make_config())
    state_b = initialize_mission_state(make_config())
    assert advance_mission(state_a) == advance_mission(state_b)


def test_distance_to_active_waypoint_is_deterministic():
    """Confirm identical state and rover position always yield the same distance."""
    state = initialize_mission_state(make_config())
    first = distance_to_active_waypoint(state, rover_x=1.0, rover_y=1.0)
    second = distance_to_active_waypoint(state, rover_x=1.0, rover_y=1.0)
    assert first == second


def test_is_localization_usable_is_deterministic():
    """Confirm identical inputs to is_localization_usable always yield the same result."""
    first = is_localization_usable(1.0, 2.0, 10.0, now_seconds=10.2, timeout_s=0.5)
    second = is_localization_usable(1.0, 2.0, 10.0, now_seconds=10.2, timeout_s=0.5)
    assert first == second
