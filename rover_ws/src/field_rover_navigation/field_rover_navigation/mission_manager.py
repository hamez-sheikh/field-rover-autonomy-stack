"""Pure Python multi-waypoint mission-sequencing state machine."""

from dataclasses import dataclass
import math

from field_rover_navigation.occupancy_grid import is_measurement_fresh


# Mission status values. A small three-state machine is enough to
# sequence an ordered waypoint list: no complex state machine is needed.
MISSION_IDLE = 'idle'
MISSION_ACTIVE = 'active'
MISSION_COMPLETE = 'complete'


@dataclass(frozen=True)
class Waypoint:
    """One ordered mission goal position, in the mission's map frame."""

    x: float
    y: float

    def __post_init__(self):
        """Reject a waypoint with a non-finite coordinate."""
        if not math.isfinite(self.x) or not math.isfinite(self.y):
            raise ValueError('Waypoint x and y must both be finite.')


def build_waypoints(waypoint_x: tuple, waypoint_y: tuple) -> tuple:
    """Validate parallel coordinate arrays and build the ordered Waypoint tuple."""
    if len(waypoint_x) != len(waypoint_y):
        raise ValueError('waypoint_x and waypoint_y must have equal length.')
    return tuple(Waypoint(x, y) for x, y in zip(waypoint_x, waypoint_y))


@dataclass(frozen=True)
class MissionConfig:
    """Store the ordered waypoint list and mission-sequencing behavior."""

    waypoint_x: tuple
    waypoint_y: tuple
    mission_rate_hz: float = 10.0
    waypoint_tolerance_m: float = 0.25
    localization_timeout_s: float = 0.50
    map_frame: str = 'map'
    auto_start: bool = True

    def __post_init__(self):
        """Reject a configuration that cannot sequence a valid, deterministic mission."""
        if not math.isfinite(self.mission_rate_hz) or self.mission_rate_hz <= 0.0:
            raise ValueError('mission_rate_hz must be positive and finite.')
        if not math.isfinite(self.waypoint_tolerance_m) or self.waypoint_tolerance_m <= 0.0:
            raise ValueError('waypoint_tolerance_m must be positive and finite.')
        if (
            not math.isfinite(self.localization_timeout_s)
            or self.localization_timeout_s <= 0.0
        ):
            raise ValueError('localization_timeout_s must be positive and finite.')
        if not self.map_frame:
            raise ValueError('map_frame must be non-empty.')
        # Raises on a length mismatch or a non-finite coordinate; the
        # resulting tuple is discarded here and rebuilt on demand by the
        # waypoints property below, so validation always matches lookup.
        build_waypoints(self.waypoint_x, self.waypoint_y)

    @property
    def waypoints(self) -> tuple:
        """Return the ordered, validated Waypoint tuple for this mission."""
        return build_waypoints(self.waypoint_x, self.waypoint_y)


DEFAULT_MISSION_CONFIG = MissionConfig(
    waypoint_x=(4.0, 7.0, 7.0),
    waypoint_y=(2.0, 2.0, 5.0),
    mission_rate_hz=10.0,
    waypoint_tolerance_m=0.25,
    localization_timeout_s=0.50,
    map_frame='map',
    auto_start=True,
)


@dataclass(frozen=True)
class MissionState:
    """Snapshot of mission-sequencing progress: waypoints, active index, and status."""

    waypoints: tuple
    index: int
    status: str


def initialize_mission_state(config: MissionConfig) -> MissionState:
    """Build the initial mission state: idle unless auto-starting a non-empty mission."""
    waypoints = config.waypoints
    if waypoints and config.auto_start:
        return MissionState(waypoints=waypoints, index=0, status=MISSION_ACTIVE)
    return MissionState(waypoints=waypoints, index=0, status=MISSION_IDLE)


def active_waypoint(state: MissionState):
    """Return the current target waypoint, or None when no waypoint is active."""
    if state.status != MISSION_ACTIVE:
        return None
    if not (0 <= state.index < len(state.waypoints)):
        return None
    return state.waypoints[state.index]


def distance_to_active_waypoint(state: MissionState, rover_x: float, rover_y: float):
    """Return the Euclidean distance to the active waypoint, or None if unavailable."""
    waypoint = active_waypoint(state)
    if waypoint is None:
        return None
    if not math.isfinite(rover_x) or not math.isfinite(rover_y):
        return None
    return math.hypot(waypoint.x - rover_x, waypoint.y - rover_y)


def is_waypoint_reached(distance_m: float, tolerance_m: float) -> bool:
    """Return whether a measured distance is within the waypoint tolerance (inclusive)."""
    return distance_m <= tolerance_m


def advance_mission(state: MissionState) -> MissionState:
    """Advance at most one waypoint index, or mark the mission complete at the last one."""
    if state.status != MISSION_ACTIVE:
        return state

    next_index = state.index + 1
    if next_index < len(state.waypoints):
        return MissionState(waypoints=state.waypoints, index=next_index, status=MISSION_ACTIVE)
    return MissionState(waypoints=state.waypoints, index=state.index, status=MISSION_COMPLETE)


def is_mission_complete(state: MissionState) -> bool:
    """Return whether the mission has finished sequencing every waypoint."""
    return state.status == MISSION_COMPLETE


def is_localization_usable(
    localization_x,
    localization_y,
    stamp_seconds,
    now_seconds: float,
    timeout_s: float,
) -> bool:
    """Return whether a localization sample is present, finite, and fresh enough to use."""
    if localization_x is None or localization_y is None:
        return False
    if not math.isfinite(localization_x) or not math.isfinite(localization_y):
        return False
    return is_measurement_fresh(stamp_seconds, now_seconds, timeout_s)
