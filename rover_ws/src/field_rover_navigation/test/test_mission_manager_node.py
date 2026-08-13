"""Unit tests for the mission manager node's ROS wiring, gating, and publishing."""

# The mission-sequencing mathematics itself is exercised through the pure
# mission_manager module tests. These checks focus on properties that only
# exist at the node level: topic wiring, one-goal-at-a-time publishing, and
# the localization-freshness gating that keeps the mission from advancing
# on missing or stale input.

from field_rover_navigation.mission_manager import (
    initialize_mission_state,
    MISSION_COMPLETE,
    MISSION_IDLE,
    MissionConfig,
)
from field_rover_navigation.mission_manager_node import MissionManagerNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    created_node = MissionManagerNode()
    yield created_node
    created_node.destroy_node()


def _make_localization(x=0.0, y=0.0, sec=10, nanosec=0):
    message = Odometry()
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    return message


def _capture_published_goals(node):
    """Replace the goal publisher's publish() with one that records messages."""
    captured = []
    node._goal_publisher.publish = lambda message: captured.append(message)
    return captured


def _make_empty_mission_node(node):
    """Reconfigure a constructed node in place to run an empty mission."""
    node._config = MissionConfig(waypoint_x=(), waypoint_y=())
    node._state = initialize_mission_state(node._config)


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------

def test_node_subscribes_to_localization_odom(node):
    """Confirm the node subscribes to /localization/odom."""
    assert node._odom_subscription.topic_name == '/localization/odom'


def test_node_publishes_goal_pose_topic(node):
    """Confirm the node publishes geometry_msgs/PoseStamped on /goal_pose."""
    assert node._goal_publisher.topic_name == '/goal_pose'
    assert node._goal_publisher.msg_type is PoseStamped


def test_node_does_not_subscribe_to_ground_truth(node):
    """Confirm no subscription to /ground_truth/odom exists anywhere on the node."""
    topics = [subscription.topic_name for subscription in node.subscriptions]
    assert '/ground_truth/odom' not in topics


def test_node_never_publishes_cmd_vel(node):
    """Confirm the node has no /cmd_vel publisher of any kind."""
    publisher_topics = [publisher.topic_name for publisher in node.publishers]
    assert '/cmd_vel' not in publisher_topics


def test_node_does_not_subscribe_to_map_or_range(node):
    """Confirm the node never subscribes to /map or any /range/<beam> topic."""
    topics = [subscription.topic_name for subscription in node.subscriptions]
    assert '/map' not in topics
    assert not any(topic.startswith('/range/') for topic in topics)


def test_default_mission_starts_active_with_sample_waypoints(node):
    """Confirm the default parameters produce the documented three-waypoint mission."""
    assert node._state.status != MISSION_IDLE
    assert len(node._state.waypoints) == 3


# ---------------------------------------------------------------------------
# Empty mission
# ---------------------------------------------------------------------------

def test_empty_mission_stays_idle_and_publishes_no_goal(node):
    """Confirm an empty mission publishes no goal and never crashes."""
    _make_empty_mission_node(node)
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=0.0, y=0.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)

    assert captured == []
    assert node._state.status == MISSION_IDLE


def test_empty_array_cli_override_does_not_crash_node_construction():
    """Confirm '-p waypoint_x:=[] -p waypoint_y:=[]' produces an idle mission, not a crash."""
    # An empty-array parameter override carries no element-type
    # information, so rclpy resolves it to an uninitialized parameter
    # rather than an empty double array, even when a non-empty default is
    # declared. This regression test runs construction through a real,
    # isolated rclpy context with those exact command-line arguments, the
    # same path a launch file or manual `ros2 run` invocation takes.
    isolated_context = Context()
    rclpy.init(
        args=['--ros-args', '-p', 'waypoint_x:=[]', '-p', 'waypoint_y:=[]'],
        context=isolated_context,
    )
    try:
        isolated_node = MissionManagerNode(context=isolated_context)
        try:
            assert isolated_node._state.status == MISSION_IDLE
            assert isolated_node._state.waypoints == ()
        finally:
            isolated_node.destroy_node()
    finally:
        rclpy.shutdown(context=isolated_context)


# ---------------------------------------------------------------------------
# One-goal-at-a-time publication and monotonic progression
# ---------------------------------------------------------------------------

def test_first_goal_is_published_once_localization_is_usable(node):
    """Confirm the first waypoint is published exactly once localization arrives."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=0.0, y=0.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)

    assert len(captured) == 1
    assert captured[0].pose.position.x == pytest.approx(4.0)
    assert captured[0].pose.position.y == pytest.approx(2.0)


def test_timer_ticks_do_not_repeatedly_republish_same_waypoint(node):
    """Confirm repeated ticks with the rover still short of the goal publish nothing new."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=0.0, y=0.0, sec=10))
    for tick in range(5):
        node._evaluate_mission(now_seconds=10.05 + tick * 0.1)

    assert len(captured) == 1


def test_reaching_waypoint_publishes_exactly_the_next_waypoint(node):
    """Confirm reaching waypoint 0 advances and publishes waypoint 1, and nothing else."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=0.0, y=0.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)
    assert len(captured) == 1

    node._handle_localization(_make_localization(x=4.0, y=2.0, sec=11))
    node._evaluate_mission(now_seconds=11.05)

    assert len(captured) == 2
    assert captured[1].pose.position.x == pytest.approx(7.0)
    assert captured[1].pose.position.y == pytest.approx(2.0)
    assert node._state.index == 1


def test_final_waypoint_completion_publishes_no_extra_waypoint(node):
    """Confirm reaching the final waypoint marks completion without an extra publish."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=4.0, y=2.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)  # publish waypoint 0, then it is reached
    node._handle_localization(_make_localization(x=7.0, y=2.0, sec=11))
    node._evaluate_mission(now_seconds=11.05)  # advance to and publish waypoint 1, reached
    node._handle_localization(_make_localization(x=7.0, y=5.0, sec=12))
    node._evaluate_mission(now_seconds=12.05)  # advance to and publish waypoint 2, reached
    node._handle_localization(_make_localization(x=7.0, y=5.0, sec=13))
    node._evaluate_mission(now_seconds=13.05)  # final waypoint reached: mission completes

    assert len(captured) == 3
    assert node._state.status == MISSION_COMPLETE

    # Further ticks must not publish anything else.
    node._evaluate_mission(now_seconds=14.0)
    assert len(captured) == 3


def test_one_tick_does_not_cascade_through_closely_spaced_waypoints(node):
    """Confirm one tick advances at most one index, even when two goals are both in range."""
    node._config = MissionConfig(waypoint_x=(1.0, 1.05, 5.0), waypoint_y=(1.0, 1.0, 1.0))
    node._state = initialize_mission_state(node._config)
    captured = _capture_published_goals(node)

    # The rover is already within tolerance of both waypoint 0 and waypoint 1.
    node._handle_localization(_make_localization(x=1.02, y=1.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)

    assert node._state.index == 1
    assert len(captured) == 2
    assert captured[0].pose.position.x == pytest.approx(1.0)
    assert captured[1].pose.position.x == pytest.approx(1.05)


# ---------------------------------------------------------------------------
# Localization freshness
# ---------------------------------------------------------------------------

def test_stale_localization_prevents_progression(node):
    """Confirm a localization sample older than the timeout blocks all advancement."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=4.0, y=2.0, sec=10))
    stale_now = 10.0 + node._config.localization_timeout_s + 1.0
    node._evaluate_mission(now_seconds=stale_now)

    assert captured == []
    assert node._state.index == 0
    assert node._state.status != MISSION_COMPLETE


def test_missing_localization_prevents_progression(node):
    """Confirm no localization at all blocks publishing and advancement."""
    captured = _capture_published_goals(node)
    node._evaluate_mission(now_seconds=100.0)
    assert captured == []
    assert node._state.index == 0


def test_localization_recovery_resumes_mission(node):
    """Confirm the mission resumes from the same waypoint once localization is fresh again."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=0.0, y=0.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)
    assert len(captured) == 1

    # Localization goes stale for a while; no progression should occur.
    stale_now = 10.0 + node._config.localization_timeout_s + 5.0
    node._evaluate_mission(now_seconds=stale_now)
    assert len(captured) == 1
    assert node._state.index == 0

    # Fresh localization arrives again, at the same waypoint 0, now in tolerance.
    node._handle_localization(_make_localization(x=4.0, y=2.0, sec=20))
    node._evaluate_mission(now_seconds=20.05)

    assert node._state.index == 1
    assert len(captured) == 2


# ---------------------------------------------------------------------------
# Zero-timestamp convention
# ---------------------------------------------------------------------------

def test_zero_stamp_localization_falls_back_to_receipt_time(node):
    """Confirm a zero header.stamp is treated as unstamped via receipt-time fallback."""
    before = node.get_clock().now().nanoseconds / 1_000_000_000.0
    node._handle_localization(_make_localization(x=1.0, y=1.0, sec=0, nanosec=0))
    after = node.get_clock().now().nanoseconds / 1_000_000_000.0

    assert node._localization_stamp_seconds is not None
    assert before <= node._localization_stamp_seconds <= after


# ---------------------------------------------------------------------------
# Goal message construction
# ---------------------------------------------------------------------------

def test_goal_frame_and_coordinates_are_correct(node):
    """Confirm the published goal has the expected frame, position, and orientation."""
    captured = _capture_published_goals(node)

    node._handle_localization(_make_localization(x=0.0, y=0.0, sec=10))
    node._evaluate_mission(now_seconds=10.05)

    goal = captured[0]
    assert goal.header.frame_id == 'map'
    assert goal.pose.position.x == pytest.approx(4.0)
    assert goal.pose.position.y == pytest.approx(2.0)
    assert goal.pose.position.z == pytest.approx(0.0)
    assert goal.pose.orientation.x == pytest.approx(0.0)
    assert goal.pose.orientation.y == pytest.approx(0.0)
    assert goal.pose.orientation.z == pytest.approx(0.0)
    assert goal.pose.orientation.w == pytest.approx(1.0)
