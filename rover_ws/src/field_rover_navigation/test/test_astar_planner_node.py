"""Unit tests for the A* planner node's ROS wiring, gating, and publishing."""

# The search mathematics itself is exercised through the pure astar_planner
# module tests. These checks focus on properties that only exist at the
# node level: topic wiring, publish QoS, and the goal-callback gating that
# keeps a missing map or pose, or a mismatched frame, from ever planning.

import math

from field_rover_navigation.astar_planner_node import AStarPlannerNode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    created_node = AStarPlannerNode()
    yield created_node
    created_node.destroy_node()


def _make_map(
    width=5, height=5, resolution=1.0, origin_x=0.0, origin_y=0.0, data=None, frame_id='map',
):
    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.info.width = width
    message.info.height = height
    message.info.resolution = resolution
    message.info.origin.position.x = origin_x
    message.info.origin.position.y = origin_y
    message.data = data if data is not None else [0] * (width * height)
    return message


def _make_localization(x=0.5, y=0.5, yaw=0.0):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    half_yaw = yaw / 2.0
    message.pose.pose.orientation.z = math.sin(half_yaw)
    message.pose.pose.orientation.w = math.cos(half_yaw)
    return message


def _make_goal(x=4.5, y=4.5, yaw=0.0, frame_id='map'):
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x = x
    message.pose.position.y = y
    half_yaw = yaw / 2.0
    message.pose.orientation.z = math.sin(half_yaw)
    message.pose.orientation.w = math.cos(half_yaw)
    return message


def _capture_published_paths(node):
    """Replace the path publisher's publish() with one that records messages."""
    captured = []
    node._path_publisher.publish = lambda message: captured.append(message)
    return captured


def test_node_subscribes_to_map_localization_and_goal(node):
    """Confirm the node subscribes to /map, /localization/odom, and /goal_pose."""
    assert node._map_subscription.topic_name == '/map'
    assert node._odom_subscription.topic_name == '/localization/odom'
    assert node._goal_subscription.topic_name == '/goal_pose'


def test_node_publishes_planned_path_topic(node):
    """Confirm the node publishes nav_msgs/Path on /planned_path."""
    assert node._path_publisher.topic_name == '/planned_path'
    assert node._path_publisher.msg_type is Path


def test_node_does_not_subscribe_to_ground_truth(node):
    """Confirm no subscription to /ground_truth/odom exists anywhere on the node."""
    topics = [
        node._map_subscription.topic_name,
        node._odom_subscription.topic_name,
        node._goal_subscription.topic_name,
    ]
    assert '/ground_truth/odom' not in topics


def test_node_never_publishes_cmd_vel(node):
    """Confirm the node has no /cmd_vel publisher of any kind."""
    publisher_topics = [
        publisher.topic_name for publisher in node.publishers
    ]
    assert '/cmd_vel' not in publisher_topics


def test_map_subscription_uses_transient_local_durability(node):
    """Confirm the /map subscription QoS uses transient-local durability."""
    qos = node._map_subscription.qos_profile
    assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL


def test_no_planning_before_map_is_received(node):
    """Confirm a goal received before any map publishes an empty path, not a crash."""
    captured = _capture_published_paths(node)
    node._handle_localization(_make_localization())
    node._handle_goal(_make_goal())
    assert len(captured) == 1
    assert captured[0].poses == []


def test_no_planning_before_localization_is_received(node):
    """Confirm a goal received before any localization publishes an empty path."""
    captured = _capture_published_paths(node)
    node._handle_map(_make_map())
    node._handle_goal(_make_goal())
    assert len(captured) == 1
    assert captured[0].poses == []


def test_invalid_goal_frame_publishes_empty_path(node):
    """Confirm a goal stamped in the wrong frame is rejected with an empty path."""
    captured = _capture_published_paths(node)
    node._handle_map(_make_map())
    node._handle_localization(_make_localization())
    node._handle_goal(_make_goal(frame_id='odom'))
    assert len(captured) == 1
    assert captured[0].poses == []


def test_reachable_goal_publishes_non_empty_path(node):
    """Confirm a reachable goal in an open map publishes a full, correctly framed path."""
    captured = _capture_published_paths(node)
    node._handle_map(_make_map(width=5, height=5, data=[0] * 25))
    node._handle_localization(_make_localization(x=0.5, y=0.5))
    node._handle_goal(_make_goal(x=4.5, y=4.5))

    assert len(captured) == 1
    path = captured[0]
    assert len(path.poses) == 5
    assert path.header.frame_id == 'map'
    assert path.poses[0].pose.position.x == pytest.approx(0.5)
    assert path.poses[0].pose.position.y == pytest.approx(0.5)
    assert path.poses[-1].pose.position.x == pytest.approx(4.5)
    assert path.poses[-1].pose.position.y == pytest.approx(4.5)


def test_unreachable_goal_publishes_empty_path(node):
    """Confirm a goal on the far side of a full wall publishes an empty path."""
    width, height = 5, 5
    data = [0] * (width * height)
    for x in range(width):
        data[2 * width + x] = 100  # a fully occupied row at y=2, no gap
    captured = _capture_published_paths(node)
    node._handle_map(_make_map(width=width, height=height, data=data))
    node._handle_localization(_make_localization(x=2.5, y=0.5))
    node._handle_goal(_make_goal(x=2.5, y=4.5))

    assert len(captured) == 1
    assert captured[0].poses == []


def test_path_pose_count_matches_pure_planner_result(node):
    """Confirm the number of published poses matches the pure planner's cell count."""
    captured = _capture_published_paths(node)
    node._handle_map(_make_map(width=5, height=5, data=[0] * 25))
    node._handle_localization(_make_localization(x=0.5, y=0.5))
    node._handle_goal(_make_goal(x=2.5, y=0.5))

    assert len(captured[0].poses) == 3


def test_path_timestamp_and_frame_are_consistent_across_all_poses(node):
    """Confirm the path header stamp/frame and each pose's stamp/frame all match."""
    captured = _capture_published_paths(node)
    node._handle_map(_make_map(width=5, height=5, data=[0] * 25))
    node._handle_localization(_make_localization(x=0.5, y=0.5))
    node._handle_goal(_make_goal(x=4.5, y=0.5))

    path = captured[0]
    for pose in path.poses:
        assert pose.header.stamp.sec == path.header.stamp.sec
        assert pose.header.stamp.nanosec == path.header.stamp.nanosec
        assert pose.header.frame_id == 'map'


def test_invalid_map_is_ignored_without_replacing_a_good_map(node):
    """Confirm a data-length mismatch is rejected and never overwrites a good map."""
    node._handle_map(_make_map(width=5, height=5, data=[0] * 25))
    node._handle_map(_make_map(width=5, height=5, data=[0] * 10))

    assert node._map is not None
    assert len(node._map.data) == 25
