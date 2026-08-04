"""Unit tests for the occupancy-grid mapping node's ROS wiring and gating."""

# The mapping mathematics itself is exercised through the pure
# occupancy_grid module tests. These checks focus on properties that only
# exist at the node level: topic wiring, publish QoS, and the gating that
# keeps a stale pose or a duplicate range sample from touching the map.

import math

from field_rover_navigation.occupancy_grid_node import (
    BEAM_ANGLES_DEG,
    OccupancyGridNode,
)
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from sensor_msgs.msg import Range


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    created_node = OccupancyGridNode()
    yield created_node
    created_node.destroy_node()


def _wheel_free_localization(x=1.0, y=1.0, yaw=0.0, sec=10, nanosec=0):
    message = Odometry()
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    half_yaw = yaw / 2.0
    message.pose.pose.orientation.z = math.sin(half_yaw)
    message.pose.pose.orientation.w = math.cos(half_yaw)
    return message


def _range_message(value, sec=10, nanosec=0, min_range=0.1, max_range=8.0):
    message = Range()
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    message.min_range = min_range
    message.max_range = max_range
    message.range = value
    return message


def test_node_subscribes_to_localization_and_all_five_range_beams(node):
    """Confirm the node has exactly six subscriptions: localization + 5 beams."""
    assert node._odom_subscription is not None
    assert len(node._range_subscriptions) == 5
    assert set(BEAM_ANGLES_DEG.keys()) == {
        'front_far_right', 'front_right', 'front', 'front_left', 'front_far_left',
    }


def test_node_publishes_map_topic(node):
    """Confirm the node publishes nav_msgs/OccupancyGrid on /map."""
    assert node._map_publisher.topic_name == '/map'
    assert node._map_publisher.msg_type is OccupancyGrid


def test_node_does_not_subscribe_to_ground_truth(node):
    """Confirm no subscription to /ground_truth/odom exists anywhere on the node."""
    all_subscription_topics = [node._odom_subscription.topic_name] + [
        sub.topic_name for sub in node._range_subscriptions
    ]
    assert '/ground_truth/odom' not in all_subscription_topics
    assert node._odom_subscription.topic_name == '/localization/odom'


def test_map_publisher_uses_transient_local_durability(node):
    """Confirm the /map publisher QoS uses transient-local durability."""
    qos = node._map_publisher.qos_profile
    assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL


def test_no_grid_update_occurs_before_localization_is_received(node):
    """Confirm a fresh range sample alone, with no pose yet, updates nothing."""
    range_message = _range_message(2.5, sec=10)
    node._beam_samples['front'].measured_range = range_message.range
    node._beam_samples['front'].min_range = range_message.min_range
    node._beam_samples['front'].max_range = range_message.max_range
    node._beam_samples['front'].stamp_seconds = 10.0

    node._maybe_update_grid(now_seconds=10.05)

    assert node._state.evidence == [0] * node._config.total_cells


def test_duplicate_range_sample_is_processed_only_once(node):
    """Confirm the same range stamp is never applied to the grid twice."""
    node._handle_localization(_wheel_free_localization(x=1.0, y=1.0, sec=10))
    node._beam_samples['front'].measured_range = 2.0
    node._beam_samples['front'].min_range = 0.1
    node._beam_samples['front'].max_range = 8.0
    node._beam_samples['front'].stamp_seconds = 10.0

    node._maybe_update_grid(now_seconds=10.05)
    evidence_after_first = list(node._state.evidence)

    node._maybe_update_grid(now_seconds=10.06)
    evidence_after_second = list(node._state.evidence)

    assert evidence_after_first == evidence_after_second


def test_stale_localization_prevents_grid_update(node):
    """Confirm a localization pose older than the timeout blocks all updates."""
    node._handle_localization(_wheel_free_localization(x=1.0, y=1.0, sec=10))
    node._beam_samples['front'].measured_range = 2.0
    node._beam_samples['front'].min_range = 0.1
    node._beam_samples['front'].max_range = 8.0
    node._beam_samples['front'].stamp_seconds = 10.0

    node._maybe_update_grid(now_seconds=10.0 + node._config.localization_timeout_s + 1.0)

    assert node._state.evidence == [0] * node._config.total_cells


def test_one_beam_updates_while_another_beam_has_no_data(node):
    """Confirm one beam can update the map while a sibling beam stays silent."""
    node._handle_localization(_wheel_free_localization(x=1.0, y=1.0, sec=10))
    node._beam_samples['front'].measured_range = 2.0
    node._beam_samples['front'].min_range = 0.1
    node._beam_samples['front'].max_range = 8.0
    node._beam_samples['front'].stamp_seconds = 10.0
    # 'front_left' and the other three beams are left with no sample at all.

    node._maybe_update_grid(now_seconds=10.05)

    assert any(value != 0 for value in node._state.evidence)


def test_occupancy_grid_metadata_matches_configuration(node):
    """Confirm published map metadata (frame, resolution, dims) matches config."""
    captured = []
    node._map_publisher.publish = lambda message: captured.append(message)

    node._publish_map()

    assert len(captured) == 1
    message = captured[0]
    assert message.header.frame_id == node._config.frame_id
    assert message.info.resolution == pytest.approx(node._config.resolution_m)
    assert message.info.width == node._config.width_cells
    assert message.info.height == node._config.height_cells
    assert len(message.data) == node._config.width_cells * node._config.height_cells
