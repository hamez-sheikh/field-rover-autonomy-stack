"""Unit tests for the localization node's message-ordering guards."""

# Every other behaviour in this package is tested through the pure
# geographic-conversion and pose-estimator modules. The two checks here —
# a GPS or IMU message arriving before the first wheel pose must not
# corrupt state — are properties of the node's callback wiring itself
# (whether self._state is still None), so they are exercised directly
# against the node's callbacks instead.

from field_rover_localization.localization_node import LocalizationNode
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import Imu, NavSatFix


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    created_node = LocalizationNode()
    yield created_node
    created_node.destroy_node()


def test_first_wheel_message_initializes_state_without_integrating(node):
    """Confirm the first /wheel/odom message seeds pose, not a jump."""
    message = Odometry()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 0
    message.pose.pose.position.x = 2.0
    message.pose.pose.position.y = -1.0
    message.pose.pose.orientation.z = 0.2474039593
    message.pose.pose.orientation.w = 0.9689124217

    node._handle_wheel_odometry(message)

    assert node._state.x == pytest.approx(2.0)
    assert node._state.y == pytest.approx(-1.0)
    assert node._state.yaw == pytest.approx(0.5, abs=1e-6)
    assert node._state.linear_velocity == pytest.approx(0.0)
    assert node._state.angular_velocity == pytest.approx(0.0)


def test_gps_before_wheel_initialization_does_not_corrupt_state(node):
    """Confirm a GPS fix arriving before any wheel pose is safely ignored."""
    gps_message = NavSatFix()
    gps_message.header.stamp.sec = 5
    gps_message.latitude = 43.27
    gps_message.longitude = -79.90

    node._handle_gps(gps_message)

    assert node._state is None


def test_imu_before_wheel_initialization_is_handled_safely(node):
    """Confirm an IMU sample arriving before any wheel pose is stored safely."""
    imu_message = Imu()
    imu_message.header.stamp.sec = 3
    imu_message.angular_velocity.z = 0.42

    node._handle_imu(imu_message)

    assert node._state is None
    assert node._latest_imu_angular_velocity_z == pytest.approx(0.42)
