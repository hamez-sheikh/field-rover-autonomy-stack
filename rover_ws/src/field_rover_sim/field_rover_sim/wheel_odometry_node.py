"""ROS 2 node publishing simulated differential-drive wheel odometry."""

import math

from field_rover_sim.range_sensor_node import quaternion_to_yaw
from field_rover_sim.wheel_odometry import (
    DEFAULT_WHEEL_ODOMETRY_CONFIG,
    integrate_wheel_odometry,
    WheelOdometryConfig,
    WheelOdometryState,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


# Ground truth publishes at 20 Hz (0.05 s). A much larger gap between
# messages (a slow tick, a paused simulator) should not be integrated as if
# the rover coasted the whole time, so elapsed time is clamped.
MAXIMUM_TIME_STEP_SECONDS = 0.5


class WheelOdometryNode(Node):
    """Dead-reckon an imperfect pose estimate from simulated wheel travel."""

    def __init__(self):
        """Declare parameters, build the calibration config, and wire ROS I/O."""
        super().__init__('wheel_odometry')

        self._config = self._build_validated_config()

        self._state = None
        self._last_stamp_seconds = None

        self._publisher = self.create_publisher(Odometry, '/wheel/odom', 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._handle_ground_truth,
            10,
        )

        self.get_logger().info(
            'Simulating wheel odometry with track width '
            f'{self._config.wheel_track_width:.2f} m, left_wheel_scale='
            f'{self._config.left_wheel_scale:.3f}, right_wheel_scale='
            f'{self._config.right_wheel_scale:.3f}.'
        )
        self.get_logger().info(
            'Dead-reckoning from /ground_truth/odom velocity; the estimate '
            'is initialized once from the first pose and never corrected.'
        )

    def _build_validated_config(self) -> WheelOdometryConfig:
        """Declare wheel-odometry parameters and build a validated config."""
        self.declare_parameter(
            'wheel_track_width',
            DEFAULT_WHEEL_ODOMETRY_CONFIG.wheel_track_width,
        )
        self.declare_parameter(
            'left_wheel_scale',
            DEFAULT_WHEEL_ODOMETRY_CONFIG.left_wheel_scale,
        )
        self.declare_parameter(
            'right_wheel_scale',
            DEFAULT_WHEEL_ODOMETRY_CONFIG.right_wheel_scale,
        )

        return WheelOdometryConfig(
            wheel_track_width=float(
                self.get_parameter('wheel_track_width').value
            ),
            left_wheel_scale=float(
                self.get_parameter('left_wheel_scale').value
            ),
            right_wheel_scale=float(
                self.get_parameter('right_wheel_scale').value
            ),
        )

    def _handle_ground_truth(self, message):
        """Initialize once from the first pose, then dead-reckon from velocity."""
        stamp = message.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec / 1_000_000_000.0

        if self._state is None:
            self._initialize_from_ground_truth(message, stamp_seconds)
            self._publish_estimate(stamp)
            return

        dt = stamp_seconds - self._last_stamp_seconds
        self._last_stamp_seconds = stamp_seconds

        if dt > MAXIMUM_TIME_STEP_SECONDS:
            dt = MAXIMUM_TIME_STEP_SECONDS

        if dt > 0.0:
            self._state = integrate_wheel_odometry(
                self._state,
                linear_velocity=message.twist.twist.linear.x,
                angular_velocity=message.twist.twist.angular.z,
                config=self._config,
                dt=dt,
            )

        self._publish_estimate(stamp)

    def _initialize_from_ground_truth(self, message, stamp_seconds):
        """Copy the first ground-truth pose once as the odometry origin."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.z, orientation.w)

        self._state = WheelOdometryState(x=position.x, y=position.y, yaw=yaw)
        self._last_stamp_seconds = stamp_seconds

        self.get_logger().info(
            'Initialized wheel-odometry estimate from ground truth at '
            f'({position.x:.2f}, {position.y:.2f}, {yaw:.2f}).'
        )

    def _publish_estimate(self, stamp):
        """Publish the current dead-reckoned pose and velocity as Odometry."""
        message = Odometry()

        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'

        message.pose.pose.position.x = self._state.x
        message.pose.pose.position.y = self._state.y
        message.pose.pose.position.z = 0.0

        half_yaw = self._state.yaw / 2.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)

        message.twist.twist.linear.x = self._state.linear_velocity
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0
        message.twist.twist.angular.x = 0.0
        message.twist.twist.angular.y = 0.0
        message.twist.twist.angular.z = self._state.angular_velocity

        self._publisher.publish(message)


def main(args=None):
    """Run the wheel-odometry node."""
    rclpy.init(args=args)
    node = WheelOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
