"""ROS 2 wrapper for the dynamic two-dimensional rover simulator."""

import math

from field_rover_sim.rover_dynamics import (
    clamp_requested_angular_speed,
    clamp_requested_linear_speed,
    DEFAULT_MOTION_LIMITS,
    MotionLimits,
    RoverState,
    update_rover_state,
)
from field_rover_sim.world_model import (
    DEFAULT_WORLD,
    INITIAL_ROVER_POSE,
    is_pose_in_collision,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


DEFAULT_UPDATE_RATE_HZ = 20.0
MAXIMUM_TIME_STEP_SECONDS = 0.1


class WorldSimulatorNode(Node):
    """Simulate constrained rover motion and publish perfect ground truth."""

    def __init__(self):
        """Initialize simulator state, parameters, ROS interfaces, and timer."""
        super().__init__('world_simulator')

        self._declare_parameters()
        self._update_rate_hz = float(
            self.get_parameter('update_rate_hz').value
        )

        if (
            not math.isfinite(self._update_rate_hz)
            or self._update_rate_hz <= 0.0
        ):
            raise ValueError('update_rate_hz must be positive and finite.')

        self._motion_limits = MotionLimits(
            max_forward_speed=float(
                self.get_parameter('max_forward_speed').value
            ),
            max_reverse_speed=float(
                self.get_parameter('max_reverse_speed').value
            ),
            max_turn_rate=float(
                self.get_parameter('max_turn_rate').value
            ),
            max_linear_acceleration=float(
                self.get_parameter('max_linear_acceleration').value
            ),
            max_linear_deceleration=float(
                self.get_parameter('max_linear_deceleration').value
            ),
            max_angular_acceleration=float(
                self.get_parameter('max_angular_acceleration').value
            ),
        )

        if is_pose_in_collision(INITIAL_ROVER_POSE, DEFAULT_WORLD):
            raise ValueError('The initial rover pose is in collision.')

        self._state = RoverState(
            x=INITIAL_ROVER_POSE.x,
            y=INITIAL_ROVER_POSE.y,
            yaw=INITIAL_ROVER_POSE.yaw,
        )
        self._requested_linear_speed = 0.0
        self._requested_angular_speed = 0.0

        self._publisher = self.create_publisher(
            Odometry,
            '/ground_truth/odom',
            10,
        )
        self._command_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._handle_command,
            10,
        )

        self._last_update_time = self.get_clock().now()
        timer_period = 1.0 / self._update_rate_hz
        self._timer = self.create_timer(
            timer_period,
            self._update_and_publish,
        )

        self.get_logger().info(
            'Initial rover pose is collision-free at '
            f'({self._state.x:.1f}, {self._state.y:.1f}, '
            f'{self._state.yaw:.1f}).'
        )
        self.get_logger().info(
            f'Updating motion and ground truth at '
            f'{self._update_rate_hz:.1f} Hz.'
        )
        self.get_logger().info(
            'Listening for manual velocity requests on /cmd_vel.'
        )

    def _declare_parameters(self):
        """Declare update-rate and motion-limit parameters."""
        self.declare_parameter(
            'update_rate_hz',
            DEFAULT_UPDATE_RATE_HZ,
        )
        self.declare_parameter(
            'max_forward_speed',
            DEFAULT_MOTION_LIMITS.max_forward_speed,
        )
        self.declare_parameter(
            'max_reverse_speed',
            DEFAULT_MOTION_LIMITS.max_reverse_speed,
        )
        self.declare_parameter(
            'max_turn_rate',
            DEFAULT_MOTION_LIMITS.max_turn_rate,
        )
        self.declare_parameter(
            'max_linear_acceleration',
            DEFAULT_MOTION_LIMITS.max_linear_acceleration,
        )
        self.declare_parameter(
            'max_linear_deceleration',
            DEFAULT_MOTION_LIMITS.max_linear_deceleration,
        )
        self.declare_parameter(
            'max_angular_acceleration',
            DEFAULT_MOTION_LIMITS.max_angular_acceleration,
        )

    def _handle_command(self, message):
        """Store clamped linear-x and angular-z velocity requests."""
        self._requested_linear_speed = clamp_requested_linear_speed(
            message.linear.x,
            self._motion_limits,
        )
        self._requested_angular_speed = clamp_requested_angular_speed(
            message.angular.z,
            self._motion_limits,
        )

    def _update_and_publish(self):
        """Advance the simulator using measured elapsed time and publish."""
        current_time = self.get_clock().now()
        dt = (
            current_time - self._last_update_time
        ).nanoseconds / 1_000_000_000.0
        self._last_update_time = current_time

        if dt > MAXIMUM_TIME_STEP_SECONDS:
            dt = MAXIMUM_TIME_STEP_SECONDS

        self._state, collision_occurred = update_rover_state(
            state=self._state,
            requested_linear_speed=self._requested_linear_speed,
            requested_angular_speed=self._requested_angular_speed,
            limits=self._motion_limits,
            world=DEFAULT_WORLD,
            dt=dt,
        )

        if collision_occurred:
            self.get_logger().warning(
                'Rejected motion that would collide with the world.',
                throttle_duration_sec=2.0,
            )
            self._requested_linear_speed = 0.0

        self._publish_ground_truth(current_time)

    def _publish_ground_truth(self, timestamp):
        """Publish the current perfect pose and velocity as Odometry."""
        message = Odometry()

        message.header.stamp = timestamp.to_msg()
        message.header.frame_id = 'map'
        message.child_frame_id = 'base_link'

        message.pose.pose.position.x = self._state.x
        message.pose.pose.position.y = self._state.y
        message.pose.pose.position.z = 0.0

        half_yaw = self._state.yaw / 2.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)

        message.twist.twist.linear.x = self._state.linear_speed
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0
        message.twist.twist.angular.x = 0.0
        message.twist.twist.angular.y = 0.0
        message.twist.twist.angular.z = self._state.angular_speed

        self._publisher.publish(message)


def main(args=None):
    """Run the dynamic world simulator node."""
    rclpy.init(args=args)
    node = WorldSimulatorNode()

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
