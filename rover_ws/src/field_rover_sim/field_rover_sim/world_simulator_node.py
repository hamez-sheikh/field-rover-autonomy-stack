"""ROS 2 node that publishes the static ground-truth rover pose."""

import math

from field_rover_sim.world_model import (
    DEFAULT_WORLD,
    INITIAL_ROVER_POSE,
    is_pose_in_collision,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class WorldSimulatorNode(Node):
    """Publish the fixed ground-truth pose for the static rover world."""

    def __init__(self):
        """Initialize the world simulator and ground-truth publisher."""
        super().__init__('world_simulator')

        self._publisher = self.create_publisher(
            Odometry,
            '/ground_truth/odom',
            10,
        )
        self._timer = self.create_timer(0.2, self._publish_ground_truth)

        if is_pose_in_collision(INITIAL_ROVER_POSE, DEFAULT_WORLD):
            self.get_logger().error(
                'The initial rover pose is in collision.',
            )
        else:
            self.get_logger().info(
                'The initial rover pose is collision-free.',
            )

        self.get_logger().info(
            'Publishing static ground truth at 5 Hz.',
        )

    def _publish_ground_truth(self):
        """Publish the unchanged ground-truth pose as an Odometry message."""
        message = Odometry()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.child_frame_id = 'base_link'

        message.pose.pose.position.x = INITIAL_ROVER_POSE.x
        message.pose.pose.position.y = INITIAL_ROVER_POSE.y
        message.pose.pose.position.z = 0.0

        half_yaw = INITIAL_ROVER_POSE.yaw / 2.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)

        message.twist.twist.linear.x = 0.0
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0
        message.twist.twist.angular.x = 0.0
        message.twist.twist.angular.y = 0.0
        message.twist.twist.angular.z = 0.0

        self._publisher.publish(message)


def main(args=None):
    """Run the static world simulator node."""
    rclpy.init(args=args)
    node = WorldSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
