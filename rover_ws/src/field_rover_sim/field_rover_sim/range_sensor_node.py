"""ROS 2 node publishing ideal directional range-sensor readings."""

import math

from field_rover_sim.range_sensor import (
    BeamDefinition,
    measure_all_beams,
    RangeSensorConfig,
)
from field_rover_sim.world_model import DEFAULT_WORLD, Pose2D
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


DEFAULT_PUBLISH_RATE_HZ = 10.0
DEFAULT_MIN_RANGE = 0.1
DEFAULT_MAX_RANGE = 8.0
DEFAULT_FIELD_OF_VIEW = 0.05

BEAM_ANGLES_DEG = {
    'front_far_right': -60.0,
    'front_right': -30.0,
    'front': 0.0,
    'front_left': 30.0,
    'front_far_left': 60.0,
}


def quaternion_to_yaw(orientation_z: float, orientation_w: float) -> float:
    """Recover the yaw angle from a planar (z, w) orientation quaternion."""
    return 2.0 * math.atan2(orientation_z, orientation_w)


class RangeSensorNode(Node):
    """Publish one ideal sensor_msgs/Range topic per configured beam."""

    def __init__(self):
        """Declare parameters, build the beam configuration, and wire ROS I/O."""
        super().__init__('range_sensor')

        self._declare_parameters()
        self._config = self._build_validated_config()

        self._latest_pose = None

        self._odom_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._handle_ground_truth,
            10,
        )

        self._range_publishers = {
            beam.name: self.create_publisher(
                Range, f'/range/{beam.name}', 10,
            )
            for beam in self._config.beams
        }

        publish_period = 1.0 / self._publish_rate_hz
        self._timer = self.create_timer(publish_period, self._publish_ranges)

        self.get_logger().info(
            f'Publishing {len(self._config.beams)} range beams at '
            f'{self._publish_rate_hz:.1f} Hz '
            f'(min_range={self._config.min_range:.2f} m, '
            f'max_range={self._config.max_range:.2f} m, '
            f'field_of_view={self._config.field_of_view:.3f} rad).'
        )
        self.get_logger().info(
            'Subscribed to ground-truth pose on /ground_truth/odom.'
        )

    def _declare_parameters(self):
        """Declare rate and range parameters, keeping beam angles fixed."""
        self.declare_parameter('publish_rate_hz', DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter('min_range', DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', DEFAULT_MAX_RANGE)
        self.declare_parameter('field_of_view', DEFAULT_FIELD_OF_VIEW)

    def _build_validated_config(self) -> RangeSensorConfig:
        """Read parameters, validate them, and build the sensor configuration."""
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)
        field_of_view = float(self.get_parameter('field_of_view').value)

        if (
            not math.isfinite(self._publish_rate_hz)
            or self._publish_rate_hz <= 0.0
        ):
            raise ValueError('publish_rate_hz must be positive and finite.')

        beams = tuple(
            BeamDefinition(name=name, relative_angle=math.radians(angle_deg))
            for name, angle_deg in BEAM_ANGLES_DEG.items()
        )

        return RangeSensorConfig(
            beams=beams,
            min_range=min_range,
            max_range=max_range,
            field_of_view=field_of_view,
        )

    def _handle_ground_truth(self, message):
        """Store the latest ground-truth pose extracted from Odometry."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.z, orientation.w)

        self._latest_pose = Pose2D(x=position.x, y=position.y, yaw=yaw)

    def _publish_ranges(self):
        """Measure all beams against the latest pose and publish one Range each."""
        if self._latest_pose is None:
            return

        stamp = self.get_clock().now().to_msg()
        readings = measure_all_beams(self._latest_pose, DEFAULT_WORLD, self._config)

        for beam_name, distance in readings:
            message = Range()
            message.header.stamp = stamp
            message.header.frame_id = f'range_{beam_name}'
            message.radiation_type = Range.INFRARED
            message.field_of_view = self._config.field_of_view
            message.min_range = self._config.min_range
            message.max_range = self._config.max_range
            message.range = distance

            self._range_publishers[beam_name].publish(message)


def main(args=None):
    """Run the directional range-sensor node."""
    rclpy.init(args=args)
    node = RangeSensorNode()

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
