"""ROS 2 node building a bounded-evidence occupancy grid from range beams."""

import math

from field_rover_navigation.occupancy_grid import (
    build_occupancy_grid_data,
    DEFAULT_OCCUPANCY_GRID_CONFIG,
    is_measurement_fresh,
    is_new_range_sample,
    OccupancyGridConfig,
    OccupancyGridState,
    update_grid_with_beam,
)
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Range


# The five fixed beams published by field_rover_sim's range_sensor node
# (see field_rover_sim/range_sensor_node.py), reproduced here rather than
# imported so this package never depends on simulator internals.
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


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS Time message into seconds as a single float."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


class BeamSample:
    """Track the latest raw range reading received for one beam."""

    __slots__ = ('measured_range', 'min_range', 'max_range', 'stamp_seconds')

    def __init__(self):
        """Start with no reading received yet."""
        self.measured_range = None
        self.min_range = None
        self.max_range = None
        self.stamp_seconds = None


class OccupancyGridNode(Node):
    """Build a persistent occupancy grid from localization pose and range beams."""

    def __init__(self):
        """Declare parameters, build the grid state, and wire ROS I/O."""
        super().__init__('occupancy_grid_publisher')

        self._config = self._build_validated_config()
        self._state = OccupancyGridState(self._config)

        self._pose_x = None
        self._pose_y = None
        self._pose_yaw = None
        self._pose_stamp_seconds = None

        self._beam_angles_rad = {
            name: math.radians(angle_deg)
            for name, angle_deg in BEAM_ANGLES_DEG.items()
        }
        self._beam_samples = {name: BeamSample() for name in BEAM_ANGLES_DEG}
        self._last_processed_stamp_seconds = {
            name: None for name in BEAM_ANGLES_DEG
        }

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_publisher = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self._odom_subscription = self.create_subscription(
            Odometry, '/localization/odom', self._handle_localization, 10,
        )
        self._range_subscriptions = [
            self.create_subscription(
                Range, f'/range/{beam_name}', self._make_range_handler(beam_name), 10,
            )
            for beam_name in BEAM_ANGLES_DEG
        ]

        update_period = 1.0 / self._config.map_update_rate_hz
        self._timer = self.create_timer(update_period, self._update_and_publish)

        self.get_logger().info(
            f'Mapping a {self._config.world_width_m:.1f} m x '
            f'{self._config.world_height_m:.1f} m world at '
            f'{self._config.resolution_m:.2f} m resolution '
            f'({self._config.width_cells} x {self._config.height_cells} cells), '
            f'updating at {self._config.map_update_rate_hz:.1f} Hz.'
        )
        self.get_logger().info(
            'Subscribed to /localization/odom and five /range/<beam> topics. '
            'This is bounded-evidence occupancy-grid mapping, not SLAM: it '
            'does not estimate pose, and it never subscribes to '
            '/ground_truth/odom.'
        )

    def _build_validated_config(self) -> OccupancyGridConfig:
        """Declare mapping parameters and build a validated configuration."""
        defaults = DEFAULT_OCCUPANCY_GRID_CONFIG

        self.declare_parameter('world_width_m', defaults.world_width_m)
        self.declare_parameter('world_height_m', defaults.world_height_m)
        self.declare_parameter('resolution_m', defaults.resolution_m)
        self.declare_parameter('origin_x_m', defaults.origin_x_m)
        self.declare_parameter('origin_y_m', defaults.origin_y_m)
        self.declare_parameter('free_evidence_delta', defaults.free_evidence_delta)
        self.declare_parameter(
            'occupied_evidence_delta', defaults.occupied_evidence_delta,
        )
        self.declare_parameter('minimum_evidence', defaults.minimum_evidence)
        self.declare_parameter('maximum_evidence', defaults.maximum_evidence)
        self.declare_parameter('free_threshold', defaults.free_threshold)
        self.declare_parameter('occupied_threshold', defaults.occupied_threshold)
        self.declare_parameter('frame_id', defaults.frame_id)
        self.declare_parameter('map_update_rate_hz', defaults.map_update_rate_hz)
        self.declare_parameter(
            'localization_timeout_s', defaults.localization_timeout_s,
        )
        self.declare_parameter('range_timeout_s', defaults.range_timeout_s)

        return OccupancyGridConfig(
            world_width_m=float(self.get_parameter('world_width_m').value),
            world_height_m=float(self.get_parameter('world_height_m').value),
            resolution_m=float(self.get_parameter('resolution_m').value),
            origin_x_m=float(self.get_parameter('origin_x_m').value),
            origin_y_m=float(self.get_parameter('origin_y_m').value),
            free_evidence_delta=int(
                self.get_parameter('free_evidence_delta').value
            ),
            occupied_evidence_delta=int(
                self.get_parameter('occupied_evidence_delta').value
            ),
            minimum_evidence=int(self.get_parameter('minimum_evidence').value),
            maximum_evidence=int(self.get_parameter('maximum_evidence').value),
            free_threshold=int(self.get_parameter('free_threshold').value),
            occupied_threshold=int(
                self.get_parameter('occupied_threshold').value
            ),
            frame_id=str(self.get_parameter('frame_id').value),
            map_update_rate_hz=float(
                self.get_parameter('map_update_rate_hz').value
            ),
            localization_timeout_s=float(
                self.get_parameter('localization_timeout_s').value
            ),
            range_timeout_s=float(self.get_parameter('range_timeout_s').value),
        )

    def _handle_localization(self, message):
        """Store the latest localization pose and timestamp; do not map here."""
        # Mapping runs on its own timer, so a pose update alone never
        # triggers a grid update — see _update_and_publish.
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        self._pose_x = position.x
        self._pose_y = position.y
        self._pose_yaw = quaternion_to_yaw(orientation.z, orientation.w)
        self._pose_stamp_seconds = stamp_to_seconds(message.header.stamp)

    def _make_range_handler(self, beam_name: str):
        """Build a callback that stores the latest raw sample for one beam."""
        def _handle_range(message):
            sample = self._beam_samples[beam_name]
            sample.measured_range = message.range
            sample.min_range = message.min_range
            sample.max_range = message.max_range
            sample.stamp_seconds = stamp_to_seconds(message.header.stamp)

        return _handle_range

    def _update_and_publish(self):
        """On each timer tick, apply any fresh unprocessed beams, then publish."""
        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self._maybe_update_grid(now_seconds)
        self._publish_map()

    def _maybe_update_grid(self, now_seconds: float):
        """Apply every fresh, not-yet-processed beam sample exactly once."""
        pose_is_fresh = is_measurement_fresh(
            self._pose_stamp_seconds, now_seconds, self._config.localization_timeout_s,
        )
        if not pose_is_fresh:
            return

        for beam_name, sample in self._beam_samples.items():
            if sample.stamp_seconds is None:
                continue

            last_processed = self._last_processed_stamp_seconds[beam_name]
            if not is_new_range_sample(sample.stamp_seconds, last_processed):
                continue

            range_is_fresh = is_measurement_fresh(
                sample.stamp_seconds, now_seconds, self._config.range_timeout_s,
            )
            if not range_is_fresh:
                continue

            update_grid_with_beam(
                self._state,
                rover_x=self._pose_x,
                rover_y=self._pose_y,
                rover_yaw=self._pose_yaw,
                beam_relative_angle=self._beam_angles_rad[beam_name],
                measured_range=sample.measured_range,
                min_range=sample.min_range,
                max_range=sample.max_range,
            )
            self._last_processed_stamp_seconds[beam_name] = sample.stamp_seconds

    def _publish_map(self):
        """Publish the current persistent grid as a nav_msgs/OccupancyGrid."""
        # Runs every timer tick regardless of whether a fresh update was
        # applied, so the map keeps publishing through a temporary sensor
        # or localization dropout instead of going silent.
        stamp = self.get_clock().now().to_msg()

        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self._config.frame_id

        message.info.map_load_time = stamp
        message.info.resolution = self._config.resolution_m
        message.info.width = self._config.width_cells
        message.info.height = self._config.height_cells
        message.info.origin.position.x = self._config.origin_x_m
        message.info.origin.position.y = self._config.origin_y_m
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.w = 1.0

        message.data = build_occupancy_grid_data(self._state)

        self._map_publisher.publish(message)


def main(args=None):
    """Run the occupancy-grid mapping node."""
    rclpy.init(args=args)
    node = OccupancyGridNode()

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
