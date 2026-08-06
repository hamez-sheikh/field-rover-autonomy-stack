"""ROS 2 node planning an A* path from the latest map, pose, and goal."""

import math

from field_rover_navigation.astar_planner import (
    AStarConfig,
    compute_path_yaws,
    DEFAULT_ASTAR_CONFIG,
    plan_grid_path,
    world_path_from_grid_path,
    yaw_to_quaternion_zw,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def quaternion_to_yaw(orientation_z: float, orientation_w: float) -> float:
    """Recover the yaw angle from a planar (z, w) orientation quaternion."""
    return 2.0 * math.atan2(orientation_z, orientation_w)


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS Time message into seconds as a single float."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


class MapSnapshot:
    """Hold the most recently received, validated OccupancyGrid."""

    __slots__ = (
        'width', 'height', 'resolution_m', 'origin_x_m', 'origin_y_m', 'data', 'frame_id',
    )

    def __init__(self, message: OccupancyGrid):
        """Copy the fields of one OccupancyGrid message needed for planning."""
        self.width = message.info.width
        self.height = message.info.height
        self.resolution_m = message.info.resolution
        self.origin_x_m = message.info.origin.position.x
        self.origin_y_m = message.info.origin.position.y
        self.data = list(message.data)
        self.frame_id = message.header.frame_id


class AStarPlannerNode(Node):
    """Plan a collision-free grid path on request, from the map and fused pose."""

    def __init__(self):
        """Declare parameters and wire the map, pose, goal, and path topics."""
        super().__init__('astar_planner')

        self._config = self._build_validated_config()
        self._map_frame = str(self.get_parameter('map_frame').value)

        self._map: MapSnapshot | None = None
        self._pose_x = None
        self._pose_y = None
        self._pose_yaw = None

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self._handle_map, map_qos,
        )
        self._odom_subscription = self.create_subscription(
            Odometry, '/localization/odom', self._handle_localization, 10,
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self._handle_goal, 10,
        )
        self._path_publisher = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info(
            'Waiting for /map, /localization/odom, and /goal_pose. '
            f'allow_diagonal={self._config.allow_diagonal}, '
            f'allow_unknown={self._config.allow_unknown}, '
            f'occupied_threshold={self._config.occupied_threshold}.'
        )
        self.get_logger().info(
            'This is path generation only: it never subscribes to '
            '/ground_truth/odom and never publishes /cmd_vel.'
        )

    def _build_validated_config(self) -> AStarConfig:
        """Declare planner parameters and build a validated AStarConfig."""
        defaults = DEFAULT_ASTAR_CONFIG

        self.declare_parameter('allow_diagonal', defaults.allow_diagonal)
        self.declare_parameter('prevent_corner_cutting', defaults.prevent_corner_cutting)
        self.declare_parameter('allow_unknown', defaults.allow_unknown)
        self.declare_parameter('occupied_threshold', defaults.occupied_threshold)
        self.declare_parameter('max_expansions', defaults.max_expansions)
        self.declare_parameter('unknown_traversal_cost', defaults.unknown_traversal_cost)
        self.declare_parameter('map_frame', 'map')

        return AStarConfig(
            allow_diagonal=bool(self.get_parameter('allow_diagonal').value),
            prevent_corner_cutting=bool(
                self.get_parameter('prevent_corner_cutting').value
            ),
            allow_unknown=bool(self.get_parameter('allow_unknown').value),
            occupied_threshold=int(self.get_parameter('occupied_threshold').value),
            max_expansions=int(self.get_parameter('max_expansions').value),
            unknown_traversal_cost=float(
                self.get_parameter('unknown_traversal_cost').value
            ),
        )

    def _handle_map(self, message: OccupancyGrid):
        """Validate and store the latest map; never plan from inside this callback."""
        width = message.info.width
        height = message.info.height
        if width <= 0 or height <= 0 or len(message.data) != width * height:
            self.get_logger().warning(
                'Ignoring an OccupancyGrid with invalid dimensions or data length '
                f'({width}x{height}, {len(message.data)} cells).'
            )
            return

        self._map = MapSnapshot(message)

    def _handle_localization(self, message: Odometry):
        """Store the latest fused pose; do not plan here."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        self._pose_x = position.x
        self._pose_y = position.y
        self._pose_yaw = quaternion_to_yaw(orientation.z, orientation.w)

    def _handle_goal(self, message: PoseStamped):
        """Plan on each new goal and publish the resulting path, or an empty one."""
        if self._map is None:
            self.get_logger().warning('Ignoring /goal_pose: no /map received yet.')
            self._publish_path([], [])
            return
        if self._pose_x is None:
            self.get_logger().warning(
                'Ignoring /goal_pose: no /localization/odom received yet.'
            )
            self._publish_path([], [])
            return
        if message.header.frame_id != self._map_frame:
            self.get_logger().warning(
                f"Ignoring /goal_pose in frame '{message.header.frame_id}', "
                f"expected '{self._map_frame}'."
            )
            self._publish_path([], [])
            return

        result = plan_grid_path(
            width_cells=self._map.width,
            height_cells=self._map.height,
            resolution_m=self._map.resolution_m,
            origin_x_m=self._map.origin_x_m,
            origin_y_m=self._map.origin_y_m,
            data=self._map.data,
            start_world=(self._pose_x, self._pose_y),
            goal_world=(message.pose.position.x, message.pose.position.y),
            config=self._config,
        )

        if not result.success:
            self.get_logger().info(f'Planning failed: {result.failure_reason}.')
            self._publish_path([], [])
            return

        world_path = world_path_from_grid_path(
            self._map.width,
            self._map.height,
            self._map.resolution_m,
            self._map.origin_x_m,
            self._map.origin_y_m,
            result.grid_path,
        )
        goal_yaw = quaternion_to_yaw(
            message.pose.orientation.z, message.pose.orientation.w,
        )
        yaws = compute_path_yaws(world_path, goal_yaw)

        self.get_logger().info(
            f'Planned a {len(result.grid_path)}-cell path, cost={result.cost:.2f}, '
            f'expanded={result.expanded_nodes}.'
        )
        self._publish_path(world_path, yaws)

    def _publish_path(self, world_path: list[tuple[float, float]], yaws: list[float]):
        """Publish a Path built from world points and per-point yaws, in the map frame."""
        stamp = self.get_clock().now().to_msg()

        message = Path()
        message.header.stamp = stamp
        message.header.frame_id = self._map_frame

        for (world_x, world_y), yaw in zip(world_path, yaws):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self._map_frame
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            orientation_z, orientation_w = yaw_to_quaternion_zw(yaw)
            pose.pose.orientation.z = orientation_z
            pose.pose.orientation.w = orientation_w
            message.poses.append(pose)

        self._path_publisher.publish(message)


def main(args=None):
    """Run the A* planner node."""
    rclpy.init(args=args)
    node = AStarPlannerNode()

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
