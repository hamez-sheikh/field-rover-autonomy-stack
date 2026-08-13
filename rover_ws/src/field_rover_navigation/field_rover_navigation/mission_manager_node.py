"""ROS 2 node sequencing a multi-waypoint mission through /goal_pose."""

from field_rover_navigation.mission_manager import (
    active_waypoint,
    advance_mission,
    DEFAULT_MISSION_CONFIG,
    distance_to_active_waypoint,
    initialize_mission_state,
    is_localization_usable,
    is_waypoint_reached,
    MISSION_ACTIVE,
    MISSION_COMPLETE,
    MissionConfig,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS Time message into seconds as a single float."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


class MissionManagerNode(Node):
    """Sequence an ordered multi-waypoint mission through /goal_pose, one goal at a time."""

    def __init__(self, **kwargs):
        """Declare parameters, build the initial mission state, and wire ROS I/O."""
        super().__init__('mission_manager', **kwargs)

        self._config = self._build_validated_config()
        self._state = initialize_mission_state(self._config)

        self._localization_x = None
        self._localization_y = None
        self._localization_stamp_seconds = None
        self._published_index = None

        self._goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._odom_subscription = self.create_subscription(
            Odometry, '/localization/odom', self._handle_localization, 10,
        )

        timer_period = 1.0 / self._config.mission_rate_hz
        self._timer = self.create_timer(timer_period, self._mission_timer_callback)

        if not self._state.waypoints:
            self.get_logger().info('No mission waypoints configured; staying idle.')
        elif self._state.status != MISSION_ACTIVE:
            self.get_logger().info(
                'auto_start is false; the mission will not begin automatically.'
            )
        else:
            self.get_logger().info(
                f'Sequencing {len(self._state.waypoints)} waypoint(s) via /goal_pose; '
                'waiting for usable /localization/odom before publishing the first goal.'
            )
        self.get_logger().info(
            'This is mission sequencing only: it never publishes /cmd_vel, never '
            'plans paths itself, and never subscribes to /ground_truth/odom.'
        )

    def _declare_waypoint_array(self, name: str, default: tuple) -> tuple:
        """Declare a double-array waypoint parameter as an ordered float tuple."""
        # A command-line or YAML override of an empty array (e.g.
        # -p waypoint_x:=[]) cannot carry element-type information, so
        # rclpy resolves it to an uninitialized parameter instead of an
        # empty list — even with a non-empty default supplied here. Since
        # an empty array is the only way to request zero waypoints through
        # the parameter interface, an uninitialized override falls back to
        # an explicit empty double array here rather than surfacing as a
        # crash.
        self.declare_parameter(name, list(default))
        empty_double_array = Parameter(name, Parameter.Type.DOUBLE_ARRAY, [])
        parameter = self.get_parameter_or(name, empty_double_array)
        return tuple(float(value) for value in parameter.value)

    def _build_validated_config(self) -> MissionConfig:
        """Declare mission parameters and build a validated MissionConfig."""
        defaults = DEFAULT_MISSION_CONFIG

        waypoint_x = self._declare_waypoint_array('waypoint_x', defaults.waypoint_x)
        waypoint_y = self._declare_waypoint_array('waypoint_y', defaults.waypoint_y)
        self.declare_parameter('mission_rate_hz', defaults.mission_rate_hz)
        self.declare_parameter('waypoint_tolerance_m', defaults.waypoint_tolerance_m)
        self.declare_parameter('localization_timeout_s', defaults.localization_timeout_s)
        self.declare_parameter('map_frame', defaults.map_frame)
        self.declare_parameter('auto_start', defaults.auto_start)

        return MissionConfig(
            waypoint_x=waypoint_x,
            waypoint_y=waypoint_y,
            mission_rate_hz=float(self.get_parameter('mission_rate_hz').value),
            waypoint_tolerance_m=float(
                self.get_parameter('waypoint_tolerance_m').value
            ),
            localization_timeout_s=float(
                self.get_parameter('localization_timeout_s').value
            ),
            map_frame=str(self.get_parameter('map_frame').value),
            auto_start=bool(self.get_parameter('auto_start').value),
        )

    def _handle_localization(self, message: Odometry):
        """Store the latest rover position and a usable freshness timestamp."""
        # Mission evaluation runs on its own timer, so a localization
        # update alone never advances the mission — see _evaluate_mission.
        position = message.pose.pose.position
        self._localization_x = position.x
        self._localization_y = position.y

        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            # An unstamped (all-zero) header would otherwise read as
            # infinitely stale; fall back to this node's receipt time, the
            # same documented convention Day 11's path_follower_node uses.
            self._localization_stamp_seconds = (
                self.get_clock().now().nanoseconds / 1_000_000_000.0
            )
        else:
            self._localization_stamp_seconds = stamp_to_seconds(stamp)

    def _mission_timer_callback(self):
        """Real timer entry point: capture the current time and evaluate once."""
        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self._evaluate_mission(now_seconds)

    def _evaluate_mission(self, now_seconds: float):
        """Advance and publish at most one waypoint step, gated on fresh localization."""
        if self._state.status != MISSION_ACTIVE:
            return

        usable = is_localization_usable(
            self._localization_x,
            self._localization_y,
            self._localization_stamp_seconds,
            now_seconds,
            self._config.localization_timeout_s,
        )
        if not usable:
            # Missing or stale localization: preserve mission state exactly
            # as-is and try again on the next tick.
            return

        self._publish_active_goal_if_new()

        distance = distance_to_active_waypoint(
            self._state, self._localization_x, self._localization_y,
        )
        if distance is None or not is_waypoint_reached(
            distance, self._config.waypoint_tolerance_m,
        ):
            return

        self._state = advance_mission(self._state)
        if self._state.status == MISSION_COMPLETE:
            self.get_logger().info('Final waypoint reached; mission complete.')
            return

        self._publish_active_goal_if_new()

    def _publish_active_goal_if_new(self):
        """Publish the active waypoint on /goal_pose, but only once per index."""
        if self._published_index == self._state.index:
            return

        waypoint = active_waypoint(self._state)
        if waypoint is None:
            return

        message = PoseStamped()
        message.header.frame_id = self._config.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = waypoint.x
        message.pose.position.y = waypoint.y
        message.pose.position.z = 0.0
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = 0.0
        message.pose.orientation.w = 1.0

        self._goal_publisher.publish(message)
        self._published_index = self._state.index

        self.get_logger().info(
            f'Publishing waypoint {self._state.index + 1}/{len(self._state.waypoints)} '
            f'at ({waypoint.x:.2f}, {waypoint.y:.2f}).'
        )


def main(args=None):
    """Run the mission manager node."""
    rclpy.init(args=args)
    node = MissionManagerNode()

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
