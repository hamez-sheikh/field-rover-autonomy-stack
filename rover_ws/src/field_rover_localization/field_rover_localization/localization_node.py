"""ROS 2 node fusing wheel odometry, IMU yaw rate, and GPS into one estimate."""

import math

from field_rover_localization.geographic_conversion import geographic_to_local
from field_rover_localization.pose_estimator import (
    apply_gps_correction,
    build_pose_covariance,
    build_twist_covariance,
    DEFAULT_LOCALIZATION_CONFIG,
    is_gps_measurement_new,
    is_imu_measurement_usable,
    LocalizationConfig,
    LocalizationState,
    predict_state,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


def quaternion_to_yaw(orientation_z: float, orientation_w: float) -> float:
    """Recover the yaw angle from a planar (z, w) orientation quaternion."""
    # A self-contained copy of the same one-line planar-yaw formula used
    # elsewhere in this project: field_rover_localization deliberately does
    # not depend on field_rover_sim, so this is not imported from there.
    return 2.0 * math.atan2(orientation_z, orientation_w)


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS Time message into seconds as a single float."""
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


class LocalizationNode(Node):
    """Fuse wheel odometry, IMU yaw rate, and GPS into one Odometry estimate."""

    def __init__(self):
        """Declare parameters, build the localization config, and wire ROS I/O."""
        super().__init__('localization')

        self._config = self._build_validated_config()

        self._state = None
        self._last_wheel_stamp_seconds = None

        self._latest_imu_stamp_seconds = None
        self._latest_imu_angular_velocity_z = None

        self._last_gps_stamp_seconds = None
        self._gps_applied_count = 0
        self._gps_rejected_count = 0

        self._publisher = self.create_publisher(
            Odometry, '/localization/odom', 10,
        )
        self._wheel_subscription = self.create_subscription(
            Odometry, '/wheel/odom', self._handle_wheel_odometry, 10,
        )
        self._imu_subscription = self.create_subscription(
            Imu, '/imu/data', self._handle_imu, 10,
        )
        self._gps_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self._handle_gps, 10,
        )

        self.get_logger().info(
            'Fusing /wheel/odom, /imu/data, and /gps/fix into '
            '/localization/odom. imu_yaw_rate_weight='
            f'{self._config.imu_yaw_rate_weight:.2f}, gps_position_gain='
            f'{self._config.gps_position_gain:.2f}, max_gps_innovation_m='
            f'{self._config.max_gps_innovation_m:.2f}.'
        )
        self.get_logger().info(
            'This is a complementary-style estimator, not an EKF: the '
            'output covariance below is a fixed nominal description, not '
            'propagated through a process/measurement model.'
        )
        self.get_logger().info(
            'Waiting for the first /wheel/odom message before publishing '
            'any fused estimate.'
        )

    def _build_validated_config(self) -> LocalizationConfig:
        """Declare localization parameters and build a validated configuration."""
        self.declare_parameter(
            'imu_yaw_rate_weight',
            DEFAULT_LOCALIZATION_CONFIG.imu_yaw_rate_weight,
        )
        self.declare_parameter(
            'imu_gyro_bias_correction',
            DEFAULT_LOCALIZATION_CONFIG.imu_gyro_bias_correction,
        )
        self.declare_parameter(
            'imu_timeout_s', DEFAULT_LOCALIZATION_CONFIG.imu_timeout_s,
        )
        self.declare_parameter(
            'gps_position_gain',
            DEFAULT_LOCALIZATION_CONFIG.gps_position_gain,
        )
        self.declare_parameter(
            'max_gps_innovation_m',
            DEFAULT_LOCALIZATION_CONFIG.max_gps_innovation_m,
        )
        self.declare_parameter('max_dt', DEFAULT_LOCALIZATION_CONFIG.max_dt)
        self.declare_parameter(
            'reference_latitude_deg',
            DEFAULT_LOCALIZATION_CONFIG.reference_latitude_deg,
        )
        self.declare_parameter(
            'reference_longitude_deg',
            DEFAULT_LOCALIZATION_CONFIG.reference_longitude_deg,
        )
        self.declare_parameter(
            'frame_id', DEFAULT_LOCALIZATION_CONFIG.frame_id,
        )
        self.declare_parameter(
            'child_frame_id', DEFAULT_LOCALIZATION_CONFIG.child_frame_id,
        )
        self.declare_parameter(
            'position_variance',
            DEFAULT_LOCALIZATION_CONFIG.position_variance,
        )
        self.declare_parameter(
            'yaw_variance', DEFAULT_LOCALIZATION_CONFIG.yaw_variance,
        )
        self.declare_parameter(
            'linear_velocity_variance',
            DEFAULT_LOCALIZATION_CONFIG.linear_velocity_variance,
        )
        self.declare_parameter(
            'angular_velocity_variance',
            DEFAULT_LOCALIZATION_CONFIG.angular_velocity_variance,
        )

        return LocalizationConfig(
            imu_yaw_rate_weight=float(
                self.get_parameter('imu_yaw_rate_weight').value
            ),
            imu_gyro_bias_correction=float(
                self.get_parameter('imu_gyro_bias_correction').value
            ),
            imu_timeout_s=float(self.get_parameter('imu_timeout_s').value),
            gps_position_gain=float(
                self.get_parameter('gps_position_gain').value
            ),
            max_gps_innovation_m=float(
                self.get_parameter('max_gps_innovation_m').value
            ),
            max_dt=float(self.get_parameter('max_dt').value),
            reference_latitude_deg=float(
                self.get_parameter('reference_latitude_deg').value
            ),
            reference_longitude_deg=float(
                self.get_parameter('reference_longitude_deg').value
            ),
            frame_id=str(self.get_parameter('frame_id').value),
            child_frame_id=str(self.get_parameter('child_frame_id').value),
            position_variance=float(
                self.get_parameter('position_variance').value
            ),
            yaw_variance=float(self.get_parameter('yaw_variance').value),
            linear_velocity_variance=float(
                self.get_parameter('linear_velocity_variance').value
            ),
            angular_velocity_variance=float(
                self.get_parameter('angular_velocity_variance').value
            ),
        )

    def _handle_wheel_odometry(self, message):
        """Initialize once from the first wheel pose, then predict and publish."""
        stamp = message.header.stamp
        stamp_seconds = stamp_to_seconds(stamp)

        if self._state is None:
            self._initialize_from_wheel_odometry(message, stamp_seconds)
            self._publish_estimate(stamp)
            return

        dt = stamp_seconds - self._last_wheel_stamp_seconds
        self._last_wheel_stamp_seconds = stamp_seconds

        imu_is_usable = is_imu_measurement_usable(
            self._latest_imu_stamp_seconds, stamp_seconds, self._config,
        )

        self._state = predict_state(
            self._state,
            wheel_linear_velocity=message.twist.twist.linear.x,
            wheel_angular_velocity=message.twist.twist.angular.z,
            imu_angular_velocity_z=self._latest_imu_angular_velocity_z,
            imu_is_usable=imu_is_usable,
            dt=dt,
            config=self._config,
        )

        self._publish_estimate(stamp)

    def _initialize_from_wheel_odometry(self, message, stamp_seconds):
        """Seed the fused pose from the first wheel-odometry message."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.z, orientation.w)

        self._state = LocalizationState(x=position.x, y=position.y, yaw=yaw)
        self._last_wheel_stamp_seconds = stamp_seconds

        self.get_logger().info(
            'Initialized the fused pose estimate from the first '
            f'/wheel/odom message at ({position.x:.2f}, {position.y:.2f}, '
            f'{yaw:.2f}).'
        )

    def _handle_imu(self, message):
        """Store the latest IMU yaw rate and timestamp; do not publish here."""
        # Prediction timing is driven entirely by /wheel/odom, so this
        # callback only records state for the next wheel-triggered update.
        self._latest_imu_stamp_seconds = stamp_to_seconds(
            message.header.stamp,
        )
        self._latest_imu_angular_velocity_z = message.angular_velocity.z

    def _handle_gps(self, message):
        """Convert and gate one GPS fix, applying it to position at most once."""
        if self._state is None:
            self.get_logger().debug(
                'Ignoring /gps/fix received before wheel-odometry '
                'initialization.',
                throttle_duration_sec=5.0,
            )
            return

        if message.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().debug(
                'Ignoring /gps/fix with no valid fix (status='
                f'{message.status.status}).',
                throttle_duration_sec=5.0,
            )
            return

        stamp_seconds = stamp_to_seconds(message.header.stamp)
        if not is_gps_measurement_new(
            stamp_seconds, self._last_gps_stamp_seconds,
        ):
            self.get_logger().debug(
                'Ignoring a duplicate or out-of-order /gps/fix (stamp='
                f'{stamp_seconds:.3f}).',
                throttle_duration_sec=5.0,
            )
            return
        self._last_gps_stamp_seconds = stamp_seconds

        gps_east_m, gps_north_m = geographic_to_local(
            message.latitude,
            message.longitude,
            self._config.reference_latitude_deg,
            self._config.reference_longitude_deg,
        )

        result = apply_gps_correction(
            self._state, gps_east_m, gps_north_m, self._config,
        )
        self._state = result.state

        if result.applied:
            self._gps_applied_count += 1
            self.get_logger().debug(
                f'Applied GPS correction #{self._gps_applied_count} '
                f'(innovation={result.innovation_distance:.2f} m).',
                throttle_duration_sec=5.0,
            )
        else:
            self._gps_rejected_count += 1
            self.get_logger().debug(
                f'Rejected GPS correction #{self._gps_rejected_count} '
                f'(innovation={result.innovation_distance:.2f} m, gate='
                f'{self._config.max_gps_innovation_m:.2f} m).',
                throttle_duration_sec=5.0,
            )

    def _publish_estimate(self, stamp):
        """Publish the current fused pose and velocity as Odometry."""
        message = Odometry()

        message.header.stamp = stamp
        message.header.frame_id = self._config.frame_id
        message.child_frame_id = self._config.child_frame_id

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

        message.pose.covariance = build_pose_covariance(self._config)
        message.twist.covariance = build_twist_covariance(self._config)

        self._publisher.publish(message)


def main(args=None):
    """Run the localization node."""
    rclpy.init(args=args)
    node = LocalizationNode()

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
