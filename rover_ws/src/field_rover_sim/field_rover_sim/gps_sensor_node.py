"""ROS 2 node publishing a simulated, low-rate GPS fix with dropouts."""

from field_rover_sim.gps_sensor import (
    calculate_horizontal_variance,
    create_dropout_rng,
    create_noise_rng,
    decide_dropout,
    DEFAULT_GPS_CONFIG,
    generate_gps_measurement,
    GpsConfig,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


# The vertical axis is not simulated (the rover moves in a 2D plane), so its
# covariance uses a large fixed value to mark it as an untrustworthy
# placeholder rather than claiming a precise reading, mirroring the
# unmodelled-axis convention used by the IMU node.
UNMODELLED_VERTICAL_VARIANCE = 1e6


class GpsSensorNode(Node):
    """Publish low-rate NavSatFix fixes derived from ground-truth position."""

    def __init__(self):
        """Declare parameters, build the GPS config, and wire ROS I/O."""
        super().__init__('gps_sensor')

        self._config = self._build_validated_config()
        self._noise_rng = create_noise_rng(self._config)
        self._dropout_rng = create_dropout_rng(self._config)

        self._latest_local_x = None
        self._latest_local_y = None
        self._has_ground_truth = False

        self._fix_count = 0
        self._dropout_count = 0

        self._publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._handle_ground_truth,
            10,
        )

        timer_period = 1.0 / self._config.publish_rate_hz
        self._timer = self.create_timer(timer_period, self._maybe_publish_fix)

        self.get_logger().info(
            'Simulating GPS at '
            f'{self._config.publish_rate_hz:.2f} Hz around reference '
            f'({self._config.reference_latitude_deg:.6f}, '
            f'{self._config.reference_longitude_deg:.6f}, '
            f'{self._config.reference_altitude_m:.2f} m).'
        )
        self.get_logger().info(
            'Bias: east='
            f'{self._config.position_bias_east_m:.2f} m, north='
            f'{self._config.position_bias_north_m:.2f} m. Noise: '
            f'position_noise_stddev={self._config.position_noise_stddev_m:.2f} '
            f'm, dropout_probability={self._config.dropout_probability:.2f}, '
            f'random_seed={self._config.random_seed}.'
        )
        self.get_logger().info(
            'Waiting for the first /ground_truth/odom message before '
            'publishing any GPS fix.'
        )

    def _build_validated_config(self) -> GpsConfig:
        """Declare GPS parameters and build a validated configuration."""
        self.declare_parameter(
            'publish_rate_hz', DEFAULT_GPS_CONFIG.publish_rate_hz,
        )
        self.declare_parameter(
            'reference_latitude_deg',
            DEFAULT_GPS_CONFIG.reference_latitude_deg,
        )
        self.declare_parameter(
            'reference_longitude_deg',
            DEFAULT_GPS_CONFIG.reference_longitude_deg,
        )
        self.declare_parameter(
            'reference_altitude_m', DEFAULT_GPS_CONFIG.reference_altitude_m,
        )
        self.declare_parameter(
            'position_bias_east_m', DEFAULT_GPS_CONFIG.position_bias_east_m,
        )
        self.declare_parameter(
            'position_bias_north_m',
            DEFAULT_GPS_CONFIG.position_bias_north_m,
        )
        self.declare_parameter(
            'position_noise_stddev_m',
            DEFAULT_GPS_CONFIG.position_noise_stddev_m,
        )
        self.declare_parameter(
            'dropout_probability', DEFAULT_GPS_CONFIG.dropout_probability,
        )
        self.declare_parameter('random_seed', DEFAULT_GPS_CONFIG.random_seed)
        self.declare_parameter('frame_id', DEFAULT_GPS_CONFIG.frame_id)

        return GpsConfig(
            publish_rate_hz=float(
                self.get_parameter('publish_rate_hz').value
            ),
            reference_latitude_deg=float(
                self.get_parameter('reference_latitude_deg').value
            ),
            reference_longitude_deg=float(
                self.get_parameter('reference_longitude_deg').value
            ),
            reference_altitude_m=float(
                self.get_parameter('reference_altitude_m').value
            ),
            position_bias_east_m=float(
                self.get_parameter('position_bias_east_m').value
            ),
            position_bias_north_m=float(
                self.get_parameter('position_bias_north_m').value
            ),
            position_noise_stddev_m=float(
                self.get_parameter('position_noise_stddev_m').value
            ),
            dropout_probability=float(
                self.get_parameter('dropout_probability').value
            ),
            random_seed=int(self.get_parameter('random_seed').value),
            frame_id=str(self.get_parameter('frame_id').value),
        )

    def _handle_ground_truth(self, message):
        """Store the latest ground-truth local position; do not publish here."""
        # GPS publishes on its own slower timer, independent of how often
        # ground truth arrives, so this callback only records state.
        position = message.pose.pose.position
        self._latest_local_x = position.x
        self._latest_local_y = position.y

        if not self._has_ground_truth:
            self._has_ground_truth = True
            self.get_logger().info(
                'Received first ground-truth pose at '
                f'({position.x:.2f}, {position.y:.2f}); GPS fixes may now '
                'be published.'
            )

    def _maybe_publish_fix(self):
        """On each timer tick, maybe drop or publish one simulated GPS fix."""
        if not self._has_ground_truth:
            return

        if decide_dropout(self._config, self._dropout_rng):
            self._dropout_count += 1
            self.get_logger().debug(
                f'Dropped a scheduled GPS update ({self._dropout_count} '
                'total dropouts so far).',
                throttle_duration_sec=5.0,
            )
            return

        measurement = generate_gps_measurement(
            self._latest_local_x,
            self._latest_local_y,
            self._config,
            self._noise_rng,
        )
        self._publish_fix(measurement)
        self._fix_count += 1

    def _publish_fix(self, measurement):
        """Populate and publish one sensor_msgs/msg/NavSatFix message."""
        message = NavSatFix()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._config.frame_id

        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS

        message.latitude = measurement.latitude_deg
        message.longitude = measurement.longitude_deg
        message.altitude = measurement.altitude_m

        horizontal_variance = calculate_horizontal_variance(self._config)
        message.position_covariance[0] = horizontal_variance
        message.position_covariance[4] = horizontal_variance
        message.position_covariance[8] = UNMODELLED_VERTICAL_VARIANCE
        message.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        self._publisher.publish(message)


def main(args=None):
    """Run the GPS sensor node."""
    rclpy.init(args=args)
    node = GpsSensorNode()

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
