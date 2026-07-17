"""ROS 2 node publishing a simulated planar IMU with bias and noise."""

from field_rover_sim.imu_sensor import (
    calculate_accel_variance,
    calculate_gyro_variance,
    create_rng,
    DEFAULT_IMU_CONFIG,
    generate_imu_measurement,
    ImuConfig,
)
from field_rover_sim.range_sensor_node import quaternion_to_yaw
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


DEFAULT_FRAME_ID = 'imu_link'

# orientation_covariance[0] == -1.0 is the sensor_msgs/Imu convention for
# "this field is not populated" (REP 145 / message comment). This sensor
# simulates gyroscope and accelerometer measurements only; it does not
# estimate orientation, so the identity quaternion below is a placeholder,
# never a fused or ground-truth heading.
ORIENTATION_UNAVAILABLE = -1.0
IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)

# angular_velocity_covariance / linear_acceleration_covariance report the
# configured Gaussian measurement-noise variance on their diagonal. They
# describe random noise spread only, not the constant bias offset, and not
# any fused/estimation uncertainty. The unmodelled axes (roll/pitch rate,
# vertical acceleration) use a large fixed variance to mark them as
# untrustworthy placeholders rather than claiming a precise zero reading.
UNMODELLED_AXIS_VARIANCE = 1e6


class ImuSensorNode(Node):
    """Derive simulated gyroscope/accelerometer readings from ground truth."""

    def __init__(self):
        """Declare parameters, build the IMU config, and wire ROS I/O."""
        super().__init__('imu_sensor')

        self._config = self._build_validated_config()
        self._rng = create_rng(self._config)
        self._frame_id = str(
            self.declare_parameter('frame_id', DEFAULT_FRAME_ID).value
        )
        if not self._frame_id:
            raise ValueError('frame_id must be non-empty.')

        self._state = None
        self._last_stamp_seconds = None

        self._publisher = self.create_publisher(Imu, '/imu/data', 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._handle_ground_truth,
            10,
        )

        self.get_logger().info(
            'Simulating IMU with gyro_bias_z='
            f'{self._config.gyro_bias_z:.4f} rad/s, accel_bias_x='
            f'{self._config.accel_bias_x:.4f} m/s^2, accel_bias_y='
            f'{self._config.accel_bias_y:.4f} m/s^2.'
        )
        self.get_logger().info(
            'Noise: gyro_noise_stddev='
            f'{self._config.gyro_noise_stddev:.4f} rad/s, '
            f'accel_noise_stddev={self._config.accel_noise_stddev:.4f} '
            f'm/s^2, random_seed={self._config.random_seed}, '
            f'max_dt={self._config.max_dt:.2f} s.'
        )
        self.get_logger().info(
            'Deriving world-frame acceleration from /ground_truth/odom '
            'velocity and rotating it into the body frame; orientation is '
            'published as unavailable.'
        )

    def _build_validated_config(self) -> ImuConfig:
        """Declare IMU parameters and build a validated configuration."""
        self.declare_parameter('gyro_bias_z', DEFAULT_IMU_CONFIG.gyro_bias_z)
        self.declare_parameter('accel_bias_x', DEFAULT_IMU_CONFIG.accel_bias_x)
        self.declare_parameter('accel_bias_y', DEFAULT_IMU_CONFIG.accel_bias_y)
        self.declare_parameter(
            'gyro_noise_stddev', DEFAULT_IMU_CONFIG.gyro_noise_stddev,
        )
        self.declare_parameter(
            'accel_noise_stddev', DEFAULT_IMU_CONFIG.accel_noise_stddev,
        )
        self.declare_parameter('random_seed', DEFAULT_IMU_CONFIG.random_seed)
        self.declare_parameter('max_dt', DEFAULT_IMU_CONFIG.max_dt)

        return ImuConfig(
            gyro_bias_z=float(self.get_parameter('gyro_bias_z').value),
            accel_bias_x=float(self.get_parameter('accel_bias_x').value),
            accel_bias_y=float(self.get_parameter('accel_bias_y').value),
            gyro_noise_stddev=float(
                self.get_parameter('gyro_noise_stddev').value
            ),
            accel_noise_stddev=float(
                self.get_parameter('accel_noise_stddev').value
            ),
            random_seed=int(self.get_parameter('random_seed').value),
            max_dt=float(self.get_parameter('max_dt').value),
        )

    def _handle_ground_truth(self, message):
        """Compute dt, derive one IMU sample, and publish it."""
        stamp = message.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec / 1_000_000_000.0

        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.z, orientation.w)
        forward_speed = message.twist.twist.linear.x
        yaw_rate = message.twist.twist.angular.z

        if self._last_stamp_seconds is None:
            dt = 0.0
        else:
            dt = stamp_seconds - self._last_stamp_seconds
        self._last_stamp_seconds = stamp_seconds

        measurement, self._state = generate_imu_measurement(
            previous_state=self._state,
            yaw=yaw,
            forward_speed=forward_speed,
            yaw_rate=yaw_rate,
            dt=dt,
            config=self._config,
            rng=self._rng,
        )

        self._publish_measurement(stamp, measurement)

    def _publish_measurement(self, stamp, measurement):
        """Populate and publish one sensor_msgs/msg/Imu message."""
        message = Imu()

        message.header.stamp = stamp
        message.header.frame_id = self._frame_id

        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = IDENTITY_QUATERNION
        message.orientation_covariance[0] = ORIENTATION_UNAVAILABLE

        message.angular_velocity.x = 0.0
        message.angular_velocity.y = 0.0
        message.angular_velocity.z = measurement.angular_velocity_z

        gyro_variance = calculate_gyro_variance(self._config)
        message.angular_velocity_covariance[0] = UNMODELLED_AXIS_VARIANCE
        message.angular_velocity_covariance[4] = UNMODELLED_AXIS_VARIANCE
        message.angular_velocity_covariance[8] = gyro_variance

        message.linear_acceleration.x = measurement.linear_acceleration_x
        message.linear_acceleration.y = measurement.linear_acceleration_y
        message.linear_acceleration.z = 0.0

        accel_variance = calculate_accel_variance(self._config)
        message.linear_acceleration_covariance[0] = accel_variance
        message.linear_acceleration_covariance[4] = accel_variance
        message.linear_acceleration_covariance[8] = UNMODELLED_AXIS_VARIANCE

        self._publisher.publish(message)


def main(args=None):
    """Run the IMU sensor node."""
    rclpy.init(args=args)
    node = ImuSensorNode()

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
