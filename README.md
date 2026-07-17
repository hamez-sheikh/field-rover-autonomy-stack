# Field Rover Autonomy Stack

A software-only ROS 2 Jazzy capstone project for developing and evaluating a simulated field rover autonomy stack.

This project is a progression beyond my completed ROS 2 Autonomous Rover Controller. It will eventually combine 2D rover simulation, noisy sensors, localization, occupancy-grid mapping, custom A* planning, C++ path following, multi-waypoint missions, replanning, safety supervision, fault injection, visualization, testing, and quantitative evaluation.

## Current Status

Day 1 foundation milestone:

- Project scope defined before implementation
- ROS 2 Jazzy development-container configuration added
- ROS 2 workspace created
- Five responsibility-based packages scaffolded

Day 2 static-world milestone:

- Static 2D world model (20.0 m x 15.0 m) with three circular obstacles
- Pure-Python obstacle and boundary collision checks
- `world_simulator` node publishing ground-truth `/ground_truth/odom`

Day 3 rover-motion milestone:

- Constrained unicycle motion with speed, acceleration, and turn-rate limits
- `/cmd_vel` (`geometry_msgs/msg/Twist`) drives the simulated rover
- Collision-safe pose integration (blocked translation still allows rotation)

Day 4 directional range-sensing milestone:

- Pure-Python ray-casting geometry (`field_rover_sim/range_sensor.py`) for
  ray-circle and ray-boundary intersection, independent of ROS 2
- `range_sensor` node publishing five fixed beams derived from
  `/ground_truth/odom`
- Ideal, noiseless distance readings; no wheel/IMU/GPS sensing yet

Day 5 wheel-odometry milestone:

- Pure-Python differential-drive wheel-odometry model
  (`field_rover_sim/wheel_odometry.py`), independent of ROS 2
- `wheel_odometry` node dead-reckoning an imperfect pose estimate from
  simulated left/right wheel travel
- Configurable per-wheel calibration scale factors that produce gradual,
  cumulative drift away from ground truth — the first intentionally
  imperfect position estimate in the project

Day 6 simulated-IMU milestone:

- Pure-Python planar IMU model (`field_rover_sim/imu_sensor.py`),
  independent of ROS 2
- `imu_sensor` node deriving gyroscope and accelerometer readings from
  `/ground_truth/odom` velocity, independent of wheel odometry and range
  sensing
- World-frame velocity differentiation, rotated into the rover body frame,
  to produce ideal forward/lateral acceleration (including lateral
  acceleration while turning)
- Configurable deterministic gyroscope/accelerometer bias and reproducible
  seeded Gaussian noise; orientation is explicitly marked unavailable
  (`orientation_covariance[0] = -1.0`) since this milestone measures motion,
  it does not estimate heading

See [`docs/PROJECT_SPEC.md`](docs/PROJECT_SPEC.md) for the complete planned scope.

### Directional range sensor

The `range_sensor` node casts five fixed beams from the rover centre, rotating
with rover yaw, and reports the distance to the nearest circular obstacle or
world boundary along each beam.

| Beam name         | Relative angle |
|--------------------|---------------:|
| `front_far_right`  | -60 deg        |
| `front_right`      | -30 deg        |
| `front`            |   0 deg        |
| `front_left`       | +30 deg        |
| `front_far_left`   | +60 deg        |

- Subscribes to `/ground_truth/odom` (`nav_msgs/msg/Odometry`) and recovers
  yaw from the orientation quaternion.
- Publishes one `sensor_msgs/msg/Range` message per beam on
  `/range/<beam_name>` (radiation type `INFRARED`), at 10 Hz by default.
- `min_range` = 0.1 m, `max_range` = 8.0 m, `field_of_view` = 0.05 rad,
  each declared as a ROS 2 parameter and validated at startup.
- No detection within `max_range` reports `max_range` (not infinity); a hit
  closer than `min_range` saturates to `min_range`.
- The ray origin is the rover centre, not the footprint edge — a
  deliberate simplification with no sensor-mounting offset yet.
- Measurements are ideal and noiseless, computed directly from ground
  truth; noise, drift, and sensor failures are not implemented.

### Wheel odometry

The `wheel_odometry` node dead-reckons an independent, imperfect pose estimate
by simulating differential-drive wheel travel from the true rover velocity,
instead of republishing ground truth with an offset.

- Subscribes to `/ground_truth/odom` (`nav_msgs/msg/Odometry`) and uses
  `twist.twist.linear.x` / `twist.twist.angular.z` as the ideal rover
  velocity that drives the simulated left/right wheels.
- Converts rover velocity to wheel velocities with
  `v_left = v - omega * L / 2`, `v_right = v + omega * L / 2`, where `L` is
  `wheel_track_width`.
- Applies configurable `left_wheel_scale` / `right_wheel_scale` calibration
  factors to each wheel's true distance increment before reconstructing
  centre travel and heading change, then integrates pose with a midpoint
  heading (`heading_mid = yaw + delta_yaw / 2`) for better accuracy through
  curves.
- Publishes `nav_msgs/msg/Odometry` on `/wheel/odom` with
  `header.frame_id = "odom"` and `child_frame_id = "base_link"`.
- Initializes its pose estimate once from the first `/ground_truth/odom`
  message, then evolves entirely through dead reckoning — it never corrects
  itself against ground truth again, so calibration error accumulates.
- Default parameters: `wheel_track_width` = 0.6 m, `left_wheel_scale` = 1.01,
  `right_wheel_scale` = 0.99. The left wheel over-reports distance and the
  right wheel under-reports it, so straight driving gradually drifts in
  heading and, through that wrong heading, in position.
- No IMU, GPS, random noise, or sensor fusion yet — only deterministic
  wheel-calibration drift.

### Simulated IMU

The `imu_sensor` node derives simulated gyroscope and accelerometer readings
from `/ground_truth/odom` instead of publishing perfect ground truth. It is
independent of `wheel_odometry` and `range_sensor` — a separate sensor with
its own imperfections, parameters, and tests.

This is a **planar simplification**, not a full 6-DOF physical IMU:

- Only yaw rate (rotation about the vertical axis) is modelled; roll and
  pitch rate are always zero.
- Only forward (body x) and lateral (body y) acceleration are modelled;
  vertical (body z) acceleration is always zero.
- Gravity is excluded — a stationary rover reads approximately zero
  acceleration (plus configured bias/noise), not ~9.81 m/s^2 upward.
- No orientation estimate is produced. `orientation_covariance[0]` is set to
  `-1.0` (the `sensor_msgs/Imu` convention for "not populated") and the
  orientation quaternion is the identity placeholder — ground-truth yaw is
  never published as if it were an IMU estimate.

Math, in order:

1. World-frame velocity from body-forward speed and yaw:
   `vx = v * cos(yaw)`, `vy = v * sin(yaw)`.
2. Numerical differentiation of world-frame velocity between consecutive
   samples gives world-frame acceleration.
3. That acceleration is rotated into the rover body frame using yaw, giving
   ideal forward (x) and lateral (y) acceleration — this is what makes
   constant-speed turning produce lateral acceleration (`a_y ≈ v * omega`),
   not just straight-line speed changes.
4. Configured constant bias and reproducible seeded Gaussian noise
   (`random.Random(random_seed)`, never the global `random` module) are
   added to both the gyroscope and accelerometer readings.

Default parameters: `gyro_bias_z` = 0.01 rad/s, `accel_bias_x` = 0.03 m/s^2,
`accel_bias_y` = -0.02 m/s^2, `gyro_noise_stddev` = 0.005 rad/s,
`accel_noise_stddev` = 0.02 m/s^2, `random_seed` = 42, `max_dt` = 0.5 s.

Covariance is populated from configured noise variance
(`stddev ** 2`) on the diagonal — it represents random measurement noise
only, not the constant bias offset and not any fused/estimation
uncertainty. Unmodelled axes (roll/pitch rate, vertical acceleration) use a
large fixed variance to mark them as untrustworthy placeholders.

The first `/ground_truth/odom` sample, and any sample with a non-positive or
excessive (`> max_dt`) time step, reports zero ideal acceleration instead of
a false spike from an unreliable time difference; the gyroscope reading
needs no history and is unaffected.

Publishes `sensor_msgs/msg/Imu` on `/imu/data` with `header.frame_id =
"imu_link"`. No GPS or sensor fusion is implemented yet — Day 6 produces
measurements only; it does not estimate pose, heading, or velocity.

Run all four nodes together:

```bash
ros2 run field_rover_sim world_simulator
ros2 run field_rover_sim range_sensor
ros2 run field_rover_sim wheel_odometry
ros2 run field_rover_sim imu_sensor
```

Override calibration at launch, e.g. to compare against perfect wheels:

```bash
ros2 run field_rover_sim wheel_odometry --ros-args \
  -p left_wheel_scale:=1.0 -p right_wheel_scale:=1.0
```

Override IMU bias/noise at launch, e.g. to inspect the noiseless ideal signal:

```bash
ros2 run field_rover_sim imu_sensor --ros-args \
  -p gyro_noise_stddev:=0.0 -p accel_noise_stddev:=0.0
```

## Package Structure

- `field_rover_sim` — simulation, sensor generation, and visualization
- `field_rover_localization` — heading and position estimation
- `field_rover_navigation` — mapping, planning, missions, replanning, and safety
- `field_rover_control` — C++ path following and motion-command limits
- `field_rover_bringup` — launch and configuration resources

## Development Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- C++ with `ament_cmake`
- `colcon`
- GitHub Codespaces

## Build

From the repository root:

```bash
cd rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
