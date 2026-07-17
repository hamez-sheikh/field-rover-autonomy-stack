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

Run both nodes together:

```bash
ros2 run field_rover_sim world_simulator
ros2 run field_rover_sim range_sensor
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
