# Field Rover Autonomy Stack

A software-only ROS 2 Jazzy capstone project for developing and evaluating a simulated field rover autonomy stack.

This project is a progression beyond my completed ROS 2 Autonomous Rover Controller. It will eventually combine 2D rover simulation, noisy sensors, localization, occupancy-grid mapping, custom A* planning, C++ path following, multi-waypoint missions, replanning, safety supervision, fault injection, visualization, testing, and quantitative evaluation.

## Current Status

July 19 foundation milestone:

- Project scope defined before implementation
- ROS 2 Jazzy development-container configuration added
- ROS 2 workspace created
- Five responsibility-based packages scaffolded
- No rover behavior or autonomy nodes implemented yet

See [`docs/PROJECT_SPEC.md`](docs/PROJECT_SPEC.md) for the complete planned scope.

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