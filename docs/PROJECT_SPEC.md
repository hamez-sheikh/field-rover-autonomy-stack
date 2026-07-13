# Field Rover Autonomy Stack — Project Specification

## Problem Statement

This project will build a software-only ROS 2 autonomy stack for a simulated field rover. The rover must estimate its position from imperfect sensors, plan collision-free routes, follow those routes, complete multi-waypoint missions, replan when conditions change, and react safely to injected faults. It is a progression beyond Rover Controller v1, which demonstrated basic reactive obstacle avoidance and stale-sensor safety.

## Main Learning Goals

- Design a multi-package ROS 2 system with clear responsibilities and interfaces.
- Simulate rover motion, obstacles, collisions, and noisy sensors.
- Implement understandable 2D localization and sensor fusion by hand.
- Implement custom A* path planning and mission-level replanning.
- Write a path-following controller in C++.
- Test, measure, record, visualize, and explain autonomous behavior.

## Must-Finish Scope

The completed core system will include a simulated 2D rover and environment; noisy wheel odometry; IMU, GPS, and range sensing; GPS dropouts; one hand-written fused position estimate; an occupancy grid; custom A* planning; a C++ path follower; multiple mission waypoints; replanning when a route becomes blocked; at least two injected faults with different safety responses; a Matplotlib visualizer; basic automated tests; at least one rosbag recording; and quantitative evaluation over approximately five to ten trials.

## Stretch Scope

Stretch work includes comparing the hand-written localizer with `robot_localization`, supporting four fault types, running twenty evaluation trials, adding GitHub Actions, and experimenting with a small vision module. Stretch items will begin only after the core system is reliable and documented.

## Non-Goals

This summer phase will not use Gazebo, RViz2, reinforcement learning, PyTorch, YOLO, Nav2, real rover hardware, or a second unrelated project. Full production-level `tf2` integration is also outside the required scope.

## Planned ROS 2 Architecture

- `field_rover_sim` (`ament_python`): `rover_simulator` for the world, rover dynamics, obstacles, collisions, ground truth, and simulated sensors; `rover_visualizer` for Matplotlib display.
- `field_rover_localization` (`ament_python`): `rover_localizer` for heading and position fusion.
- `field_rover_navigation` (`ament_python`): `occupancy_grid_publisher`, `astar_planner`, `mission_manager`, and `safety_supervisor`.
- `field_rover_control` (`ament_cmake`, C++): `path_follower` for path tracking, command filtering, speed limits, and turn-rate limits.
- `field_rover_bringup` (`ament_cmake`): launch files and configuration files; no behavior node of its own.

## Planned Message Interfaces

The primary standard messages will be:

- `nav_msgs/msg/Odometry`
- `sensor_msgs/msg/Imu`
- `sensor_msgs/msg/NavSatFix`
- `sensor_msgs/msg/Range`
- `nav_msgs/msg/OccupancyGrid`
- `nav_msgs/msg/Path`
- `geometry_msgs/msg/Twist`

Custom messages will be avoided unless a clear requirement appears later.

## Coordinate Approach

The first implementation will use a two-dimensional world with position `(x, y)` in metres and heading `yaw` in radians. Positive yaw will rotate counter-clockwise. Conceptual `map` and `base_link` frames will be represented through message frame IDs and clear pose calculations. GPS will be converted into a local Cartesian approximation. Core calculations will use understandable trigonometry in project code; actual `tf2` broadcasting remains optional.

## Safety and Fault Tolerance

The rover must default to a safe stop when command, localization, or critical sensor data becomes stale. A temporary GPS dropout should trigger degraded dead-reckoning behavior rather than an immediate stop, while a critical stale-data or collision-risk condition should stop motion. Fault injection must be repeatable and visible in logs and the visualizer.

## Testing and Measurement

Tests will cover important calculations and safety transitions. Evaluation will measure average localization position error, waypoint completion, navigation success rate, planning time, replanning events, and safety response. At least one representative run will be recorded with rosbag.

## Final Success Criteria

The project succeeds when a documented ROS 2 launch starts the complete simulated system; the rover localizes itself, plans and follows routes through multiple waypoints, replans around a newly blocked route, and responds correctly to at least two faults. The repository must include tests, measurements, a rosbag recording, setup instructions, architecture documentation, and a clear demonstration.