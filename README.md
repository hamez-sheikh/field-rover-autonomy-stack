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

Day 7 simulated-GPS milestone:

- Pure-Python GPS model (`field_rover_sim/gps_sensor.py`), independent of
  ROS 2, converting local east/north metres into latitude/longitude with a
  small-distance local-plane approximation
- `gps_sensor` node publishing low-rate `sensor_msgs/msg/NavSatFix` fixes
  from `/ground_truth/odom`, independent of wheel odometry, the IMU, and
  range sensing — GPS is a measurement source only, it does not correct or
  fuse with any other estimate
- Configurable constant horizontal bias, reproducible seeded Gaussian
  noise, and reproducible seeded dropouts that skip a scheduled update
  entirely (no fake or repeated fix); a ROS timer publishes independently
  of how often ground truth arrives, at a lower rate than the simulator

Day 8: Multi-sensor localization milestone:

- Pure-Python planar pose estimator
  (`field_rover_localization/pose_estimator.py`), independent of ROS 2,
  fusing wheel odometry, IMU yaw rate, and GPS position into one estimate
- Pure-Python geographic-to-local conversion
  (`field_rover_localization/geographic_conversion.py`), the inverse of
  the Day 7 GPS approximation, reimplemented rather than imported so this
  package never depends on `field_rover_sim`
- `localization` node subscribing to `/wheel/odom`, `/imu/data`, and
  `/gps/fix`, publishing a single fused `nav_msgs/msg/Odometry` on
  `/localization/odom`
- A **complementary-style** estimator, not an EKF: no covariance
  propagation through a process model, only a fixed nominal output
  covariance
- Continues dead-reckoning through GPS dropouts and IMU staleness; never
  hard-resets to a GPS fix and never subscribes to `/ground_truth/odom`

Day 9: Occupancy-grid mapping milestone:

- Pure-Python bounded-evidence occupancy-grid model
  (`field_rover_navigation/occupancy_grid.py`), independent of ROS 2,
  covering grid geometry, integer Bresenham ray traversal, evidence
  updates, and occupancy encoding
- `occupancy_grid_publisher` node subscribing to `/localization/odom` and
  the five `/range/<beam>` topics, publishing `nav_msgs/msg/OccupancyGrid`
  on `/map`
- A **bounded evidence model, not a Bayesian log-odds filter**: each cell's
  integer evidence is nudged by a fixed free/occupied delta and clamped, so
  repeated observations can always overturn stale evidence
- A timer drives mapping so each fresh, not-yet-processed beam sample is
  applied exactly once; the map keeps publishing through a temporary
  sensor or localization dropout, and never subscribes to
  `/ground_truth/odom`
- This is mapping, not SLAM: it consumes the Day 8 pose estimate as given
  and does not estimate or correct the rover's own position; Day 10 adds
  path planning on top of this map

Day 10: A* path planning milestone:

- Pure-Python eight/four-connected A* planner
  (`field_rover_navigation/astar_planner.py`), independent of ROS 2, reusing
  Day 9's `world_to_grid` / `grid_to_world_center` conversion helpers rather
  than duplicating that math
- `astar_planner` node subscribing to `/map`, `/localization/odom`, and
  `/goal_pose`, publishing `nav_msgs/msg/Path` on `/planned_path`
- Eight-connected search by default (`allow_diagonal=true`), Euclidean-cost
  diagonal steps (`sqrt(2)`), an octile heuristic, and corner-cut
  prevention so a diagonal move is only allowed when both orthogonal cells
  it passes between are also traversable
- Unknown cells are blocked by default (`allow_unknown=false`); occupied
  cells (`>= occupied_threshold` = 50) are always blocked
- Publishes an empty `Path` on any planning failure (occupied, unknown, or
  out-of-map start/goal, or no route) and logs a concise reason; the node
  stays alive and accepts the next goal
- This is **path generation only**: it never subscribes to
  `/ground_truth/odom` and never publishes `/cmd_vel`; path following and
  automatic replanning are not implemented yet

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

### Simulated GPS

The `gps_sensor` node converts the rover's true local `(x, y)` position into
a simulated, imperfect geographic fix. It is independent of `wheel_odometry`,
`imu_sensor`, and `range_sensor` — a separate low-rate sensor with its own
bias, noise, and dropouts. **It never corrects or fuses with any other
estimate**; Day 7 produces measurements only.

This is a **local tangent-plane approximation**, not general-purpose geodesy:

- World `x` is treated as east metres and world `y` as north metres, both
  measured from a configurable geographic reference point
  (`reference_latitude_deg`, `reference_longitude_deg`).
- Latitude/longitude use a small-distance spherical-Earth approximation —
  accurate for this ~20 m x 15 m world, not suitable for long-distance
  navigation. No GeographicLib, PROJ, or UTM dependency is used.
- Altitude is always the constant `reference_altitude_m`; vertical rover
  motion is not simulated.

Math, in order:

1. Constant bias and reproducible seeded Gaussian noise are added to the
   true local east/north position, in metres (`position_bias_east_m`,
   `position_bias_north_m`, `position_noise_stddev_m`).
2. `delta_latitude_rad = north_m / EARTH_RADIUS_M`;
   `delta_longitude_rad = east_m / (EARTH_RADIUS_M * cos(reference_latitude))`
   — longitude depends on the reference latitude because a degree of
   longitude covers fewer metres away from the equator.
3. The reference latitude/longitude plus those deltas give the fix.

Subscribes to `/ground_truth/odom` (`nav_msgs/msg/Odometry`) and only stores
the latest position — it does not publish inside that callback. A separate
ROS timer (`publish_rate_hz` = 2.0 Hz by default, versus the simulator's
20 Hz) makes an independent, seeded Bernoulli dropout decision
(`dropout_probability` = 0.10 by default) on every scheduled update; a
dropout skips publication entirely for that tick (no fake or repeated fix).
Noise and dropout draw from two separate seeded `random.Random` streams
(`random_seed` and `random_seed + 1`) so one behaviour can't accidentally
perturb the other. No fix is published before the first `/ground_truth/odom`
message arrives.

Publishes `sensor_msgs/msg/NavSatFix` on `/gps/fix` with
`header.frame_id = "gps_link"`, `status.status = STATUS_FIX`,
`status.service = SERVICE_GPS`, and diagonal
`position_covariance` (`COVARIANCE_TYPE_DIAGONAL_KNOWN`): east/north
variance equal to `position_noise_stddev_m ** 2` (random noise only, not
the constant bias), and a large fixed vertical variance marking the
unmodelled altitude axis as untrustworthy.

Default parameters: `publish_rate_hz` = 2.0, `reference_latitude_deg` =
43.2609, `reference_longitude_deg` = -79.9192, `reference_altitude_m` = 0.0,
`position_bias_east_m` = 0.40 m, `position_bias_north_m` = -0.25 m,
`position_noise_stddev_m` = 1.0 m, `dropout_probability` = 0.10,
`random_seed` = 42. The reference point is just a configurable local origin,
not a claim about where the simulated world physically sits.

### Multi-sensor localization

The `localization` node fuses three independent, imperfect measurements —
`/wheel/odom` (`nav_msgs/msg/Odometry`), `/imu/data` (`sensor_msgs/msg/Imu`),
and `/gps/fix` (`sensor_msgs/msg/NavSatFix`) — into one continuous planar
pose estimate, published as `nav_msgs/msg/Odometry` on `/localization/odom`
with `header.frame_id = "map"` and `child_frame_id = "base_link"`. It never
subscribes to `/ground_truth/odom`; ground truth exists only for evaluating
this estimate afterwards, never as an estimator input.

This is a **complementary-style estimator, not an Extended Kalman Filter**:
there is no state covariance propagated through a process model and no
Kalman gain computed from measurement/process noise. Fusion instead uses
fixed, hand-picked weights and gains, and the published covariance below is
a static nominal description of confidence, not a probabilistically
propagated one.

Prediction versus correction:

- **Prediction** runs on every `/wheel/odom` message and owns the
  estimator's timing — `dt` comes from consecutive wheel-message
  timestamps. Wheel `twist.twist.linear.x` drives forward motion; wheel
  `twist.twist.angular.z` is the baseline turn rate. When a fresh IMU
  sample is available, it is bias-corrected
  (`omega_imu_corrected = omega_imu_raw - imu_gyro_bias_correction`) and
  blended with the wheel turn rate by `imu_yaw_rate_weight`
  (`omega_fused = weight * omega_imu_corrected + (1 - weight) *
  omega_wheel`); a missing, invalid, future-dated, or stale
  (`age > imu_timeout_s`) IMU sample falls back to the wheel rate alone.
  Pose then integrates with a midpoint heading
  (`heading_mid = yaw + delta_yaw / 2`), matching the same integration
  style as `wheel_odometry`.
- **Correction** runs once per accepted `/gps/fix`. Latitude/longitude is
  converted to local east/north metres by
  `field_rover_localization/geographic_conversion.py` — the inverse of the
  Day 7 GPS approximation, reimplemented rather than imported, so this
  package never depends on `field_rover_sim`. The resulting innovation
  (`gps_position - estimated_position`) is rejected outright if its
  distance exceeds `max_gps_innovation_m` (an implausible fix, e.g. a bad
  read or a dropout-adjacent spike); otherwise only a `gps_position_gain`
  fraction of it is applied
  (`estimated_position += gps_position_gain * innovation`). GPS never
  touches yaw or velocity, and a dropped/late/duplicate fix is never
  reapplied — a missing `/gps/fix` update simply leaves prediction running
  on wheel/IMU alone, so a GPS dropout degrades smoothly instead of
  freezing or resetting the estimate.

Initialization: the estimate is seeded once from the very first
`/wheel/odom` message (`x`, `y`, and `yaw` copied directly from its pose;
no integration on that first message), then evolves through prediction and
correction from there. A GPS or IMU message that arrives before that first
wheel pose is safely ignored/stored rather than corrupting an estimate that
does not exist yet — localization does not wait for GPS to begin.

Covariance is a fixed nominal diagonal, not shrinking after a correction or
growing during dead reckoning: `position_variance` for `x`/`y` and
`yaw_variance` for yaw, `linear_velocity_variance` for linear-x and
`angular_velocity_variance` for angular-z, with a large fixed variance for
every unmodelled axis (`z`, roll, pitch, linear-y/z, angular-x/y) to mark
them as untrustworthy placeholders.

Default parameters: `imu_yaw_rate_weight` = 0.70, `imu_gyro_bias_correction`
= 0.01 rad/s, `imu_timeout_s` = 0.20 s, `gps_position_gain` = 0.20,
`max_gps_innovation_m` = 5.0 m, `max_dt` = 0.50 s, `reference_latitude_deg`
= 43.2609, `reference_longitude_deg` = -79.9192 (matching the Day 7 GPS
reference), `position_variance` = 0.50, `yaw_variance` = 0.05,
`linear_velocity_variance` = 0.10, `angular_velocity_variance` = 0.02.

Mapping and path planning are not implemented yet — Day 8 produces a fused
pose estimate only.

### Occupancy-grid mapping

The `occupancy_grid_publisher` node (package `field_rover_navigation`) builds
a persistent 2D map from `/localization/odom` and the five `/range/<beam>`
topics, publishing `nav_msgs/msg/OccupancyGrid` on `/map` with
`header.frame_id = "map"`. It never subscribes to `/ground_truth/odom` and
never imports simulator or localization internals — only standard ROS 2
messages go in. This is **occupancy-grid mapping, not SLAM**: it consumes
Day 8's pose estimate as given and does not estimate or correct the rover's
pose itself; Day 10 adds A* path planning on top of this map.

The mapped world matches the Day 2 static world: `world_width_m` = 20.0 m,
`world_height_m` = 15.0 m, at `resolution_m` = 0.25 m, giving an 80 x 60
(4800-cell) grid anchored at `origin_x_m` = 0.0, `origin_y_m` = 0.0.

This is a **bounded evidence model, not a Bayesian log-odds filter**: each
cell holds an integer evidence value starting at 0 (unknown). Every
processed beam sample nudges the cells it touches by a fixed delta —
`free_evidence_delta` = -1 for a free cell, `occupied_evidence_delta` = +3
for an occupied cell — clamped to `[minimum_evidence, maximum_evidence]` =
`[-5, 5]` so repeated contradictory observations can always overturn stale
evidence instead of it saturating forever. Evidence is encoded into the
published `OccupancyGrid` cell values (`-1` unknown, `0` free, `100`
occupied) by two thresholds: evidence `<= free_threshold` (-1) is free,
evidence `>= occupied_threshold` (+1) is occupied, and anything strictly
between stays unknown.

For each beam, cells are traced from the rover's grid cell to the measured
endpoint with an integer Bresenham line: intermediate cells are marked
free, and the endpoint is marked occupied — unless the reading is at
`max_range` (the range sensor's no-detection value), in which case the
whole beam only marks free space and no endpoint is occupied. The rover's
own cell is always marked free, never occupied, even for a very short
(minimum-range) reading. A measured endpoint that falls outside the mapped
world only frees the in-map portion of the ray; a hit exactly on the map's
outer edge resolves to the last valid cell rather than being discarded. An
invalid reading (`NaN`, infinite, or outside `[min_range, max_range]` by
more than a small floating-point tolerance) is rejected outright and never
touches the grid.

A `map_update_rate_hz` = 5.0 Hz timer drives mapping instead of updating
directly from each subscription callback: on every tick, it applies every
range sample that is both fresh (`age <= range_timeout_s` = 0.5 s) and not
already processed (a strictly newer timestamp than the last one applied to
that beam), but only while the latest `/localization/odom` pose is itself
fresh (`age <= localization_timeout_s` = 0.5 s). One beam can update while
another sits stale or missing entirely. The timer always republishes the
current grid on every tick regardless of whether a fresh update landed, so
the map keeps publishing through a temporary sensor or localization
dropout instead of going silent. The `/map` publisher uses a
transient-local, depth-1 QoS profile so a subscriber that starts late still
receives the latest map.

Ground truth is never read by this node; it exists only to evaluate the
resulting map afterwards, exactly as with Day 8 localization.

Run the full pipeline, including mapping:

```bash
ros2 run field_rover_sim world_simulator
ros2 run field_rover_sim range_sensor
ros2 run field_rover_sim wheel_odometry
ros2 run field_rover_sim imu_sensor
ros2 run field_rover_sim gps_sensor
ros2 run field_rover_localization localization
ros2 run field_rover_navigation occupancy_grid_publisher
```

Override mapping parameters at launch, e.g. to update the map faster and
require stricter evidence before marking a cell occupied:

```bash
ros2 run field_rover_navigation occupancy_grid_publisher --ros-args \
  -p map_update_rate_hz:=10.0 -p occupied_threshold:=3
```

Run all six sensing/localization nodes together:

```bash
ros2 run field_rover_sim world_simulator
ros2 run field_rover_sim range_sensor
ros2 run field_rover_sim wheel_odometry
ros2 run field_rover_sim imu_sensor
ros2 run field_rover_sim gps_sensor
ros2 run field_rover_localization localization
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

Override GPS noise/dropout at launch, e.g. to inspect the ideal fix:

```bash
ros2 run field_rover_sim gps_sensor --ros-args \
  -p position_noise_stddev_m:=0.0 -p dropout_probability:=0.0
```

Override localization fusion parameters at launch, e.g. to trust the IMU
turn rate completely and correct fully to each GPS fix:

```bash
ros2 run field_rover_localization localization --ros-args \
  -p imu_yaw_rate_weight:=1.0 -p gps_position_gain:=1.0
```

Wheel odometry, the IMU, and GPS each still publish an independent,
imperfect measurement; `localization` is the only node in this repository
that combines them into a single fused position estimate.

### A* path planning

The `astar_planner` node (package `field_rover_navigation`) plans a
collision-free grid path from the rover's current fused pose to a
requested goal. It subscribes to `/map` (`nav_msgs/msg/OccupancyGrid`),
`/localization/odom` (`nav_msgs/msg/Odometry`), and `/goal_pose`
(`geometry_msgs/msg/PoseStamped`), and publishes `nav_msgs/msg/Path` on
`/planned_path` with `header.frame_id = "map"`. It never subscribes to
`/ground_truth/odom`, never imports simulator internals, and never
publishes `/cmd_vel` — this is **path generation only**: path following
and automatic replanning are not implemented yet.

All search mathematics lives in a ROS-independent pure module,
`field_rover_navigation/astar_planner.py`. It reuses Day 9's
`world_to_grid` and `grid_to_world_center` conversion helpers (via a small
duck-typed `GridGeometry`) rather than reimplementing that math a second
time, so a world coordinate maps to the same grid cell everywhere in this
package.

By default the planner searches **eight-connected**
(`allow_diagonal=true`): orthogonal steps cost `1.0`, diagonal steps cost
`sqrt(2)`, and the A* heuristic is the matching **octile distance**
(`max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)`), which never overestimates
the true movement cost. Setting `allow_diagonal=false` switches to
four-connected search with a Manhattan heuristic instead. Every diagonal
move is rejected unless both orthogonal cells it passes between are also
traversable (`prevent_corner_cutting=true` by default), so a path can
never squeeze through the corner between two touching blocked cells.

Occupancy is read directly from the incoming `OccupancyGrid` values: a
cell `>= occupied_threshold` (50) is always blocked; a cell equal to `-1`
(unknown) is blocked unless `allow_unknown=true`, in which case it can be
entered at an extra `unknown_traversal_cost` (5.0) on top of the normal
step cost. The default stays conservative (`allow_unknown=false`) — the
planner never silently treats unmapped space as safe. Planning is
rejected outright, with a concise logged reason and an **empty published
`Path`**, whenever the map metadata or data length is invalid, the start
or goal falls outside the map, or the start or goal cell is occupied or
(by default) unknown. A goal inside the rover's current cell succeeds
trivially with a one-cell, zero-cost path. Search is bounded by
`max_expansions` (100000, far above the Day 9 map's 4800 cells) so a
provably unreachable goal fails cleanly instead of searching forever.

The open set is a `heapq` priority queue keyed on `(f_score, tie_break,
g_score, cell)`, where `tie_break` is a monotonically increasing counter —
so two cells with an identical priority always pop in the same order, and
planning the same map/start/goal/configuration repeatedly always returns
the same path and cost. Each published pose is oriented to face the next
point on the path (`atan2` between consecutive world points); the final
pose keeps the heading of the segment that reaches it, or the requested
goal orientation for a single-cell path.

Run the full pipeline, including planning:

```bash
ros2 run field_rover_sim world_simulator
ros2 run field_rover_sim range_sensor
ros2 run field_rover_sim wheel_odometry
ros2 run field_rover_sim imu_sensor
ros2 run field_rover_sim gps_sensor
ros2 run field_rover_localization localization
ros2 run field_rover_navigation occupancy_grid_publisher
ros2 run field_rover_navigation astar_planner
```

Publish a sample goal once the map has observed enough free space around
it:

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 1.0}}}"
```

Override planning parameters at launch, e.g. to require a wider safety
margin from occupied evidence and disable diagonal movement:

```bash
ros2 run field_rover_navigation astar_planner --ros-args \
  -p occupied_threshold:=1 -p allow_diagonal:=false
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
