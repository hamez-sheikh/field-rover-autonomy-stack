// Pure C++ lookahead path-following controller.
//
// Everything in this header is independent of ROS 2: no rclcpp, no ROS
// message types, no ROS time. It only knows about plain points, poses,
// and velocity commands, so it can be unit tested directly and reused by
// any ROS wrapper node without dragging ROS types into the math.
#ifndef FIELD_ROVER_CONTROL__PATH_FOLLOWER_HPP_
#define FIELD_ROVER_CONTROL__PATH_FOLLOWER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace field_rover_control
{

/// A single planar point, in metres.
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// A planar rover pose: position in metres, heading in radians.
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// Tunable behaviour of the path follower. See is_valid_config() for the
/// exact bounds each field must satisfy.
struct PathFollowerConfig
{
  double control_rate_hz = 20.0;
  double lookahead_distance_m = 0.60;
  double goal_tolerance_m = 0.25;
  double max_linear_speed_mps = 0.45;
  double max_angular_speed_radps = 0.80;
  double linear_gain = 0.80;
  double angular_gain = 1.80;
  double turn_in_place_threshold_rad = 1.00;
  double localization_timeout_s = 0.50;
  std::string map_frame = "map";
  std::string base_frame = "base_link";
};

/// The path follower's evolving progress along the current path.
struct PathFollowerState
{
  std::vector<Point2D> path;
  std::size_t current_path_index = 0;
  bool path_active = false;
  bool goal_reached = false;
};

/// A bounded velocity command. Never contains lateral/vertical motion.
struct ControlCommand
{
  double linear_x = 0.0;
  double angular_z = 0.0;
};

/// The outcome of one control step.
struct FollowResult
{
  ControlCommand command;
  bool goal_reached = false;
  /// True when the controller actually computed a tracking command this
  /// step (as opposed to returning a safe stop for an invalid pose or an
  /// inactive/empty path).
  bool valid = false;
};

/// Return true only when every field of config is within a safe range.
bool is_valid_config(const PathFollowerConfig & config);

/// Normalize an angle into the interval [-pi, pi).
double normalize_angle(double angle);

/// Return the shortest signed angular error from current_yaw to
/// desired_heading, normalized into [-pi, pi).
double circular_heading_error(double desired_heading, double current_yaw);

/// Return the straight-line distance between two points.
double euclidean_distance(const Point2D & a, const Point2D & b);

/// Return true when value is neither NaN nor infinite.
bool is_finite_value(double value);

/// Return true when both coordinates of a point are finite.
bool is_finite_point(const Point2D & point);

/// Return true when every field of a pose is finite.
bool is_finite_pose(const Pose2D & pose);

/// Clamp value into the inclusive range [min_value, max_value].
double clamp_value(double value, double min_value, double max_value);

/// Return true when path is non-empty and every point in it is finite.
bool is_path_valid(const std::vector<Point2D> & path);

/// Replace state's path. A valid, non-empty path becomes active with
/// progress reset to the start and the goal-reached flag cleared. An
/// empty or non-finite path instead deactivates state, identically to
/// clear_path().
void reset_path(PathFollowerState & state, const std::vector<Point2D> & new_path);

/// Deactivate state: drop the path, reset progress, and clear
/// goal-reached.
void clear_path(PathFollowerState & state);

/// Return the index of the closest point to position, searching only the
/// suffix of path starting at start_index. Restricting the search to
/// that suffix is what keeps progress monotonic: the returned index is
/// always >= start_index. path must be non-empty.
std::size_t find_nearest_index(
  const std::vector<Point2D> & path,
  const Point2D & position,
  std::size_t start_index);

/// Return the index of the first point at or after nearest_index that is
/// at least lookahead_distance_m from position. Falls back to the final
/// index when no later point is far enough away. path must be non-empty
/// and nearest_index must be a valid index into it.
std::size_t select_lookahead_index(
  const std::vector<Point2D> & path,
  const Point2D & position,
  std::size_t nearest_index,
  double lookahead_distance_m);

/// Compute a bounded velocity command that drives pose toward target,
/// using proportional heading control with heading-error slowdown and a
/// turn-in-place threshold. The result always respects config's velocity
/// limits and never commands reverse motion.
ControlCommand compute_control_command(
  const Pose2D & pose,
  const Point2D & target,
  const PathFollowerConfig & config);

/// Run one control step: update state's progress along its path and
/// return the resulting command.
///
/// Returns a safe zero-velocity command, without advancing progress,
/// whenever pose is non-finite or state has no active/non-empty path.
/// Once the final path point is within config's goal tolerance, state is
/// marked goal-reached and every subsequent call keeps returning a zero
/// command until a new path is set with reset_path().
FollowResult step(
  PathFollowerState & state,
  const Pose2D & pose,
  const PathFollowerConfig & config);

}  // namespace field_rover_control

#endif  // FIELD_ROVER_CONTROL__PATH_FOLLOWER_HPP_
