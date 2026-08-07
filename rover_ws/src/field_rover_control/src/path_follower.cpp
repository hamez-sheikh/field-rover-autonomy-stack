#include "field_rover_control/path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace field_rover_control
{

namespace
{
bool is_positive_finite(double value)
{
  return is_finite_value(value) && value > 0.0;
}
}  // namespace

bool is_valid_config(const PathFollowerConfig & config)
{
  if (!is_positive_finite(config.control_rate_hz)) {
    return false;
  }
  if (!is_positive_finite(config.lookahead_distance_m)) {
    return false;
  }
  if (!is_positive_finite(config.goal_tolerance_m)) {
    return false;
  }
  if (!is_positive_finite(config.max_linear_speed_mps)) {
    return false;
  }
  if (!is_positive_finite(config.max_angular_speed_radps)) {
    return false;
  }
  if (!is_positive_finite(config.linear_gain)) {
    return false;
  }
  if (!is_positive_finite(config.angular_gain)) {
    return false;
  }
  if (!is_positive_finite(config.localization_timeout_s)) {
    return false;
  }
  if (!is_finite_value(config.turn_in_place_threshold_rad) ||
    config.turn_in_place_threshold_rad <= 0.0 ||
    config.turn_in_place_threshold_rad > M_PI)
  {
    return false;
  }
  if (config.map_frame.empty() || config.base_frame.empty()) {
    return false;
  }
  return true;
}

double normalize_angle(double angle)
{
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

double circular_heading_error(double desired_heading, double current_yaw)
{
  return normalize_angle(desired_heading - current_yaw);
}

double euclidean_distance(const Point2D & a, const Point2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool is_finite_value(double value)
{
  return std::isfinite(value);
}

bool is_finite_point(const Point2D & point)
{
  return is_finite_value(point.x) && is_finite_value(point.y);
}

bool is_finite_pose(const Pose2D & pose)
{
  return is_finite_value(pose.x) && is_finite_value(pose.y) && is_finite_value(pose.yaw);
}

double clamp_value(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

bool is_path_valid(const std::vector<Point2D> & path)
{
  if (path.empty()) {
    return false;
  }
  for (const auto & point : path) {
    if (!is_finite_point(point)) {
      return false;
    }
  }
  return true;
}

void clear_path(PathFollowerState & state)
{
  state.path.clear();
  state.current_path_index = 0;
  state.path_active = false;
  state.goal_reached = false;
}

void reset_path(PathFollowerState & state, const std::vector<Point2D> & new_path)
{
  if (!is_path_valid(new_path)) {
    clear_path(state);
    return;
  }
  state.path = new_path;
  state.current_path_index = 0;
  state.path_active = true;
  state.goal_reached = false;
}

std::size_t find_nearest_index(
  const std::vector<Point2D> & path,
  const Point2D & position,
  std::size_t start_index)
{
  const std::size_t clamped_start = std::min(start_index, path.size() - 1);

  std::size_t best_index = clamped_start;
  double best_distance = euclidean_distance(position, path[clamped_start]);

  for (std::size_t i = clamped_start + 1; i < path.size(); ++i) {
    const double distance = euclidean_distance(position, path[i]);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  return best_index;
}

std::size_t select_lookahead_index(
  const std::vector<Point2D> & path,
  const Point2D & position,
  std::size_t nearest_index,
  double lookahead_distance_m)
{
  const std::size_t clamped_nearest = std::min(nearest_index, path.size() - 1);

  for (std::size_t i = clamped_nearest; i < path.size(); ++i) {
    if (euclidean_distance(position, path[i]) >= lookahead_distance_m) {
      return i;
    }
  }

  return path.size() - 1;
}

ControlCommand compute_control_command(
  const Pose2D & pose,
  const Point2D & target,
  const PathFollowerConfig & config)
{
  const double desired_heading = std::atan2(target.y - pose.y, target.x - pose.x);
  const double error = circular_heading_error(desired_heading, pose.yaw);

  const double angular = clamp_value(
    config.angular_gain * error,
    -config.max_angular_speed_radps,
    config.max_angular_speed_radps);

  const double distance = euclidean_distance(Point2D{pose.x, pose.y}, target);
  double linear = std::min(config.max_linear_speed_mps, config.linear_gain * distance);

  const double heading_scale = std::max(0.0, std::cos(error));
  linear *= heading_scale;

  if (std::abs(error) >= config.turn_in_place_threshold_rad) {
    linear = 0.0;
  }

  return ControlCommand{linear, angular};
}

FollowResult step(
  PathFollowerState & state,
  const Pose2D & pose,
  const PathFollowerConfig & config)
{
  if (!is_finite_pose(pose)) {
    return FollowResult{ControlCommand{}, state.goal_reached, false};
  }
  if (!state.path_active || state.path.empty()) {
    return FollowResult{ControlCommand{}, false, false};
  }
  if (state.goal_reached) {
    return FollowResult{ControlCommand{}, true, true};
  }

  const Point2D position{pose.x, pose.y};

  const std::size_t nearest_index = find_nearest_index(
    state.path, position, state.current_path_index);
  state.current_path_index = nearest_index;

  const Point2D & final_point = state.path.back();
  const double distance_to_goal = euclidean_distance(position, final_point);
  if (distance_to_goal <= config.goal_tolerance_m) {
    state.goal_reached = true;
    return FollowResult{ControlCommand{}, true, true};
  }

  const std::size_t target_index = select_lookahead_index(
    state.path, position, nearest_index, config.lookahead_distance_m);
  const Point2D & target = state.path[target_index];

  const ControlCommand command = compute_control_command(pose, target, config);
  return FollowResult{command, false, true};
}

}  // namespace field_rover_control
