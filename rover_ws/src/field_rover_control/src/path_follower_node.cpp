#include "field_rover_control/path_follower_node.hpp"

#include <cmath>
#include <stdexcept>

namespace field_rover_control
{

namespace
{
/// Recover the yaw angle from a planar (z, w) orientation quaternion, the
/// same one-line convention used by the Python nodes in this project.
double quaternion_to_yaw(double orientation_z, double orientation_w)
{
  return 2.0 * std::atan2(orientation_z, orientation_w);
}
}  // namespace

PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions & options)
: Node("path_follower", options)
{
  config_ = build_validated_config();

  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  path_subscription_ = create_subscription<nav_msgs::msg::Path>(
    "/planned_path", 10,
    std::bind(&PathFollowerNode::handle_path, this, std::placeholders::_1));

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odom", 10,
    std::bind(&PathFollowerNode::handle_localization, this, std::placeholders::_1));

  const auto timer_period = std::chrono::duration<double>(1.0 / config_.control_rate_hz);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&PathFollowerNode::control_step, this));

  RCLCPP_INFO(
    get_logger(),
    "Following /planned_path (frame '%s') using /localization/odom, publishing "
    "/cmd_vel at %.1f Hz. lookahead_distance_m=%.2f, goal_tolerance_m=%.2f.",
    config_.map_frame.c_str(), config_.control_rate_hz,
    config_.lookahead_distance_m, config_.goal_tolerance_m);
  RCLCPP_INFO(
    get_logger(),
    "This is path following only: it never subscribes to /ground_truth/odom "
    "and never plans or replans a path.");
}

std::vector<std::string> PathFollowerNode::subscribed_topic_names() const
{
  return {path_subscription_->get_topic_name(), odom_subscription_->get_topic_name()};
}

std::string PathFollowerNode::published_cmd_vel_topic() const
{
  return cmd_vel_publisher_->get_topic_name();
}

PathFollowerConfig PathFollowerNode::build_validated_config()
{
  PathFollowerConfig defaults;

  declare_parameter("control_rate_hz", defaults.control_rate_hz);
  declare_parameter("lookahead_distance_m", defaults.lookahead_distance_m);
  declare_parameter("goal_tolerance_m", defaults.goal_tolerance_m);
  declare_parameter("max_linear_speed_mps", defaults.max_linear_speed_mps);
  declare_parameter("max_angular_speed_radps", defaults.max_angular_speed_radps);
  declare_parameter("linear_gain", defaults.linear_gain);
  declare_parameter("angular_gain", defaults.angular_gain);
  declare_parameter("turn_in_place_threshold_rad", defaults.turn_in_place_threshold_rad);
  declare_parameter("localization_timeout_s", defaults.localization_timeout_s);
  declare_parameter("map_frame", defaults.map_frame);
  declare_parameter("base_frame", defaults.base_frame);

  PathFollowerConfig config;
  config.control_rate_hz = get_parameter("control_rate_hz").as_double();
  config.lookahead_distance_m = get_parameter("lookahead_distance_m").as_double();
  config.goal_tolerance_m = get_parameter("goal_tolerance_m").as_double();
  config.max_linear_speed_mps = get_parameter("max_linear_speed_mps").as_double();
  config.max_angular_speed_radps = get_parameter("max_angular_speed_radps").as_double();
  config.linear_gain = get_parameter("linear_gain").as_double();
  config.angular_gain = get_parameter("angular_gain").as_double();
  config.turn_in_place_threshold_rad = get_parameter("turn_in_place_threshold_rad").as_double();
  config.localization_timeout_s = get_parameter("localization_timeout_s").as_double();
  config.map_frame = get_parameter("map_frame").as_string();
  config.base_frame = get_parameter("base_frame").as_string();

  if (!is_valid_config(config)) {
    RCLCPP_FATAL(get_logger(), "Refusing to start with an invalid path_follower configuration.");
    throw std::invalid_argument("Invalid PathFollowerConfig.");
  }

  return config;
}

void PathFollowerNode::handle_path(const nav_msgs::msg::Path::SharedPtr message)
{
  if (!message->poses.empty() && message->header.frame_id != config_.map_frame) {
    RCLCPP_WARN(
      get_logger(), "Ignoring /planned_path in frame '%s', expected '%s'.",
      message->header.frame_id.c_str(), config_.map_frame.c_str());
    clear_path(state_);
    return;
  }

  std::vector<Point2D> points;
  points.reserve(message->poses.size());
  for (const auto & pose : message->poses) {
    points.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
  }

  if (!is_path_valid(points)) {
    RCLCPP_INFO(get_logger(), "Received an empty or invalid path; stopping.");
    clear_path(state_);
    return;
  }

  reset_path(state_, points);
  RCLCPP_INFO(get_logger(), "Following a new %zu-point path.", points.size());
}

void PathFollowerNode::handle_localization(const nav_msgs::msg::Odometry::SharedPtr message)
{
  const auto & position = message->pose.pose.position;
  const auto & orientation = message->pose.pose.orientation;

  // A degenerate (z, w) quaternion carries no usable heading; ignore the
  // sample rather than feeding atan2(0, 0) into the controller.
  if (!is_finite_value(orientation.z) || !is_finite_value(orientation.w) ||
    (orientation.z == 0.0 && orientation.w == 0.0))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring /localization/odom with an invalid orientation quaternion.");
    return;
  }

  const Pose2D pose{position.x, position.y, quaternion_to_yaw(orientation.z, orientation.w)};

  if (!is_finite_pose(pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring /localization/odom with a non-finite pose.");
    return;
  }

  latest_pose_ = pose;
  localization_received_ = true;

  // Both branches must share the node clock's type, or later subtracting
  // this stamp from get_clock()->now() throws (rclcpp refuses to mix
  // clock sources). header.stamp of exactly zero means the publisher
  // never stamped the message; fall back to this node's receipt time so
  // freshness can still be judged instead of always reading as infinitely
  // stale.
  const auto & stamp = message->header.stamp;
  latest_localization_stamp_ = (stamp.sec == 0 && stamp.nanosec == 0) ?
    get_clock()->now() :
    rclcpp::Time(stamp, get_clock()->get_clock_type());
}

bool PathFollowerNode::is_localization_usable() const
{
  if (!localization_received_) {
    return false;
  }
  const double age_s = (get_clock()->now() - latest_localization_stamp_).seconds();
  return age_s >= 0.0 && age_s <= config_.localization_timeout_s;
}

void PathFollowerNode::control_step()
{
  geometry_msgs::msg::Twist twist;  // Defaults to all-zero.

  if (is_localization_usable()) {
    const FollowResult result = step(state_, latest_pose_, config_);
    twist.linear.x = clamp_value(result.command.linear_x, 0.0, config_.max_linear_speed_mps);
    twist.angular.z = clamp_value(
      result.command.angular_z,
      -config_.max_angular_speed_radps,
      config_.max_angular_speed_radps);
  }

  cmd_vel_publisher_->publish(twist);
}

}  // namespace field_rover_control
