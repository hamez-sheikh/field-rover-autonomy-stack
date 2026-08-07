// ROS 2 wrapper around the pure field_rover_control::step() controller.
//
// This node only translates ROS messages into the plain Point2D/Pose2D
// types the pure library understands, tracks localization freshness, and
// publishes the resulting command. All path-following math lives in
// path_follower.cpp/.hpp, independent of ROS. The class is declared here
// (rather than only in a .cpp) so tests can construct it directly, the
// same way the executable's main() does.
#ifndef FIELD_ROVER_CONTROL__PATH_FOLLOWER_NODE_HPP_
#define FIELD_ROVER_CONTROL__PATH_FOLLOWER_NODE_HPP_

#include <string>
#include <vector>

#include "field_rover_control/path_follower.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace field_rover_control
{

class PathFollowerNode : public rclcpp::Node
{
public:
  explicit PathFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Topic names this node actually subscribes to, for direct testing
  /// without relying on ROS graph discovery timing.
  std::vector<std::string> subscribed_topic_names() const;

  /// The topic this node publishes velocity commands on.
  std::string published_cmd_vel_topic() const;

private:
  PathFollowerConfig build_validated_config();
  void handle_path(const nav_msgs::msg::Path::SharedPtr message);
  void handle_localization(const nav_msgs::msg::Odometry::SharedPtr message);
  bool is_localization_usable() const;
  void control_step();

  PathFollowerConfig config_;
  PathFollowerState state_;

  bool localization_received_ = false;
  Pose2D latest_pose_;
  rclcpp::Time latest_localization_stamp_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace field_rover_control

#endif  // FIELD_ROVER_CONTROL__PATH_FOLLOWER_NODE_HPP_
