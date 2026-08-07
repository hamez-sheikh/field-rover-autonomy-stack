// Node-level tests for PathFollowerNode. These talk to the node purely
// over its real ROS topics and parameters -- the same interface any other
// node would use -- rather than reaching into its private state. The
// controller math itself is covered directly in test_path_follower.cpp.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "field_rover_control/path_follower_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace field_rover_control
{
namespace
{

geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  const double half_yaw = yaw / 2.0;
  pose.pose.orientation.z = std::sin(half_yaw);
  pose.pose.orientation.w = std::cos(half_yaw);
  return pose;
}

nav_msgs::msg::Path make_path(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & [x, y] : points) {
    path.poses.push_back(make_pose(x, y));
  }
  return path;
}

nav_msgs::msg::Odometry make_odometry(
  const rclcpp::Time & stamp, double x, double y, double yaw = 0.0)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.header.stamp = stamp;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  const double half_yaw = yaw / 2.0;
  odom.pose.pose.orientation.z = std::sin(half_yaw);
  odom.pose.pose.orientation.w = std::cos(half_yaw);
  return odom;
}

/// A minimal test harness: publishes /planned_path and /localization/odom,
/// and records every /cmd_vel message the node under test publishes back.
class TestHarness
{
public:
  TestHarness()
  : node(std::make_shared<rclcpp::Node>("path_follower_test_harness"))
  {
    path_publisher = node->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
    cmd_vel_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        latest_cmd_vel = *message;
        cmd_vel_count += 1;
      });
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;
  geometry_msgs::msg::Twist latest_cmd_vel;
  int cmd_vel_count = 0;
};

/// Spin both the node under test and the harness until predicate is true
/// or timeout_ms elapses, polling every 5 ms.
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  int timeout_ms = 3000)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

/// Build a node under test with a fast control loop, so tests do not need
/// to wait through the ~20 Hz production default.
std::shared_ptr<PathFollowerNode> make_fast_node()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("control_rate_hz", 50.0)});
  return std::make_shared<PathFollowerNode>(options);
}

class PathFollowerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(PathFollowerNodeTest, SubscribesToPlannedPathAndLocalization)
{
  auto node = make_fast_node();
  const auto topics = node->subscribed_topic_names();
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/planned_path"), topics.end());
  EXPECT_NE(std::find(topics.begin(), topics.end(), "/localization/odom"), topics.end());
}

TEST_F(PathFollowerNodeTest, PublishesCmdVel)
{
  auto node = make_fast_node();
  EXPECT_EQ(node->published_cmd_vel_topic(), "/cmd_vel");
}

TEST_F(PathFollowerNodeTest, DoesNotSubscribeToGroundTruth)
{
  auto node = make_fast_node();
  const auto topics = node->subscribed_topic_names();
  EXPECT_EQ(std::find(topics.begin(), topics.end(), "/ground_truth/odom"), topics.end());
}

TEST_F(PathFollowerNodeTest, EmptyPathProducesZeroCommand)
{
  auto node = make_fast_node();
  TestHarness harness;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(harness.node);

  harness.odom_publisher->publish(make_odometry(node->now(), 0.0, 0.0, 0.0));

  ASSERT_TRUE(spin_until(executor, [&] {return harness.cmd_vel_count > 0;}));
  EXPECT_NEAR(harness.latest_cmd_vel.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.latest_cmd_vel.angular.z, 0.0, 1e-9);
}

TEST_F(PathFollowerNodeTest, StaleLocalizationProducesZeroCommand)
{
  auto node = make_fast_node();
  TestHarness harness;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(harness.node);

  harness.path_publisher->publish(make_path({{0.0, 0.0}, {5.0, 0.0}}));
  // Well past the default 0.5 s localization_timeout_s, regardless of how
  // fast this test happens to run.
  const rclcpp::Time stale_stamp = node->now() - rclcpp::Duration::from_seconds(5.0);
  harness.odom_publisher->publish(make_odometry(stale_stamp, 0.0, 0.0, 0.0));

  ASSERT_TRUE(spin_until(executor, [&] {return harness.cmd_vel_count > 0;}));
  EXPECT_NEAR(harness.latest_cmd_vel.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.latest_cmd_vel.angular.z, 0.0, 1e-9);
}

TEST_F(PathFollowerNodeTest, ValidPathAndPoseProducesBoundedCommand)
{
  auto node = make_fast_node();
  TestHarness harness;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(harness.node);

  harness.path_publisher->publish(make_path({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}}));
  harness.odom_publisher->publish(make_odometry(node->now(), 0.0, 0.0, 0.0));

  ASSERT_TRUE(
    spin_until(
      executor, [&] {
        harness.odom_publisher->publish(make_odometry(node->now(), 0.0, 0.0, 0.0));
        return harness.cmd_vel_count > 0 && harness.latest_cmd_vel.linear.x > 0.0;
      }));

  EXPECT_GE(harness.latest_cmd_vel.linear.x, 0.0);
  EXPECT_LE(harness.latest_cmd_vel.linear.x, 0.45 + 1e-9);
  EXPECT_LE(std::abs(harness.latest_cmd_vel.angular.z), 0.80 + 1e-9);
}

TEST_F(PathFollowerNodeTest, GoalCompletionProducesZeroCommand)
{
  auto node = make_fast_node();
  TestHarness harness;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(harness.node);

  harness.path_publisher->publish(make_path({{0.0, 0.0}}));

  ASSERT_TRUE(
    spin_until(
      executor, [&] {
        harness.odom_publisher->publish(make_odometry(node->now(), 0.0, 0.0, 0.0));
        return harness.cmd_vel_count > 0;
      }));
  EXPECT_NEAR(harness.latest_cmd_vel.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.latest_cmd_vel.angular.z, 0.0, 1e-9);
}

TEST_F(PathFollowerNodeTest, PathReplacementResetsProgress)
{
  auto node = make_fast_node();
  TestHarness harness;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(harness.node);

  // First path is a single point the rover already sits on: the node
  // reaches the goal immediately and should keep publishing zero.
  harness.path_publisher->publish(make_path({{0.0, 0.0}}));
  ASSERT_TRUE(
    spin_until(
      executor, [&] {
        harness.odom_publisher->publish(make_odometry(node->now(), 0.0, 0.0, 0.0));
        return harness.cmd_vel_count > 0;
      }));
  EXPECT_NEAR(harness.latest_cmd_vel.linear.x, 0.0, 1e-9);

  // A replacement path that is not yet complete must clear the stale
  // goal-reached state and resume tracking.
  harness.cmd_vel_count = 0;
  harness.path_publisher->publish(make_path({{0.0, 0.0}, {5.0, 0.0}}));
  ASSERT_TRUE(
    spin_until(
      executor, [&] {
        harness.odom_publisher->publish(make_odometry(node->now(), 0.0, 0.0, 0.0));
        return harness.cmd_vel_count > 0 && harness.latest_cmd_vel.linear.x > 0.0;
      }));
  EXPECT_GT(harness.latest_cmd_vel.linear.x, 0.0);
}

}  // namespace
}  // namespace field_rover_control

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
