// Executable entry point for the path_follower node. Kept separate from
// path_follower_node.cpp so tests can construct PathFollowerNode directly
// without linking a second main().
#include <memory>

#include "field_rover_control/path_follower_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<field_rover_control::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
