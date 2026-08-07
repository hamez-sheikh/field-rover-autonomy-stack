cmake_minimum_required(VERSION 3.8)
project(field_rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

# Pure controller math plus the ROS node class, independent of the
# executable's main() so tests can link against the same code the
# executable runs.
add_library(field_rover_control_lib
  src/path_follower.cpp
  src/path_follower_node.cpp
)
target_include_directories(field_rover_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(field_rover_control_lib rclcpp geometry_msgs nav_msgs)

add_executable(path_follower src/path_follower_node_main.cpp)
target_link_libraries(path_follower field_rover_control_lib)

install(TARGETS field_rover_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS path_follower
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_path_follower test/test_path_follower.cpp)
  target_link_libraries(test_path_follower field_rover_control_lib)

  ament_add_gtest(test_path_follower_node test/test_path_follower_node.cpp)
  target_link_libraries(test_path_follower_node field_rover_control_lib)
endif()

ament_package()
