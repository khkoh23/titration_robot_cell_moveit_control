cmake_minimum_required(VERSION 3.8)
project(titration_robot_cell_moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(titration_robot_interfaces REQUIRED)

include_directories(include)

add_executable(move_pose_server src/move_pose_server.cpp)
ament_target_dependencies(move_pose_server
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  titration_robot_interfaces)

add_executable(move_state_server src/move_state_server.cpp)
ament_target_dependencies(move_state_server
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  titration_robot_interfaces)

# add_executable(titration_task src/titration_task.cpp)
# ament_target_dependencies(titration_task
#   behaviortree_cpp
#   geometry_msgs
#   moveit_ros_planning_interface
#   rclcpp
#   rclcpp_action
#   sensor_msgs
#   titration_robot_interfaces)

add_executable(titration_executor 
  src/titration_executor.cpp)
ament_target_dependencies(titration_executor 
  behaviortree_cpp
  rclcpp
  rclcpp_action
  sensor_msgs
  titration_robot_interfaces)

install(TARGETS 
  titration_executor
  move_pose_server
  move_state_server
  # titration_task
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  tree
  DESTINATION share/${PROJECT_NAME})

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
endif()

ament_package()
