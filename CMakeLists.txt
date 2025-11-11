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
find_package(titration_robot_interfaces REQUIRED)

# add_executable(joint_space_planning src/joint_space_planning.cpp)
# ament_target_dependencies(joint_space_planning PUBLIC moveit_ros_planning_interface rclcpp)

# add_executable(pipette_tip_target src/pipette_tip_target.cpp)
# ament_target_dependencies(pipette_tip_target PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_analyte_to_ready src/move_analyte_to_ready.cpp)
# ament_target_dependencies(move_analyte_to_ready PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_analyte_to_titrant src/move_analyte_to_titrant.cpp)
# ament_target_dependencies(move_analyte_to_titrant PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_home_to_ready src/move_home_to_ready.cpp)
# ament_target_dependencies(move_home_to_ready PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_ready_to_home src/move_ready_to_home.cpp)
# ament_target_dependencies(move_ready_to_home PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_ready_to_titrant src/move_ready_to_titrant.cpp)
# ament_target_dependencies(move_ready_to_titrant PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_titrant_to_analyte src/move_titrant_to_analyte.cpp)
# ament_target_dependencies(move_titrant_to_analyte PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

# add_executable(move_to_pose_server src/move_to_pose_server.cpp)
# ament_target_dependencies(move_to_pose_server
#   geometry_msgs
#   moveit_ros_planning_interface
#   rclcpp
#   rclcpp_action
#   titration_robot_interfaces)

# add_executable(move_to_pose_client src/move_to_pose_client.cpp)
# ament_target_dependencies(move_to_pose_client
#   geometry_msgs
#   moveit_ros_planning_interface
#   rclcpp
#   rclcpp_action
#   titration_robot_interfaces)

add_executable(move_pose_server src/move_pose_server.cpp)
ament_target_dependencies(move_pose_server
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  titration_robot_interfaces)

add_executable(titration_task src/titration_task.cpp)
ament_target_dependencies(titration_task
  behaviortree_cpp
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  titration_robot_interfaces)


install(TARGETS 
  # joint_space_planning
  # pipette_tip_target
  # move_analyte_to_ready
  # move_analyte_to_titrant
  # move_home_to_ready
  # move_ready_to_home
  # move_ready_to_titrant
  # move_titrant_to_analyte
  # move_to_pose_server
  # move_to_pose_client
  move_pose_server
  titration_task
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
