cmake_minimum_required(VERSION 3.8)
project(titration_robot_cell_moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(joint_space_planning src/joint_space_planning.cpp)
ament_target_dependencies(joint_space_planning PUBLIC moveit_ros_planning_interface rclcpp)

add_executable(pipette_tip_target src/pipette_tip_target.cpp)
ament_target_dependencies(pipette_tip_target PUBLIC geometry_msgs moveit_ros_planning_interface rclcpp)

install(TARGETS 
  joint_space_planning
  pipette_tip_target
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
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
