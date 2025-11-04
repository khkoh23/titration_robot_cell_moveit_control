#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto node = rclcpp::Node::make_shared("move_analyte_to_ready");
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_arm");
  geometry_msgs::msg::PoseStamped target;
  bool success;

  /* Pose B1
  */
  target.header.frame_id = move_group.getPlanningFrame();
  target.pose.position.x = 0.225;
  target.pose.position.y = 0.200;
  target.pose.position.z = 1.100;
  target.pose.orientation.w = 0.0000;
  target.pose.orientation.x = 0.7071;
  target.pose.orientation.y = 0.7071;
  target.pose.orientation.z = 0.0000;
  move_group.setPoseTarget(target.pose);
  move_group.setMaxVelocityScalingFactor(0.5);
	move_group.setMaxAccelerationScalingFactor(0.5);
  success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) move_group.move();

  /* Pose Ready
  */
  target.header.frame_id = move_group.getPlanningFrame();
  target.pose.position.x = 0.225;
  target.pose.position.y = 0.000;
  target.pose.position.z = 1.100;
  target.pose.orientation.w = 0.0000;
  target.pose.orientation.x = 0.7071;
  target.pose.orientation.y = 0.7071;
  target.pose.orientation.z = 0.0000;
  move_group.setPoseTarget(target.pose);
  move_group.setMaxVelocityScalingFactor(0.5);
	move_group.setMaxAccelerationScalingFactor(0.5);
  success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) move_group.move();

  return 0;
}
