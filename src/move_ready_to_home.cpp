#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto node = rclcpp::Node::make_shared("move_home_to_ready");
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_arm");
  bool within_bounds, success;
  std::vector<double> joint_target;
  geometry_msgs::msg::PoseStamped target;

  /* Home Pose
  */
  joint_target = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
  within_bounds = move_group.setJointValueTarget(joint_target);
	if( !within_bounds ) {
		std::cout << "Joint outer bounds" << std::endl;
		return -1;
	}
	move_group.setMaxVelocityScalingFactor(0.5);
	move_group.setMaxAccelerationScalingFactor(0.5);
	success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success) move_group.move(); 

  return 0;
}
