#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur5_pipette_tip_target_rclcpp");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "ur_arm"); // tip = ur5_tcp

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = move_group.getPlanningFrame(); // usually "world"
  // target.pose.position.x = 0.225;
  // target.pose.position.y = 0.000;
  // target.pose.position.z = 1.100;
  // target.pose.orientation.w = 0.0000;
  // target.pose.orientation.x = 0.7071;
  // target.pose.orientation.y = 0.7071;
  // target.pose.orientation.z = 0.0000;

  target.pose.position.x = 0.225;
  target.pose.position.y = 0.000;
  target.pose.position.z = 1.100;
  target.pose.orientation.w = 0.0000;
  target.pose.orientation.x = 0.7071;
  target.pose.orientation.y = 0.7071;
  target.pose.orientation.z = 0.0000;

  // Preferred: tip link already ur5_tcp
  move_group.setPoseTarget(target.pose);
  move_group.setMaxVelocityScalingFactor(1.0);
	move_group.setMaxAccelerationScalingFactor(1.0);

  auto const success = static_cast<bool>(move_group.plan(plan));
  if (success) move_group.move();

  // Explicit variant:
//   move_group.setEndEffectorLink("ur5_tcp");
//   move_group.setPoseTarget(target.pose, "ur5_tcp");
//   if (move_group.plan(/*Plan*/)) move_group.move();

  rclcpp::shutdown();
  return 0;
}
