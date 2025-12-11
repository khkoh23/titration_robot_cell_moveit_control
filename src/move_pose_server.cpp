#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <titration_robot_interfaces/action/move_pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class MovePoseActionServer : public rclcpp::Node {
public:
    using MovePose = titration_robot_interfaces::action::MovePose;
    using GoalHandleMovePose = rclcpp_action::ServerGoalHandle<MovePose>;

    explicit MovePoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("move_pose_server", options), 
    move_group_interface_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](auto){}), "ur_arm")) {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<MovePose>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_pose",
            std::bind(&MovePoseActionServer::handle_goal, this, _1, _2),
            std::bind(&MovePoseActionServer::handle_cancel, this, _1),
            std::bind(&MovePoseActionServer::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Move-pose action server started");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp_action::Server<MovePose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MovePose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target position (%.3f, %.3f, %.3f)",
        goal->position_x, goal->position_y, goal->position_z);
        (void)uuid;
        if (goal->position_x > 0.9) { // TODO: Check goal reject criteria
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMovePose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMovePose> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&MovePoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMovePose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal ...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MovePose::Feedback>();
        auto result = std::make_shared<MovePose::Result>();
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "world";
        target.pose.position.x = goal->position_x;
        target.pose.position.y = goal->position_y;
        target.pose.position.z = goal->position_z;
        target.pose.orientation.w = goal->orientation_w;
        target.pose.orientation.x = goal->orientation_x;
        target.pose.orientation.y = goal->orientation_y;
        target.pose.orientation.z = goal->orientation_z;
        // --- Cartesian Path Planning Logic ---
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target.pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01; // 1 cm
        double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, trajectory);
        bool success = (fraction >= 0.9); // Define success: computing 90% or more of the requested path
        move_group_interface_->setMaxVelocityScalingFactor(0.5);
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        if(success) {
            RCLCPP_INFO(this->get_logger(), "Cartesian path computed (%.2f%% covered) - Succeeded", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory = trajectory;
            moveit::core::MoveItErrorCode execute_success = move_group_interface_->execute(plan);
            if (execute_success == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Move group execute - Succeeded");
                result->outcome = true;
                goal_handle->succeed(result);
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Move group execute - Failed");
                result->outcome = false;
                goal_handle->abort(result);
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path computation failed (%.2f%% covered)", fraction * 100.0);
            result->outcome = false;
            goal_handle->abort(result);
        }
        // TODO: Publish feedback
    }

};  // class MovePoseActionServer

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::ExecutorOptions options;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto action_server = std::make_shared<MovePoseActionServer>(node_options);
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}