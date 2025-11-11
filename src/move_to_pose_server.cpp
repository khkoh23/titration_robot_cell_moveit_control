#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <titration_robot_interfaces/action/move_to_pose.hpp>

class MoveToPoseActionServer : public rclcpp::Node {
public:
    using MoveToPose = titration_robot_interfaces::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    explicit MoveToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("move_to_pose_server", options), move_group_interface_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](auto){}), "ur_arm")) {
        using namespace std::placeholders;
        // move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "your_planning_group_name");
        this->action_server_ = rclcpp_action::create_server<MoveToPose>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_to_pose",
            std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Move-to-pose action server started");
    }

private:
    // std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target_pose");
        (void)uuid;
        if (goal->target_pose.pose.position.x > 0.9) { // To add goal reject criteria
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&MoveToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal ...");
        // rclcpp::Rate loop_rate(1); // 1 Hz loop rate
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();
        // geometry_msgs::msg::PoseStamped target;
        // target.header.frame_id = move_group_interface_->getPlanningFrame();
        // target.pose.position.x = goal->target_pose;
        // move_group_interface_->setPoseTarget(target.pose);
        move_group_interface_->setPoseTarget(goal->target_pose);
        move_group_interface_->setMaxVelocityScalingFactor(0.5);
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            // geometry_msgs::msg::PoseStamped target_pose = goal->target_pose;
            // float timeout = goal->timeout;
            // static const std::string planning_group_name = goal->planning_group_name;
            // float current_x = 0.0; 
            // float current_y = 0.0; 
            // float current_z = 0.0; 
            // auto & current_pose = feedback->current_pose;
            // auto start_time = this->now();
            // while ((this->now() - start_time).seconds() < timeout) {
            //     if (goal_handle->is_canceling()) { // Check if there is a cancel request
            //         goal_handle->canceled(result);
            //         RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            //         return;
            //     }
            //     // Update 
            //     // Publish feedback
            //     // feedback->current_pose = ;
            //     goal_handle->publish_feedback(feedback);
            //     RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f, %f)", current_x, current_y, current_z);
            //     loop_rate.sleep();
            // }

        if(success) {
            moveit::core::MoveItErrorCode execute_success = move_group_interface_->execute(plan);
            if (execute_success == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
                result->success = true;
                goal_handle->succeed(result);
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Goal Failed during execution");
                result->success = false;
                goal_handle->abort(result);
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Goal Failed during planning");
            result->success = false;
            goal_handle->abort(result);
        }

    }

};  // class MoveToPoseActionServer

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::ExecutorOptions options;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto action_server = std::make_shared<MoveToPoseActionServer>(node_options);
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}