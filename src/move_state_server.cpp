#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <titration_robot_interfaces/action/move_state.hpp>
#include <string>

class MoveStateActionServer : public rclcpp::Node {
public:
    using MoveState = titration_robot_interfaces::action::MoveState;
    using GoalHandleMoveState = rclcpp_action::ServerGoalHandle<MoveState>;

    explicit MoveStateActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("move_state_server", options), 
    move_group_interface_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](auto){}), "ur_arm")) {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<MoveState>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_state",
            std::bind(&MoveStateActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveStateActionServer::handle_cancel, this, _1),
            std::bind(&MoveStateActionServer::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Move-state action server started");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp_action::Server<MoveState>::SharedPtr action_server_;
    std::string home = "home";
    std::string normal = "normal";

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveState::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target named (%s)", goal->group_state.c_str());
        (void)uuid;
        // TODO: Add goal reject criteria
        // return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveState> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveState> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&MoveStateActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveState> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal ...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveState::Feedback>();
        auto result = std::make_shared<MoveState::Result>();
        move_group_interface_->setNamedTarget(goal->group_state);
        move_group_interface_->setMaxVelocityScalingFactor(0.5);
        move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            RCLCPP_INFO(this->get_logger(), "Move group plan - Succeeded");
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
            RCLCPP_ERROR(this->get_logger(), "Move group plan - Failed");
            result->outcome = false;
            goal_handle->abort(result);
        }
        // TODO: Publish feedback
    }

};  // class MoveStateActionServer

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::ExecutorOptions options;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto action_server = std::make_shared<MoveStateActionServer>(node_options);
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}