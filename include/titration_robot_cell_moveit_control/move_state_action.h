#ifndef MOVE_STATE_ACTION_H
#define MOVE_STATE_ACTION_H

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp> 

#include <titration_robot_interfaces/action/move_state.hpp>
#include <memory>
#include <chrono>

class MoveStateAction : public BT::StatefulActionNode {
public:
    using MoveState = titration_robot_interfaces::action::MoveState;
    using GoalHandleMoveState = rclcpp_action::ClientGoalHandle<MoveState>;
    using WrappedResult = GoalHandleMoveState::WrappedResult;
    using ResultFuture = std::shared_future<WrappedResult>;

    MoveStateAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
        bt_node_name_ = name;
        client_ = rclcpp_action::create_client<MoveState>(node_, "move_state");
    }

    BT::NodeStatus onStart() override {
        clearGoalState();
        if (!getInput<std::string>("group_state", group_state_)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input port: group_state", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "BT action %s is started.", bt_node_name_.c_str());
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Action server 'move_state' is not available!", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        MoveState::Goal goal_msg;
        goal_msg.group_state = group_state_;
        rclcpp_action::Client<MoveState>::SendGoalOptions send_goal_options;
        try { 
            future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
        } catch (const std::exception& e) { 
            RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while sending goal: %s", bt_node_name_.c_str(), e.what());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "[%s] Action goal for move_state is sent. Waiting for response...", bt_node_name_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] ROS shutting down.", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (!goal_handle_) { // Wait for the goal handle once
            if (!future_goal_handle_.valid() || future_goal_handle_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                return BT::NodeStatus::RUNNING;
            }
            try {
                goal_handle_ = future_goal_handle_.get(); 
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] UnknownGoalHandleError while obtaining goal handle: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while obtaining goal handle: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            if (!goal_handle_) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Action goal for move_state was rejected by the server.", bt_node_name_.c_str());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            try { // Once have a valid handle, request the result exactly once
                future_result_ = client_->async_get_result(goal_handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] UnknownGoalHandleError in async_get_result: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception in async_get_result: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
        }
        // Poll for the result
        if (future_result_.valid() && future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            WrappedResult wrapped_result;
            try {
                wrapped_result = future_result_.get();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while getting result: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Action goal for move_state succeeded!", bt_node_name_.c_str());
                clearGoalState();
                return BT::NodeStatus::SUCCESS;
            } 
            else {
                RCLCPP_WARN(node_->get_logger(), "[%s] Action goal for move_state failed. Result code: %d", bt_node_name_.c_str(), static_cast<int>(wrapped_result.code));
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { 
        RCLCPP_WARN(node_->get_logger(), "BT action MoveState is halted externally.");
        if (goal_handle_) {
            try {
                (void)client_->async_cancel_goal(goal_handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_WARN(node_->get_logger(), "[%s] UnknownGoalHandleError while canceling: %s", bt_node_name_.c_str(), e.what());
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Exception while canceling goal: %s", bt_node_name_.c_str(), e.what());
            }
        }
        clearGoalState();
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("group_state"),
        };
    }

private:
    void clearGoalState() {
        goal_handle_.reset();
        future_goal_handle_ = std::shared_future<GoalHandleMoveState::SharedPtr>();
        future_result_ = ResultFuture();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<MoveState>::SharedPtr client_;
    std::shared_future<GoalHandleMoveState::SharedPtr> future_goal_handle_;
    GoalHandleMoveState::SharedPtr goal_handle_{nullptr};
    ResultFuture future_result_;

    std::string group_state_;
    std::string bt_node_name_;

}; 

#endif // MOVE_STATE_ACTION_H