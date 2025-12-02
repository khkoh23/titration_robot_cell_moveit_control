#ifndef MOVE_POSE_ACTION_H
#define MOVE_POSE_ACTION_H

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp> 

#include <titration_robot_interfaces/action/move_pose.hpp>
#include <memory>
#include <chrono>

class MovePoseAction : public BT::StatefulActionNode {
public:
    using MovePose = titration_robot_interfaces::action::MovePose;
    using GoalHandleMovePose = rclcpp_action::ClientGoalHandle<MovePose>;
    using WrappedResult = GoalHandleMovePose::WrappedResult;
    using ResultFuture = std::shared_future<WrappedResult>;

    MovePoseAction(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
        bt_node_name_ = name;
        client_ = rclcpp_action::create_client<MovePose>(node_, "move_pose");
    }

    BT::NodeStatus onStart() override {
        clearGoalState();
        if (!getInput<float>("position_x", position_x_) 
        || !getInput<float>("position_y", position_y_) 
        || !getInput<float>("position_z", position_z_)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required input ports: position_x/y/z", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "BT action %s is started.", bt_node_name_.c_str());
        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Action server 'move_pose' is not available!", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        MovePose::Goal goal_msg;
        goal_msg.position_x = position_x_;
        goal_msg.position_y = position_y_;
        goal_msg.position_z = position_z_;
        goal_msg.orientation_w = 0.0000f;
        goal_msg.orientation_x = 0.7071f;
        goal_msg.orientation_y = 0.7071f;
        goal_msg.orientation_z = 0.0000f;
        rclcpp_action::Client<MovePose>::SendGoalOptions send_goal_options;
        try {
            future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while sending move_pose goal: %s", bt_node_name_.c_str(), e.what());
            return BT::NodeStatus::FAILURE;
        }
//        RCLCPP_INFO(node_->get_logger(), "[%s] Action goal for move_pose is sent. Waiting for response...", bt_node_name_.c_str());
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
                RCLCPP_ERROR(node_->get_logger(), "[%s] UnknownGoalHandleError while obtaining move_pose goal handle: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while obtaining move_pose goal handle: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            if (!goal_handle_) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Action goal for move_pose was rejected by the server.", bt_node_name_.c_str());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            try { // Once have a valid handle, request the result exactly once
                future_result_ = client_->async_get_result(goal_handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] UnknownGoalHandleError in async_get_result (move_pose): %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception in async_get_result (move_pose): %s", bt_node_name_.c_str(), e.what());
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
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception while getting move_pose result: %s", bt_node_name_.c_str(), e.what());
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//                RCLCPP_INFO(node_->get_logger(), "[%s] Action goal for move_pose succeeded!", bt_node_name_.c_str());
                clearGoalState();
                return BT::NodeStatus::SUCCESS;
            } 
            else {
                RCLCPP_WARN(node_->get_logger(), "[%s] Action goal for move_pose failed. Result code: %d", bt_node_name_.c_str(), static_cast<int>(wrapped_result.code));
                clearGoalState();
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { 
        RCLCPP_WARN(node_->get_logger(), "BT action MovePose is halted externally.");
        if (goal_handle_) {
            try {
                (void)client_->async_cancel_goal(goal_handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                RCLCPP_WARN(node_->get_logger(), "[%s] UnknownGoalHandleError while canceling move_pose goal: %s", bt_node_name_.c_str(), e.what());
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Exception while canceling move_pose goal: %s", bt_node_name_.c_str(), e.what());
            }
        }
        clearGoalState();
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("position_x"),
            BT::InputPort<float>("position_y"),
            BT::InputPort<float>("position_z"),
        };
    }

private:
    void clearGoalState() {
        goal_handle_.reset();
        future_goal_handle_ = std::shared_future<GoalHandleMovePose::SharedPtr>();
        future_result_ = ResultFuture();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<MovePose>::SharedPtr client_;
    std::shared_future<GoalHandleMovePose::SharedPtr> future_goal_handle_;
    GoalHandleMovePose::SharedPtr goal_handle_{nullptr};
    ResultFuture future_result_;

    float position_x_{0.f}, position_y_{0.f}, position_z_{0.f};
    float orientation_w_{0.0f}, orientation_x_{0.7071f}, orientation_y_{0.7071f}, orientation_z_{0.0f};
    std::string bt_node_name_;

}; 

#endif // MOVE_POSE_ACTION_H