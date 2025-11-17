#ifndef IS_STATE_NORMAL_CONDITION_H
#define IS_STATE_NORMAL_CONDITION_H

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <chrono>

class IsStateNormalCondition : public BT::ConditionNode {
public:
    IsStateNormalCondition(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_(node_ptr) {
        RCLCPP_INFO(node_->get_logger(), "BT condition %s is started. Subscribing to /joint_states.", name.c_str());
        subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(), 
            std::bind(&IsStateNormalCondition::jointStateCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts() {
        return {
            // BT::InputPort<std::string>("joint_name", "joint_1", "Joint to check"),
            // BT::InputPort<double>("position_threshold", 0.1, "Threshold for position")
        };
    }

    BT::NodeStatus tick() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!last_joint_state_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
            "[%s] Waiting for /joint_states...", name().c_str());
            return BT::NodeStatus::RUNNING;
        }

        const auto& js = *last_joint_state_;

        // Basic sanity checks
        if (js.position.size() < normal_.size()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Received %zu joint positions, but expected at least %zu.", 
            name().c_str(), js.position.size(), normal_.size());
            return BT::NodeStatus::FAILURE;
        }

        // Compare first N joints against the fixed normal_ vector (index-based match)
        for (size_t i = 0; i < normal_.size(); ++i) {
            const double actual = js.position[i];
            const double target = normal_[i];
            const double err = std::fabs(actual - target);
            if (err > kToleranceRad_) {
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                "[%s] Joint %zu off by %.6f rad (target=%.6f, actual=%.6f, tol=%.6f)", 
                name().c_str(), i, err, target, actual, kToleranceRad_);
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::mutex mutex_; // Protects last_joint_state_ from concurrent access

    // Normal pose in radians (6 DOF)
    static constexpr std::array<double, 6> normal_ = {
        -1.5708, -1.5708, 0.0000, -1.5708,  1.5708, 0.0000
    };

    // Per-joint absolute tolerance (radians)
    static constexpr double kToleranceRad_ = 0.02; // ≈ 1.15°

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_joint_state_ = msg;
    }
    
};

#endif // IS_STATE_NORMAL_CONDITION_H