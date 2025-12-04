#ifndef IS_TO_REPLENISH_PIPETTE_CONDITION_H
#define IS_TO_REPLENISH_PIPETTE_CONDITION_H

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <titration_robot_interfaces/srv/pipette.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>     // for std::this_thread::sleep_for
#include <rmw/rmw.h>  // rmw_request_id_t

class IsToReplenishPipetteCondition : public BT::ConditionNode {
public:
    using Pipette = titration_robot_interfaces::srv::Pipette;
    IsToReplenishPipetteCondition(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr) 
    : BT::ConditionNode(name, config), node_(std::move(node_ptr)), bt_node_name_(name) {
        client_ = node_->create_client<Pipette>("pipette");
        RCLCPP_INFO(node_->get_logger(), "BT condition %s started.", bt_node_name_.c_str());
    }

    static BT::PortsList providedPorts() {
        return {}; 
    }

    BT::NodeStatus tick() override {
        using namespace std::chrono_literals;
        if (!client_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[%s] Service server 'pipette' not available!", bt_node_name_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        auto req = std::make_shared<Pipette::Request>();
        req->command = 124;
        (void)client_->async_send_request(req);
        // Poll for the response with a bounded wait.
        constexpr auto kPollSleep = 10ms;      // poll granularity
        constexpr auto kMaxWait   = 100ms;     // total max wait per tick (tune this)
        const auto t_start = std::chrono::steady_clock::now();
        Pipette::Response resp_msg;
        rmw_request_id_t resp_header;
        while (true) {
            // Attempt to take a response; returns true if one was available.
            if (client_->take_response(resp_msg, resp_header)) {
                const bool hv = static_cast<bool>(resp_msg.to_replenish);
                return (hv) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            // No response yet; bail out if we exceeded max wait
            if (std::chrono::steady_clock::now() - t_start >= kMaxWait) {
                RCLCPP_WARN(node_->get_logger(), "[%s] No response within %ld ms; treating as FAILURE this tick.", bt_node_name_.c_str(), 
                std::chrono::duration_cast<std::chrono::milliseconds>(kMaxWait).count());
                return BT::NodeStatus::FAILURE;
            }
            std::this_thread::sleep_for(kPollSleep);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Pipette>::SharedPtr client_;
    std::string bt_node_name_;
};

#endif // IS_TO_REPLENISH_PIPETTE_CONDITION_H