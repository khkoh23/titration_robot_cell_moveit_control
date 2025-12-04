#ifndef BT_EXECUTOR_EQUIVALENCE_POINT_H
#define BT_EXECUTOR_EQUIVALENCE_POINT_H

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <stdexcept>

#include "titration_robot_cell_moveit_control/check_dph_dv_condition.h"
#include "titration_robot_cell_moveit_control/check_ph_by_range_condition.h"
#include "titration_robot_cell_moveit_control/is_ph_stable_condition.h"
#include "titration_robot_cell_moveit_control/is_state_normal_condition.h"
#include "titration_robot_cell_moveit_control/is_to_replenish_pipette_condition.h"
#include "titration_robot_cell_moveit_control/aspire_action.h"
#include "titration_robot_cell_moveit_control/dispense_action.h"
#include "titration_robot_cell_moveit_control/move_pose_action.h"
#include "titration_robot_cell_moveit_control/move_state_action.h"
#include "titration_robot_cell_moveit_control/set_dispense_volume_action.h"

class BTExecutor : public rclcpp::Node {
public:
    BTExecutor() : Node("equivalence_point_titration_executor") {

    }

    void initialize() {
        RCLCPP_INFO(this->get_logger(), "BTExecutor node starting up ...");
        blackboard_ = BT::Blackboard::create();
        auto shared_this = shared_from_this();

        factory_.registerBuilder<CheckDphDvCondition>("CheckDphDv", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<CheckDphDvCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<CheckPhByRangeCondition>("CheckPhByRange", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<CheckPhByRangeCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<IsPhStableCondition>("IsPhStable", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<IsPhStableCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<IsStateNormalCondition>("IsStateNormal", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<IsStateNormalCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<IsToReplenishPipetteCondition>("IsToReplenishPipette", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<IsToReplenishPipetteCondition>(name, config, shared_this);
            });
        factory_.registerBuilder<AspireAction>("Aspire",
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<AspireAction>(name, config, shared_this);
            });
        factory_.registerBuilder<DispenseAction>("Dispense",
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<DispenseAction>(name, config, shared_this);
            });
        factory_.registerBuilder<MovePoseAction>("MovePose", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<MovePoseAction>(name, config, shared_this);
            });
        factory_.registerBuilder<MoveStateAction>("MoveState", 
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<MoveStateAction>(name, config, shared_this);
            });
        factory_.registerBuilder<SetDispenseVolumeAction>("SetDispenseVolume",
            [&](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<SetDispenseVolumeAction>(name, config, shared_this);
            });
        
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
        
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&BTExecutor::tick_callback, this));
    }

    ~BTExecutor() {
        RCLCPP_INFO(this->get_logger(), "BTExecutor node shutting down. Cleaning up resources ...");
        if (timer_) {
            timer_->cancel();
            timer_.reset();
        }
    }

private:
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;

    void tick_callback() {
        auto s = tree_.rootNode()->status();
        if (s == BT::NodeStatus::IDLE || s == BT::NodeStatus::RUNNING) {
          tree_.tickOnce();
        }
    }

}; 

#endif // BT_EXECUTOR_EQUIVALENCE_POINT_H