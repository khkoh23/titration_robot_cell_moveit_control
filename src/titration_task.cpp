#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <titration_robot_interfaces/action/move_pose.hpp>
#include <titration_robot_interfaces/action/move_state.hpp>
#include <string>

using namespace std::chrono_literals;
using MovePoseAction = titration_robot_interfaces::action::MovePose;
using MoveStateAction = titration_robot_interfaces::action::MoveState;

enum class ActionResult : uint8_t {  // TODO: To match with rclcpp_action::ResultCode
    ActionNotCompleted,
    ActionFailed,
    ActionCancelled,
    ActionSucceded
};

BT::NodeStatus IsTitrationDone() {
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsPipetteEmpty() {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DispenseStepVolume() {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Aspire() {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsStateNormal() {
    return BT::NodeStatus::FAILURE;
}


class CallMovePose : public BT::StatefulActionNode {
public:
    CallMovePose(const std::string & action_name, const BT::NodeConfig & conf) : BT::StatefulActionNode(action_name, conf) {
        node_ = rclcpp::Node::make_shared("move_pose_action_client_node");
    }

    BT::NodeStatus onStart() {
        getInput<float>("position_x", position_x_);
        getInput<float>("position_y", position_y_);
        getInput<float>("position_z", position_z_);
        action_result_ = ActionResult::ActionNotCompleted;
        server_called_ = false;
        client_ = rclcpp_action::create_client<MovePoseAction>(node_, "move_pose");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        if (!server_called_) {
            auto goal = MovePoseAction::Goal();
            goal.position_x = position_x_;
            goal.position_y = position_y_;
            goal.position_z = position_z_;
            goal.orientation_w = 0.0000;
            goal.orientation_x = 0.7071;
            goal.orientation_y = 0.7071;
            goal.orientation_z = 0.0000;
            server_called_ = true;
            auto send_goal_future = client_->async_send_goal(goal);
            if (rclcpp::spin_until_future_complete(node_, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
                std::cout << "Failed to send move_pose goal" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            auto goal_handle = send_goal_future.get();
            if (!goal_handle) {
                std::cout << "Goal was rejected by move_pose server" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            client_->async_get_result(goal_handle, std::bind(&CallMovePose::result_callback, this, std::placeholders::_1));
        }

        rclcpp::spin_some(node_);

        switch (action_result_) {
            case ActionResult::ActionSucceded:
                std::cout << "Call move_pose action: Succeded" << std::endl;
                return BT::NodeStatus::SUCCESS;
                break;
            case ActionResult::ActionFailed:
                std::cout << "Call move_pose action: Failed" << std::endl;
                return BT::NodeStatus::FAILURE;
                break;
            case ActionResult::ActionCancelled:
                std::cout << "Call move_pose action : Cancelled" << std::endl;
                return BT::NodeStatus::FAILURE;
                break;
            case ActionResult::ActionNotCompleted:
                break;
        }

        std::cout << "Call move_pose action: Waiting server execution" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("position_x"),
            BT::InputPort<float>("position_y"),
            BT::InputPort<float>("position_z"),
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<MovePoseAction>::SharedPtr client_;
    float position_x_, position_y_, position_z_, orientation_w_, orientation_x_, orientation_y_, orientation_z_;
    bool server_called_;
    ActionResult action_result_;

    void result_callback(const rclcpp_action::ClientGoalHandle<MovePoseAction>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_result_ = ActionResult::ActionSucceded;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                action_result_ = ActionResult::ActionFailed;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                action_result_ = ActionResult::ActionCancelled;
                break;
            default:
                break;
        }
    }
}; // class CallMovePose




class CallMoveState : public BT::StatefulActionNode {
public:
    CallMoveState(const std::string & action_name, const BT::NodeConfig & conf) : BT::StatefulActionNode(action_name, conf) {
        node_ = rclcpp::Node::make_shared("move_state_action_client_node");
    }

    BT::NodeStatus onStart() {
        getInput<std::string>("group_state", group_state_);
        action_result_ = ActionResult::ActionNotCompleted;
        server_called_ = false;
        client_ = rclcpp_action::create_client<MoveStateAction>(node_, "move_state");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        if (!server_called_) {
            auto goal = MoveStateAction::Goal();
            goal.group_state = group_state_;
            server_called_ = true;
            auto send_goal_future = client_->async_send_goal(goal);
            if (rclcpp::spin_until_future_complete(node_, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
                std::cout << "Failed to send move_state goal" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            auto goal_handle = send_goal_future.get();
            if (!goal_handle) {
                std::cout << "Goal was rejected by move_state server" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            client_->async_get_result(goal_handle, std::bind(&CallMoveState::result_callback, this, std::placeholders::_1));
        }

        rclcpp::spin_some(node_);

        switch (action_result_) {
            case ActionResult::ActionSucceded:
                std::cout << "Call move_state action: Succeded" << std::endl;
                return BT::NodeStatus::SUCCESS;
                break;
            case ActionResult::ActionFailed:
                std::cout << "Call move_state action: Failed" << std::endl;
                return BT::NodeStatus::FAILURE;
                break;
            case ActionResult::ActionCancelled:
                std::cout << "Call move_state action : Cancelled" << std::endl;
                return BT::NodeStatus::FAILURE;
                break;
            case ActionResult::ActionNotCompleted:
                break;
        }

        std::cout << "Call move_state action: Waiting server execution" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("group_state"),
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<MoveStateAction>::SharedPtr client_;
    std::string group_state_;
    bool server_called_;
    ActionResult action_result_;

    void result_callback(const rclcpp_action::ClientGoalHandle<MoveStateAction>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_result_ = ActionResult::ActionSucceded;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                action_result_ = ActionResult::ActionFailed;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                action_result_ = ActionResult::ActionCancelled;
                break;
            default:
                break;
        }
    }
}; // class CallMoveState



class BTExecutor : public rclcpp::Node {
public:
    BTExecutor() : Node("bt_executor") {
        first_ = true;
        timer_ = this->create_wall_timer( 0.5s, std::bind(&BTExecutor::tick_function, this));
        blackboard_ = BT::Blackboard::create();
    }

private:
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_;

    void init_btree() {
        blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        factory_.registerNodeType<CallMovePose>("MovePose");
        factory_.registerSimpleCondition("IsTitrationDone", std::bind(IsTitrationDone));
        factory_.registerSimpleCondition("IsPipetteEmpty", std::bind(IsPipetteEmpty));
        factory_.registerSimpleCondition("DispenseStepVolume", std::bind(DispenseStepVolume));
        factory_.registerSimpleCondition("Aspire", std::bind(Aspire));
        factory_.registerSimpleCondition("IsStateNormal", std::bind(IsStateNormal));
        factory_.registerNodeType<CallMoveState>("MoveState");
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
    }

    void tick_function() {
      if( first_) {
          init_btree();
          first_ = false;
      } 
      tree_.tickOnce();
    }

}; // class BTExecutor



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTExecutor>();
    rclcpp::spin( node );
    rclcpp::shutdown();
    return 0;
}