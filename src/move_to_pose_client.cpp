#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <titration_robot_interfaces/action/move_to_pose.hpp>

class MoveToPoseActionClient : public rclcpp::Node {
public:
    using MoveToPose = titration_robot_interfaces::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

    explicit MoveToPoseActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("move_to_pose_action_client", node_options), goal_done_(false) {
        this->client_ptr_ = rclcpp_action::create_client<MoveToPose>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "move_to_pose");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&MoveToPoseActionClient::send_goal, this));
    }

    bool is_goal_done() const {
        return this->goal_done_;
    }

    void send_goal() {
        using namespace std::placeholders;
        this->timer_->cancel();
        this->goal_done_ = false;
        if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }
        auto goal_msg = MoveToPose::Goal();
        goal_msg.target_pose.header.frame_id = "world";
        goal_msg.target_pose.pose.position.x = 0.225;
        goal_msg.target_pose.pose.position.y = -0.225;
        goal_msg.target_pose.pose.position.z = 1.100;
        goal_msg.target_pose.pose.orientation.w = 0.0000;
        goal_msg.target_pose.pose.orientation.x = 0.7071;
        goal_msg.target_pose.pose.orientation.y = 0.7071;
        goal_msg.target_pose.pose.orientation.z = 0.0000;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MoveToPoseActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&MoveToPoseActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&MoveToPoseActionClient::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }


private:
    rclcpp_action::Client<MoveToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    void goal_response_callback(GoalHandleMoveToPose::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleMoveToPose::SharedPtr, const std::shared_ptr<const MoveToPose::Feedback> feedback) {
        // RCLCPP_INFO(this->get_logger(), "Next number in sequence received: %" PRId32, feedback->sequence.back());
    }

    void result_callback(const GoalHandleMoveToPose::WrappedResult & result) {
        this->goal_done_ = true;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received");
        // for (auto number : result.result->sequence) {
        //     RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
        // }
    }
};  // class MoveToPoseActionClient


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MoveToPoseActionClient>();
    while (!action_client->is_goal_done()) {
        rclcpp::spin_some(action_client);
    }
    rclcpp::shutdown();
    return 0;
}