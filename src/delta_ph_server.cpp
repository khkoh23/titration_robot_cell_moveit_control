#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <titration_robot_interfaces/srv/delta_ph.hpp>

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <iostream>

using namespace std::chrono_literals;

class DeltaPhServiceServer : public rclcpp::Node {
public:
    using DeltaPh = titration_robot_interfaces::srv::DeltaPh;
    DeltaPhServiceServer() : Node("delta_ph_server") {
        ph_subscription_ = this->create_subscription<std_msgs::msg::Float32>("ph", 10, std::bind(&DeltaPhServiceServer::topic_callback, this, std::placeholders::_1));
        delta_ph_service_ = this->create_service<titration_robot_interfaces::srv::DeltaPh>("delta_ph", std::bind(&DeltaPhServiceServer::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "DeltaPh service server started.");
    }

private:
    struct PhDataPoint {
        rclcpp::Time timestamp;
        float value;
    };
    std::deque<PhDataPoint> data_buffer_;
    std::mutex buffer_mutex_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ph_subscription_;
    rclcpp::Service<DeltaPh>::SharedPtr delta_ph_service_;

    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        PhDataPoint new_point;
        new_point.timestamp = this->now();
        new_point.value = msg->data;
        data_buffer_.push_back(new_point);
    }

    void handle_service_request(const std::shared_ptr<DeltaPh::Request> request, std::shared_ptr<DeltaPh::Response> response) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        auto window_duration = std::chrono::seconds(request->window_sec);
        rclcpp::Time current_time = this->now();
        rclcpp::Time window_start_time = current_time - window_duration;
        // Remove old data points from the buffer (garbage collection)
        while (!data_buffer_.empty() && data_buffer_.front().timestamp < window_start_time) {
            data_buffer_.pop_front();
        }
        if (data_buffer_.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough data points in the last %d seconds to calculate a delta.", request->window_sec);
            response->delta_ph = 0.0f; // Return 0 if calculation isn't possible
            return;
        }
        // The delta is the difference between the most recent value and the oldest value currently in the window
        float oldest_value = data_buffer_.front().value;
        float newest_value = data_buffer_.back().value;
        response->delta_ph = newest_value - oldest_value;
        RCLCPP_INFO(this->get_logger(), "Calculated delta_ph over %d sec window: %f", request->window_sec, response->delta_ph);
    }
}; // class DeltaPhServiceServer

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeltaPhServiceServer>());
    rclcpp::shutdown();
    return 0;
}