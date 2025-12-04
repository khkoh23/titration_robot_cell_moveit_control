#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <titration_robot_interfaces/srv/derivative_ph_volume.hpp>

#include <deque>
#include <mutex>

class DerivativePhVolumeServer : public rclcpp::Node {
public:
  DerivativePhVolumeServer() : Node("derivative_ph_volume_server") {
    ph_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "ph", 10, std::bind(&DerivativePhVolumeServer::ph_callback, this, std::placeholders::_1));
    titration_vol_subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
      "titration_vol", 10, std::bind(&DerivativePhVolumeServer::titration_vol_callback, this, std::placeholders::_1));
    service_ = this->create_service<titration_robot_interfaces::srv::DerivativePhVolume>( 
      "derivative_ph_volume", std::bind(&DerivativePhVolumeServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "DerivativePhVolume service server started.");
  }

private:
  void ph_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_ph_ = msg->data;
  }

  void titration_vol_callback(const std_msgs::msg::UInt32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Store (volume, pH) pairs. Limit history to a few points for derivative calculation.
    if (!ph_vol_data_.empty() && ph_vol_data_.back().first == msg->data) {
        // Avoid duplicate volume entries if titration_vol is published rapidly without pH change
        ph_vol_data_.back().second = latest_ph_;
    } else {
        ph_vol_data_.push_back({static_cast<float>(msg->data), latest_ph_});
        if (ph_vol_data_.size() > 5) { // Keep a small history for derivative
            ph_vol_data_.pop_front();
        }
    }
  }

  void handle_service(
    const std::shared_ptr<titration_robot_interfaces::srv::DerivativePhVolume::Request> request, 
    std::shared_ptr<titration_robot_interfaces::srv::DerivativePhVolume::Response> response) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (ph_vol_data_.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough data points to calculate derivative.");
      response->dph_dv = 0.0; // Or handle as an error
      return;
    }
    // Calculate derivative using the last two points
    float dV = ph_vol_data_.back().first - ph_vol_data_[ph_vol_data_.size() - 2].first;
    float dPh = ph_vol_data_.back().second - ph_vol_data_[ph_vol_data_.size() - 2].second;
    if (dV == 0.0f) {
      RCLCPP_WARN(this->get_logger(), "Change in volume is zero, cannot calculate derivative.");
      response->dph_dv = 0.0;
      return;
    }
    response->dph_dv = dPh / dV;
    RCLCPP_INFO(this->get_logger(), "Calculated dPh/dV: %f", response->dph_dv);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ph_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr titration_vol_subscription_;
  rclcpp::Service<titration_robot_interfaces::srv::DerivativePhVolume>::SharedPtr service_; 

  std::deque<std::pair<float, float>> ph_vol_data_; // Stores {volume, pH} pairs
  float latest_ph_ = 0.0f;
  std::mutex data_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DerivativePhVolumeServer>());
  rclcpp::shutdown();
  return 0;
}