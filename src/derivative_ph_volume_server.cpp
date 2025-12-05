#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <titration_robot_interfaces/srv/derivative_ph_volume.hpp>

#include <deque>
#include <mutex>
#include <chrono>

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
  // Each stored sample carries the cumulative titration volume (0.1 µL units),
  // the pH sampled at that time, a 'repaired' flag (whether we've overwritten pH with a stable reading),
  // and a timestamp for observability / potential future guards.
  struct VolPhSample {
    float volume{0.0f}; // cumulative volume in 0.1 µL units
    float ph{0.0f};     // pH value
    bool repaired{false};
    std::chrono::steady_clock::time_point t{};
  };

  void ph_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_ph_ = msg->data;
  }

  void titration_vol_callback(const std_msgs::msg::UInt32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto now = std::chrono::steady_clock::now();
    // Store (volume, pH_at_dispense) pairs. Limit history to a few points for derivative calculation.
    if (!ph_vol_data_.empty() && ph_vol_data_.back().volume == static_cast<float>(msg->data)) {
      // Defensive path: in case the same titration_vol is republished, refresh the pH for that volume.
      ph_vol_data_.back().ph = latest_ph_;
      ph_vol_data_.back().t = now;
      // Do not set repaired here; this is the "at-publish" refresh, not the post-stability repair.
    }
    else {
      ph_vol_data_.push_back(VolPhSample{
        static_cast<float>(msg->data), // 0.1 µL units
        latest_ph_,                    // pH sampled now (at dispense publish time)
        false,                         // not repaired yet
        now
      });
      if (ph_vol_data_.size() > 5) { // Keep a small history for derivative
        ph_vol_data_.pop_front();
      }
    }
  }

  void handle_service(const std::shared_ptr<titration_robot_interfaces::srv::DerivativePhVolume::Request> /*request*/,
    std::shared_ptr<titration_robot_interfaces::srv::DerivativePhVolume::Response> response) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // --- Repair-on-request: assume BT ensured stability before this call ---
    // Overwrite the last sample's pH with the stabilized latest_ph_ exactly once.
    if (!ph_vol_data_.empty()) {
      auto &last = ph_vol_data_.back();
      if (!last.repaired) {
        last.ph = latest_ph_;
        last.repaired = true;
        // (optional) last.t = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Repaired last pair to stable pH: V=%.1f (0.1 uL), pH=%.3f", last.volume, last.ph);
      }
    }
    if (ph_vol_data_.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough data points to calculate derivative.");
      response->dph_dv = 0.0; // To add a guard
      return;
    }
    const auto &s2 = ph_vol_data_[ph_vol_data_.size() - 1];
    const auto &s1 = ph_vol_data_[ph_vol_data_.size() - 2];
    float dV_raw = s2.volume - s1.volume; // difference in 0.1 µL units
    float dPh    = s2.ph     - s1.ph;
    if (dV_raw == 0.0f) {
      RCLCPP_WARN(this->get_logger(), "Change in volume is zero, cannot calculate derivative.");
      response->dph_dv = 0.0; // To add a guard
      return;
    }
    response->dph_dv = dPh / dV_raw;
    RCLCPP_INFO(this->get_logger(), "Calculated dPh/dV (per 0.1 uL): %f", response->dph_dv);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ph_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr titration_vol_subscription_;
  rclcpp::Service<titration_robot_interfaces::srv::DerivativePhVolume>::SharedPtr service_; 

  std::deque<VolPhSample> ph_vol_data_; // Stores {volume, pH, repaired, t} 
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