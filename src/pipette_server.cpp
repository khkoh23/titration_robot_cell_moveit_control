#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <titration_robot_interfaces/srv/pipette.hpp>

#include <memory>

class PipetteServiceServer : public rclcpp::Node {
public:

    using Pipette = titration_robot_interfaces::srv::Pipette;
    explicit PipetteServiceServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("pipette_server", options) {
        using namespace std::placeholders;
        pipette_cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>("pipette_cmd", 
            rclcpp::QoS(10).reliable());
        pipette_set_vol_pub_ = this->create_publisher<std_msgs::msg::UInt16>("pipette_set_vol", 
            rclcpp::QoS(1).reliable().transient_local());
        pipette_step_vol_pub_ = this->create_publisher<std_msgs::msg::UInt16>("pipette_step_vol", 
            rclcpp::QoS(1).reliable().transient_local());
        titration_vol_pub_ = this->create_publisher<std_msgs::msg::UInt32>("titration_vol", 
            rclcpp::QoS(1).reliable().transient_local());
        this->service_server_ = this->create_service<Pipette>(
            "pipette", 
            std::bind(&PipetteServiceServer::handle_request, this, _1, _2)
        );
        RCLCPP_INFO(this->get_logger(), "Pipette service server started.");
    }

private:
    rclcpp::Service<Pipette>::SharedPtr service_server_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pipette_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pipette_set_vol_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pipette_step_vol_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr titration_vol_pub_;
    const int16_t capacity_volume_ = 2200; // maximum 220.0 uL pipette tip volume
    int16_t holding_volume_ = 0; // curent liquid held by the pipette
    uint16_t aspire_volume_ = 2000; // 200.0 uL
    uint16_t dispense_volume_ = 200; // 20.0 uL
    uint32_t titration_volume = 0;

    inline void help_publish_int8(rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub, int8_t value) {
        std_msgs::msg::Int8 msg;
        msg.data = value;
        pub->publish(msg);
    }

    inline void help_publish_uint16(rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub, uint16_t value) {
        std_msgs::msg::UInt16 msg;
        msg.data = value;
        pub->publish(msg);
    }

    inline void help_publish_uint32(rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub, uint16_t value) {
        std_msgs::msg::UInt32 msg;
        msg.data = value;
        pub->publish(msg);
    }

    void handle_request(const std::shared_ptr<Pipette::Request> request, std::shared_ptr<Pipette::Response> response) {
        switch (request->command) {
            case 7:
                if (holding_volume_ + aspire_volume_ <= capacity_volume_) {
                    holding_volume_ += aspire_volume_;
                    help_publish_int8(pipette_cmd_pub_, 7);
                    RCLCPP_INFO(this->get_logger(), "Aspired. Holding volume: %d/%d.", holding_volume_, capacity_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Aspire failed: Holding volume after Aspire %d is greater than max volume %d.", aspire_volume_ + holding_volume_, capacity_volume_);
                break;
            case 71:
                if (request->aspire_volume >= 50 && request->aspire_volume <= 2000) { // accept range 5 uL to 200 uL
                    aspire_volume_ = request->aspire_volume;
                    help_publish_uint16(pipette_set_vol_pub_, aspire_volume_);
                    RCLCPP_INFO(this->get_logger(), "Aspire volume is set to: %d.", aspire_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Aspire volume is outside range 50 to 2000.");
                break;
            case 9:
                if (holding_volume_ >= dispense_volume_) {
                    holding_volume_ -= dispense_volume_;
                    help_publish_int8(pipette_cmd_pub_, 9);
                    titration_volume += dispense_volume_;
                    help_publish_uint32(titration_vol_pub_, titration_volume);
                    RCLCPP_INFO(this->get_logger(), "Dispensed. Holding volume: %d/%d.", holding_volume_, capacity_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Dispense failed: Holding volume %d is lesser than dispense step volume %d.", holding_volume_, dispense_volume_);
                break;
            case 91:
                if (request->dispense_volume >= 50 && request->dispense_volume <= 2000) { // accept range 5 uL to 200 uL
                    dispense_volume_ = request->dispense_volume;
                    help_publish_uint16(pipette_step_vol_pub_, dispense_volume_);
                    RCLCPP_INFO(this->get_logger(), "Dispense volume is set to: %d.", dispense_volume_);
                }
                else RCLCPP_WARN(this->get_logger(), "Dispense volume is outside range 50 to 2000.");
                break;
            case 123:
                RCLCPP_INFO(this->get_logger(), "Response to holding volume enquiry: %d/%d.", holding_volume_, capacity_volume_);
                break;
            case 124:
                if (dispense_volume_ > holding_volume_) {
                    RCLCPP_WARN(this->get_logger(), "Yes to replenish ...");
                    response->to_replenish = true;
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "No need to replenish.");
                    response->to_replenish = false;
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command code: %d", static_cast<int>(request->command));
                break;
        }
        response->holding_volume = holding_volume_;
    }

}; // class PipetteServiceServer

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // rclcpp::ExecutorOptions options;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto service_server = std::make_shared<PipetteServiceServer>(node_options);
    rclcpp::spin(service_server);
    rclcpp::shutdown();
    return 0;
}