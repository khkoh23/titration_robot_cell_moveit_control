#include <rclcpp/rclcpp.hpp>
#include "titration_robot_cell_moveit_control/bt_executor_equivalence_point.h"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto executor_node = std::make_shared<BTExecutor>();
    executor_node->initialize();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(executor_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}