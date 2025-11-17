#include <rclcpp/rclcpp.hpp>
#include "titration_robot_cell_moveit_control/bt_executor.h"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto executor_node = std::make_shared<BTExecutor>();
    executor_node->initialize();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(executor_node);
    // while (rclcpp::ok() 
    // && tree.rootNode()->status() != BT::NodeStatus::SUCCESS 
    // && tree.rootNode()->status() != BT::NodeStatus::FAILURE) {
    //     tree.tickOnce();
    //     executor.spin_some(); 
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
    executor.spin();
    rclcpp::shutdown();
    return 0;
}