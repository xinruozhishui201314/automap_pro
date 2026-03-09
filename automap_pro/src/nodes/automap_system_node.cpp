#include "automap_pro/system/automap_system.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <iostream>

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[automap_system_node] rclcpp::init failed: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.use_intra_process_comms(true);

    try {
        auto node = std::make_shared<automap_pro::AutoMapSystem>(options);
        // 多线程 Executor：odom/cloud/kfinfo 等回调可并行，避免单线程下背压或慢回调阻塞其它回调，充分发挥多核
        rclcpp::executors::MultiThreadedExecutor exec(
            rclcpp::ExecutorOptions(), 4);
        exec.add_node(node);
        exec.spin();
        exec.remove_node(node);
    } catch (const std::exception& e) {
        std::cerr << "[automap_system_node] Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        std::cerr << "[automap_system_node] Unknown exception" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
