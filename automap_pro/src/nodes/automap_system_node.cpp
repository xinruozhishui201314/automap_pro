#include "automap_pro/system/automap_system.h"
#include <rclcpp/rclcpp.hpp>
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
    // 启用进程内通信（Intra-Process Communication）
    // 当与 fast_livo 在同一 Component Container 中运行时，自动零拷贝
    options.use_intra_process_comms(true);

    try {
        auto node = std::make_shared<automap_pro::AutoMapSystem>(options);
        rclcpp::spin(node);
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
