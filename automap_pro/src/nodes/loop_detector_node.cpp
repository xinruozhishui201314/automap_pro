#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/config_manager.h"
#include "automap_pro/loop_closure/loop_detector.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loop_detector_node");

    std::string config_path;
    node->declare_parameter<std::string>("config", "");
    node->get_parameter("config", config_path);
    if (!config_path.empty()) {
        automap_pro::ConfigManager::instance().loadFromFile(config_path);
    }

    automap_pro::LoopDetector detector;
    detector.init(node);
    detector.start();

    RCLCPP_INFO(node->get_logger(), "[loop_detector_node] Running...");
    rclcpp::spin(node);
    detector.stop();
    rclcpp::shutdown();
    return 0;
}
