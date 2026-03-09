#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/config_manager.h"
#include "automap_pro/submap/submap_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("submap_manager_node");

    std::string config_path;
    node->declare_parameter<std::string>("config", "");
    node->get_parameter("config", config_path);
    if (!config_path.empty()) {
        automap_pro::ConfigManager::instance().loadFromFile(config_path);
    }

    automap_pro::SubMapManager submap_mgr;
    submap_mgr.init(node, 0);

    RCLCPP_INFO(node->get_logger(), "[submap_manager_node] Running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
