/**
 * @file nodes/sensor_sync_node.cpp
 * @brief 独立 ROS 节点入口。
 */
#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/config_manager.h"
#include "automap_pro/sensor/sensor_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensor_sync_node");

    std::string config_path;
    node->declare_parameter<std::string>("config", "");
    node->get_parameter("config", config_path);
    if (!config_path.empty()) {
        automap_pro::ConfigManager::instance().loadFromFile(config_path);
    }

    automap_pro::SensorManager sensor_mgr;
    sensor_mgr.init(node);

    RCLCPP_INFO(node->get_logger(), "[sensor_sync_node] Running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
