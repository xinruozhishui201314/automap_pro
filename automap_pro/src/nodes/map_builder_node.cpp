#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/config_manager.h"
#include "automap_pro/map/map_builder.h"
#include "automap_pro/map/map_filter.h"
#include "automap_pro/map/map_exporter.h"
#include "automap_pro/visualization/rviz_publisher.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("map_builder_node");

    std::string config_path;
    node->declare_parameter<std::string>("config", "");
    node->get_parameter("config", config_path);
    if (!config_path.empty()) {
        automap_pro::ConfigManager::instance().loadFromFile(config_path);
    }

    automap_pro::MapBuilder builder;
    automap_pro::MapFilter filter;
    automap_pro::MapExporter exporter;
    automap_pro::RvizPublisher rviz;
    rviz.init(node);

    RCLCPP_INFO(node->get_logger(), "[map_builder_node] Running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
