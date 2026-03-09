#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/config_manager.h"
#include "automap_pro/backend/hba_wrapper.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("optimizer_node");

    std::string config_path;
    node->declare_parameter<std::string>("config", "");
    node->get_parameter("config", config_path);
    if (!config_path.empty()) {
        automap_pro::ConfigManager::instance().loadFromFile(config_path);
    }

    automap_pro::HBAWrapper hba;
    hba.init(node);
    hba.start();

    RCLCPP_INFO(node->get_logger(), "[optimizer_node] Running...");
    rclcpp::spin(node);
    hba.stop();
    rclcpp::shutdown();
    return 0;
}
