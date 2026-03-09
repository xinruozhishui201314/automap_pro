#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/config_manager.h"
#include "automap_pro/sensor/sensor_manager.h"
#include "automap_pro/frontend/fast_livo2_wrapper.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fast_livo2_node");

    std::string config_path;
    node->declare_parameter<std::string>("config", "");
    node->get_parameter("config", config_path);
    if (!config_path.empty()) {
        automap_pro::ConfigManager::instance().loadFromFile(config_path);
    }

    automap_pro::SensorManager sensor_mgr;
    sensor_mgr.init(node);

    automap_pro::FastLIVO2Wrapper frontend;
    frontend.init(node, sensor_mgr.imuBuffer(), sensor_mgr.gpsBuffer());

    sensor_mgr.imu().registerCallback([&frontend](const automap_pro::ImuData& imu) {
        frontend.feedImu(imu);
    });
    sensor_mgr.gps().registerCallback([&frontend](const automap_pro::GPSMeasurement& gps) {
        frontend.feedGPS(gps);
    });
    sensor_mgr.lidar().registerCallback([&frontend](const automap_pro::LidarFrame::Ptr& f) {
        frontend.feedLidar(f);
    });

    RCLCPP_INFO(node->get_logger(), "[fast_livo2_node] Running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
