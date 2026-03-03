#include "automap_pro/sensor/sensor_manager.h"
#include "automap_pro/core/config_manager.h"

namespace automap_pro {

SensorManager::SensorManager()
    : lidar_(std::make_unique<LidarProcessor>())
    , imu_  (std::make_unique<ImuProcessor>())
    , gps_  (std::make_unique<GPSProcessor>())
{}

void SensorManager::init(rclcpp::Node::SharedPtr node, bool subscribe_lidar_imu) {
    if (subscribe_lidar_imu) {
        imu_->init(node, imu_buffer_);
        lidar_->init(node, imu_buffer_);
    }

    gps_->registerCallback([this](const GPSMeasurement& meas) {
        gps_buffer_.push(meas.timestamp, meas);
    });
    gps_->init(node);

    if (subscribe_lidar_imu) {
        RCLCPP_INFO(node->get_logger(), "[SensorManager] Initialized lidar/IMU/GPS (internal frontend).");
    } else {
        RCLCPP_INFO(node->get_logger(), "[SensorManager] Initialized GPS only (external fast_livo = single data entry for lidar/IMU/image).");
    }
}

LidarProcessor&          SensorManager::lidar()     { return *lidar_; }
ImuProcessor&            SensorManager::imu()       { return *imu_; }
GPSProcessor&            SensorManager::gps()       { return *gps_; }
TimedBuffer<ImuData>&    SensorManager::imuBuffer() { return imu_buffer_; }
TimedBuffer<GPSMeasurement>& SensorManager::gpsBuffer() { return gps_buffer_; }

bool SensorManager::isReady() const {
    return !imu_buffer_.empty() && !gps_buffer_.empty();
}

}  // namespace automap_pro
