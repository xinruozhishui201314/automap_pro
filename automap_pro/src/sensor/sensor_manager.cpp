/**
 * @file sensor/sensor_manager.cpp
 * @brief 传感器驱动与同步实现。
 */
#include "automap_pro/sensor/sensor_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

namespace automap_pro {

SensorManager::SensorManager()
    : lidar_(std::make_unique<LidarProcessor>())
    , imu_  (std::make_unique<ImuProcessor>())
    , gps_  (std::make_unique<GPSProcessor>())
{
    ALOG_DEBUG("SensorManager", "SensorManager constructed");
}

void SensorManager::init(rclcpp::Node::SharedPtr node, bool subscribe_lidar_imu) {
    try {
        if (!node) {
            throw std::invalid_argument("SensorManager::init: node is null");
        }
        
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
        
        ALOG_INFO("SensorManager", "SensorManager initialization completed");
    } catch (const std::exception& e) {
        ALOG_ERROR("SensorManager", "init failed: {}", e.what());
        throw;
    } catch (...) {
        ALOG_ERROR("SensorManager", "init failed: unknown exception");
        throw;
    }
}

LidarProcessor&          SensorManager::lidar()     { return *lidar_; }
ImuProcessor&            SensorManager::imu()       { return *imu_; }
GPSProcessor&            SensorManager::gps()       { return *gps_; }
TimedBuffer<ImuData>&    SensorManager::imuBuffer() { return imu_buffer_; }
TimedBuffer<GPSMeasurement>& SensorManager::gpsBuffer() { return gps_buffer_; }

bool SensorManager::isReady() const {
    try {
        return !imu_buffer_.empty() && !gps_buffer_.empty();
    } catch (const std::exception& e) {
        ALOG_ERROR("SensorManager", "isReady check failed: {}", e.what());
        return false;
    }
}

}  // namespace automap_pro
