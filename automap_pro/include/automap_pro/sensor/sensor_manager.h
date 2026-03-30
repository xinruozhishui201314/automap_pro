#pragma once
/**
 * @file sensor/sensor_manager.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/sensor/lidar_processor.h"
#include "automap_pro/sensor/imu_processor.h"
#include "automap_pro/sensor/gps_processor.h"
#include "automap_pro/sensor/time_sync.h"

namespace automap_pro {

class SensorManager {
public:
    SensorManager();
    ~SensorManager() = default;

    /** 初始化传感器管理。subscribe_lidar_imu=true 时订阅 lidar/IMU/GPS 供内部前端；false 时仅订阅 GPS（external_fast_livo 时由 fast_livo 节点作为唯一数据入口）。 */
    void init(rclcpp::Node::SharedPtr node, bool subscribe_lidar_imu = true);

    LidarProcessor&          lidar();
    ImuProcessor&            imu();
    GPSProcessor&            gps();
    TimedBuffer<ImuData>&    imuBuffer();
    TimedBuffer<GPSMeasurement>& gpsBuffer();

    bool isReady() const;

private:
    std::unique_ptr<LidarProcessor> lidar_;
    std::unique_ptr<ImuProcessor>   imu_;
    std::unique_ptr<GPSProcessor>   gps_;

    TimedBuffer<ImuData>         imu_buffer_{50000, 10.0};
    TimedBuffer<GPSMeasurement>  gps_buffer_{1000, 30.0};
};

}  // namespace automap_pro
