#pragma once

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

    void init(rclcpp::Node::SharedPtr node);

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
