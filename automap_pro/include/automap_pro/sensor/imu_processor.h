#pragma once

#include <deque>
#include <mutex>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/sensor/time_sync.h"

namespace automap_pro {

class ImuProcessor {
public:
    using ImuCallback = std::function<void(const ImuData&)>;

    ImuProcessor();
    ~ImuProcessor() = default;

    void init(rclcpp::Node::SharedPtr node, TimedBuffer<ImuData>& imu_buffer);
    void registerCallback(ImuCallback cb);
    void process(const sensor_msgs::msg::Imu::SharedPtr msg);

    struct PreintResult {
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
        Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
        double dt_total = 0.0;
    };

    PreintResult preintegrate(const std::vector<ImuData>& imu_data,
                               double t_start, double t_end,
                               const Eigen::Vector3d& gyro_bias,
                               const Eigen::Vector3d& accel_bias,
                               const Eigen::Vector3d& gravity) const;

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    std::vector<ImuCallback> callbacks_;
    TimedBuffer<ImuData>* imu_buffer_ = nullptr;
    double time_offset_ = 0.0;
    double gravity_     = 9.81;
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

}  // namespace automap_pro
