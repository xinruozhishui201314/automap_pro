#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

class GPSProcessor {
public:
    using MeasurementCallback = std::function<void(const GPSMeasurement&)>;

    GPSProcessor();
    ~GPSProcessor() = default;

    void init(rclcpp::Node::SharedPtr node);

    void setENUOrigin(double lat, double lon, double alt);
    void registerCallback(MeasurementCallback cb);
    void process(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    GPSState  currentState()  const;
    ENUOrigin enuOrigin()     const;
    bool      hasOrigin()     const;
    void updateOdometryPose(double timestamp, const Eigen::Vector3d& pos);

private:
    GPSQuality assessQuality(const sensor_msgs::msg::NavSatFix& msg) const;
    Eigen::Matrix3d qualityToCovariance(GPSQuality q, double hdop) const;
    bool jumpDetection(const Eigen::Vector3d& pos_enu, double dt);
    bool consistencyCheck(const GPSMeasurement& meas);
    void updateState(bool valid);

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
    rclcpp::Logger logger_{rclcpp::get_logger("automap_pro.gps_processor")};
    std::vector<MeasurementCallback> callbacks_;

    ENUOrigin enu_origin_;
    GPSState state_ = GPSState::INIT;
    std::deque<std::pair<double, Eigen::Vector3d>> gps_history_;
    int consecutive_outliers_  = 0;
    int consecutive_valid_     = 0;
    std::deque<std::pair<double, Eigen::Vector3d>> odom_history_;

    double excellent_hdop_;
    double high_hdop_;
    double medium_hdop_;
    double max_jump_;
    double max_velocity_;
    double chi2_threshold_;
    int    consecutive_valid_required_;
    bool   jump_detection_enabled_;
    bool   consistency_check_enabled_;
    mutable std::mutex mutex_;

    uint64_t msg_count_ = 0;
    double last_msg_ts_ = 0.0;
    double last_log_time_ = -1e9;
    std::string topic_name_;
    static constexpr double kDataFlowLogInterval = 2.0;

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

}  // namespace automap_pro
