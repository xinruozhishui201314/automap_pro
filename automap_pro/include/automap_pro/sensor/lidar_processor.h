#pragma once
/**
 * @file sensor/lidar_processor.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <deque>
#include <mutex>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "automap_pro/core/data_types.h"
#include "automap_pro/sensor/time_sync.h"

namespace automap_pro {

struct LidarFrame {
    double  timestamp_start = 0.0;
    double  timestamp_end   = 0.0;
    CloudXYZIPtr cloud_raw;
    CloudXYZIPtr cloud_undistorted;
    using Ptr = std::shared_ptr<LidarFrame>;
};

class LidarProcessor {
public:
    using FrameCallback = std::function<void(const LidarFrame::Ptr&)>;

    LidarProcessor();
    ~LidarProcessor() = default;

    void init(rclcpp::Node::SharedPtr node, const TimedBuffer<ImuData>& imu_buffer);
    void registerCallback(FrameCallback cb);
    void process(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    CloudXYZIPtr undistort(const CloudXYZIPtr& raw, double t_start, double t_end,
                           const std::vector<ImuData>& imu_data);
    CloudXYZIPtr rangeFilter(const CloudXYZIPtr& cloud, double min_range, double max_range);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::vector<FrameCallback> callbacks_;
    const TimedBuffer<ImuData>* imu_buffer_ = nullptr;
    double min_range_;
    double max_range_;
    bool   undistort_enabled_;
    double time_offset_;
    rclcpp::Logger logger_{rclcpp::get_logger("automap_pro.lidar_processor")};
    std::string topic_name_;
    uint64_t msg_count_ = 0;
    double last_msg_ts_ = 0.0;
    double last_log_time_ = -1e9;
    static constexpr double kDataFlowLogInterval = 2.0;
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

}  // namespace automap_pro
