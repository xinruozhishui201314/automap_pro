#pragma once
/**
 * @file sensor/camera_processor.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <functional>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

class CameraProcessor {
public:
    using ImageCallback = std::function<void(double, const cv::Mat&)>;

    CameraProcessor() = default;
    ~CameraProcessor() = default;

    void init(rclcpp::Node::SharedPtr node);
    void registerCallback(ImageCallback cb);
    void process(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::vector<ImageCallback> callbacks_;
    double time_offset_ = 0.0;
    rclcpp::Logger logger_{rclcpp::get_logger("automap_pro.camera_processor")};
    std::string topic_name_;
    uint64_t msg_count_ = 0;
    double last_msg_ts_ = 0.0;
    double last_log_time_ = -1e9;
    static constexpr double kDataFlowLogInterval = 2.0;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace automap_pro
