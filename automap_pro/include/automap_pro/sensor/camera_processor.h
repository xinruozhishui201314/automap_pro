#pragma once

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
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace automap_pro
