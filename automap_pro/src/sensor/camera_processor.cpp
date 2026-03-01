#include "automap_pro/sensor/camera_processor.h"
#include "automap_pro/core/config_manager.h"

namespace automap_pro {

void CameraProcessor::init(rclcpp::Node::SharedPtr node) {
    logger_ = node->get_logger();
    if (!ConfigManager::instance().cameraEnabled()) {
        RCLCPP_INFO(logger_, "[CameraProcessor] Camera disabled in config.");
        return;
    }
    std::string topic = ConfigManager::instance().cameraTopic();
    sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        topic, 10, std::bind(&CameraProcessor::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[CameraProcessor] Subscribing to %s", topic.c_str());
}

void CameraProcessor::registerCallback(ImageCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void CameraProcessor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    process(msg);
}

void CameraProcessor::process(const sensor_msgs::msg::Image::SharedPtr msg) {
    double ts = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
    cv::Mat image;
    try {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN(logger_, "[CameraProcessor] cv_bridge exception: %s", e.what());
        return;
    }
    for (auto& cb : callbacks_) cb(ts, image);
}

}  // namespace automap_pro
