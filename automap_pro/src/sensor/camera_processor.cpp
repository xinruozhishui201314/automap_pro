#include "automap_pro/sensor/camera_processor.h"
#include "automap_pro/core/config_manager.h"

#include <chrono>

namespace automap_pro {

void CameraProcessor::init(rclcpp::Node::SharedPtr node) {
    logger_ = node->get_logger();
    if (!ConfigManager::instance().cameraEnabled()) {
        RCLCPP_INFO(logger_, "[CameraProcessor] Camera disabled in config.");
        return;
    }
    topic_name_ = ConfigManager::instance().cameraTopic();
    sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        topic_name_, 10, std::bind(&CameraProcessor::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[CameraProcessor] Subscribing to %s", topic_name_.c_str());
}

void CameraProcessor::registerCallback(ImageCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void CameraProcessor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    process(msg);
    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    last_msg_ts_ = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
    msg_count_++;
    if (now - last_log_time_ >= kDataFlowLogInterval) {
        RCLCPP_INFO(logger_, "[DataFlow] Camera | topic=%s | count=%lu | last_ts=%.3f | %dx%d",
                    topic_name_.c_str(), static_cast<unsigned long>(msg_count_), last_msg_ts_,
                    msg->width, msg->height);
        msg_count_ = 0;
        last_log_time_ = now;
    }
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
