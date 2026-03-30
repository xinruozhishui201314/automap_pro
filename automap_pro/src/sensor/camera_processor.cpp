/**
 * @file sensor/camera_processor.cpp
 * @brief 传感器驱动与同步实现。
 */
#include "automap_pro/sensor/camera_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

#include <chrono>

#define MOD "CameraProcessor"

namespace automap_pro {

void CameraProcessor::init(rclcpp::Node::SharedPtr node) {
    try {
        if (!node) {
            throw std::invalid_argument("CameraProcessor::init: node is null");
        }
        
        logger_ = node->get_logger();
        
        if (!ConfigManager::instance().cameraEnabled()) {
            RCLCPP_INFO(logger_, "[CameraProcessor] Camera disabled in config.");
            ALOG_INFO(MOD, "CameraProcessor disabled by config");
            return;
        }
        
        topic_name_ = ConfigManager::instance().cameraTopic();
        sub_ = node->create_subscription<sensor_msgs::msg::Image>(
            topic_name_, 10, std::bind(&CameraProcessor::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(logger_, "[CameraProcessor][TOPIC] subscribe: %s", topic_name_.c_str());
        ALOG_INFO(MOD, "CameraProcessor initialized: topic={}", topic_name_);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "CameraProcessor::init failed: {}", e.what());
        throw;
    }
}

void CameraProcessor::registerCallback(ImageCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void CameraProcessor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        if (!msg) {
            ALOG_WARN(MOD, "CameraProcessor: received null message");
            return;
        }
        
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
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "CameraProcessor::imageCallback exception: {}", e.what());
    } catch (...) {
        ALOG_ERROR(MOD, "CameraProcessor::imageCallback unknown exception");
    }
}

void CameraProcessor::process(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        if (!msg) {
            ALOG_WARN(MOD, "CameraProcessor::process: null message");
            return;
        }
        
        // 验证图像数据有效性
        if (msg->width == 0 || msg->height == 0) {
            ALOG_WARN(MOD, "CameraProcessor: received empty image (0x0)");
            return;
        }
        
        if (msg->data.empty()) {
            ALOG_WARN(MOD, "CameraProcessor: received image with no data");
            return;
        }
        
        double ts = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
        
        // 验证时间戳有效性
        if (!std::isfinite(ts) || ts <= 0) {
            ALOG_WARN(MOD, "CameraProcessor: invalid timestamp {:.3f}", ts);
            return;
        }
        
        cv::Mat image;
        try {
            // 根据编码格式选择转换方式
            std::string encoding = msg->encoding;
            
            if (encoding == sensor_msgs::image_encodings::BGR8 ||
                encoding == sensor_msgs::image_encodings::RGB8 ||
                encoding == sensor_msgs::image_encodings::MONO8 ||
                encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                // 直接支持的格式
                image = cv_bridge::toCvCopy(msg, encoding)->image;
            } else {
                // 尝试转换为 BGR8（通用格式）
                image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            }
        } catch (const cv_bridge::Exception& e) {
            ALOG_WARN(MOD, "cv_bridge exception (encoding={}): {}", msg->encoding, e.what());
            return;
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "cv_bridge conversion failed: {}", e.what());
            return;
        }
        
        // 验证转换后的图像
        if (image.empty()) {
            ALOG_WARN(MOD, "CameraProcessor: converted image is empty");
            return;
        }
        
        // 检查图像数据有效性
        if (!image.data || image.cols <= 0 || image.rows <= 0) {
            ALOG_WARN(MOD, "CameraProcessor: invalid image data after conversion");
            return;
        }
        
        // 检查图像是否包含有效像素（非全黑）
        if (cv::countNonZero(image) == 0 && image.channels() == 1) {
            ALOG_DEBUG(MOD, "CameraProcessor: received all-black image");
        }
        
        for (auto& cb : callbacks_) {
            try {
                cb(ts, image);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "Camera callback exception: {}", e.what());
            }
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "CameraProcessor::process exception: {}", e.what());
    } catch (...) {
        ALOG_ERROR(MOD, "CameraProcessor::process unknown exception");
    }
}

}  // namespace automap_pro
