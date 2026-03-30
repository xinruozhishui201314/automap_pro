#pragma once
/**
 * @file sensor/image_decompressor.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

namespace automap_pro {

/**
 * @brief CompressedImage解压缩节点
 * 
 * 功能：
 * 1. 订阅CompressedImage话题，解压为Image话题
 * 2. 支持多种压缩格式（JPEG/PNG）
 * 3. 性能监控：统计解压延迟、FPS、数据大小
 * 4. 自动质量评估：检测模糊、过曝等问题
 * 
 * 使用场景：
 * - M2DGR数据集：/camera/head/image_raw/compressed -> /camera/head/image_raw
 * - 任何使用CompressedImage的数据集
 */
class ImageDecompressor : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param node_options ROS2节点选项
     */
    explicit ImageDecompressor(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("image_decompressor", node_options) {
        
        // 声明参数
        this->declare_parameter("input_topic", std::string(""));
        this->declare_parameter("output_topic", std::string(""));
        this->declare_parameter("queue_size", 10);
        this->declare_parameter("enable_quality_check", true);
        this->declare_parameter("log_interval_sec", 2.0);
        this->declare_parameter("warn_latency_ms", 50.0);
        
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        int queue_size = this->get_parameter("queue_size").as_int();
        enable_quality_check_ = this->get_parameter("enable_quality_check").as_bool();
        log_interval_ = this->get_parameter("log_interval_sec").as_double();
        warn_latency_ms_ = this->get_parameter("warn_latency_ms").as_double();
        
        RCLCPP_INFO(this->get_logger(), 
                    "[ImageDecompressor] Initializing...");
        RCLCPP_INFO(this->get_logger(), 
                    "[ImageDecompressor] Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), 
                    "[ImageDecompressor] Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), 
                    "[ImageDecompressor] Queue size: %d", queue_size);
        
        // 创建订阅者和发布者
        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic, queue_size,
            std::bind(&ImageDecompressor::compressedCallback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, queue_size);
        
        // 性能统计初始化
        last_log_time_ = this->now();
        frame_count_ = 0;
        total_latency_ms_ = 0.0;
        total_compressed_bytes_ = 0;
        total_decompressed_bytes_ = 0;
        
        RCLCPP_INFO(this->get_logger(), 
                    "[ImageDecompressor] Ready!");
    }
    
private:
    /**
     * @brief CompressedImage回调函数
     * @param msg 压缩图像消息
     */
    void compressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto start_time = this->now();
        
        try {
            // 解压图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            
            // 质量检查
            if (enable_quality_check_) {
                checkImageQuality(image);
            }
            
            // 转换为Image消息
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_ptr->toImageMsg();
            img_msg->header = msg->header;  // 保持原始时间戳
            
            // 发布
            pub_->publish(*img_msg);
            
            // 性能统计
            auto end_time = this->now();
            double latency_ms = (end_time - start_time).seconds() * 1000.0;
            
            frame_count_++;
            total_latency_ms_ += latency_ms;
            total_compressed_bytes_ += msg->data.size();
            total_decompressed_bytes_ += image.total() * image.elemSize();
            
            // 延迟警告
            if (latency_ms > warn_latency_ms_) {
                RCLCPP_WARN(this->get_logger(),
                           "[ImageDecompressor] High decompression latency: %.2f ms (threshold: %.2f ms)",
                           latency_ms, warn_latency_ms_);
            }
            
            // 定期日志
            auto now = this->now();
            if ((now - last_log_time_).seconds() >= log_interval_) {
                logStatistics();
                last_log_time_ = now;
                frame_count_ = 0;
                total_latency_ms_ = 0.0;
                total_compressed_bytes_ = 0;
                total_decompressed_bytes_ = 0;
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                        "[ImageDecompressor] cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                        "[ImageDecompressor] Exception: %s", e.what());
        }
    }
    
    /**
     * @brief 图像质量检查
     * @param image OpenCV图像
     */
    void checkImageQuality(const cv::Mat& image) {
        // 检查图像尺寸
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                        "[ImageDecompressor] Empty image detected!");
            return;
        }
        
        // 检查模糊度（使用Laplacian方差）
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat laplacian;
        cv::Laplacian(gray, laplacian, CV_64F);
        cv::Scalar mu, sigma;
        cv::meanStdDev(laplacian, mu, sigma);
        double sharpness = sigma.val[0] * sigma.val[0];
        
        // 检查过曝/欠曝
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar mean_value = cv::mean(hsv);
        double brightness = mean_value.val[2];  // V通道
        
        // 阈值警告
        if (sharpness < 10.0) {
            RCLCPP_WARN(this->get_logger(),
                       "[ImageDecompressor] Low sharpness detected: %.2f (threshold: 10.0)",
                       sharpness);
        }
        
        if (brightness > 240.0) {
            RCLCPP_WARN(this->get_logger(),
                       "[ImageDecompressor] Overexposed image detected: %.1f (threshold: 240)",
                       brightness);
        } else if (brightness < 15.0) {
            RCLCPP_WARN(this->get_logger(),
                       "[ImageDecompressor] Underexposed image detected: %.1f (threshold: 15)",
                       brightness);
        }
        
        // 记录统计
        last_sharpness_ = sharpness;
        last_brightness_ = brightness;
    }
    
    /**
     * @brief 记录性能统计
     */
    void logStatistics() {
        if (frame_count_ == 0) return;
        
        double avg_latency_ms = total_latency_ms_ / frame_count_;
        double fps = frame_count_ / log_interval_;
        double compression_ratio = total_compressed_bytes_ > 0 ? 
                                  (double)total_decompressed_bytes_ / total_compressed_bytes_ : 0.0;
        double avg_compressed_kb = (double)total_compressed_bytes_ / frame_count_ / 1024.0;
        double avg_decompressed_kb = (double)total_decompressed_bytes_ / frame_count_ / 1024.0;
        
        RCLCPP_INFO(this->get_logger(),
                    "[ImageDecompressor] Stats: FPS=%.1f | AvgLatency=%.2fms | "
                    "CompressionRatio=%.1fx | AvgSize=%.1fKB->%.1fKB | "
                    "Sharpness=%.1f | Brightness=%.1f",
                    fps, avg_latency_ms, compression_ratio,
                    avg_compressed_kb, avg_decompressed_kb,
                    last_sharpness_, last_brightness_);
    }
    
    // ROS2节点组件
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    
    // 配置参数
    bool enable_quality_check_;
    double log_interval_;
    double warn_latency_ms_;
    
    // 性能统计
    rclcpp::Time last_log_time_;
    uint64_t frame_count_;
    double total_latency_ms_;
    uint64_t total_compressed_bytes_;
    uint64_t total_decompressed_bytes_;
    
    // 质量指标
    double last_sharpness_ = 0.0;
    double last_brightness_ = 0.0;
};

}  // namespace automap_pro
