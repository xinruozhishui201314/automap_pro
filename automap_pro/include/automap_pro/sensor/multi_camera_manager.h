#pragma once
/**
 * @file sensor/multi_camera_manager.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <unordered_map>
#include <functional>
#include <string>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

/**
 * @brief 相机类型枚举（支持M2DGR多相机）
 */
enum class CameraType {
    HEAD,       // 前视相机 (Cam-head 3199)
    LEFT,       // 左相机 (Cam-left 8823)
    RIGHT,      // 右相机 (Cam-right 8828)
    MID_LEFT,   // 中左相机 (Cam-midleft 8830)
    MID_RIGHT,  // 中右相机 (Cam-midright 8827)
    BACK_LEFT,  // 后左相机 (Cam-backleft 6450)
    BACK_RIGHT, // 后右相机 (Cam-backright 6548)
    THERMAL,    // 热成像相机
    PINHOLE     // RealSense D435i RGB
};

/**
 * @brief 相机配置结构体
 */
struct CameraConfig {
    CameraType type;
    std::string topic;
    std::string frame_id;
    double fx, fy, cx, cy;      // 内参
    double k1, k2, p1, p2;      // 畸变系数
    int width, height;
    cv::Mat R;                  // 旋转矩阵（到LiDAR）
    cv::Mat t;                  // 平移向量（到LiDAR）
    bool enabled;
    
    // 外参Rcl和Pcl (FAST-LIVO格式)
    std::vector<double> Rcl;     // 旋转矩阵(行优先)
    std::vector<double> Pcl;     // 平移向量
};

/**
 * @brief 多相机管理器
 * 
 * 功能：
 * 1. 支持M2DGR数据集多相机配置（Cam-head/Cam-left/Cam-right等）
 * 2. 运行时切换活跃相机
 * 3. 多相机同步订阅
 * 4. 相机外参自动管理
 * 5. 支持相机标定文件加载
 * 
 * 使用场景：
 * - M2DGR数据集多相机测试
 * - 不同场景选择不同相机（前视/侧视/后视）
 * - 多相机融合建图
 */
class MultiCameraManager : public rclcpp::Node {
public:
    using ImageCallback = std::function<void(CameraType, double, const cv::Mat&)>;
    
    explicit MultiCameraManager(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("multi_camera_manager", node_options) {
        
        // 声明参数
        this->declare_parameter("active_camera", std::string("head"));
        this->declare_parameter("enable_multi_camera", false);
        this->declare_parameter("calibration_file", std::string(""));
        this->declare_parameter("auto_quality_switch", false);
        this->declare_parameter("quality_threshold", 50.0);
        this->declare_parameter("queue_size", 10);
        
        // 获取参数
        active_camera_name_ = this->get_parameter("active_camera").as_string();
        enable_multi_camera_ = this->get_parameter("enable_multi_camera").as_bool();
        calib_file_path_ = this->get_parameter("calibration_file").as_string();
        auto_quality_switch_ = this->get_parameter("auto_quality_switch").as_bool();
        quality_threshold_ = this->get_parameter("quality_threshold").as_double();
        queue_size_ = this->get_parameter("queue_size").as_int();
        
        RCLCPP_INFO(this->get_logger(), 
                    "[MultiCameraManager] Initializing...");
        RCLCPP_INFO(this->get_logger(), 
                    "[MultiCameraManager] Active camera: %s", active_camera_name_.c_str());
        RCLCPP_INFO(this->get_logger(), 
                    "[MultiCameraManager] Multi-camera mode: %s", 
                    enable_multi_camera_ ? "enabled" : "disabled");
        
        // 初始化M2DGR相机配置
        initM2DGRCameras();
        
        // 加载标定文件（如果提供）
        if (!calib_file_path_.empty()) {
            loadCalibrationFile(calib_file_path_);
        }
        
        // 创建订阅者
        setupSubscriptions();
        
        RCLCPP_INFO(this->get_logger(), 
                    "[MultiCameraManager] Ready!");
    }
    
    /**
     * @brief 注册图像回调
     * @param cb 回调函数
     */
    void registerCallback(ImageCallback cb) {
        callbacks_.push_back(std::move(cb));
    }
    
    /**
     * @brief 切换活跃相机
     * @param camera_name 相机名称 (head/left/right等)
     * @return 是否成功
     */
    bool switchCamera(const std::string& camera_name) {
        auto type = stringToCameraType(camera_name);
        if (type == CameraType::PINHOLE && camera_name != "pinhole") {
            RCLCPP_ERROR(this->get_logger(), 
                        "[MultiCameraManager] Unknown camera: %s", camera_name.c_str());
            return false;
        }
        
        active_camera_name_ = camera_name;
        RCLCPP_INFO(this->get_logger(), 
                    "[MultiCameraManager] Switched active camera to: %s", camera_name.c_str());
        return true;
    }
    
    /**
     * @brief 获取活跃相机配置
     * @return 相机配置
     */
    CameraConfig getActiveCameraConfig() const {
        auto type = stringToCameraType(active_camera_name_);
        auto it = camera_configs_.find(type);
        if (it != camera_configs_.end()) {
            return it->second;
        }
        return CameraConfig{};
    }
    
    /**
     * @brief 获取指定相机配置
     * @param type 相机类型
     * @return 相机配置
     */
    CameraConfig getCameraConfig(CameraType type) const {
        auto it = camera_configs_.find(type);
        if (it != camera_configs_.end()) {
            return it->second;
        }
        return CameraConfig{};
    }
    
    /**
     * @brief 获取所有相机配置
     * @return 相机配置列表
     */
    std::vector<CameraConfig> getAllCameraConfigs() const {
        std::vector<CameraConfig> configs;
        for (const auto& [type, config] : camera_configs_) {
            configs.push_back(config);
        }
        return configs;
    }
    
    /**
     * @brief 启用指定相机
     * @param camera_name 相机名称
     */
    void enableCamera(const std::string& camera_name) {
        auto type = stringToCameraType(camera_name);
        auto it = camera_configs_.find(type);
        if (it != camera_configs_.end()) {
            it->second.enabled = true;
            RCLCPP_INFO(this->get_logger(), 
                        "[MultiCameraManager] Camera enabled: %s", camera_name.c_str());
        }
    }
    
    /**
     * @brief 禁用指定相机
     * @param camera_name 相机名称
     */
    void disableCamera(const std::string& camera_name) {
        auto type = stringToCameraType(camera_name);
        auto it = camera_configs_.find(type);
        if (it != camera_configs_.end()) {
            it->second.enabled = false;
            RCLCPP_INFO(this->get_logger(), 
                        "[MultiCameraManager] Camera disabled: %s", camera_name.c_str());
        }
    }
    
    /**
     * @brief 打印所有相机状态
     */
    void printCameraStatus() const {
        RCLCPP_INFO(this->get_logger(), "[MultiCameraManager] Camera Status:");
        RCLCPP_INFO(this->get_logger(), "  %-15s %-8s %-30s %-10s", 
                    "Camera", "Enabled", "Topic", "Resolution");
        RCLCPP_INFO(this->get_logger(), "  %s", 
                    "----------------------------------------------------------------");
        
        for (const auto& [type, config] : camera_configs_) {
            std::string type_str = cameraTypeToString(type);
            std::string enabled_str = config.enabled ? "YES" : "NO";
            std::string topic_str = config.topic;
            if (topic_str.length() > 28) topic_str = topic_str.substr(0, 28) + "..";
            std::string res_str = std::to_string(config.width) + "x" + std::to_string(config.height);
            
            RCLCPP_INFO(this->get_logger(), "  %-15s %-8s %-30s %-10s",
                        type_str.c_str(), enabled_str.c_str(), 
                        topic_str.c_str(), res_str.c_str());
        }
    }

private:
    /**
     * @brief 初始化M2DGR相机配置
     */
    void initM2DGRCameras() {
        // Cam-head (前视相机, ID:3199)
        CameraConfig head_config;
        head_config.type = CameraType::HEAD;
        head_config.topic = "/camera/head/image_raw/compressed";
        head_config.frame_id = "camera_head";
        head_config.fx = 542.993253538048;
        head_config.fy = 541.3882904458247;
        head_config.cx = 629.0025857364897;
        head_config.cy = 503.71809588651786;
        head_config.k1 = -0.057963907006683066;
        head_config.k2 = -0.026465594265953234;
        head_config.p1 = 0.011980216320790046;
        head_config.p2 = -0.003041081642470451;
        head_config.width = 1280;
        head_config.height = 1024;
        head_config.Rcl = {0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0};
        head_config.Pcl = {0.07410, 0.00127, 0.65608};
        head_config.enabled = (active_camera_name_ == "head");
        camera_configs_[CameraType::HEAD] = head_config;
        
        // Cam-left (左相机, ID:8823)
        CameraConfig left_config;
        left_config.type = CameraType::LEFT;
        left_config.topic = "/camera/left/image_raw/compressed";
        left_config.frame_id = "camera_left";
        left_config.fx = 540.645056202188;
        left_config.fy = 539.8545023658869;
        left_config.cx = 626.4125666883942;
        left_config.cy = 523.947634226782;
        left_config.k1 = -0.07015146608431883;
        left_config.k2 = 0.008586142263125124;
        left_config.p1 = -0.021968993685891842;
        left_config.p2 = 0.007442211946112636;
        left_config.width = 1280;
        left_config.height = 1024;
        left_config.Rcl = {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
        left_config.Pcl = {0.24221, 0.16123, -0.16711};
        left_config.enabled = (active_camera_name_ == "left");
        camera_configs_[CameraType::LEFT] = left_config;
        
        // Cam-right (右相机, ID:8828)
        CameraConfig right_config;
        right_config.type = CameraType::RIGHT;
        right_config.topic = "/camera/right/image_raw/compressed";
        right_config.frame_id = "camera_right";
        right_config.fx = 540.6832252229977;
        right_config.fy = 539.3921307247979;
        right_config.cx = 632.9173957218305;
        right_config.cy = 503.3766864767991;
        right_config.k1 = -0.07147685334620411;
        right_config.k2 = 0.006423830171528276;
        right_config.p1 = -0.02354604292216998;
        right_config.p2 = 0.009181757660952325;
        right_config.width = 1280;
        right_config.height = 1024;
        right_config.Rcl = {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
        right_config.Pcl = {0.242013, -0.16025, -0.16724};
        right_config.enabled = (active_camera_name_ == "right");
        camera_configs_[CameraType::RIGHT] = right_config;
        
        // 打印初始化状态
        printCameraStatus();
    }
    
    /**
     * @brief 设置订阅者
     */
    void setupSubscriptions() {
        if (enable_multi_camera_) {
            // 多相机模式：订阅所有启用的相机
            for (auto& [type, config] : camera_configs_) {
                if (config.enabled) {
                    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
                        config.topic, queue_size_,
                        [this, type](const sensor_msgs::msg::Image::SharedPtr msg) {
                            imageCallback(type, msg);
                        });
                    subs_[type] = sub;
                    RCLCPP_INFO(this->get_logger(), 
                                "[MultiCameraManager] Subscribed: %s", config.topic.c_str());
                }
            }
        } else {
            // 单相机模式：只订阅活跃相机
            auto type = stringToCameraType(active_camera_name_);
            auto it = camera_configs_.find(type);
            if (it != camera_configs_.end() && it->second.enabled) {
                auto sub = this->create_subscription<sensor_msgs::msg::Image>(
                    it->second.topic, queue_size_,
                    [this, type](const sensor_msgs::msg::Image::SharedPtr msg) {
                        imageCallback(type, msg);
                    });
                subs_[type] = sub;
                RCLCPP_INFO(this->get_logger(), 
                            "[MultiCameraManager] Subscribed (active): %s", 
                            it->second.topic.c_str());
            }
        }
    }
    
    /**
     * @brief 图像回调函数
     * @param type 相机类型
     * @param msg 图像消息
     */
    void imageCallback(CameraType type, const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            double ts = rclcpp::Time(msg->header.stamp).seconds();
            
            // 质量检查（如果启用自动切换）
            if (auto_quality_switch_) {
                double quality = computeImageQuality(image);
                if (quality < quality_threshold_) {
                    RCLCPP_WARN(this->get_logger(),
                               "[MultiCameraManager] Low quality detected: %.2f (threshold: %.2f)",
                               quality, quality_threshold_);
                    // 可以在这里添加自动切换逻辑
                }
            }
            
            // 触发回调
            for (auto& cb : callbacks_) {
                cb(type, ts, image);
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                        "[MultiCameraManager] cv_bridge exception: %s", e.what());
        }
    }
    
    /**
     * @brief 计算图像质量（Laplacian方差）
     * @param image OpenCV图像
     * @return 质量分数
     */
    double computeImageQuality(const cv::Mat& image) {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat laplacian;
        cv::Laplacian(gray, laplacian, CV_64F);
        cv::Scalar mu, sigma;
        cv::meanStdDev(laplacian, mu, sigma);
        return sigma.val[0] * sigma.val[0];
    }
    
    /**
     * @brief 加载标定文件
     * @param filepath 标定文件路径
     * @return 是否成功
     */
    bool loadCalibrationFile(const std::string& filepath) {
        RCLCPP_INFO(this->get_logger(), 
                    "[MultiCameraManager] Loading calibration from: %s", filepath.c_str());
        
        try {
            cv::FileStorage fs(filepath, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", filepath.c_str());
                return false;
            }

            for (auto& [type, config] : camera_configs_) {
                std::string prefix = cameraTypeToString(type);
                cv::FileNode node = fs[prefix];
                if (node.empty()) continue;

                node["fx"] >> config.fx;
                node["fy"] >> config.fy;
                node["cx"] >> config.cx;
                node["cy"] >> config.cy;
                node["k1"] >> config.k1;
                node["k2"] >> config.k2;
                node["p1"] >> config.p1;
                node["p2"] >> config.p2;
                node["width"] >> config.width;
                node["height"] >> config.height;
                
                cv::Mat R, t;
                node["R"] >> R;
                node["t"] >> t;
                if (!R.empty()) config.R = R.clone();
                if (!t.empty()) config.t = t.clone();

                RCLCPP_INFO(this->get_logger(), "[MultiCameraManager] Loaded calibration for: %s", prefix.c_str());
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading calibration file: %s", e.what());
            return false;
        }
    }
    
    /**
     * @brief 相机类型转字符串
     * @param type 相机类型
     * @return 字符串
     */
    std::string cameraTypeToString(CameraType type) const {
        switch (type) {
            case CameraType::HEAD: return "head";
            case CameraType::LEFT: return "left";
            case CameraType::RIGHT: return "right";
            case CameraType::MID_LEFT: return "mid_left";
            case CameraType::MID_RIGHT: return "mid_right";
            case CameraType::BACK_LEFT: return "back_left";
            case CameraType::BACK_RIGHT: return "back_right";
            case CameraType::THERMAL: return "thermal";
            case CameraType::PINHOLE: return "pinhole";
            default: return "unknown";
        }
    }
    
    /**
     * @brief 字符串转相机类型
     * @param str 字符串
     * @return 相机类型
     */
    CameraType stringToCameraType(const std::string& str) const {
        if (str == "head") return CameraType::HEAD;
        if (str == "left") return CameraType::LEFT;
        if (str == "right") return CameraType::RIGHT;
        if (str == "mid_left") return CameraType::MID_LEFT;
        if (str == "mid_right") return CameraType::MID_RIGHT;
        if (str == "back_left") return CameraType::BACK_LEFT;
        if (str == "back_right") return CameraType::BACK_RIGHT;
        if (str == "thermal") return CameraType::THERMAL;
        if (str == "pinhole") return CameraType::PINHOLE;
        return CameraType::PINHOLE;
    }
    
    // 相机配置
    std::unordered_map<CameraType, CameraConfig> camera_configs_;
    
    // ROS2订阅者
    std::unordered_map<CameraType, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
    
    // 回调函数
    std::vector<ImageCallback> callbacks_;
    
    // 配置参数
    std::string active_camera_name_;
    bool enable_multi_camera_;
    std::string calib_file_path_;
    bool auto_quality_switch_;
    double quality_threshold_;
    int queue_size_;
};

}  // namespace automap_pro
