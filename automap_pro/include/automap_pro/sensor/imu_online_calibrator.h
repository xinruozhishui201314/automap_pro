#pragma once
/**
 * @file sensor/imu_online_calibrator.h
 * @brief 传感器：LiDAR/IMU/GPS/相机/时间同步与在线标定。
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>
#include <Eigen/Dense>
#include <memory>
#include <functional>

namespace automap_pro {

/**
 * @brief IMU在线标定器
 * 
 * 功能：
 * 1. IMU偏置在线估计 (静止时估计加速度/陀螺仪零偏)
 * 2. 动态调整IMU噪声参数 (根据实际数据统计)
 * 3. IMU健康监控 (检测异常数据)
 * 4. 支持多IMU切换
 * 
 * 使用场景：
 * - Xsens IMU偏置估计
 * - 实时调整fast_livo的IMU噪声参数
 * - IMU故障检测
 */
class IMUOnlineCalibrator : public rclcpp::Node {
public:
    explicit IMUOnlineCalibrator(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("imu_online_calibrator", node_options) {
        
        // 声明参数
        this->declare_parameter("imu_topic", std::string("/handsfree/imu"));
        this->declare_parameter("queue_size", 1000);
        this->declare_parameter("bias_window_size", 200);  // 估计偏置的窗口大小
        this->declare_parameter("noise_window_size", 1000); // 估计噪声的窗口大小
        this->declare_parameter("static_threshold_acc", 0.1);  // 静止加速度阈值 (m/s^2)
        this->declare_parameter("static_threshold_gyr", 0.05); // 静止角速度阈值 (rad/s)
        this->declare_parameter("min_static_samples", 50);      // 最小静止样本数
        this->declare_parameter("bias_update_interval", 1.0);    // 偏置更新间隔 (秒)
        this->declare_parameter("noise_update_interval", 2.0);   // 噪声更新间隔 (秒)
        this->declare_parameter("enable_auto_noise_adjust", true); // 启用自动噪声调整
        this->declare_parameter("noise_adjust_factor", 1.5);     // 噪声调整因子
        
        // 获取参数
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        queue_size_ = this->get_parameter("queue_size").as_int();
        bias_window_size_ = this->get_parameter("bias_window_size").as_int();
        noise_window_size_ = this->get_parameter("noise_window_size").as_int();
        static_threshold_acc_ = this->get_parameter("static_threshold_acc").as_double();
        static_threshold_gyr_ = this->get_parameter("static_threshold_gyr").as_double();
        min_static_samples_ = this->get_parameter("min_static_samples").as_int();
        bias_update_interval_ = this->get_parameter("bias_update_interval").as_double();
        noise_update_interval_ = this->get_parameter("noise_update_interval").as_double();
        enable_auto_noise_adjust_ = this->get_parameter("enable_auto_noise_adjust").as_bool();
        noise_adjust_factor_ = this->get_parameter("noise_adjust_factor").as_double();
        
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] Initializing...");
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] IMU topic: %s", imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] Bias window size: %d", bias_window_size_);
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] Noise window size: %d", noise_window_size_);
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] Auto noise adjust: %s", 
                    enable_auto_noise_adjust_ ? "enabled" : "disabled");
        
        // 创建订阅者
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, queue_size_,
            std::bind(&IMUOnlineCalibrator::imuCallback, this, std::placeholders::_1));
        
        // 创建发布者（发布标定后的IMU数据）
        calibrated_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            imu_topic_ + "/calibrated", queue_size_);
        
        // 创建发布者（发布偏置和噪声参数）
        bias_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/imu_calibrator/bias_acc", 10);
        bias_gyr_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/imu_calibrator/bias_gyr", 10);
        noise_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/imu_calibrator/noise", 10);
        static_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/imu_calibrator/is_static", 10);
        
        // 初始化状态
        bias_acc_ = Eigen::Vector3d::Zero();
        bias_gyr_ = Eigen::Vector3d::Zero();
        noise_acc_ = Eigen::Vector3d::Zero();
        noise_gyr_ = Eigen::Vector3d::Zero();
        last_bias_update_time_ = this->now();
        last_noise_update_time_ = this->now();
        static_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] Ready!");
    }
    
    /**
     * @brief 获取加速度计偏置
     */
    Eigen::Vector3d getAccBias() const { return bias_acc_; }
    
    /**
     * @brief 获取陀螺仪偏置
     */
    Eigen::Vector3d getGyrBias() const { return bias_gyr_; }
    
    /**
     * @brief 获取加速度计噪声
     */
    Eigen::Vector3d getAccNoise() const { return noise_acc_; }
    
    /**
     * @brief 获取陀螺仪噪声
     */
    Eigen::Vector3d getGyrNoise() const { return noise_gyr_; }
    
    /**
     * @brief 设置初始偏置
     */
    void setInitialBias(const Eigen::Vector3d& acc_bias, const Eigen::Vector3d& gyr_bias) {
        bias_acc_ = acc_bias;
        bias_gyr_ = gyr_bias;
        RCLCPP_INFO(this->get_logger(), 
                    "[IMUOnlineCalibrator] Initial bias set: acc=[%.4f,%.4f,%.4f], gyr=[%.4f,%.4f,%.4f]",
                    acc_bias[0], acc_bias[1], acc_bias[2],
                    gyr_bias[0], gyr_bias[1], gyr_bias[2]);
    }
    
    /**
     * @brief 重置标定状态
     */
    void reset() {
        bias_acc_ = Eigen::Vector3d::Zero();
        bias_gyr_ = Eigen::Vector3d::Zero();
        noise_acc_ = Eigen::Vector3d::Zero();
        noise_gyr_ = Eigen::Vector3d::Zero();
        acc_buffer_.clear();
        gyr_buffer_.clear();
        static_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "[IMUOnlineCalibrator] Calibration reset");
    }
    
    /**
     * @brief 获取噪声协方差矩阵（用于fast_livo）
     */
    double getAccCovariance() const {
        double max_noise = noise_acc_.maxCoeff();
        return max_noise * max_noise * noise_adjust_factor_;
    }
    
    /**
     * @brief 获取陀螺仪噪声协方差
     */
    double getGyrCovariance() const {
        double max_noise = noise_gyr_.maxCoeff();
        return max_noise * max_noise * noise_adjust_factor_;
    }
    
    /**
     * @brief 获取偏置随机游走协方差
     */
    double getBiasAccCovariance() const {
        return getAccCovariance() * 0.01;  // 偏置随机游走约为噪声的1%
    }
    
    /**
     * @brief 获取陀螺偏置随机游走协方差
     */
    double getBiasGyrCovariance() const {
        return getGyrCovariance() * 0.01;  // 偏置随机游走约为噪声的1%
    }

private:
    /**
     * @brief IMU回调函数
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 提取加速度和角速度
        Eigen::Vector3d acc(msg->linear_acceleration.x,
                           msg->linear_acceleration.y,
                           msg->linear_acceleration.z);
        Eigen::Vector3d gyr(msg->angular_velocity.x,
                           msg->angular_velocity.y,
                           msg->angular_velocity.z);
        
        // 判断是否静止
        bool is_static = checkStatic(acc, gyr);
        
        // 发布静止状态
        std_msgs::msg::Bool static_msg;
        static_msg.data = is_static;
        static_status_pub_->publish(static_msg);
        
        // 静止时更新偏置估计
        if (is_static) {
            updateBiasEstimate(acc, gyr);
            static_count_++;
        } else {
            static_count_ = 0;
        }
        
        // 更新噪声估计
        updateNoiseEstimate(acc, gyr);
        
        // 发布标定后的IMU数据
        publishCalibratedIMU(msg, is_static);
        
        // 定期发布偏置和噪声参数
        publishCalibrationParams();
    }
    
    /**
     * @brief 检查是否静止
     */
    bool checkStatic(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) const {
        Eigen::Vector3d acc_diff = acc - last_acc_;
        Eigen::Vector3d gyr_diff = gyr - last_gyr_;
        
        double acc_norm = acc_diff.norm();
        double gyr_norm = gyr_diff.norm();
        
        return (acc_norm < static_threshold_acc_ && gyr_norm < static_threshold_gyr_);
    }
    
    /**
     * @brief 更新偏置估计
     */
    void updateBiasEstimate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
        // 添加到缓冲区
        acc_buffer_.push_back(acc);
        gyr_buffer_.push_back(gyr);
        
        // 限制缓冲区大小
        if (acc_buffer_.size() > bias_window_size_) {
            acc_buffer_.pop_front();
            gyr_buffer_.pop_front();
        }
        
        // 定期更新偏置
        auto now = this->now();
        if ((now - last_bias_update_time_).seconds() >= bias_update_interval_ &&
            acc_buffer_.size() >= min_static_samples_) {
            
            // 计算平均值作为偏置估计
            Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
            
            for (const auto& a : acc_buffer_) acc_mean += a;
            for (const auto& g : gyr_buffer_) gyr_mean += g;
            
            acc_mean /= acc_buffer_.size();
            gyr_mean /= gyr_buffer_.size();
            
            // 更新偏置（加速度计偏置需要减去重力）
            Eigen::Vector3d gravity(0.0, 0.0, 9.81);
            bias_acc_ = acc_mean - gravity;
            bias_gyr_ = gyr_mean;
            
            last_bias_update_time_ = now;
            
            RCLCPP_INFO(this->get_logger(),
                        "[IMUOnlineCalibrator] Bias updated: acc=[%.4f,%.4f,%.4f], gyr=[%.4f,%.4f,%.4f]",
                        bias_acc_[0], bias_acc_[1], bias_acc_[2],
                        bias_gyr_[0], bias_gyr_[1], bias_gyr_[2]);
        }
    }
    
    /**
     * @brief 更新噪声估计
     */
    void updateNoiseEstimate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
        if (!enable_auto_noise_adjust_) return;
        
        // 使用滑动窗口计算标准差
        acc_noise_buffer_.push_back(acc);
        gyr_noise_buffer_.push_back(gyr);
        
        if (acc_noise_buffer_.size() > noise_window_size_) {
            acc_noise_buffer_.pop_front();
            gyr_noise_buffer_.pop_front();
        }
        
        auto now = this->now();
        if ((now - last_noise_update_time_).seconds() >= noise_update_interval_ &&
            acc_noise_buffer_.size() >= min_static_samples_) {
            
            // 计算方差
            Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d acc_sq_sum = Eigen::Vector3d::Zero();
            for (const auto& a : acc_noise_buffer_) {
                acc_sum += a;
                acc_sq_sum += a.cwiseProduct(a);
            }
            
            size_t n = acc_noise_buffer_.size();
            Eigen::Vector3d acc_mean = acc_sum / n;
            Eigen::Vector3d acc_var = (acc_sq_sum / n - acc_mean.cwiseProduct(acc_mean)).cwiseMax(0.0);
            noise_acc_ = acc_var.cwiseSqrt();
            
            Eigen::Vector3d gyr_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyr_sq_sum = Eigen::Vector3d::Zero();
            for (const auto& g : gyr_noise_buffer_) {
                gyr_sum += g;
                gyr_sq_sum += g.cwiseProduct(g);
            }
            
            Eigen::Vector3d gyr_mean = gyr_sum / n;
            Eigen::Vector3d gyr_var = (gyr_sq_sum / n - gyr_mean.cwiseProduct(gyr_mean)).cwiseMax(0.0);
            noise_gyr_ = gyr_var.cwiseSqrt();
            
            last_noise_update_time_ = now;
            
            RCLCPP_DEBUG(this->get_logger(),
                        "[IMUOnlineCalibrator] Noise updated: acc=[%.4f,%.4f,%.4f], gyr=[%.4f,%.4f,%.4f]",
                        noise_acc_[0], noise_acc_[1], noise_acc_[2],
                        noise_gyr_[0], noise_gyr_[1], noise_gyr_[2]);
        }
    }
    
    /**
     * @brief 发布标定后的IMU数据
     */
    void publishCalibratedIMU(const sensor_msgs::msg::Imu::SharedPtr msg, bool is_static) {
        sensor_msgs::msg::Imu calibrated_msg;
        calibrated_msg.header = msg->header;
        
        // 减去偏置
        calibrated_msg.linear_acceleration.x = msg->linear_acceleration.x - bias_acc_[0];
        calibrated_msg.linear_acceleration.y = msg->linear_acceleration.y - bias_acc_[1];
        calibrated_msg.linear_acceleration.z = msg->linear_acceleration.z - bias_acc_[2];
        
        calibrated_msg.angular_velocity.x = msg->angular_velocity.x - bias_gyr_[0];
        calibrated_msg.angular_velocity.y = msg->angular_velocity.y - bias_gyr_[1];
        calibrated_msg.angular_velocity.z = msg->angular_velocity.z - bias_gyr_[2];
        
        // 复制协方差（实际应该重新估计）
        calibrated_msg.linear_acceleration_covariance = msg->linear_acceleration_covariance;
        calibrated_msg.angular_velocity_covariance = msg->angular_velocity_covariance;
        
        calibrated_pub_->publish(calibrated_msg);
    }
    
    /**
     * @brief 发布标定参数
     */
    void publishCalibrationParams() {
        auto now = this->now();
        if ((now - last_bias_update_time_).seconds() < 0.1) {  // 只在更新后发布
            geometry_msgs::msg::Vector3 bias_msg;
            bias_msg.x = bias_acc_[0];
            bias_msg.y = bias_acc_[1];
            bias_msg.z = bias_acc_[2];
            bias_pub_->publish(bias_msg);
            
            geometry_msgs::msg::Vector3 bias_gyr_msg;
            bias_gyr_msg.x = bias_gyr_[0];
            bias_gyr_msg.y = bias_gyr_[1];
            bias_gyr_msg.z = bias_gyr_[2];
            bias_gyr_pub_->publish(bias_gyr_msg);
            
            geometry_msgs::msg::Vector3 noise_msg;
            noise_msg.x = getAccCovariance();
            noise_msg.y = getGyrCovariance();
            noise_msg.z = getBiasGyrCovariance();
            noise_pub_->publish(noise_msg);
        }
    }
    
    // ROS2节点组件
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr calibrated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bias_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bias_gyr_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr noise_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr static_status_pub_;
    
    // 配置参数
    std::string imu_topic_;
    int queue_size_;
    int bias_window_size_;
    int noise_window_size_;
    double static_threshold_acc_;
    double static_threshold_gyr_;
    int min_static_samples_;
    double bias_update_interval_;
    double noise_update_interval_;
    bool enable_auto_noise_adjust_;
    double noise_adjust_factor_;
    
    // 标定状态
    Eigen::Vector3d bias_acc_;
    Eigen::Vector3d bias_gyr_;
    Eigen::Vector3d noise_acc_;
    Eigen::Vector3d noise_gyr_;
    
    // 数据缓冲区
    std::deque<Eigen::Vector3d> acc_buffer_;
    std::deque<Eigen::Vector3d> gyr_buffer_;
    std::deque<Eigen::Vector3d> acc_noise_buffer_;
    std::deque<Eigen::Vector3d> gyr_noise_buffer_;
    
    // 时间戳
    Eigen::Vector3d last_acc_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_gyr_{Eigen::Vector3d::Zero()};
    rclcpp::Time last_bias_update_time_;
    rclcpp::Time last_noise_update_time_;
    
    // 静止计数
    int static_count_;
};

}  // namespace automap_pro
