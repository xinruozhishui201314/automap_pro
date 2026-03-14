#include "automap_pro/sensor/imu_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

#include <chrono>

#define MOD "ImuProcessor"

namespace automap_pro {

ImuProcessor::ImuProcessor() = default;

void ImuProcessor::init(rclcpp::Node::SharedPtr node, TimedBuffer<ImuData>& imu_buffer) {
    try {
        if (!node) {
            throw std::invalid_argument("ImuProcessor::init: node is null");
        }
        
        imu_buffer_ = &imu_buffer;
        logger_ = node->get_logger();
        const auto& cfg = ConfigManager::instance();
        time_offset_ = 0.0;
        gravity_     = cfg.imuGravity();
        topic_name_ = cfg.imuTopic();
        sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
            topic_name_, 2000, std::bind(&ImuProcessor::imuCallback, this, std::placeholders::_1));
        RCLCPP_INFO(logger_, "[ImuProcessor][TOPIC] subscribe: %s (gravity=%.2f)", 
                    topic_name_.c_str(), gravity_);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "ImuProcessor::init failed: {}", e.what());
        throw;
    }
}

void ImuProcessor::registerCallback(ImuCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void ImuProcessor::process(const sensor_msgs::msg::Imu::SharedPtr msg) {
    try {
        if (!msg) {
            ALOG_WARN(MOD, "ImuProcessor: received null message");
            return;
        }
        
        // 验证数据有效性
        bool has_invalid = false;
        if (!std::isfinite(msg->angular_velocity.x) ||
            !std::isfinite(msg->angular_velocity.y) ||
            !std::isfinite(msg->angular_velocity.z)) {
            ALOG_WARN(MOD, "ImuProcessor: angular velocity contains NaN/Inf, skipping");
            has_invalid = true;
        }
        if (!std::isfinite(msg->linear_acceleration.x) ||
            !std::isfinite(msg->linear_acceleration.y) ||
            !std::isfinite(msg->linear_acceleration.z)) {
            ALOG_WARN(MOD, "ImuProcessor: linear acceleration contains NaN/Inf, skipping");
            has_invalid = true;
        }
        
        if (has_invalid) {
            return;
        }
        
        ImuData data;
        data.timestamp = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
        data.angular_velocity   = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        data.linear_acceleration= {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        
        if (imu_buffer_) {
            imu_buffer_->push(data.timestamp, data);
        }
        
        for (auto& cb : callbacks_) {
            try {
                cb(data);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "IMU callback exception: {}", e.what());
            }
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "ImuProcessor::process exception: {}", e.what());
    }
}

void ImuProcessor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    try {
        process(msg);
        double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        double msg_ts = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
        last_msg_ts_ = msg_ts;
        msg_count_++;
        if (now - last_log_time_ >= kDataFlowLogInterval) {
            RCLCPP_INFO(logger_, "[DataFlow] IMU | topic=%s | count=%lu | last_ts=%.3f",
                        topic_name_.c_str(), static_cast<unsigned long>(msg_count_), last_msg_ts_);
            msg_count_ = 0;
            last_log_time_ = now;
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "ImuProcessor::imuCallback exception: {}", e.what());
    }
}

ImuProcessor::PreintResult ImuProcessor::preintegrate(
        const std::vector<ImuData>& imu_data,
        double t_start, double t_end,
        const Eigen::Vector3d& gyro_bias,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gravity) const {
    PreintResult result;
    
    try {
        if (imu_data.empty()) {
            ALOG_DEBUG(MOD, "preintegrate: empty IMU data");
            return result;
        }
        
        if (t_end <= t_start) {
            ALOG_WARN(MOD, "preintegrate: invalid time range (t_start={}, t_end={})", t_start, t_end);
            return result;
        }
        
        // 验证 bias 有效性
        if (!gyro_bias.allFinite() || !accel_bias.allFinite()) {
            ALOG_WARN(MOD, "preintegrate: bias contains NaN/Inf, using zero bias");
            Eigen::Vector3d gyro_bias_valid = gyro_bias.allFinite() ? gyro_bias : Eigen::Vector3d::Zero();
            Eigen::Vector3d accel_bias_valid = accel_bias.allFinite() ? accel_bias : Eigen::Vector3d::Zero();
            return preintegrate(imu_data, t_start, t_end, gyro_bias_valid, accel_bias_valid, gravity);
        }
        
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        double t_prev = t_start;
        
        for (size_t i = 0; i < imu_data.size(); ++i) {
            const auto& imu = imu_data[i];
            
            // 时间范围检查
            if (imu.timestamp < t_start) continue;
            if (imu.timestamp > t_end)   break;
            
            double dt = imu.timestamp - t_prev;
            if (dt <= 0.0) continue;
            
            // 检查 IMU 数据有效性
            if (!imu.angular_velocity.allFinite() || !imu.linear_acceleration.allFinite()) {
                ALOG_WARN(MOD, "preintegrate: skipping invalid IMU data at index {}", i);
                t_prev = imu.timestamp;
                continue;
            }
            
            t_prev = imu.timestamp;
            
            Eigen::Vector3d omega = imu.angular_velocity - gyro_bias;
            Eigen::Vector3d acc   = imu.linear_acceleration - accel_bias;
            
            // 限制角速度和加速度幅度（防止数值问题）
            double omega_norm = omega.norm();
            constexpr double kMaxOmega = 10.0;  // rad/s
            if (omega_norm > kMaxOmega) {
                ALOG_WARN(MOD, "preintegrate: clamping angular velocity norm from {:.2f} to {:.2f}", 
                          omega_norm, kMaxOmega);
                omega = omega.normalized() * kMaxOmega;
            }
            
            double acc_norm = acc.norm();
            constexpr double kMaxAcc = 100.0;  // m/s^2
            if (acc_norm > kMaxAcc) {
                ALOG_WARN(MOD, "preintegrate: clamping acceleration norm from {:.2f} to {:.2f}", 
                          acc_norm, kMaxAcc);
                acc = acc.normalized() * kMaxAcc;
            }
            
            // 旋转积分
            double angle = omega_norm * dt;
            if (angle > 1e-10) {
                Eigen::AngleAxisd aa(angle, omega.normalized());
                R = R * aa.toRotationMatrix();
            }
            
            // 平移和速度积分
            Eigen::Vector3d acc_world = R * acc + gravity;
            p += v * dt + 0.5 * acc_world * dt * dt;
            v += acc_world * dt;
            result.dt_total += dt;
        }
        
        // 验证结果有效性
        if (!R.allFinite() || !v.allFinite() || !p.allFinite()) {
            ALOG_ERROR(MOD, "preintegrate: result contains NaN/Inf, returning identity");
            result.delta_R = Eigen::Matrix3d::Identity();
            result.delta_v = Eigen::Vector3d::Zero();
            result.delta_p = Eigen::Vector3d::Zero();
            return result;
        }

        // 修复: 对旋转矩阵进行正交化，确保满足SO(3)性质
        // 使用SVD分解进行正交化: R = U * V^T，确保行列式为正
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_orthogonalized = svd.matrixU() * svd.matrixV().transpose();
        // 确保行列式为+1（避免反射）
        if (R_orthogonalized.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R_orthogonalized = svd.matrixU() * V.transpose();
        }
        R = R_orthogonalized;

        result.delta_R = R;
        result.delta_v = v;
        result.delta_p = p;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "preintegrate exception: {}", e.what());
    }
    
    return result;
}

}  // namespace automap_pro
