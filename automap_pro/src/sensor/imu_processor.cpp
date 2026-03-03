#include "automap_pro/sensor/imu_processor.h"
#include "automap_pro/core/config_manager.h"

#include <chrono>

namespace automap_pro {

ImuProcessor::ImuProcessor() = default;

void ImuProcessor::init(rclcpp::Node::SharedPtr node, TimedBuffer<ImuData>& imu_buffer) {
    imu_buffer_ = &imu_buffer;
    logger_ = node->get_logger();
    const auto& cfg = ConfigManager::instance();
    time_offset_ = 0.0;
    gravity_     = cfg.imuGravity();
    topic_name_ = cfg.imuTopic();
    sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
        topic_name_, 2000, std::bind(&ImuProcessor::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[ImuProcessor] Subscribing to %s", topic_name_.c_str());
}

void ImuProcessor::registerCallback(ImuCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void ImuProcessor::process(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuData data;
    data.timestamp = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
    data.angular_velocity   = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    data.linear_acceleration= {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    if (imu_buffer_) imu_buffer_->push(data.timestamp, data);
    for (auto& cb : callbacks_) cb(data);
}

void ImuProcessor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
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
}

ImuProcessor::PreintResult ImuProcessor::preintegrate(
        const std::vector<ImuData>& imu_data,
        double t_start, double t_end,
        const Eigen::Vector3d& gyro_bias,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gravity) const {
    PreintResult result;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double t_prev = t_start;
    for (size_t i = 0; i < imu_data.size(); ++i) {
        const auto& imu = imu_data[i];
        if (imu.timestamp < t_start) continue;
        if (imu.timestamp > t_end)   break;
        double dt = imu.timestamp - t_prev;
        if (dt <= 0.0) continue;
        t_prev = imu.timestamp;
        Eigen::Vector3d omega = imu.angular_velocity   - gyro_bias;
        Eigen::Vector3d acc   = imu.linear_acceleration - accel_bias;
        double angle = omega.norm() * dt;
        if (angle > 1e-10) {
            Eigen::AngleAxisd aa(angle, omega.normalized());
            R = R * aa.toRotationMatrix();
        }
        Eigen::Vector3d acc_world = R * acc + gravity;
        p += v * dt + 0.5 * acc_world * dt * dt;
        v += acc_world * dt;
        result.dt_total += dt;
    }
    result.delta_R = R;
    result.delta_v = v;
    result.delta_p = p;
    return result;
}

}  // namespace automap_pro
