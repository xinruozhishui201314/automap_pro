#include "automap_pro/sensor/lidar_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <chrono>

namespace automap_pro {

LidarProcessor::LidarProcessor() {
    const auto& cfg = ConfigManager::instance();
    min_range_         = cfg.lidarMinRange();
    max_range_         = cfg.lidarMaxRange();
    undistort_enabled_ = cfg.lidarUndistort();
    time_offset_       = 0.0;
}

void LidarProcessor::init(rclcpp::Node::SharedPtr node, const TimedBuffer<ImuData>& imu_buffer) {
    imu_buffer_ = &imu_buffer;
    logger_ = node->get_logger();
    topic_name_ = ConfigManager::instance().lidarTopic();
    sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name_, 10, std::bind(&LidarProcessor::lidarCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[LidarProcessor] Subscribing to %s", topic_name_.c_str());
}

void LidarProcessor::registerCallback(FrameCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void LidarProcessor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    process(msg);
    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double msg_ts = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
    last_msg_ts_ = msg_ts;
    msg_count_++;
    if (now - last_log_time_ >= kDataFlowLogInterval) {
        RCLCPP_INFO(logger_, "[DataFlow] Lidar | topic=%s | count=%lu | last_ts=%.3f | points=%u",
                    topic_name_.c_str(), static_cast<unsigned long>(msg_count_), last_msg_ts_,
                    msg->width * msg->height);
        msg_count_ = 0;
        last_log_time_ = now;
    }
}

void LidarProcessor::process(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto frame = std::make_shared<LidarFrame>();
    frame->cloud_raw = std::make_shared<CloudXYZI>();
    pcl::fromROSMsg(*msg, *frame->cloud_raw);

    double stamp = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
    frame->timestamp_start = stamp;
    frame->timestamp_end   = stamp + 0.1;

    frame->cloud_raw = rangeFilter(frame->cloud_raw, min_range_, max_range_);

    if (undistort_enabled_ && imu_buffer_ && !imu_buffer_->empty()) {
        auto imu_data = imu_buffer_->range(
            frame->timestamp_start - 0.05, frame->timestamp_end + 0.05);
        frame->cloud_undistorted = undistort(
            frame->cloud_raw, frame->timestamp_start, frame->timestamp_end, imu_data);
    } else {
        frame->cloud_undistorted = frame->cloud_raw;
    }

    for (auto& cb : callbacks_) cb(frame);
}

CloudXYZIPtr LidarProcessor::rangeFilter(const CloudXYZIPtr& cloud,
                                           double min_r, double max_r) {
    CloudXYZIPtr out(new CloudXYZI);
    out->reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (r >= static_cast<float>(min_r) && r <= static_cast<float>(max_r)) {
            out->push_back(pt);
        }
    }
    return out;
}

CloudXYZIPtr LidarProcessor::undistort(const CloudXYZIPtr& raw,
                                         double t_start, double t_end,
                                         const std::vector<ImuData>& imu_data) {
    if (imu_data.empty() || raw->empty()) return raw;

    CloudXYZIPtr out(new CloudXYZI);
    out->resize(raw->size());

    // 预处理：为每个IMU数据计算累积旋转到end
    // 修复：从后往前积分（正确的积分方向）
    std::vector<Eigen::Matrix3d> R_to_end(imu_data.size(), Eigen::Matrix3d::Identity());
    for (int j = imu_data.size() - 2; j >= 0; --j) {
        const auto& imu = imu_data[j];
        const auto& imu_next = imu_data[j + 1];
        double dt = imu_next.timestamp - imu.timestamp;
        if (dt > 0) {
            Eigen::Vector3d omega = imu.angular_velocity;
            double angle = omega.norm() * dt;
            if (angle > 1e-10) {
                Eigen::AngleAxisd aa(angle, omega.normalized());
                R_to_end[j] = aa.toRotationMatrix() * R_to_end[j + 1];
            } else {
                R_to_end[j] = R_to_end[j + 1];
            }
        }
    }
    
    Eigen::Matrix3d R_ref = Eigen::Matrix3d::Identity();
    double dt_total = t_end - t_start;
    if (dt_total <= 0.0) return raw;

    for (size_t i = 0; i < raw->size(); ++i) {
        const auto& pt = raw->points[i];
        // 修正：使用 size() - 1 避免除以零
        double alpha = static_cast<double>(i) / (raw->size() > 1 ? (raw->size() - 1) : 1);
        double t_pt  = t_start + alpha * dt_total;

        // 找到最近的IMU数据
        size_t imu_idx = 0;
        for (; imu_idx < imu_data.size(); ++imu_idx) {
            if (imu_data[imu_idx].timestamp >= t_pt) break;
        }
        if (imu_idx >= imu_data.size()) imu_idx = imu_data.size() - 1;

        // 插值旋转
        Eigen::Matrix3d R_pt_to_end;
        if (imu_idx == 0) {
            R_pt_to_end = R_to_end[0];
        } else {
            // 在两个IMU之间插值（使用四元数更精确）
            double ratio = (t_pt - imu_data[imu_idx-1].timestamp) / 
                          (imu_data[imu_idx].timestamp - imu_data[imu_idx-1].timestamp);
            ratio = std::max(0.0, std::min(1.0, ratio));  // clamp
            
            Eigen::Quaterniond q1(R_to_end[imu_idx]);
            Eigen::Quaterniond q2(R_to_end[imu_idx-1]);
            Eigen::Quaterniond q_interp = q1.slerp(ratio, q2);
            R_pt_to_end = q_interp.toRotationMatrix();
        }
        
        // 补偿：p_comp = R_to_end * p（正确的补偿方向）
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_comp = R_pt_to_end * p;

        auto& pt_out = out->points[i];
        pt_out.x = static_cast<float>(p_comp.x());
        pt_out.y = static_cast<float>(p_comp.y());
        pt_out.z = static_cast<float>(p_comp.z());
        pt_out.intensity = pt.intensity;
    }
    return out;
}

}  // namespace automap_pro
