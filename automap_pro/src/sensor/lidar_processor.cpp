#include "automap_pro/sensor/lidar_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

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
    std::string topic = ConfigManager::instance().lidarTopic();
    sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, 10, std::bind(&LidarProcessor::lidarCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node->get_logger(), "[LidarProcessor] Subscribing to %s", topic.c_str());
}

void LidarProcessor::registerCallback(FrameCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void LidarProcessor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    process(msg);
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

    Eigen::Matrix3d R_ref = Eigen::Matrix3d::Identity();
    double dt_total = t_end - t_start;
    if (dt_total <= 0.0) return raw;

    for (size_t i = 0; i < raw->size(); ++i) {
        const auto& pt = raw->points[i];
        double alpha = static_cast<double>(i) / raw->size();
        double t_pt  = t_start + alpha * dt_total;

        Eigen::Matrix3d R_pt = Eigen::Matrix3d::Identity();
        double t_prev = t_pt;
        for (size_t j = 0; j < imu_data.size(); ++j) {
            const auto& imu = imu_data[j];
            if (imu.timestamp < t_pt)   continue;
            if (imu.timestamp > t_end)  break;
            double dt = imu.timestamp - t_prev;
            if (dt <= 0.0) continue;
            t_prev = imu.timestamp;
            Eigen::Vector3d omega = imu.angular_velocity;
            double angle = omega.norm() * dt;
            if (angle > 1e-10) {
                Eigen::AngleAxisd aa(angle, omega.normalized());
                R_pt = R_pt * aa.toRotationMatrix();
            }
        }

        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_comp = R_ref.transpose() * R_pt * p;

        auto& pt_out = out->points[i];
        pt_out.x = static_cast<float>(p_comp.x());
        pt_out.y = static_cast<float>(p_comp.y());
        pt_out.z = static_cast<float>(p_comp.z());
        pt_out.intensity = pt.intensity;
    }
    return out;
}

}  // namespace automap_pro
