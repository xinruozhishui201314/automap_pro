/**
 * @file sensor/lidar_processor.cpp
 * @brief 传感器驱动与同步实现。
 */
#include "automap_pro/sensor/lidar_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/core/logger.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <chrono>

#define MOD "LidarProcessor"

namespace automap_pro {

LidarProcessor::LidarProcessor() {
    try {
        const auto& cfg = ConfigManager::instance();
        min_range_         = cfg.lidarMinRange();
        max_range_         = cfg.lidarMaxRange();
        undistort_enabled_ = cfg.lidarUndistort();
        time_offset_       = 0.0;
        
        ALOG_INFO(MOD, "LidarProcessor initialized: min_range={:.2f}m max_range={:.2f}m undistort={}",
                  min_range_, max_range_, undistort_enabled_);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "LidarProcessor init from config failed: {}, using defaults", e.what());
        min_range_ = 0.3;
        max_range_ = 100.0;
        undistort_enabled_ = false;
        time_offset_ = 0.0;
    }
}

void LidarProcessor::init(rclcpp::Node::SharedPtr node, const TimedBuffer<ImuData>& imu_buffer) {
    try {
        if (!node) {
            throw std::invalid_argument("LidarProcessor::init: node is null");
        }
        
        imu_buffer_ = &imu_buffer;
        logger_ = node->get_logger();
        topic_name_ = ConfigManager::instance().lidarTopic();
        sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_, 10, std::bind(&LidarProcessor::lidarCallback, this, std::placeholders::_1));
        RCLCPP_INFO(logger_, "[LidarProcessor][TOPIC] subscribe: %s", topic_name_.c_str());
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "LidarProcessor::init failed: {}", e.what());
        throw;
    }
}

void LidarProcessor::registerCallback(FrameCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void LidarProcessor::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
        if (!msg) {
            ALOG_WARN(MOD, "LidarProcessor: received null message");
            return;
        }
        
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
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "LidarProcessor::lidarCallback exception: {}", e.what());
    } catch (...) {
        ALOG_ERROR(MOD, "LidarProcessor::lidarCallback unknown exception");
    }
}

void LidarProcessor::process(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
        auto frame = std::make_shared<LidarFrame>();
        frame->cloud_raw = std::make_shared<CloudXYZI>();
        
        try {
            pcl::fromROSMsg(*msg, *frame->cloud_raw);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "pcl::fromROSMsg failed: {}", e.what());
            return;
        }

        if (frame->cloud_raw->empty()) {
            ALOG_WARN(MOD, "LidarProcessor: received empty point cloud");
            return;
        }

        double stamp = rclcpp::Time(msg->header.stamp).seconds() + time_offset_;
        frame->timestamp_start = stamp;
        frame->timestamp_end   = stamp + 0.1;

        try {
            frame->cloud_raw = rangeFilter(frame->cloud_raw, min_range_, max_range_);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "rangeFilter failed: {}", e.what());
        }

        if (frame->cloud_raw->empty()) {
            ALOG_DEBUG(MOD, "LidarProcessor: cloud empty after range filter");
            return;
        }

        if (undistort_enabled_ && imu_buffer_ && !imu_buffer_->empty()) {
            try {
                auto imu_data = imu_buffer_->range(
                    frame->timestamp_start - 0.05, frame->timestamp_end + 0.05);
                frame->cloud_undistorted = undistort(
                    frame->cloud_raw, frame->timestamp_start, frame->timestamp_end, imu_data);
            } catch (const std::exception& e) {
                ALOG_WARN(MOD, "Undistortion failed: {}, using raw cloud", e.what());
                frame->cloud_undistorted = frame->cloud_raw;
            }
        } else {
            frame->cloud_undistorted = frame->cloud_raw;
        }

        for (auto& cb : callbacks_) {
            try {
                cb(frame);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "Frame callback exception: {}", e.what());
            }
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "LidarProcessor::process exception: {}", e.what());
    } catch (...) {
        ALOG_ERROR(MOD, "LidarProcessor::process unknown exception");
    }
}

CloudXYZIPtr LidarProcessor::rangeFilter(const CloudXYZIPtr& cloud,
                                           double min_r, double max_r) {
    if (!cloud || cloud->empty()) {
        ALOG_WARN(MOD, "rangeFilter: input cloud is null or empty");
        return cloud;
    }
    
    CloudXYZIPtr out(new CloudXYZI);
    out->reserve(cloud->size());
    
    int filtered_count = 0;
    for (const auto& pt : cloud->points) {
        // 检查点是否有效（非 NaN/Inf）
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            filtered_count++;
            continue;
        }
        
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (r >= static_cast<float>(min_r) && r <= static_cast<float>(max_r)) {
            out->push_back(pt);
        }
    }
    
    if (filtered_count > 0) {
        ALOG_DEBUG(MOD, "rangeFilter: filtered {} invalid points", filtered_count);
    }
    
    return out;
}

CloudXYZIPtr LidarProcessor::undistort(const CloudXYZIPtr& raw,
                                         double t_start, double t_end,
                                         const std::vector<ImuData>& imu_data) {
    if (imu_data.empty() || raw->empty()) return raw;

    try {
        CloudXYZIPtr out(new CloudXYZI);
        out->resize(raw->size());

        // 预处理：为每个IMU数据计算累积旋转到end
        std::vector<Eigen::Matrix3d> R_to_end(imu_data.size(), Eigen::Matrix3d::Identity());
        for (int j = imu_data.size() - 2; j >= 0; --j) {
            const auto& imu = imu_data[j];
            const auto& imu_next = imu_data[j + 1];
            double dt = imu_next.timestamp - imu.timestamp;
            if (dt > 0) {
                Eigen::Vector3d omega = imu.angular_velocity;
                double angle = omega.norm() * dt;
                if (angle > 1e-10 && std::isfinite(angle)) {
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
            
            // 检查点有效性
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                out->points[i] = pt;
                continue;
            }
            
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
                double denom = imu_data[imu_idx].timestamp - imu_data[imu_idx-1].timestamp;
                if (std::abs(denom) < 1e-9) {
                    R_pt_to_end = R_to_end[imu_idx];
                } else {
                    double ratio = (t_pt - imu_data[imu_idx-1].timestamp) / denom;
                    ratio = std::max(0.0, std::min(1.0, ratio));
                    
                    Eigen::Quaterniond q1(R_to_end[imu_idx]);
                    Eigen::Quaterniond q2(R_to_end[imu_idx-1]);
                    Eigen::Quaterniond q_interp = q1.slerp(ratio, q2);
                    R_pt_to_end = q_interp.toRotationMatrix();
                }
            }
            
            // 补偿
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_comp = R_pt_to_end * p;

            auto& pt_out = out->points[i];
            pt_out.x = static_cast<float>(p_comp.x());
            pt_out.y = static_cast<float>(p_comp.y());
            pt_out.z = static_cast<float>(p_comp.z());
            pt_out.intensity = pt.intensity;
        }
        return out;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "undistort exception: {}", e.what());
        return raw;
    }
}

}  // namespace automap_pro
