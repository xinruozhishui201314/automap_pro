/**
 * @file sensor/time_sync.cpp
 * @brief 传感器驱动与同步实现。
 */
#include "automap_pro/sensor/time_sync.h"
#include "automap_pro/core/logger.h"

#define MOD "TimeSync"

namespace automap_pro {

TimeSync::TimeSync() = default;

void TimeSync::setLidarTimeOffset (double o) { 
    if (!std::isfinite(o)) {
        ALOG_ERROR(MOD, "setLidarTimeOffset: invalid offset (NaN/Inf)");
        return;
    }
    lidar_offset_ = o; 
}

void TimeSync::setImuTimeOffset   (double o) { 
    if (!std::isfinite(o)) {
        ALOG_ERROR(MOD, "setImuTimeOffset: invalid offset (NaN/Inf)");
        return;
    }
    imu_offset_ = o; 
}

void TimeSync::setGpsTimeOffset   (double o) { 
    if (!std::isfinite(o)) {
        ALOG_ERROR(MOD, "setGpsTimeOffset: invalid offset (NaN/Inf)");
        return;
    }
    gps_offset_ = o; 
}

void TimeSync::setCameraTimeOffset(double o) { 
    if (!std::isfinite(o)) {
        ALOG_ERROR(MOD, "setCameraTimeOffset: invalid offset (NaN/Inf)");
        return;
    }
    camera_offset_ = o; 
}

double TimeSync::correctLidar (double t) const { 
    return t + lidar_offset_; 
}

double TimeSync::correctImu   (double t) const { 
    return t + imu_offset_; 
}

double TimeSync::correctGps   (double t) const { 
    return t + gps_offset_; 
}

double TimeSync::correctCamera(double t) const { 
    return t + camera_offset_; 
}

ImuData TimeSync::interpolateImu(
    const ImuData& imu0, const ImuData& imu1, double t) const {
    try {
        if (!std::isfinite(t)) {
            ALOG_WARN(MOD, "interpolateImu: invalid timestamp, returning imu0");
            return imu0;
        }
        
        if (imu1.timestamp <= imu0.timestamp) {
            ALOG_DEBUG(MOD, "interpolateImu: timestamp non-increasing, returning imu0");
            return imu0;
        }
        
        double dt = imu1.timestamp - imu0.timestamp;
        if (dt <= 1e-9) {
            ALOG_DEBUG(MOD, "interpolateImu: timestamps too close, returning imu1");
            return imu1;
        }
        
        double alpha = (t - imu0.timestamp) / dt;
        alpha = std::max(0.0, std::min(1.0, alpha));
        
        ImuData out;
        out.timestamp = t;
        
        // 插值角速度（限制过大值）
        Eigen::Vector3d omega0 = imu0.angular_velocity;
        Eigen::Vector3d omega1 = imu1.angular_velocity;
        Eigen::Vector3d omega_interp = omega0 * (1 - alpha) + omega1 * alpha;
        
        // 验证插值结果
        if (!omega_interp.allFinite()) {
            ALOG_WARN(MOD, "interpolateImu: angular velocity interpolation produced NaN/Inf");
            out.angular_velocity = omega0;
        } else {
            // 限制角速度范围
            double max_omega = 10.0;  // rad/s
            if (omega_interp.norm() > max_omega) {
                out.angular_velocity = omega_interp.normalized() * max_omega;
            } else {
                out.angular_velocity = omega_interp;
            }
        }
        
        // 插值加速度
        Eigen::Vector3d acc0 = imu0.linear_acceleration;
        Eigen::Vector3d acc1 = imu1.linear_acceleration;
        Eigen::Vector3d acc_interp = acc0 * (1 - alpha) + acc1 * alpha;
        
        if (!acc_interp.allFinite()) {
            ALOG_WARN(MOD, "interpolateImu: acceleration interpolation produced NaN/Inf");
            out.linear_acceleration = acc0;
        } else {
            // 限制加速度范围
            double max_acc = 50.0;  // m/s^2
            if (acc_interp.norm() > max_acc) {
                out.linear_acceleration = acc_interp.normalized() * max_acc;
            } else {
                out.linear_acceleration = acc_interp;
            }
        }
        
        return out;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "interpolateImu exception: {}", e.what());
        return imu0;
    } catch (...) {
        ALOG_ERROR(MOD, "interpolateImu unknown exception");
        return imu0;
    }
}

}  // namespace automap_pro
