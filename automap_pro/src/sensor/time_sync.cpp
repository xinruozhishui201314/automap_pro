#include "automap_pro/sensor/time_sync.h"

namespace automap_pro {

TimeSync::TimeSync() = default;

void TimeSync::setLidarTimeOffset (double o) { lidar_offset_  = o; }
void TimeSync::setImuTimeOffset   (double o) { imu_offset_    = o; }
void TimeSync::setGpsTimeOffset   (double o) { gps_offset_    = o; }
void TimeSync::setCameraTimeOffset(double o) { camera_offset_ = o; }

double TimeSync::correctLidar (double t) const { return t + lidar_offset_; }
double TimeSync::correctImu   (double t) const { return t + imu_offset_; }
double TimeSync::correctGps   (double t) const { return t + gps_offset_; }
double TimeSync::correctCamera(double t) const { return t + camera_offset_; }

ImuData TimeSync::interpolateImu(
    const ImuData& imu0, const ImuData& imu1, double t) const {
    if (imu1.timestamp <= imu0.timestamp) return imu0;
    double alpha = (t - imu0.timestamp) / (imu1.timestamp - imu0.timestamp);
    alpha = std::max(0.0, std::min(1.0, alpha));
    ImuData out;
    out.timestamp          = t;
    out.angular_velocity   = imu0.angular_velocity   * (1 - alpha) + imu1.angular_velocity   * alpha;
    out.linear_acceleration= imu0.linear_acceleration* (1 - alpha) + imu1.linear_acceleration* alpha;
    return out;
}

}  // namespace automap_pro
