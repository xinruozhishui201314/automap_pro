#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#ifdef USE_GEOGRAPHIC_LIB
#include <GeographicLib/LocalCartesian.hpp>
#else
// Minimal WGS84 → ENU fallback
#endif

namespace automap_pro {
namespace utils {

// ──────────────────────────────────────────────────────────
// Time
// ──────────────────────────────────────────────────────────
double nowSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

Timer::Timer() { reset(); }
void Timer::reset() { t0 = std::chrono::steady_clock::now(); }
double Timer::elapsedMs() const {
    auto dt = std::chrono::steady_clock::now() - t0;
    return std::chrono::duration<double, std::milli>(dt).count();
}

// ──────────────────────────────────────────────────────────
// Math / Geometry
// ──────────────────────────────────────────────────────────
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<  0,    -v.z(),  v.y(),
          v.z(),  0,    -v.x(),
         -v.y(),  v.x(),  0;
    return m;
}

Eigen::Matrix4d isometry3dToMatrix4d(const Pose3d& T) {
    return T.matrix();
}

Pose3d matrix4dToIsometry3d(const Eigen::Matrix4d& M) {
    Pose3d T;
    T.linear()      = M.block<3,3>(0,0);
    T.translation() = M.block<3,1>(0,3);
    return T;
}

Pose3d interpolatePose(const Pose3d& T0, const Pose3d& T1, double t) {
    // t in [0,1]
    Eigen::Quaterniond q0(T0.rotation()), q1(T1.rotation());
    Pose3d T;
    T.linear()      = q0.slerp(t, q1).toRotationMatrix();
    T.translation() = T0.translation() * (1.0 - t) + T1.translation() * t;
    return T;
}

double translationNorm(const Pose3d& T) {
    return T.translation().norm();
}

double rotationAngleDeg(const Pose3d& T) {
    Eigen::AngleAxisd aa(T.rotation());
    return aa.angle() * 180.0 / M_PI;
}

Vec6d poseToVec6d(const Pose3d& T) {
    Vec6d v;
    v.head<3>() = T.translation();
    Eigen::AngleAxisd aa(T.rotation());
    v.tail<3>() = aa.axis() * aa.angle();
    return v;
}

Pose3d vec6dToPose(const Vec6d& v) {
    Pose3d T;
    T.translation() = v.head<3>();
    double angle = v.tail<3>().norm();
    if (angle < 1e-10) {
        T.linear() = Eigen::Matrix3d::Identity();
    } else {
        Eigen::AngleAxisd aa(angle, v.tail<3>() / angle);
        T.linear() = aa.toRotationMatrix();
    }
    return T;
}

Mat66d poseCovariance6d(double trans_sigma, double rot_sigma) {
    Mat66d cov = Mat66d::Zero();
    cov.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * trans_sigma * trans_sigma;
    cov.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * rot_sigma  * rot_sigma;
    return cov;
}

// ──────────────────────────────────────────────────────────
// GPS / Geographic
// ──────────────────────────────────────────────────────────
void wgs84ToENU(double lat, double lon, double alt,
                double lat0, double lon0, double alt0,
                double& e, double& n, double& u) {
#ifdef USE_GEOGRAPHIC_LIB
    GeographicLib::LocalCartesian proj(lat0, lon0, alt0);
    proj.Forward(lat, lon, alt, e, n, u);
#else
    // Simple flat-earth approximation for fallback
    const double R = 6378137.0;  // WGS84 equatorial radius
    const double deg2rad = M_PI / 180.0;
    double dlat = (lat - lat0) * deg2rad;
    double dlon = (lon - lon0) * deg2rad;
    double lat0r = lat0 * deg2rad;
    n = R * dlat;
    e = R * std::cos(lat0r) * dlon;
    u = alt - alt0;
#endif
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6378137.0;
    const double d2r = M_PI / 180.0;
    double dlat = (lat2 - lat1) * d2r;
    double dlon = (lon2 - lon1) * d2r;
    double a = std::sin(dlat/2)*std::sin(dlat/2)
             + std::cos(lat1*d2r)*std::cos(lat2*d2r)
             * std::sin(dlon/2)*std::sin(dlon/2);
    return 2.0 * R * std::asin(std::sqrt(a));
}

double mahalanobisDistance3d(const Eigen::Vector3d& delta,
                              const Eigen::Matrix3d& cov) {
    Eigen::Matrix3d cov_inv = cov.inverse();
    return std::sqrt(delta.transpose() * cov_inv * delta);
}

// ──────────────────────────────────────────────────────────
// Point Cloud
// ──────────────────────────────────────────────────────────
CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, double voxel_size) {
    if (!cloud || cloud->empty()) return std::make_shared<CloudXYZI>();
    pcl::VoxelGrid<PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(static_cast<float>(voxel_size),
                   static_cast<float>(voxel_size),
                   static_cast<float>(voxel_size));
    CloudXYZIPtr out(new CloudXYZI);
    vg.filter(*out);
    return out;
}

CloudXYZIPtr statisticalOutlierRemoval(const CloudXYZIPtr& cloud,
                                        int mean_k, double std_mul) {
    if (!cloud || cloud->empty()) return std::make_shared<CloudXYZI>();
    pcl::StatisticalOutlierRemoval<PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_mul);
    CloudXYZIPtr out(new CloudXYZI);
    sor.filter(*out);
    return out;
}

CloudXYZIPtr transformCloud(const CloudXYZIPtr& cloud, const Pose3d& T) {
    if (!cloud || cloud->empty()) return std::make_shared<CloudXYZI>();
    CloudXYZIPtr out(new CloudXYZI);
    pcl::transformPointCloud(*cloud, *out, T.matrix().cast<float>());
    return out;
}

CloudXYZIPtr mergePointClouds(const std::vector<CloudXYZIPtr>& clouds) {
    CloudXYZIPtr merged(new CloudXYZI);
    for (const auto& c : clouds) {
        if (c && !c->empty()) *merged += *c;
    }
    return merged;
}

// ──────────────────────────────────────────────────────────
// File / IO
// ──────────────────────────────────────────────────────────
bool createDirectories(const std::string& path) {
    try {
        std::filesystem::create_directories(path);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"), "[Utils] Failed to create dir %s: %s", path.c_str(), e.what());
        return false;
    }
}

bool fileExists(const std::string& path) {
    return std::filesystem::exists(path);
}

bool savePosesTUM(const std::string& path,
                  const std::vector<double>& timestamps,
                  const std::vector<Pose3d>& poses) {
    if (timestamps.size() != poses.size()) return false;
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].translation();
        Eigen::Quaterniond q(poses[i].rotation());
        ofs << timestamps[i] << " "
            << p.x() << " " << p.y() << " " << p.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    return true;
}

bool savePosesKITTI(const std::string& path,
                    const std::vector<Pose3d>& poses) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << std::fixed << std::setprecision(9);
    for (const auto& T : poses) {
        const auto& M = T.matrix();
        ofs << M(0,0) << " " << M(0,1) << " " << M(0,2) << " " << M(0,3) << " "
            << M(1,0) << " " << M(1,1) << " " << M(1,2) << " " << M(1,3) << " "
            << M(2,0) << " " << M(2,1) << " " << M(2,2) << " " << M(2,3) << "\n";
    }
    return true;
}

// ──────────────────────────────────────────────────────────
// Logging
// ──────────────────────────────────────────────────────────
static LogLevel g_log_level = LogLevel::INFO;

void setLogLevel(LogLevel level) { g_log_level = level; }

void logDebug(const std::string& msg) {
    if (g_log_level <= LogLevel::DEBUG) RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"), "%s", msg.c_str());
}
void logInfo(const std::string& msg) {
    if (g_log_level <= LogLevel::INFO)  RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "%s", msg.c_str());
}
void logWarn(const std::string& msg) {
    if (g_log_level <= LogLevel::WARN)  RCLCPP_WARN(rclcpp::get_logger("automap_pro"), "%s", msg.c_str());
}
void logError(const std::string& msg) {
    if (g_log_level <= LogLevel::ERROR) RCLCPP_ERROR(rclcpp::get_logger("automap_pro"), "%s", msg.c_str());
}

}  // namespace utils

// ── Data type string helpers ──────────────────────────────
std::string gpsQualityToString(GPSQuality q) {
    switch (q) {
        case GPSQuality::INVALID:   return "INVALID";
        case GPSQuality::LOW:       return "LOW";
        case GPSQuality::MEDIUM:    return "MEDIUM";
        case GPSQuality::HIGH:      return "HIGH";
        case GPSQuality::EXCELLENT: return "EXCELLENT";
    }
    return "UNKNOWN";
}

std::string gpsStateToString(GPSState s) {
    switch (s) {
        case GPSState::INIT:     return "INIT";
        case GPSState::TRACKING: return "TRACKING";
        case GPSState::DEGRADED: return "DEGRADED";
        case GPSState::LOST:     return "LOST";
    }
    return "UNKNOWN";
}

std::string subMapStateToString(SubMapState s) {
    switch (s) {
        case SubMapState::ACTIVE:   return "ACTIVE";
        case SubMapState::MATURE:   return "MATURE";
        case SubMapState::FROZEN:   return "FROZEN";
        case SubMapState::UPDATED:  return "UPDATED";
        case SubMapState::ARCHIVED: return "ARCHIVED";
    }
    return "UNKNOWN";
}

// ── SubMap methods ──────────────────────────────────────
bool SubMap::isFull(int max_kf, double max_spatial, double max_temporal) const {
    if (static_cast<int>(keyframes.size()) >= max_kf) return true;
    if (spatial_extent >= max_spatial) return true;
    if (t_end - t_start >= max_temporal) return true;
    return false;
}

void SubMap::addKeyFrame(const KeyFrame::Ptr& kf) {
    if (keyframes.empty()) {
        anchor_keyframe_id = static_cast<int>(kf->id);
        pose_w_anchor = kf->T_w_b;
        t_start = kf->timestamp;
    }
    keyframes.push_back(kf);
    kf->submap_id = id;
    t_end = kf->timestamp;

    // Update relative poses
    Pose3d T_anchor_kf = pose_w_anchor.inverse() * kf->T_w_b;
    relative_poses.push_back(T_anchor_kf);

    // Update spatial extent
    spatial_extent = std::max(spatial_extent,
        (kf->T_w_b.translation() - pose_w_anchor.translation()).norm());

    // Collect GPS
    if (kf->has_valid_gps) {
        gps_measurements.push_back(kf->gps);
        has_valid_gps = true;
        // Running average of GPS center
        size_t n = gps_measurements.size();
        gps_center = ((n - 1) * gps_center + kf->gps.position_enu) / n;
    }
}

void SubMap::freeze() {
    state = SubMapState::FROZEN;
}

void SubMap::updateAnchorPose(const Pose3d& new_pose) {
    pose_w_anchor_optimized = new_pose;
    state = SubMapState::UPDATED;
}

void SubMap::reproject() {
    if (keyframes.empty()) return;
    merged_cloud = std::make_shared<CloudXYZI>();
    const Pose3d& T_w_anchor = pose_w_anchor_optimized;
    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        if (!kf->cloud_ds_body || kf->cloud_ds_body->empty()) continue;
        // T_w_kf = T_w_anchor * T_anchor_kf
        Pose3d T_w_kf = T_w_anchor * relative_poses[i];
        auto cloud_world = utils::transformCloud(kf->cloud_ds_body, T_w_kf);
        *merged_cloud += *cloud_world;
    }
    downsampled_cloud = utils::voxelDownsample(merged_cloud, 0.5);
}

}  // namespace automap_pro
