#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <filesystem>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "automap_pro/core/data_types.h"

namespace automap_pro {
namespace utils {

// ──────────────────────────────────────────────────────────
// Time utilities
// ──────────────────────────────────────────────────────────
double nowSeconds();

struct Timer {
    std::chrono::steady_clock::time_point t0;
    Timer();
    double elapsedMs() const;
    void   reset();
};

// ──────────────────────────────────────────────────────────
// Math / Geometry
// ──────────────────────────────────────────────────────────
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

Eigen::Matrix4d isometry3dToMatrix4d(const Pose3d& T);
Pose3d          matrix4dToIsometry3d(const Eigen::Matrix4d& M);

Pose3d interpolatePose(const Pose3d& T0, const Pose3d& T1, double t);

double translationNorm(const Pose3d& T);
double rotationAngleDeg(const Pose3d& T);

// 6-DoF vector [tx, ty, tz, rx, ry, rz] (angle-axis)
Vec6d   poseToVec6d(const Pose3d& T);
Pose3d  vec6dToPose(const Vec6d& v);

Mat66d  poseCovariance6d(double trans_sigma, double rot_sigma);

// ──────────────────────────────────────────────────────────
// GPS / Geographic
// ──────────────────────────────────────────────────────────
void   wgs84ToENU(double lat, double lon, double alt,
                  double lat0, double lon0, double alt0,
                  double& e, double& n, double& u);

double haversineDistance(double lat1, double lon1,
                         double lat2, double lon2);

double mahalanobisDistance3d(const Eigen::Vector3d& delta,
                              const Eigen::Matrix3d& cov);

// ──────────────────────────────────────────────────────────
// Point Cloud
// ──────────────────────────────────────────────────────────
CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, double voxel_size);

CloudXYZIPtr statisticalOutlierRemoval(const CloudXYZIPtr& cloud,
                                        int mean_k = 50,
                                        double std_mul = 2.0);

CloudXYZIPtr transformCloud(const CloudXYZIPtr& cloud, const Pose3d& T);

// Merge list of clouds (already in world frame)
CloudXYZIPtr mergePointClouds(const std::vector<CloudXYZIPtr>& clouds);

// ──────────────────────────────────────────────────────────
// File / IO
// ──────────────────────────────────────────────────────────
bool   createDirectories(const std::string& path);
bool   fileExists(const std::string& path);
bool   savePosesTUM(const std::string& path,
                    const std::vector<double>& timestamps,
                    const std::vector<Pose3d>& poses);
bool   savePosesKITTI(const std::string& path,
                      const std::vector<Pose3d>& poses);

// ──────────────────────────────────────────────────────────
// Logging
// ──────────────────────────────────────────────────────────
enum class LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };
void setLogLevel(LogLevel level);
void logDebug(const std::string& msg);
void logInfo (const std::string& msg);
void logWarn (const std::string& msg);
void logError(const std::string& msg);

}  // namespace utils
}  // namespace automap_pro
