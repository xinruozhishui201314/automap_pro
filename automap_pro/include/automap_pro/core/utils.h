#pragma once

#include <string>
#include <chrono>
#include "automap_pro/core/data_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace automap_pro {

namespace utils {

/** Voxel downsample point cloud. */
CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size);

/** @return true if path exists (file or directory). */
bool fileExists(const std::string& path);

/** Create directory and parents; no-op if already exists. */
void createDirectories(const std::string& path);

/** Simple wall-clock timer, elapsed time in ms. */
class Timer {
public:
    Timer() : t0_(std::chrono::steady_clock::now()) {}
    double elapsedMs() const {
        auto now = std::chrono::steady_clock::now();
        return 1e-3 * static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now - t0_).count());
    }
private:
    std::chrono::steady_clock::time_point t0_;
};

/** Pose3d (Isometry3d) to 6D vector [tx,ty,tz, rx,ry,rz] (rotation as Euler or log-map; here simplified as translation + euler). */
Vec6d poseToVec6d(const Pose3d& T);

/** 6D vector to Pose3d. */
Pose3d vec6dToPose(const Vec6d& v);

}  // namespace utils

}  // namespace automap_pro
