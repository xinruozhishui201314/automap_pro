#pragma once

#include <string>
#include <chrono>
#include "automap_pro/core/data_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace automap_pro {

namespace utils {

/** Remove NaN/Inf and optionally clip points to ±max_abs_coord. Returns new cloud; may be empty. */
CloudXYZIPtr sanitizePointCloudForVoxel(const CloudXYZIPtr& cloud, float max_abs_coord = 1e6f);

/** 点云体素滤波参数下限，避免过小导致 PCL 索引溢出/崩溃。所有 leaf_size 会 clamp 至此值。 */
constexpr float kMinVoxelLeafSize = 0.2f;

/** True if (max-min)/leaf on any axis would exceed safe limit (PCL VoxelGrid int overflow). */
bool voxelGridWouldOverflow(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, float leaf_size);

/** Voxel downsample point cloud. leaf_size 会 clamp 至至少 kMinVoxelLeafSize；输入内部会 sanitize；溢出风险时放大 leaf 或返回副本。 */
CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size);

/** 分块体素滤波：按空间网格分块，每块单独滤波再合并，避免大范围点云单次滤波导致 PCL 溢出/崩溃。
 *  chunk_size_m 为每块边长（米），建议 40～80；若 <=0 则退化为单次 voxelDownsample。 */
CloudXYZIPtr voxelDownsampleChunked(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m = 50.0f);

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
