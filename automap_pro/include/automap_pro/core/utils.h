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

/** 带超时保护的体素下采样。
 *  @param cloud 输入点云
 *  @param leaf_size 体素大小
 *  @param timeout_ms 超时时间（毫秒），默认 5000ms (5秒)
 *  @param timed_out 如果非空，超时时会被设置为 true
 *  @return 降采样后的点云；超时或异常时返回原始点云的副本
 */
CloudXYZIPtr voxelDownsampleWithTimeout(const CloudXYZIPtr& cloud, float leaf_size, 
                                         int timeout_ms = 5000, bool* timed_out = nullptr);

/** 分块体素滤波：按空间网格分块，每块单独滤波再合并，避免大范围点云单次滤波导致 PCL 溢出/崩溃。
 *  chunk_size_m 为每块边长（米），建议 40～80；若 <=0 则按 leaf 自动计算安全块大小。 */
CloudXYZIPtr voxelDownsampleChunked(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m = 50.0f);

/** 带超时的分块体素：大点云时在异步任务中执行 voxelDownsampleChunked（内部 OpenMP 并行），超时返回 sanitized 副本。
 *  用于 merge 等路径，兼顾并行加速与超时保护。chunk_size_m<=0 表示自动。 */
CloudXYZIPtr voxelDownsampleChunkedWithTimeout(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m,
                                                int timeout_ms = 15000, bool* timed_out = nullptr);

/** 安全体素下采样（推荐）：保证不出现 overflow，且不放大 leaf、保持分辨率。
 *  内部在可能溢出时走分块流程：分割 → 每块降采样 → 拼接；chunk_size_m<=0 时自动计算块大小。
 *  @param cloud 输入点云
 *  @param leaf_size 体素边长（米）
 *  @param chunk_size_m 分块边长（米），<=0 表示自动
 *  @return 降采样后的点云 */
CloudXYZIPtr voxelDownsampleSafe(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m = 0.0f);

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
