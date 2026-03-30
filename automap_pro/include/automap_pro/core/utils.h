#pragma once
/**
 * @file utils.h
 * @brief 点云几何预处理、体素安全下采样、文件系统与位姿/GNSS 工具函数。
 *
 * @details
 * - 体素单元内通常取**质心**：@f$\mathbf{c}=\frac{1}{|V|}\sum_{i\in V}\mathbf{p}_i@f$（实现依 PCL 或自定义哈希格）。
 * - 溢出安全：单轴体素格数需满足 @f$d_x d_y d_z \le 2^{31}-1@f$ 量级约束，见 .cpp 中分块策略。
 * - WGS84→ENU：以原点为参考的局部切平面（GeographicLib::LocalCartesian）。
 */
#include <string>
#include <chrono>
#include "automap_pro/core/data_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace automap_pro {

/** @brief 无状态工具函数命名空间（点云/IO/坐标/计时）。 */
namespace utils {

/** @brief 去除 NaN/Inf，可选将坐标裁剪到 @f$\pm\texttt{max\_abs\_coord}@f$；返回新云。 */
CloudXYZIPtr sanitizePointCloudForVoxel(const CloudXYZIPtr& cloud, float max_abs_coord = 1e6f);

/** 点云体素滤波参数下限，避免过小导致 PCL 索引溢出/崩溃。所有 leaf_size 会 clamp 至此值。 */
constexpr float kMinVoxelLeafSize = 0.2f;

/** True if (max-min)/leaf on any axis would exceed safe limit (PCL VoxelGrid int overflow). */
bool voxelGridWouldOverflow(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, float leaf_size);

/** Voxel downsample point cloud. leaf_size 会 clamp 至至少 kMinVoxelLeafSize；输入内部会 sanitize；溢出风险时放大 leaf 或返回副本。
 *  @param parallel 是否使用 OpenMP 并行（默认为 true，建议在大点云时开启） */
CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size, bool parallel = true);

/** 带超时保护的体素下采样。
 *  @param cloud 输入点云
 *  @param leaf_size 体素大小
 *  @param timeout_ms 超时时间（毫秒），默认 5000ms (5秒)
 *  @param timed_out 如果非空，超时时会被设置为 true
 *  @param parallel 是否使用并行（默认为 true）
 *  @return 降采样后的点云；超时或异常时返回原始点云的副本
 */
CloudXYZIPtr voxelDownsampleWithTimeout(const CloudXYZIPtr& cloud, float leaf_size, 
                                         int timeout_ms = 5000, bool* timed_out = nullptr,
                                         bool parallel = true);

/** 分块体素滤波：按空间网格分块，每块单独滤波再合并，避免大范围点云单次滤波导致 PCL 溢出/崩溃。
 *  chunk_size_m 为每块边长（米），建议 40～80；若 <=0 则按 leaf 自动计算安全块大小。
 *  @param parallel 是否对分块执行 OpenMP 并行（默认为 true） */
CloudXYZIPtr voxelDownsampleChunked(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m = 50.0f, bool parallel = true);

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

/**
 * Body-frame voxel: centroid xyz; if sem_labels size matches body, intensity = majority semantic class id from
 * sem_labels->points[i].intensity; otherwise intensity = mean lidar intensity from body.
 * leaf_size is clamped to at least kMinVoxelLeafSize.
 */
CloudXYZIPtr voxelDownsampleBodyWithSemanticLabels(const CloudXYZIPtr& body,
                                                   const CloudXYZIPtr& sem_labels,
                                                   float leaf_size);

/**
 * Voxel downsample: centroid per cell, intensity = majority vote of rounded intensity (semantic class id).
 * Input is sanitized like voxelDownsample; on grid overflow falls back to voxelDownsample (first point per voxel).
 */
CloudXYZIPtr voxelDownsampleMajorityIntensity(const CloudXYZIPtr& cloud, float leaf_size, bool parallel = true);

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

// --- GPS 转换 ---
/** 将 WGS84 (lat, lon, alt) 转换为 ENU 坐标系 */
Eigen::Vector3d wgs84ToEnu(double lat, double lon, double alt, 
                           double origin_lat, double origin_lon, double origin_alt);

}  // namespace utils

}  // namespace automap_pro
