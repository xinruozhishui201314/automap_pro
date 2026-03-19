#pragma once

/**
 * iSAM2 因子类型定义模块
 *
 * 定义 iSAM2 优化器使用的所有数据类型，包括：
 * - 里程计因子
 * - 回环因子
 * - GPS 因子
 * - 子图数据
 * - 优化结果等
 */

#include "automap_pro/core/data_types.h"

#include <gtsam/geometry/Pose3.h>

#include <functional>
#include <vector>

namespace automap_pro {

/**
 * 优化任务类型枚举
 */
enum class OptimTaskType {
    LOOP_FACTOR,    // 回环因子任务
    GPS_FACTOR,     // GPS 因子任务
    BATCH_UPDATE,   // 批量更新任务
    REBUILD         // 重建任务（GPS对齐后重建）
};

/**
 * GPS 因子项结构
 */
struct GPSFactorItem {
    int sm_id;
    Eigen::Vector3d pos;
    Eigen::Matrix3d cov;
};

/**
 * 子图间里程计因子结构体（用于保存和恢复）
 */
struct OdomFactorItem {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 回环因子结构体（用于保存和恢复）
 */
struct LoopFactorItem {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 子图数据（用于 GPS 对齐后重建）
 */
struct SubmapData {
    int id;
    Pose3d pose;
    bool is_fixed;
    bool has_gps;
    Eigen::Vector3d gps_center;
    Eigen::Matrix3d gps_cov;
};

/**
 * 关键帧数据结构（用于重建）
 */
struct KeyFrameData {
    int id;
    Pose3d pose;
    bool fixed;
    bool is_first_kf_of_submap;
};

/**
 * 关键帧间里程计因子结构体
 */
struct OdomFactorItemKF {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 关键帧间回环因子结构体
 */
struct LoopFactorItemKF {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 优化任务结构体
 */
struct OptimTask {
    OptimTaskType type;
    int from_id = 0;                     // 起始节点 ID
    int to_id = 0;                        // 目标节点 ID
    Pose3d rel_pose;                     // 相对位姿
    Mat66d info_matrix;                   // 信息矩阵
    Eigen::Vector3d gps_pos;              // GPS 位置
    Eigen::Matrix3d gps_cov;             // GPS 协方差
    std::function<void()> action;        // 批量更新回调
    // REBUILD 任务专用字段
    std::vector<SubmapData> submap_data;
    std::vector<OdomFactorItem> odom_factors;
    std::vector<LoopFactorItem> loop_factors;
    std::vector<KeyFrameData> keyframe_data;
    std::vector<OdomFactorItemKF> kf_odom_factors;
    std::vector<LoopFactorItemKF> kf_loop_factors;
};

/**
 * 优化器健康状态
 */
struct ISAM2HealthStatus {
    bool is_healthy = true;
    int consecutive_failures = 0;
    int total_optimizations = 0;
    int failed_optimizations = 0;
    double last_success_time_ms = 0.0;
    std::string last_error_message;
};

}  // namespace automap_pro
