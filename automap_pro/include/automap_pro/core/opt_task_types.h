#pragma once

/**
 * 任务数据类型定义
 *
 * 定义系统中的任务类型，避免头文件循环依赖。
 */

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/isam2_factor_types.h"

namespace automap_pro {

/**
 * 后端优化任务类型
 */
struct OptTaskItem {
    enum class Type { 
        LOOP_FACTOR, GPS_FACTOR, SUBMAP_NODE, ODOM_FACTOR, REBUILD, 
        GPS_ALIGN_COMPLETE, RESET,
        // 新增：关键帧和子图管理任务（解耦backend_worker与其他模块）
        KEYFRAME_CREATE,
        // 新增：强制更新任务（用于 handleTriggerOptimize 服务，统一由 opt_worker 处理）
        FORCE_UPDATE
    } type;
    int from_id = 0;
    int to_id = 0;
    Pose3d rel_pose;
    Mat66d info_matrix;
    Eigen::Vector3d gps_pos;
    Eigen::Matrix3d gps_cov;
    LoopConstraint::Ptr loop_constraint;
    // REBUILD / GPS_ALIGN_COMPLETE 任务专用字段
    Eigen::Matrix3d R_enu_to_map;
    Eigen::Vector3d t_enu_to_map;
    std::vector<SubmapData> submap_data;
    std::vector<OdomFactorItem> odom_factors;
    std::vector<LoopFactorItem> loop_factors;
    
    // KEYFRAME_CREATE 任务专用字段
    KeyFrame::Ptr keyframe;
    bool has_prev_kf = false;
    int prev_kf_id = 0;
    bool gps_aligned = false;
    Eigen::Matrix3d gps_transform_R;
    Eigen::Vector3d gps_transform_t;
};

}  // namespace automap_pro
