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
        KEYFRAME_CREATE,
        FORCE_UPDATE,
        // 子图内回环批量：由 intra_loop_worker 投递，opt_worker 执行 addSubMapNode + addLoopFactorDeferred + forceUpdate，与主线程完全异步
        INTRA_LOOP_BATCH
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

    // INTRA_LOOP_BATCH：子图内回环检测结果，由 intra_loop_worker 投递
    SubMap::Ptr intra_loop_submap;
    std::vector<LoopConstraint::Ptr> intra_loop_constraints;
};

}  // namespace automap_pro
