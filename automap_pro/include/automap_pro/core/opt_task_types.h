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
        /** V3: GPS 对齐后批量为历史关键帧添加 GPS 因子（等价于 V2 addBatchGPSFactors） */
        GPS_BATCH_KF,
        // 子图内回环批量
        INTRA_LOOP_BATCH,
        // 活跃子图关键帧GPS绑定
        ACTIVE_SUBMAP_GPS_BIND,
        // 圆柱地标因子
        CYLINDER_LANDMARK_FACTOR,
        // 平面地标因子
        PLANE_LANDMARK_FACTOR
    } type;
    int from_id = 0;
    int to_id = 0;
    
    // LANDMARK 专用字段
    std::vector<CylinderFactorItemKF> cylinder_factors;
    std::vector<PlaneFactorItemKF> plane_factors;
    
    /** SUBMAP_NODE：首个子图锚点等场景传入 IncrementalOptimizer::addSubMapNode(..., fixed) */
    bool fixed = false;
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
    std::vector<KeyFrameData> keyframe_data;
    std::vector<OdomFactorItemKF> kf_odom_factors;
    std::vector<LoopFactorItemKF> kf_loop_factors;
    
    // KEYFRAME_CREATE 任务专用字段
    KeyFrame::Ptr keyframe;
    bool has_prev_kf = false;
    int prev_kf_id = 0;
    KeyFrame::Ptr prev_keyframe;  // ✅ V2 修复：存储前一关键帧指针，避免异步滞后导致的 ID 匹配失败
    bool gps_aligned = false;
    Eigen::Matrix3d gps_transform_R;
    Eigen::Vector3d gps_transform_t;

    // INTRA_LOOP_BATCH：子图内回环检测结果，由 intra_loop_worker 投递
    SubMap::Ptr intra_loop_submap;
    std::vector<LoopConstraint::Ptr> intra_loop_constraints;
    /** 子图内回环的 query 关键帧索引（用于节流：成功检测后隔 N 帧再检测） */
    int intra_loop_query_keyframe_idx = -1;
};

}  // namespace automap_pro
