#pragma once

/**
 * @file pose_chain.hpp
 * @brief 共享位姿链变换内核（建图 ODOM→MAP 升级、可视化锚点链、漂移混合）
 *
 * 所有消费方应调用本头文件中的函数，避免在 Mapping / Visualization / Semantic 中重复手写同一公式导致长期漂移。
 * 坐标系语义：T_map_odom 表示 odom→map（与 Registry 中 R_enu_to_map / t_enu_to_map 组成的 Isometry 一致）。
 */

#include "automap_pro/core/data_types.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>

namespace automap_pro::v3::pose_chain {

/** GPS 对齐后：body 在 map 下位姿 = T_map_odom * T_odom_b（与 MappingModule::processFrame 一致）。 */
inline Pose3d mapBodyFromOdomBody(const Pose3d& T_map_odom, const Pose3d& T_odom_b) {
    return T_map_odom * T_odom_b;
}

/**
 * 关键帧存储的 T_map_b_optimized 若在 ODOM 语义下，需先乘 T_map_odom 再参与 map 系几何。
 */
inline Pose3d keyframeOptimizedInMapFrame(const Pose3d& T_map_odom_ev, bool gps_elev_odom_kf,
                                            PoseFrame kf_pose_frame, const Pose3d& T_map_b_stored) {
    if (kf_pose_frame == PoseFrame::ODOM && gps_elev_odom_kf) {
        return T_map_odom_ev * T_map_b_stored;
    }
    return T_map_b_stored;
}

/** 可视化：world(=odom) 点云到 map 的链式外参 T_world_to_map = T_k_map * inv(T_k_odom)。 */
inline Pose3d worldToMapFromAnchorChain(const Pose3d& T_k_in_map_frame, const Pose3d& T_k_odom) {
    return T_k_in_map_frame * T_k_odom.inverse();
}

/** V1：按事件 session 取 T_map_odom；无会话记录时回退全局 GPS 对齐。 */
inline Pose3d resolveTMapOdomFromSnapshot(const PoseSnapshot& snap, uint64_t ev_session_id) {
    if (ev_session_id != 0) {
        const auto it = snap.session_alignments.find(ev_session_id);
        if (it != snap.session_alignments.end() && it->second.aligned) {
            return it->second.T_map_odom;
        }
    }
    Pose3d T = Pose3d::Identity();
    if (snap.gps_aligned) {
        T.linear() = snap.R_enu_to_map;
        T.translation() = snap.t_enu_to_map;
    }
    return T;
}

inline bool sessionGpsActiveForViz(const PoseSnapshot& snap, uint64_t ev_session_id) {
    if (ev_session_id != 0) {
        const auto it = snap.session_alignments.find(ev_session_id);
        if (it != snap.session_alignments.end()) {
            return it->second.aligned;
        }
    }
    return snap.gps_aligned;
}

/** V2：平移漂移越大，链式权重越低。 */
inline double chainWeightFromDriftTranslation(double delta_norm_m) {
    constexpr double kLo = 0.25;
    constexpr double kHi = 2.0;
    if (delta_norm_m <= kLo) {
        return 1.0;
    }
    if (delta_norm_m >= kHi) {
        return 0.0;
    }
    return 1.0 - (delta_norm_m - kLo) / (kHi - kLo);
}

inline Pose3d blendWorldToMapSE3(const Pose3d& T_chain, const Pose3d& T_direct, double w_chain) {
    w_chain = std::clamp(w_chain, 0.0, 1.0);
    const Eigen::Quaterniond qc(T_chain.linear());
    const Eigen::Quaterniond qd(T_direct.linear());
    const Eigen::Quaterniond qb = qc.slerp(1.0 - w_chain, qd);
    Pose3d out = Pose3d::Identity();
    out.linear() = qb.toRotationMatrix();
    out.translation() =
        w_chain * T_chain.translation() + (1.0 - w_chain) * T_direct.translation();
    return out;
}

} // namespace automap_pro::v3::pose_chain
