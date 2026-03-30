#pragma once
/**
 * @file v3/semantic_backend_gates.h
 * @brief V3 微内核：模块编排、事件总线、Registry、前端/语义/优化流水线。
 */


#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/data_types.h"
#include <cmath>
#include <vector>

namespace automap_pro::v3 {

/** 与 MappingModule 一致：圆柱因子观测点（体轴系质心）。 */
inline Eigen::Vector3d semanticBackendCylinderSampleBody(const CylinderLandmark::Ptr& l_kf) {
    if (!l_kf) {
        return Eigen::Vector3d::Zero();
    }
    if (l_kf->points && !l_kf->points->empty()) {
        Eigen::Vector3d s = Eigen::Vector3d::Zero();
        size_t n = 0;
        for (const auto& p : l_kf->points->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }
            s += Eigen::Vector3d(p.x, p.y, p.z);
            ++n;
        }
        if (n > 0) {
            return s / static_cast<double>(n);
        }
    }
    return l_kf->root;
}

/** 子图系：采样点到规范轴线的垂直距离与规范半径之差是否在允许范围。max_abs_err_m<=0 关闭。 */
inline bool semanticBackendCylinderRadialConsistent(const Pose3d& T_submap_kf,
                                                    const CylinderLandmark::Ptr& l_kf,
                                                    const CylinderLandmark::Ptr& l_sm,
                                                    double max_abs_err_m) {
    if (max_abs_err_m <= 0.0 || !l_kf || !l_sm) {
        return true;
    }
    if (!T_submap_kf.matrix().allFinite()) {
        return true;
    }
    const Eigen::Vector3d p_b = semanticBackendCylinderSampleBody(l_kf);
    const Eigen::Vector3d p_s = T_submap_kf * p_b;
    const Eigen::Vector3d r = l_sm->root;
    Eigen::Vector3d u = l_sm->ray;
    if (!p_s.allFinite() || !r.allFinite() || !u.allFinite() || u.norm() < 1e-9) {
        return false;
    }
    u.normalize();
    const Eigen::Vector3d v = p_s - r;
    const double d_perp = (v - v.dot(u) * u).norm();
    const double rad = l_sm->radius;
    if (!std::isfinite(d_perp) || !std::isfinite(rad)) {
        return false;
    }
    return std::abs(d_perp - rad) <= max_abs_err_m;
}

/**
 * 语义圆柱因子入图前一致性（MappingModule / HBA 复用）。
 * l_sm 为空时跳过径向一致性（仅做观测侧门控）。
 */
inline bool semanticBackendCylinderPassesGating(const ConfigManager& cfg,
                                                const Pose3d& T_submap_kf,
                                                const CylinderLandmark::Ptr& l_kf,
                                                const CylinderLandmark::Ptr& l_sm) {
    if (!cfg.semanticBackendGatingEnabled() || !l_kf) {
        return true;
    }
    const double min_pl = cfg.semanticBackendGatingCylinderMinPrimitiveLinearity();
    if (min_pl >= 0.0 && l_kf->primitive_linearity >= 0.0 && l_kf->primitive_linearity + 1e-9 < min_pl) {
        return false;
    }
    const int min_dp = cfg.semanticBackendGatingCylinderMinDetectionPoints();
    if (min_dp > 0 && l_kf->detection_point_count > 0 && l_kf->detection_point_count < min_dp) {
        return false;
    }
    const double min_kfc = cfg.semanticBackendGatingCylinderMinKfConfidence();
    if (min_kfc > 0.0 && l_kf->confidence + 1e-12 < min_kfc) {
        return false;
    }
    const double max_rad_err = cfg.semanticBackendGatingCylinderMaxAbsRadialErrorM();
    if (l_sm && !semanticBackendCylinderRadialConsistent(T_submap_kf, l_kf, l_sm, max_rad_err)) {
        return false;
    }
    return true;
}

inline bool semanticBackendPlanePassesGating(const ConfigManager& cfg, const PlaneLandmark::Ptr& p_kf) {
    if (!cfg.semanticBackendGatingEnabled() || !p_kf) {
        return true;
    }
    const int pmin = cfg.semanticBackendGatingPlaneMinDetectionPoints();
    if (pmin > 0 && p_kf->detection_point_count > 0 && p_kf->detection_point_count < pmin) {
        return false;
    }
    // Optional geometric filter for short/low planes (e.g. car bodies / guardrails):
    // when both height and tangent span stay below the configured thresholds,
    // treat this as a non-structural plane and do not create strong map factors.
    const double h_min = cfg.semanticBackendGatingPlaneMinHeightM();
    const double t_max = cfg.semanticBackendGatingPlaneMaxTangentM();
    if (h_min > 0.0 && t_max > 0.0 &&
        p_kf->vertical_span_m > 0.0 && p_kf->tangent_span_m > 0.0 &&
        p_kf->vertical_span_m < h_min && p_kf->tangent_span_m < t_max) {
        return false;
    }
    return true;
}

/**
 * Additional guard for tree trunks close to strong walls: when the trunk root
 * (in submap frame) lies closer than min_plane_distance_m to any sufficiently
 * supported plane, we treat it as a likely wall-attached artifact and can
 * choose to suppress the landmark factor.
 */
inline bool semanticBackendCylinderTooCloseToPlanes(const ConfigManager& cfg,
                                                    const Pose3d& T_submap_kf,
                                                    const CylinderLandmark::Ptr& l_kf,
                                                    const std::vector<PlaneLandmark::Ptr>& planes) {
    const double min_dist = cfg.semanticBackendGatingCylinderMinPlaneDistanceM();
    const int min_support = cfg.semanticBackendGatingCylinderMinPlaneSupport();
    if (min_dist <= 0.0 || !l_kf) {
        return false;
    }
    if (!T_submap_kf.matrix().allFinite()) {
        return false;
    }
    const Eigen::Vector3d root_s = T_submap_kf * l_kf->root;
    if (!root_s.allFinite()) {
        return false;
    }
    for (const auto& pl : planes) {
        if (!pl || !pl->isValid()) {
            continue;
        }
        if (min_support > 0 && static_cast<int>(pl->support_count) < min_support) {
            continue;
        }
        Eigen::Vector3d n = pl->normal;
        if (!n.allFinite() || n.norm() < 1e-6) {
            continue;
        }
        n.normalize();
        const double d_signed = n.dot(root_s) + pl->distance;
        const double d_abs = std::abs(d_signed);
        if (!std::isfinite(d_abs)) {
            continue;
        }
        if (d_abs < min_dist) {
            return true;
        }
    }
    return false;
}

}  // namespace automap_pro::v3
