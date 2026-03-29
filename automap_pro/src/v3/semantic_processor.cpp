#include "automap_pro/v3/semantic_processor.h"
#include "automap_pro/v3/semantic_segmentor_factory.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/landmark_id.h"
#include <algorithm>
#include <exception>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <array>
#include <limits>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <vector>
#include <string>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cinttypes>
#include <optional>

// sloam_rec：include 根目录为包内 thrid_party/sloam_rec/sloam/include（由 CMake 注入）
#include <segmentation/trellis.h>
#include <helpers/definitions.h>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace automap_pro::v3 {

namespace {

rclcpp::Clock& semanticDiagThrottleClock() {
    static rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock;
}

std::string toLowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool geoInfoEnabled(const SemanticProcessor::Config& cfg) {
    const std::string level = toLowerCopy(cfg.geometric_log_level);
    return level != "off";
}

bool geoDebugEnabled(const SemanticProcessor::Config& cfg) {
    const std::string level = toLowerCopy(cfg.geometric_log_level);
    if (level == "off") return false;
    return level == "debug" || cfg.geometric_log_detail;
}

double fuseGeoPrimitiveConfidence(double residual,
                                  float range_view_score,
                                  float fusion_rv_boost_scale) {
    if (!(range_view_score > 1e-6f) || !(fusion_rv_boost_scale > 1e-9f)) {
        return residual;
    }
    return std::min(1.0,
        residual * (1.0 + static_cast<double>(fusion_rv_boost_scale) *
                               static_cast<double>(range_view_score)));
}

int resolveTreeLabel(const SemanticProcessor::Config& cfg) {
    if (cfg.diag_override_tree_class_id >= 0) {
        return cfg.diag_override_tree_class_id;
    }
    if (cfg.tree_class_id >= 0) {
        return cfg.tree_class_id;
    }
    // Auto fallback by backend contract:
    // - sloam writes trunk as 255 in its mask output.
    // - lsk3dnet/lsk3dnet_hybrid outputs learning-label IDs; SemanticKITTI trunk=16.
    if (cfg.model_type == "lsk3dnet" || cfg.model_type == "lsk3dnet_hybrid") {
        return 16;
    }
    return 255;
}

void appendNestedException(std::ostringstream& oss, const std::exception& e, int depth) {
    oss << "[depth=" << depth << "] " << e.what();
    try {
        std::rethrow_if_nested(e);
    } catch (const std::exception& nested) {
        oss << " | ";
        appendNestedException(oss, nested, depth + 1);
    } catch (...) {
        oss << " | [depth=" << (depth + 1) << "] <non-std nested exception>";
    }
}

std::string describeExceptionFull(const std::exception& e) {
    std::ostringstream oss;
    appendNestedException(oss, e, 0);
    return oss.str();
}

std::string describeCurrentExceptionFull() {
    try {
        throw;
    } catch (const std::exception& e) {
        return describeExceptionFull(e);
    } catch (...) {
        return "<non-std exception>";
    }
}

/// Trellis 聚类点与 flipped_cloud 一致；对应车体系点为 (x, -y, z)。
inline Eigen::Vector3d flippedTreePointToBody(const pcl::PointXYZI& p) {
    return Eigen::Vector3d(static_cast<double>(p.x), -static_cast<double>(p.y), static_cast<double>(p.z));
}

/**
 * 用车体系 Patchwork 地面点在簇水平邻域（相对 up 的正交平面内）的采样，取沿 up 的低分位数作为局部地面标量 s；
 * 点数不足返回 nullopt（由调用方用簇内沿 up 的最低高度回退）。
 */
std::optional<double> estimateLocalGroundSupportAlongUp(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ground_cloud,
    double cx_body,
    double cy_body,
    const Eigen::Vector3d& up_unit,
    float search_radius_m,
    int min_samples,
    float ground_percentile) {
    if (!ground_cloud || ground_cloud->empty() || search_radius_m <= 0.f || min_samples < 3) {
        return std::nullopt;
    }
    Eigen::Vector3d up = up_unit;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    const Eigen::Vector3d horiz_anchor(cx_body, cy_body, 0.0);
    std::vector<double> supports;
    supports.reserve(std::min(ground_cloud->size(), size_t{20000}));
    const double r = static_cast<double>(search_radius_m);
    const double r2 = r * r;
    for (const auto& g : ground_cloud->points) {
        if (!std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) {
            continue;
        }
        const Eigen::Vector3d gv(static_cast<double>(g.x), static_cast<double>(g.y), static_cast<double>(g.z));
        const Eigen::Vector3d delta = gv - horiz_anchor;
        const Eigen::Vector3d horiz = delta - up * delta.dot(up);
        if (horiz.squaredNorm() <= r2) {
            supports.push_back(gv.dot(up));
        }
    }
    if (static_cast<int>(supports.size()) < min_samples) {
        return std::nullopt;
    }
    const double pct = std::clamp(static_cast<double>(ground_percentile), 0.01, 0.45);
    const size_t k = static_cast<size_t>(std::floor(pct * static_cast<double>(supports.size() - 1)));
    std::nth_element(supports.begin(), supports.begin() + static_cast<std::ptrdiff_t>(k), supports.end());
    return supports[k];
}

std::vector<pcl::PointXYZI> trunkBandPointsBodyFrame(
    const std::vector<pcl::PointXYZI>& points_body,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ground_cloud,
    const Eigen::Vector3d& vertical_ref_body,
    float rel_h_min,
    float rel_h_max,
    float ground_search_radius_m,
    int ground_min_samples,
    float ground_percentile) {
    std::vector<pcl::PointXYZI> out;
    if (points_body.size() < 3) {
        return out;
    }
    Eigen::Vector3d up = vertical_ref_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& p : points_body) {
        cx += static_cast<double>(p.x);
        cy += static_cast<double>(p.y);
    }
    const double inv = 1.0 / static_cast<double>(points_body.size());
    cx *= inv;
    cy *= inv;
    double support_min_along_up = std::numeric_limits<double>::infinity();
    for (const auto& p : points_body) {
        const Eigen::Vector3d pb(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        support_min_along_up = std::min(support_min_along_up, pb.dot(up));
    }
    const std::optional<double> s_ground_opt = estimateLocalGroundSupportAlongUp(
        ground_cloud,
        cx,
        cy,
        up,
        ground_search_radius_m,
        ground_min_samples,
        ground_percentile);
    const double s_ground = s_ground_opt.value_or(support_min_along_up);
    const double h_lo = static_cast<double>(std::min(rel_h_min, rel_h_max));
    const double h_hi = static_cast<double>(std::max(rel_h_min, rel_h_max));
    out.reserve(points_body.size());
    for (const auto& p : points_body) {
        const Eigen::Vector3d pb(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        const double h = pb.dot(up) - s_ground;
        if (h >= h_lo && h <= h_hi) {
            out.push_back(p);
        }
    }
    return out;
}

struct SparseTrunkColumnEval {
    bool gate_ok = false;
    size_t n_column = 0;
    size_t n_upper = 0;
    double vertical_extent_m = 0.0;
    double h_min = 0.0;
    double h_max = 0.0;
    std::string reject_reason;
    std::vector<pcl::PointXYZI> samples;
    /// 标准柱内点数/上半段阈值未过，但「竖向延伸 + 锥度 + 冠层点簇」结构证据通过。
    bool used_structural_evidence = false;
};

inline double distPointToLineBody(const Eigen::Vector3d& p,
                                  const Eigen::Vector3d& p0,
                                  const Eigen::Vector3d& dir_unit) {
    const Eigen::Vector3d v = p - p0;
    const Eigen::Vector3d proj = v.dot(dir_unit) * dir_unit;
    return (v - proj).norm();
}

void horizontalBasisFromUp(Eigen::Vector3d up, Eigen::Vector3d* bx, Eigen::Vector3d* by) {
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    const Eigen::Vector3d ref = (std::abs(up.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    *bx = ref.cross(up);
    if (bx->norm() < 1e-9) {
        *bx = Eigen::Vector3d::UnitX().cross(up).normalized();
    } else {
        bx->normalize();
    }
    *by = up.cross(*bx).normalized();
}

bool shaftCellNearLinePrimitiveUv(double cu,
                                  double cv,
                                  const std::vector<GeometricResult::Primitive>& primitives,
                                  const Eigen::Vector3d& bx,
                                  const Eigen::Vector3d& by,
                                  double skip_xy_m) {
    if (skip_xy_m <= 0.0) {
        return false;
    }
    const double skip2 = skip_xy_m * skip_xy_m;
    for (const auto& pr : primitives) {
        if (pr.type != GeometricResult::Primitive::Type::LINE) {
            continue;
        }
        if (!pr.points || pr.points->empty()) {
            continue;
        }
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        for (const auto& q : pr.points->points) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
                continue;
            }
            const Eigen::Vector3d v(static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));
            sum += v;
        }
        sum /= static_cast<double>(pr.points->size());
        const double du = cu - sum.dot(bx);
        const double dv = cv - sum.dot(by);
        if (du * du + dv * dv < skip2) {
            return true;
        }
    }
    return false;
}

struct ShaftCandidateWork {
    double extent_m = 0.0;
    std::vector<pcl::PointXYZI> refined;
};

/**
 * geometric_only：2D 栅格粗判竖直柱 → 柱周 refine 收集地面–树冠间点（与 RANSAC LINE 互补）。
 */
std::vector<std::vector<pcl::PointXYZI>> collectGeometricOnlyShaftRefinedColumns(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ng,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ground,
    const Eigen::Vector3d& vertical_ref_body,
    const SemanticProcessor::Config& cfg,
    const std::vector<GeometricResult::Primitive>& primitives) {
    std::vector<std::vector<pcl::PointXYZI>> out;
    if (!cfg.geometric_only_shaft_enable || !ng || ng->empty()) {
        return out;
    }
    Eigen::Vector3d up = vertical_ref_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    Eigen::Vector3d bx;
    Eigen::Vector3d by;
    horizontalBasisFromUp(up, &bx, &by);

    const double cell = std::max(0.12, static_cast<double>(cfg.geometric_only_shaft_xy_cell_m));
    std::map<std::pair<int, int>, std::vector<pcl::PointXYZI>> cells;

    for (const auto& p : ng->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        const Eigen::Vector3d q(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        const double u = q.dot(bx);
        const double v = q.dot(by);
        const int iu = static_cast<int>(std::floor(u / cell));
        const int iv = static_cast<int>(std::floor(v / cell));
        cells[{iu, iv}].push_back(p);
    }

    std::vector<ShaftCandidateWork> works;
    works.reserve(std::min<size_t>(cells.size(), 256));

    const int min_pts = std::max(6, cfg.geometric_only_shaft_min_points);
    const double min_ext = std::max(0.5, static_cast<double>(cfg.geometric_only_shaft_min_extent_m));
    const double max_spread = std::max(0.15, static_cast<double>(cfg.geometric_only_shaft_max_xy_spread_m));
    const double refine_r = std::max(0.08, static_cast<double>(cfg.geometric_only_shaft_refine_radius_m));
    const double h_cap = std::max(6.0, static_cast<double>(cfg.geometric_only_shaft_rel_z_max_m));
    const float g_r = cfg.cylinder_fit_ground_search_radius_m;
    const int g_n = cfg.cylinder_fit_ground_min_samples;
    const float g_pct = cfg.cylinder_fit_ground_percentile;

    for (auto& kv : cells) {
        auto& bucket = kv.second;
        if (static_cast<int>(bucket.size()) < min_pts) {
            continue;
        }
        Eigen::Vector3d sum(0.0, 0.0, 0.0);
        for (const auto& p : bucket) {
            sum += Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        }
        const Eigen::Vector3d cmean = sum / static_cast<double>(bucket.size());
        const double cu = cmean.dot(bx);
        const double cv = cmean.dot(by);
        if (shaftCellNearLinePrimitiveUv(cu, cv, primitives, bx, by,
                static_cast<double>(cfg.geometric_only_shaft_skip_near_line_xy_m))) {
            continue;
        }

        const std::optional<double> s_ground_opt = estimateLocalGroundSupportAlongUp(
            ground,
            cmean.x(),
            cmean.y(),
            up,
            g_r,
            g_n,
            g_pct);
        double s_ground = s_ground_opt.value_or(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(s_ground)) {
            double s_min = std::numeric_limits<double>::infinity();
            for (const auto& p : bucket) {
                const Eigen::Vector3d q(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
                s_min = std::min(s_min, q.dot(up));
            }
            s_ground = std::isfinite(s_min) ? s_min : 0.0;
        }

        double h_min = std::numeric_limits<double>::infinity();
        double h_max = -std::numeric_limits<double>::infinity();
        for (const auto& p : bucket) {
            const Eigen::Vector3d q(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
            const double h = q.dot(up) - s_ground;
            h_min = std::min(h_min, h);
            h_max = std::max(h_max, h);
        }
        const double h_lo = std::max(0.0, h_min);
        const double extent = h_max - h_lo;
        if (!(extent >= min_ext)) {
            continue;
        }

        double sum_sq = 0.0;
        const Eigen::Vector3d chor = cmean - up * cmean.dot(up);
        for (const auto& p : bucket) {
            const Eigen::Vector3d q(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
            const Eigen::Vector3d hor = q - up * q.dot(up);
            const Eigen::Vector3d d = hor - chor;
            sum_sq += d.squaredNorm();
        }
        const double rms = std::sqrt(sum_sq / static_cast<double>(bucket.size()));
        if (rms > max_spread) {
            continue;
        }

        const Eigen::Vector3d p_axis = cmean + (s_ground - cmean.dot(up)) * up;

        std::vector<pcl::PointXYZI> refined;
        refined.reserve(std::min<size_t>(ng->size(), 8192u));
        for (const auto& np : ng->points) {
            if (!std::isfinite(np.x) || !std::isfinite(np.y) || !std::isfinite(np.z)) {
                continue;
            }
            const Eigen::Vector3d q(static_cast<double>(np.x), static_cast<double>(np.y), static_cast<double>(np.z));
            if (distPointToLineBody(q, p_axis, up) > refine_r) {
                continue;
            }
            const double h = q.dot(up) - s_ground;
            if (h < -0.42 || h > h_cap + 0.85) {
                continue;
            }
            refined.push_back(np);
        }
        if (static_cast<int>(refined.size()) < min_pts) {
            continue;
        }

        ShaftCandidateWork w;
        w.extent_m = extent;
        w.refined = std::move(refined);
        works.push_back(std::move(w));
    }

    std::sort(works.begin(), works.end(), [](const ShaftCandidateWork& a, const ShaftCandidateWork& b) {
        return a.extent_m > b.extent_m;
    });
    const int cap = std::max(1, std::min(256, cfg.geometric_only_shaft_max_candidates));
    for (int i = 0; i < std::min(cap, static_cast<int>(works.size())); ++i) {
        out.push_back(std::move(works[static_cast<size_t>(i)].refined));
    }
    return out;
}

GeometricResult::Primitive makeShaftSyntheticLinePrimitive(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pts,
                                                           const Eigen::Vector3d& up_unit,
                                                           double s_ground) {
    GeometricResult::Primitive prim;
    prim.type = GeometricResult::Primitive::Type::LINE;
    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    for (const auto& p : pts->points) {
        sum += Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
    }
    const Eigen::Vector3d c = sum / static_cast<double>(pts->size());
    const Eigen::Vector3d p0 = c + (s_ground - c.dot(up_unit)) * up_unit;
    prim.model_coeffs.setZero();
    prim.model_coeffs(0) = p0.x();
    prim.model_coeffs(1) = p0.y();
    prim.model_coeffs(2) = p0.z();
    prim.model_coeffs(3) = up_unit.x();
    prim.model_coeffs(4) = up_unit.y();
    prim.model_coeffs(5) = up_unit.z();
    prim.points = pts;
    prim.linearity = 0.62;
    prim.planarity = 0.12;
    prim.residual = 0.38;
    return prim;
}

static double medianSorted(std::vector<double>& v) {
    if (v.empty()) {
        return 0.0;
    }
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    return v[mid];
}

/// 柱内点在中低高度段径向分布相对偏高段的「下粗上细」关系（稀疏时样本少则跳过）。
static bool sparseTrunkRadialTaperOk(const std::vector<pcl::PointXYZI>& col,
                                     const Eigen::Vector3d& p0,
                                     const Eigen::Vector3d& dir_unit,
                                     const Eigen::Vector3d& up_unit,
                                     double s_ground,
                                     double max_rel_z_for_taper_m,
                                     double min_ratio,
                                     int min_pts_for_check) {
    if (static_cast<int>(col.size()) < min_pts_for_check) {
        return true;
    }
    std::vector<std::pair<double, double>> hd;
    hd.reserve(col.size());
    for (const auto& q : col) {
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
            continue;
        }
        const Eigen::Vector3d pb(static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));
        const double h = pb.dot(up_unit) - s_ground;
        if (h < 0.08 || h > max_rel_z_for_taper_m) {
            continue;
        }
        const double d = distPointToLineBody(pb, p0, dir_unit);
        hd.emplace_back(h, d);
    }
    if (hd.size() < static_cast<size_t>(min_pts_for_check)) {
        return true;
    }
    std::sort(hd.begin(), hd.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    const size_t n = hd.size();
    const size_t mid = n / 2;
    std::vector<double> r_low;
    std::vector<double> r_high;
    r_low.reserve(mid + 1);
    r_high.reserve(n - mid + 1);
    for (size_t i = 0; i < mid; ++i) {
        r_low.push_back(hd[i].second);
    }
    for (size_t i = mid; i < n; ++i) {
        r_high.push_back(hd[i].second);
    }
    const double m_low = medianSorted(r_low);
    const double m_high = medianSorted(r_high);
    if (m_high < 1e-4) {
        return m_low >= 1e-4;
    }
    return m_low >= m_high * min_ratio;
}

std::vector<pcl::PointXYZI> mergeTrunkBandAndColumnDedup(const std::vector<pcl::PointXYZI>& band,
                                                         const std::vector<pcl::PointXYZI>& column,
                                                         float voxel_m,
                                                         int max_pts);

SparseTrunkColumnEval evalSparseTrunkColumnForLine(
    const GeometricResult::Primitive& prim,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& nonground,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ground_cloud,
    const Eigen::Vector3d& vertical_ref_body,
    const SemanticProcessor::Config& cfg,
    bool log_column_stages,
    double log_ts,
    int log_line_idx) {
    SparseTrunkColumnEval out;
    auto log_reject = [&](const char* step, const char* reason) {
        if (!log_column_stages) {
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_COLUMN] ts=%.3f line_idx=%d step=%s reject=%s",
            log_ts,
            log_line_idx,
            step,
            reason);
    };
    if (!cfg.sparse_trunk_column_enable) {
        out.reject_reason = "disabled";
        log_reject("gate", "disabled");
        return out;
    }
    if (!nonground || nonground->empty()) {
        out.reject_reason = "no_nonground";
        log_reject("gate", "no_nonground");
        return out;
    }
    if (prim.model_coeffs.size() < 6) {
        out.reject_reason = "bad_line_model";
        log_reject("line_model", "bad_line_model");
        return out;
    }
    Eigen::Vector3d p0 = prim.model_coeffs.head<3>();
    Eigen::Vector3d dir(prim.model_coeffs(3), prim.model_coeffs(4), prim.model_coeffs(5));
    if (dir.norm() < 1e-9) {
        out.reject_reason = "zero_dir";
        log_reject("line_model", "zero_dir");
        return out;
    }
    dir.normalize();
    Eigen::Vector3d up = vertical_ref_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    if (dir.dot(up) < 0.0) {
        dir = -dir;
    }

    double cx = p0.x();
    double cy = p0.y();
    if (prim.points && !prim.points->empty()) {
        cx = 0.0;
        cy = 0.0;
        for (const auto& q : prim.points->points) {
            cx += static_cast<double>(q.x);
            cy += static_cast<double>(q.y);
        }
        cx /= static_cast<double>(prim.points->size());
        cy /= static_cast<double>(prim.points->size());
    }

    const std::optional<double> s_ground_opt = estimateLocalGroundSupportAlongUp(
        ground_cloud,
        cx,
        cy,
        up,
        cfg.cylinder_fit_ground_search_radius_m,
        cfg.cylinder_fit_ground_min_samples,
        cfg.cylinder_fit_ground_percentile);

    double support_min_along_up = std::numeric_limits<double>::infinity();
    if (prim.points) {
        for (const auto& q : prim.points->points) {
            const Eigen::Vector3d pb(static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));
            support_min_along_up = std::min(support_min_along_up, pb.dot(up));
        }
    }
    const double s_ground =
        s_ground_opt.value_or(std::isfinite(support_min_along_up) ? support_min_along_up : p0.dot(up));

    const double r_col = std::max(0.05, static_cast<double>(cfg.sparse_trunk_column_radius_m));
    const double r_canopy = std::max(r_col, static_cast<double>(cfg.sparse_trunk_connectivity_canopy_radius_m));
    const double h_upper_lo =
        std::min(static_cast<double>(cfg.sparse_trunk_upper_rel_z_min_m),
                 static_cast<double>(cfg.sparse_trunk_upper_rel_z_max_m));
    const double h_upper_hi =
        std::max(static_cast<double>(cfg.sparse_trunk_upper_rel_z_min_m),
                 static_cast<double>(cfg.sparse_trunk_upper_rel_z_max_m));
    const double h_scan_cap = h_upper_hi + 3.0;

    std::vector<pcl::PointXYZI> col;
    col.reserve(std::min<size_t>(nonground->size(), 8192));
    double h_min_p = std::numeric_limits<double>::infinity();
    double h_max_p = -std::numeric_limits<double>::infinity();

    // 🏛️ [Advanced Research] 用于三层连通性分析的辅助计数（兼顾树冠发散特征）
    int n_root_count = 0;
    int n_stem_count = 0;
    int n_canopy_count = 0;

    for (const auto& np : nonground->points) {
        if (!std::isfinite(np.x) || !std::isfinite(np.y) || !std::isfinite(np.z)) {
            continue;
        }
        const Eigen::Vector3d p(static_cast<double>(np.x), static_cast<double>(np.y), static_cast<double>(np.z));
        const double d_line = distPointToLineBody(p, p0, dir);
        const double h = p.dot(up) - s_ground;

        // 三层连通性判定：冠层判定从 1.5m 开始（兼顾低矮枝叶），并使用更大的搜索半径 r_canopy
        if (cfg.sparse_trunk_connectivity_recall_enable) {
            if (h < 0.5) {
                if (d_line <= r_col) ++n_root_count;
            } else if (h < 1.5) {
                if (d_line <= r_col) ++n_stem_count;
            } else if (h < 15.0) {
                if (d_line <= r_canopy) ++n_canopy_count;
            }
        }

        // 维持原有 col 集合（仅窄径点），用于圆柱拟合和锥度校验，保证拟合不被树冠噪点带偏
        if (d_line > r_col) {
            continue;
        }
        if (h < -0.35) {
            continue;
        }
        if (h > h_scan_cap) {
            continue;
        }
        col.push_back(np);
        h_min_p = std::min(h_min_p, h);
        h_max_p = std::max(h_max_p, h);
    }

    if (log_column_stages || cfg.diag_trunk_chain_log) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_COLUMN][DETAIL] ts=%.3f line_idx=%d step=connectivity "
            "root=%d stem=%d canopy=%d radii=[r_col:%.2f, r_canopy:%.2f] s_ground=%.2f",
            log_ts, log_line_idx, n_root_count, n_stem_count, n_canopy_count, r_col, r_canopy, s_ground);
    }

    out.n_column = col.size();
    out.h_min = h_min_p;
    out.h_max = h_max_p;
    if (col.empty() && !cfg.sparse_trunk_connectivity_recall_enable) {
        out.reject_reason = "empty_column";
        log_reject("collect", "empty_column");
        return out;
    }

    size_t n_upper = 0;
    for (const auto& np : col) {
        const Eigen::Vector3d p(static_cast<double>(np.x), static_cast<double>(np.y), static_cast<double>(np.z));
        const double h = p.dot(up) - s_ground;
        if (h >= h_upper_lo && h <= h_upper_hi) {
            ++n_upper;
        }
    }
    out.n_upper = n_upper;

    const double h_lo_clamped = std::max(0.0, h_min_p);
    out.vertical_extent_m = h_max_p - h_lo_clamped;

    const double min_vert = static_cast<double>(cfg.sparse_trunk_min_vertical_extent_m);
    const bool std_extent_ok = out.vertical_extent_m >= min_vert;
    const bool std_upper_ok = static_cast<int>(out.n_upper) >= cfg.sparse_trunk_min_upper_points;
    const bool std_col_ok = static_cast<int>(out.n_column) >= cfg.sparse_trunk_min_column_points;
    const bool standard_gate = std_extent_ok && std_upper_ok && std_col_ok;

    auto finalize_pass = [&](std::vector<pcl::PointXYZI>&& col_in) {
        std::vector<pcl::PointXYZI> work = std::move(col_in);
        const int cap = std::max(80, cfg.sparse_trunk_max_fit_points);
        if (static_cast<int>(work.size()) > cap) {
            std::vector<pcl::PointXYZI> sub;
            sub.reserve(static_cast<size_t>(cap));
            const size_t step = std::max<size_t>(1, work.size() / static_cast<size_t>(cap));
            for (size_t i = 0; i < work.size() && static_cast<int>(sub.size()) < cap; i += step) {
                sub.push_back(work[i]);
            }
            work.swap(sub);
        }
        out.samples = std::move(work);
        out.gate_ok = true;
    };

    if (standard_gate) {
        finalize_pass(std::move(col));
        if (log_column_stages) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_COLUMN] ts=%.3f line_idx=%d step=pass samples=%zu n_up=%zu extent_m=%.2f",
                log_ts,
                log_line_idx,
                out.samples.size(),
                out.n_upper,
                out.vertical_extent_m);
        }
        return out;
    }

    if (!cfg.sparse_trunk_structural_enable) {
        if (!std_extent_ok) {
            out.reject_reason = "extent";
            log_reject("gate", "extent");
            return out;
        }
        if (!std_upper_ok) {
            out.reject_reason = "upper_pts";
            log_reject("gate", "upper_pts");
            return out;
        }
        out.reject_reason = "column_pts";
        log_reject("gate", "column_pts");
        return out;
    }

    const double s_min_ext = static_cast<double>(cfg.sparse_trunk_structural_min_extent_m);
    if (out.vertical_extent_m < s_min_ext && !cfg.sparse_trunk_connectivity_recall_enable) {
        out.reject_reason = "extent";
        log_reject("structural", "extent");
        return out;
    }
    if (static_cast<int>(out.n_column) < cfg.sparse_trunk_structural_min_column_points && !cfg.sparse_trunk_connectivity_recall_enable) {
        out.reject_reason = "structural_column_pts";
        log_reject("structural", "column_pts");
        return out;
    }

    const double r_fol = std::max(static_cast<double>(cfg.sparse_trunk_structural_foliage_radius_m), r_col);
    const double f_lo = std::min(static_cast<double>(cfg.sparse_trunk_structural_foliage_rel_z_min_m),
                                 static_cast<double>(cfg.sparse_trunk_structural_foliage_rel_z_max_m));
    const double f_hi = std::max(static_cast<double>(cfg.sparse_trunk_structural_foliage_rel_z_min_m),
                                 static_cast<double>(cfg.sparse_trunk_structural_foliage_rel_z_max_m));

    std::vector<pcl::PointXYZI> foliage;
    foliage.reserve(std::min<size_t>(nonground->size(), 8192u));
    for (const auto& np : nonground->points) {
        if (!std::isfinite(np.x) || !std::isfinite(np.y) || !std::isfinite(np.z)) {
            continue;
        }
        const Eigen::Vector3d p(static_cast<double>(np.x), static_cast<double>(np.y), static_cast<double>(np.z));
        const double d_line = distPointToLineBody(p, p0, dir);
        if (d_line > r_fol) {
            continue;
        }
        const double h = p.dot(up) - s_ground;
        if (h < f_lo || h > f_hi) {
            continue;
        }
        foliage.push_back(np);
    }

    const size_t n_foliage_window = foliage.size();
    bool foliage_pts_ok = static_cast<int>(n_foliage_window) >= cfg.sparse_trunk_structural_min_foliage_points;

    const bool taper_ok = sparseTrunkRadialTaperOk(
        col,
        p0,
        dir,
        up,
        s_ground,
        static_cast<double>(cfg.sparse_trunk_structural_taper_max_rel_z_m),
        static_cast<double>(cfg.sparse_trunk_structural_taper_min_ratio),
        cfg.sparse_trunk_structural_taper_min_column_pts);

    // 🏛 : 改进后的连通性召回：地/干窄搜(r_col)，1.5m以上冠层广搜(r_canopy)
    bool connectivity_recall = false;
    if (cfg.sparse_trunk_connectivity_recall_enable) {
        const int min_p = cfg.sparse_trunk_connectivity_min_per_layer;
        // 冠层（1.5m以上）通常点云极多且发散，要求其点数显著高于树干层（建议 3-5 倍于 min_p）
        if (n_root_count >= min_p && n_stem_count >= min_p && n_canopy_count >= (min_p * 3)) {
            connectivity_recall = true;
        }
    }

    if (taper_ok || (foliage_pts_ok && connectivity_recall)) {
        // 环境点密度检查（Ambient Check）仍保留，防止在墙边搜到大量 canopy 点导致误触发
        bool ambient_check_ok = true;
        std::string ambient_reject_detail;
        if (cfg.sparse_trunk_structural_ambient_check_enable && cfg.sparse_trunk_structural_ambient_max_points > 0) {
            const double z_pad = static_cast<double>(cfg.sparse_trunk_structural_ambient_rel_z_pad_m);
            const double h_a_lo = std::max(0.0, h_min_p - z_pad);
            const double h_a_hi = h_max_p + z_pad;
            const double r_in = r_col + static_cast<double>(cfg.sparse_trunk_structural_ambient_inner_margin_m);
            double r_out = static_cast<double>(cfg.sparse_trunk_structural_ambient_outer_radius_m);
            if (!(r_out > r_in + 0.05)) {
                r_out = r_in + 0.25;
            }
            size_t n_ambient = 0;
            for (const auto& np : nonground->points) {
                if (!std::isfinite(np.x) || !std::isfinite(np.y) || !std::isfinite(np.z)) {
                    continue;
                }
                const Eigen::Vector3d p(static_cast<double>(np.x), static_cast<double>(np.y), static_cast<double>(np.z));
                const double d_line = distPointToLineBody(p, p0, dir);
                if (d_line <= r_in || d_line > r_out) {
                    continue;
                }
                const double h = p.dot(up) - s_ground;
                if (h < h_a_lo || h > h_a_hi) {
                    continue;
                }
                ++n_ambient;
            }
            if (static_cast<int>(n_ambient) > cfg.sparse_trunk_structural_ambient_max_points) {
                ambient_check_ok = false;
                if (log_column_stages || cfg.diag_trunk_chain_log) {
                    std::ostringstream oss;
                    oss << "n_ambient=" << n_ambient << " > max=" << cfg.sparse_trunk_structural_ambient_max_points;
                    ambient_reject_detail = oss.str();
                }
            }
        }

        if (!ambient_check_ok) {
            out.reject_reason = "structural_ambient";
            if (!ambient_reject_detail.empty()) out.reject_reason += " (" + ambient_reject_detail + ")";
            log_reject("structural", out.reject_reason.c_str());
            return out;
        }
        out.used_structural_evidence = true;
        if (connectivity_recall && !taper_ok) {
            out.reject_reason = "connectivity_recall_win";
            if (log_column_stages || cfg.diag_trunk_chain_log) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][TRUNK_COLUMN] ts=%.3f line_idx=%d step=RECALL_WIN reason=connectivity_survived_low_taper",
                    log_ts, log_line_idx);
            }
        }

        // 合并树干点和树冠点，确保地标点云完整
        const int fol_cap = cfg.sparse_trunk_structural_merge_foliage_max_points;
        if (static_cast<int>(foliage.size()) > fol_cap) {
            std::vector<pcl::PointXYZI> sub;
            sub.reserve(static_cast<size_t>(fol_cap));
            const size_t step = std::max<size_t>(1, foliage.size() / static_cast<size_t>(fol_cap));
            for (size_t i = 0; i < foliage.size() && static_cast<int>(sub.size()) < fol_cap; i += step) {
                sub.push_back(foliage[i]);
            }
            foliage.swap(sub);
        }
        const int merge_max = std::max(200, cfg.sparse_trunk_max_fit_points);
        std::vector<pcl::PointXYZI> merged = mergeTrunkBandAndColumnDedup(col, foliage, 0.08f, merge_max);

        finalize_pass(std::move(merged));
        return out;
    }

    out.reject_reason = "structural_taper_or_connectivity";
    if (log_column_stages || cfg.diag_trunk_chain_log) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_COLUMN][REJECT] ts=%.3f line_idx=%d taper_ok=%d conn_ok=%d fol_ok=%d reason=structural_failed",
            log_ts, log_line_idx, taper_ok ? 1 : 0, connectivity_recall ? 1 : 0, foliage_pts_ok ? 1 : 0);
    }
    log_reject("structural", "taper_or_connectivity");
    return out;
}

std::vector<pcl::PointXYZI> mergeTrunkBandAndColumnDedup(const std::vector<pcl::PointXYZI>& band,
                                                         const std::vector<pcl::PointXYZI>& column,
                                                         float voxel_m,
                                                         int max_pts) {
    const float vm = std::max(0.03f, voxel_m);
    auto key_of = [&](const pcl::PointXYZI& p) -> uint64_t {
        const auto ix = static_cast<int32_t>(std::floor(static_cast<double>(p.x) / static_cast<double>(vm)));
        const auto iy = static_cast<int32_t>(std::floor(static_cast<double>(p.y) / static_cast<double>(vm)));
        const auto iz = static_cast<int32_t>(std::floor(static_cast<double>(p.z) / static_cast<double>(vm)));
        const uint64_t a = static_cast<uint64_t>(static_cast<uint32_t>(ix));
        const uint64_t b = static_cast<uint64_t>(static_cast<uint32_t>(iy));
        const uint64_t c = static_cast<uint64_t>(static_cast<uint32_t>(iz));
        return (a << 42) | (b << 21) | (c & ((1ULL << 21) - 1ULL));
    };
    std::unordered_set<uint64_t> seen;
    std::vector<pcl::PointXYZI> out;
    out.reserve(std::min<size_t>(band.size() + column.size(), static_cast<size_t>(max_pts) * 2));
    for (const auto& p : band) {
        const uint64_t k = key_of(p);
        if (seen.insert(k).second) {
            out.push_back(p);
        }
    }
    for (const auto& p : column) {
        const uint64_t k = key_of(p);
        if (seen.insert(k).second) {
            out.push_back(p);
        }
    }
    if (static_cast<int>(out.size()) > max_pts) {
        std::vector<pcl::PointXYZI> sub;
        const size_t step = std::max<size_t>(1, out.size() / static_cast<size_t>(max_pts));
        for (size_t i = 0; i < out.size() && static_cast<int>(sub.size()) < max_pts; i += step) {
            sub.push_back(out[i]);
        }
        out.swap(sub);
    }
    return out;
}

/// 合并 band+column 后点云主方向：最大特征值对应特征向量；与重力对齐不足时退回竖直轴（离散竖条/冠层噪声时更稳）。
bool computeSparseTrunkDominantAxisFromPoints(const std::vector<pcl::PointXYZI>& pts,
                                              const Eigen::Vector3d& vertical_ref_body,
                                              double min_abs_up_cos,
                                              Eigen::Vector3d* out_axis) {
    if (!out_axis || pts.size() < 3) {
        return false;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    int n_fin = 0;
    for (const auto& p : pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        mean += Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        ++n_fin;
    }
    if (n_fin < 3) {
        return false;
    }
    mean /= static_cast<double>(n_fin);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        Eigen::Vector3d d =
            Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)) - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(n_fin);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.info() != Eigen::Success) {
        return false;
    }
    Eigen::Vector3d up = vertical_ref_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    Eigen::Vector3d v = es.eigenvectors().col(2);
    if (v.norm() < 1e-9) {
        *out_axis = up;
        return true;
    }
    v.normalize();
    const double align = std::abs(v.dot(up));
    if (align < min_abs_up_cos) {
        *out_axis = up;
    } else {
        if (v.dot(up) < 0.0) {
            v = -v;
        }
        *out_axis = v;
    }
    return true;
}

/// SemanticProcessor::FitRejectReason 数值与枚举声明顺序一致（仅用于日志字符串）。
const char* trunkFitRejectCStr(int code) {
    switch (code) {
        case 0:
            return "success";
        case 1:
            return "ransac_reject";
        case 2:
            return "ceres_fail";
        case 3:
            return "reject_radius";
        case 4:
            return "reject_tilt";
        case 5:
            return "reject_height_window";
        case 6:
            return "reject_radial_cv";
        case 7:
            return "reject_axial_span";
        default:
            return "unknown";
    }
}

// |ray·Z| below this: axis is too horizontal to infer "up" reliably — keep solver sign (no flip).
constexpr double kCylinderAxisMinUpCos = 0.15;

/// Flip axis so it points into the +up_ref hemisphere when the trunk is clearly upright; PCL may return either sign.
inline void canonicalizeCylinderAxisUp(Eigen::Vector3d* ray, const Eigen::Vector3d& up_ref) {
    if (!ray || ray->norm() < 1e-9) {
        return;
    }
    ray->normalize();
    Eigen::Vector3d up = up_ref;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    const double align = ray->dot(up);
    if (std::abs(align) < kCylinderAxisMinUpCos) {
        return;
    }
    if (align < 0.0) {
        *ray = -*ray;
    }
}

// Along-axis position at this lower tail of t (robust vs a single noise spike at min-t).
constexpr double kAnchorAxialPercentile = 0.10;

/// Slide root along axis using a lower percentile of axial coordinates (not raw min — avoids one outlier underground).
template <typename PointRange>
void anchorCylinderRootOnAxisFootprint(Eigen::Vector3d* root,
                                       const Eigen::Vector3d& ray_unit,
                                       const PointRange& range) {
    if (!root) {
        return;
    }
    std::vector<double> ts;
    for (const auto& p : range) {
        const Eigen::Vector3d pt(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        ts.push_back((pt - *root).dot(ray_unit));
    }
    if (ts.empty()) {
        return;
    }
    std::sort(ts.begin(), ts.end());
    const size_t n = ts.size();
    const size_t idx =
        std::min(n - 1, static_cast<size_t>(std::floor(kAnchorAxialPercentile * static_cast<double>(n - 1))));
    const double t_anchor = ts[idx];
    if (std::isfinite(t_anchor)) {
        *root += t_anchor * ray_unit;
    }
}

/// 稀疏柱门控已通过（接地、竖向延伸、上半段点数），但圆柱拟合失败时：用几何 LINE 轴向 + 点云到轴径向距离的中位数估计半径，
/// 根部用轴线低分位锚定（与 fitCylinder 后 anchor 一致）。
CylinderLandmark::Ptr buildSparseTrunkFallbackLandmark(const GeometricResult::Primitive& prim,
                                                       const std::vector<pcl::PointXYZI>& pts_body,
                                                       const Eigen::Vector3d& vertical_ref_body,
                                                       const SemanticProcessor::Config& cfg,
                                                       bool log_detail,
                                                       double log_ts,
                                                       int log_line_idx,
                                                       std::string* fail_out,
                                                       const Eigen::Vector3d* axis_override,
                                                       double r_med_slack_mult) {
    const bool use_axis_override = axis_override && axis_override->norm() > 1e-9;
    auto set_fail = [&](const char* code) {
        if (fail_out) {
            *fail_out = code;
        }
        if (log_detail) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_FALLBACK] ts=%.3f line_idx=%d step=reject reason=%s pts=%zu",
                log_ts,
                log_line_idx,
                code,
                pts_body.size());
        }
    };
    if (!use_axis_override && prim.model_coeffs.size() < 6) {
        set_fail("bad_line_coeffs");
        return nullptr;
    }
    if (pts_body.size() < 10) {
        set_fail("pts_lt_10");
        return nullptr;
    }
    Eigen::Vector3d up = vertical_ref_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    Eigen::Vector3d ray;
    Eigen::Vector3d root;
    if (use_axis_override) {
        ray = axis_override->normalized();
        if (ray.dot(up) < 0.0) {
            ray = -ray;
        }
        canonicalizeCylinderAxisUp(&ray, up);
        root = Eigen::Vector3d::Zero();
        int n_fin = 0;
        for (const auto& p : pts_body) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }
            root += Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
            ++n_fin;
        }
        if (n_fin < 1) {
            set_fail("no_finite_pts");
            return nullptr;
        }
        root /= static_cast<double>(n_fin);
    } else {
        Eigen::Vector3d p0 = prim.model_coeffs.head<3>();
        ray = Eigen::Vector3d(prim.model_coeffs(3), prim.model_coeffs(4), prim.model_coeffs(5));
        if (ray.norm() < 1e-9) {
            set_fail("zero_dir");
            return nullptr;
        }
        ray.normalize();
        if (ray.dot(up) < 0.0) {
            ray = -ray;
        }
        canonicalizeCylinderAxisUp(&ray, up);
        root = p0;
    }
    const double tilt_deg =
        std::acos(std::clamp(std::abs(ray.dot(up)), 0.0, 1.0)) * 180.0 / M_PI;
    const double tilt_max_deg =
        use_axis_override
            ? (static_cast<double>(cfg.max_axis_theta) +
               static_cast<double>(cfg.sparse_trunk_fallback_pca_extra_tilt_deg))
            : static_cast<double>(cfg.max_axis_theta);
    if (tilt_deg > tilt_max_deg) {
        set_fail("axis_tilt");
        return nullptr;
    }
    anchorCylinderRootOnAxisFootprint(&root, ray, pts_body);
    std::vector<double> radial;
    radial.reserve(pts_body.size());
    for (const auto& p : pts_body) {
        const Eigen::Vector3d pt(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        const Eigen::Vector3d v = pt - root;
        const double along = v.dot(ray);
        const Eigen::Vector3d perp = v - along * ray;
        radial.push_back(perp.norm());
    }
    std::sort(radial.begin(), radial.end());
    const double r_med = radial[radial.size() / 2];
    const double r_max = static_cast<double>(cfg.max_tree_radius);
    const double r_slack = (r_med_slack_mult > 1.0 + 1e-6) ? r_med_slack_mult : 1.12;
    if (!std::isfinite(r_med) || r_med > r_max * r_slack) {
        set_fail(!std::isfinite(r_med) ? "r_med_nonfinite" : "r_med_too_large");
        return nullptr;
    }
    const double r_floor = std::max(0.06, static_cast<double>(cfg.default_tree_radius) * 0.5);
    const double r = std::clamp(r_med, r_floor, r_max);
    auto lm = std::make_shared<CylinderLandmark>();
    lm->root = root;
    lm->ray = ray;
    lm->radius = r;
    double conf =
        std::max(static_cast<double>(prim.residual), static_cast<double>(cfg.sparse_trunk_fallback_min_confidence));
    conf += std::min(0.12, 0.0010 * static_cast<double>(pts_body.size()));
    lm->confidence = std::min(0.82, conf);
    lm->primitive_linearity = static_cast<double>(prim.linearity);
    lm->detection_point_count = static_cast<int>(pts_body.size());
    lm->id = allocateLandmarkId();
    if (log_detail) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_FALLBACK] ts=%.3f line_idx=%d step=%s r_med=%.3f r_used=%.3f conf=%.3f pts=%zu "
            "r_slack=%.2f tilt=%.1f tilt_max=%.1f",
            log_ts,
            log_line_idx,
            use_axis_override ? "build_ok_pca_axis" : "build_ok",
            r_med,
            r,
            lm->confidence,
            pts_body.size(),
            r_slack,
            tilt_deg,
            tilt_max_deg);
    }
    return lm;
}

/// 结构化判树（下粗上细+冠层）通过后：用重力轴 + 合并样本直接构造圆柱；半径取轴向下半/上半径向中位数的加权（近似圆台的有效半径），不再走 RANSAC/Ceres 与 fallback。
CylinderLandmark::Ptr buildStructuralTrunkCylinderFromSamples(const GeometricResult::Primitive& prim,
                                                              const std::vector<pcl::PointXYZI>& pts_body,
                                                              const Eigen::Vector3d& vertical_ref_body,
                                                              const SemanticProcessor::Config& cfg,
                                                              bool log_detail,
                                                              double log_ts,
                                                              int log_line_idx) {
    if (pts_body.size() < 3) {
        if (log_detail) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_STRUCTURAL] ts=%.3f line_idx=%d step=reject reason=pts_lt_3 pts=%zu",
                log_ts,
                log_line_idx,
                pts_body.size());
        }
        return nullptr;
    }
    Eigen::Vector3d up = vertical_ref_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }
    Eigen::Vector3d ray = up;
    canonicalizeCylinderAxisUp(&ray, up);

    Eigen::Vector3d root = Eigen::Vector3d::Zero();
    int n_fin = 0;
    for (const auto& p : pts_body) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        root += Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        ++n_fin;
    }
    if (n_fin < 3) {
        if (log_detail) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_STRUCTURAL] ts=%.3f line_idx=%d step=reject reason=no_finite_pts",
                log_ts,
                log_line_idx);
        }
        return nullptr;
    }
    root /= static_cast<double>(n_fin);
    anchorCylinderRootOnAxisFootprint(&root, ray, pts_body);

    std::vector<std::pair<double, double>> along_radial;
    along_radial.reserve(pts_body.size());
    for (const auto& p : pts_body) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        const Eigen::Vector3d pt(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        const Eigen::Vector3d v = pt - root;
        const double along = v.dot(ray);
        const Eigen::Vector3d perp = v - along * ray;
        along_radial.emplace_back(along, perp.norm());
    }
    if (along_radial.size() < 3) {
        return nullptr;
    }
    std::sort(along_radial.begin(), along_radial.end());

    const double r_max = static_cast<double>(cfg.max_tree_radius);
    const double r_floor = std::max(0.06, static_cast<double>(cfg.default_tree_radius) * 0.45);
    double r_eff = 0.0;
    double r_base_log = -1.0;
    double r_top_log = -1.0;
    if (along_radial.size() >= 6) {
        const size_t mid = along_radial.size() / 2;
        std::vector<double> r_lo;
        std::vector<double> r_hi;
        r_lo.reserve(mid);
        r_hi.reserve(along_radial.size() - mid);
        for (size_t i = 0; i < mid; ++i) {
            r_lo.push_back(along_radial[i].second);
        }
        for (size_t i = mid; i < along_radial.size(); ++i) {
            r_hi.push_back(along_radial[i].second);
        }
        const double r_base = medianSorted(r_lo);
        const double r_top = medianSorted(r_hi);
        r_base_log = r_base;
        r_top_log = r_top;
        // 近似圆台：下半更粗权重大，避免冠层大半径把圆柱半径整体抬得过高
        r_eff = 0.58 * r_base + 0.42 * r_top;
    } else {
        std::vector<double> all_r;
        all_r.reserve(along_radial.size());
        for (const auto& ar : along_radial) {
            all_r.push_back(ar.second);
        }
        r_eff = medianSorted(all_r);
    }
    if (!std::isfinite(r_eff) || r_eff < 1e-6) {
        if (log_detail) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_STRUCTURAL] ts=%.3f line_idx=%d step=reject reason=r_eff_bad",
                log_ts,
                log_line_idx);
        }
        return nullptr;
    }
    r_eff = std::clamp(r_eff, r_floor, r_max);

    auto lm = std::make_shared<CylinderLandmark>();
    lm->root = root;
    lm->ray = ray;
    lm->radius = r_eff;
    double conf = std::max(static_cast<double>(prim.residual),
                           static_cast<double>(cfg.sparse_trunk_fallback_min_confidence));
    conf += 0.10;
    conf += std::min(0.10, 0.0008 * static_cast<double>(along_radial.size()));
    lm->confidence = std::min(0.80, conf);
    lm->primitive_linearity =
        (prim.linearity > 1e-6f) ? static_cast<double>(prim.linearity) : 0.52;
    lm->detection_point_count = static_cast<int>(pts_body.size());
    lm->id = allocateLandmarkId();
    if (log_detail) {
        if (r_base_log >= 0.0 && r_top_log >= 0.0) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_STRUCTURAL] ts=%.3f line_idx=%d step=direct_cylinder_ok r_eff=%.3f "
                "r_lo_med=%.3f r_hi_med=%.3f conf=%.3f pts=%zu",
                log_ts,
                log_line_idx,
                r_eff,
                r_base_log,
                r_top_log,
                lm->confidence,
                pts_body.size());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_STRUCTURAL] ts=%.3f line_idx=%d step=direct_cylinder_ok r_eff=%.3f "
                "r_med_single conf=%.3f pts=%zu",
                log_ts,
                log_line_idx,
                r_eff,
                lm->confidence,
                pts_body.size());
        }
    }
    return lm;
}

}  // namespace

struct CylinderFittingCost {
    CylinderFittingCost(const Eigen::Vector3d& point) : point_(point) {}

    template <typename T>
    bool operator()(const T* const root, const T* const ray, const T* const radius, T* residual) const {
        Eigen::Matrix<T, 3, 1> p{T(point_.x()), T(point_.y()), T(point_.z())};
        Eigen::Matrix<T, 3, 1> r{root[0], root[1], root[2]};
        Eigen::Matrix<T, 3, 1> v{ray[0], ray[1], ray[2]};
        T rad = radius[0];

        // Distance from point p to line (r, v)
        // d = |(p - r) x v| / |v|
        Eigen::Matrix<T, 3, 1> pr = p - r;
        Eigen::Matrix<T, 3, 1> cross = pr.cross(v);
        T v_norm = v.norm();
        if (v_norm < T(1e-6)) return false;
        
        T dist_to_axis = cross.norm() / v_norm;
        residual[0] = dist_to_axis - rad;
        return true;
    }

    Eigen::Vector3d point_;
};

SemanticProcessor::SemanticProcessor(const Config& config) : config_(config) {
    try {
        if (config_.diag_cluster_profile == "relaxed") {
            // Diagnostic-only profile for quickly testing "cluster threshold too strict".
            config_.beam_cluster_threshold = std::max(config_.beam_cluster_threshold, 0.5f);
            config_.max_dist_to_centroid = std::max(config_.max_dist_to_centroid, 0.5f);
            config_.min_vertex_size = std::min(config_.min_vertex_size, 2);
            config_.min_landmark_size = std::min(config_.min_landmark_size, 2);
            config_.min_landmark_height = std::min(config_.min_landmark_height, 0.3f);
        }

        SegmentorConfig seg_cfg;
        seg_cfg.model_type = config.model_type;
        seg_cfg.model_path = config.model_path;
        seg_cfg.lsk3dnet_model_path = config.lsk3dnet_model_path;
        seg_cfg.lsk3dnet_device = config.lsk3dnet_device;
        seg_cfg.lsk3dnet_repo_root = config.lsk3dnet_repo_root;
        seg_cfg.lsk3dnet_config_yaml = config.lsk3dnet_config_yaml;
        seg_cfg.lsk3dnet_checkpoint = config.lsk3dnet_checkpoint;
        seg_cfg.lsk3dnet_classifier_torchscript = config.lsk3dnet_classifier_torchscript;
        seg_cfg.lsk3dnet_python_exe = config.lsk3dnet_python_exe;
        seg_cfg.lsk3dnet_worker_script = config.lsk3dnet_worker_script;
        seg_cfg.lsk3dnet_hybrid_normal_mode = config.lsk3dnet_hybrid_normal_mode;
        seg_cfg.lsk3dnet_normal_fov_up_deg = config.lsk3dnet_normal_fov_up_deg;
        seg_cfg.lsk3dnet_normal_fov_down_deg = config.lsk3dnet_normal_fov_down_deg;
        seg_cfg.lsk3dnet_normal_proj_h = config.lsk3dnet_normal_proj_h;
        seg_cfg.lsk3dnet_normal_proj_w = config.lsk3dnet_normal_proj_w;
        seg_cfg.fov_up = config.fov_up;
        seg_cfg.fov_down = config.fov_down;
        seg_cfg.img_w = config.img_w;
        seg_cfg.img_h = config.img_h;
        seg_cfg.lsk_num_vote = std::max(1, std::min(32, config.lsk_num_vote));
        seg_cfg.lsk_training_volume_crop = config.lsk_training_volume_crop;
        seg_cfg.lsk_volume_bounds_valid = config.lsk_volume_bounds_valid;
        seg_cfg.lsk_vol_min_x = config.lsk_vol_min_x;
        seg_cfg.lsk_vol_min_y = config.lsk_vol_min_y;
        seg_cfg.lsk_vol_min_z = config.lsk_vol_min_z;
        seg_cfg.lsk_vol_max_x = config.lsk_vol_max_x;
        seg_cfg.lsk_vol_max_y = config.lsk_vol_max_y;
        seg_cfg.lsk_vol_max_z = config.lsk_vol_max_z;
        seg_cfg.input_channels = config.input_channels;
        seg_cfg.num_classes = config.num_classes;
        seg_cfg.tree_class_id = config.tree_class_id;
        seg_cfg.input_mean = config.input_mean;
        seg_cfg.input_std = config.input_std;
        seg_cfg.do_destagger = config.do_destagger;
        if (config.mode == "geometric_only") {
            seg_cfg.model_type = "noop_geometric";
        }
        segmentor_ = SemanticSegmentorFactory::Create(seg_cfg);

        instance_detector_ = std::make_unique<Instance>();
        Instance::Params params;
        params.beam_cluster_threshold = config.beam_cluster_threshold;
        params.max_dist_to_centroid = config.max_dist_to_centroid;
        params.min_vertex_size = config.min_vertex_size;
        params.min_landmark_size = config.min_landmark_size;
        params.min_landmark_height = config.min_landmark_height;
        params.min_cluster_points = std::max(1, config.diag_trellis_min_cluster_points);
        params.min_tree_vertices = std::max(1, config.diag_trellis_min_tree_vertices);
        instance_detector_->set_params(params);
        
        if (config_.geometric_enabled) {
            GeometricProcessorConfig geo_cfg;
            geo_cfg.patchwork.sensor_height = config_.patchwork.sensor_height;
            geo_cfg.patchwork.num_iter = config_.patchwork.num_iter;
            geo_cfg.patchwork.th_dist = config_.patchwork.th_dist;
            geo_cfg.patchwork.max_range = config_.patchwork.max_range;
            geo_cfg.patchwork.auto_sensor_height = config_.patchwork.auto_sensor_height;
            geo_cfg.patchwork.auto_height_min_xy_m = config_.patchwork.auto_height_min_xy_m;
            geo_cfg.patchwork.auto_height_max_xy_m = config_.patchwork.auto_height_max_xy_m;
            geo_cfg.patchwork.auto_height_min_samples = config_.patchwork.auto_height_min_samples;
            geo_cfg.patchwork.auto_height_percentile = config_.patchwork.auto_height_percentile;
            geo_cfg.patchwork.auto_height_clamp_min_m = config_.patchwork.auto_height_clamp_min_m;
            geo_cfg.patchwork.auto_height_clamp_max_m = config_.patchwork.auto_height_clamp_max_m;
            geo_cfg.patchwork.auto_height_ema_alpha = config_.patchwork.auto_height_ema_alpha;
            geo_cfg.patchwork.auto_height_max_z_over_r = config_.patchwork.auto_height_max_z_over_r;
            geo_cfg.patchwork.use_odom_gravity = config_.patchwork.use_odom_gravity;
            geo_cfg.patchwork.odom_up_axis = config_.patchwork.odom_up_axis;
            geo_cfg.patchwork.level_cloud_for_patchwork = config_.patchwork.level_cloud_for_patchwork;
            geo_cfg.wall_ransac.enabled = config_.wall_ransac.enabled;
            geo_cfg.wall_ransac.distance_threshold = config_.wall_ransac.distance_threshold;
            geo_cfg.wall_ransac.min_inliers = config_.wall_ransac.min_inliers;
            geo_cfg.wall_ransac.max_normal_tilt_deg = config_.wall_ransac.max_normal_tilt_deg;
            geo_cfg.wall_ransac.line_distance_threshold = config_.wall_ransac.line_distance_threshold;
            geo_cfg.wall_ransac.line_min_inliers = config_.wall_ransac.line_min_inliers;
            geo_cfg.wall_ransac.plane_distance_threshold = config_.wall_ransac.plane_distance_threshold;
            geo_cfg.wall_ransac.plane_min_inliers = config_.wall_ransac.plane_min_inliers;
            geo_cfg.accumulator.enabled = config_.accumulator.enabled;
            geo_cfg.accumulator.max_frames = config_.accumulator.max_frames;
            geo_cfg.accumulator.tag_intensity_with_scan_seq = config_.accumulator.tag_intensity_with_scan_seq;
            geo_cfg.accumulator.save_debug_pcd = config_.accumulator.save_debug_pcd;
            geo_cfg.accumulator.save_merged_cloud_dir = config_.accumulator.save_merged_cloud_dir;
            geo_cfg.accumulator.save_merged_cloud_every_n = config_.accumulator.save_merged_cloud_every_n;
            geo_cfg.accumulator.save_accum_body_pcd = config_.accumulator.save_accum_body_pcd;
            geo_cfg.accumulator.save_primitive_input_cloud = config_.accumulator.save_primitive_input_cloud;
            geo_cfg.primitive_classifier.enabled = config_.primitive_classifier.enabled;
            geo_cfg.primitive_classifier.linearity_threshold = config_.primitive_classifier.linearity_threshold;
            geo_cfg.primitive_classifier.planarity_threshold = config_.primitive_classifier.planarity_threshold;
            geo_cfg.euclidean_cluster.tolerance_m = config_.euclidean_cluster.tolerance_m;
            geo_cfg.euclidean_cluster.min_points = config_.euclidean_cluster.min_points;
            geo_cfg.euclidean_cluster.max_points = config_.euclidean_cluster.max_points;
            geo_cfg.primitive_roi.enabled = config_.primitive_roi.enabled;
            geo_cfg.primitive_roi.body_xy_radius_m = config_.primitive_roi.body_xy_radius_m;
            geo_cfg.primitive_roi.ring_min_xy_m = config_.primitive_roi.ring_min_xy_m;
            geo_cfg.primitive_roi.ring_max_xy_m = config_.primitive_roi.ring_max_xy_m;
            geo_cfg.primitive_roi.voxel_leaf_m = config_.primitive_roi.voxel_leaf_m;
            geo_cfg.range_view.enabled = config_.range_view.enabled;
            geo_cfg.range_view.mode = config_.range_view.mode;
            geo_cfg.range_view.image_width = config_.range_view.image_width;
            geo_cfg.range_view.image_height = config_.range_view.image_height;
            geo_cfg.range_view.min_range_m = config_.range_view.min_range_m;
            geo_cfg.range_view.max_range_m = config_.range_view.max_range_m;
            geo_cfg.range_view.elev_min_deg = config_.range_view.elev_min_deg;
            geo_cfg.range_view.elev_max_deg = config_.range_view.elev_max_deg;
            geo_cfg.range_view.grad_mag_norm_thresh = config_.range_view.grad_mag_norm_thresh;
            geo_cfg.range_view.dilate_iterations = config_.range_view.dilate_iterations;
            geo_cfg.range_view.min_cc_pixels = config_.range_view.min_cc_pixels;
            geo_cfg.range_view.max_cc_pixels = config_.range_view.max_cc_pixels;
            geo_cfg.range_view.wall_min_width_u = config_.range_view.wall_min_width_u;
            geo_cfg.range_view.wall_max_aspect_h_over_w = config_.range_view.wall_max_aspect_h_over_w;
            geo_cfg.range_view.trunk_max_width_u = config_.range_view.trunk_max_width_u;
            geo_cfg.range_view.trunk_min_aspect_h_over_w = config_.range_view.trunk_min_aspect_h_over_w;
            geo_cfg.range_view.bbox_margin_u = config_.range_view.bbox_margin_u;
            geo_cfg.range_view.bbox_margin_v = config_.range_view.bbox_margin_v;
            geo_cfg.range_view.max_patches_per_frame = config_.range_view.max_patches_per_frame;
            geo_cfg.range_view.max_patch_points = config_.range_view.max_patch_points;
            geo_cfg.range_view.fallback_full_cloud = config_.range_view.fallback_full_cloud;
            geo_cfg.range_view.onnx_model_path = config_.range_view.onnx_model_path;
            geo_cfg.range_view.onnx_input_width = config_.range_view.onnx_input_width;
            geo_cfg.range_view.onnx_input_height = config_.range_view.onnx_input_height;
            geo_cfg.range_view.onnx_n_classes = config_.range_view.onnx_n_classes;
            geo_cfg.range_view.onnx_wall_class_id = config_.range_view.onnx_wall_class_id;
            geo_cfg.range_view.onnx_trunk_class_id = config_.range_view.onnx_trunk_class_id;
            geo_cfg.range_view.fusion_rv_boost_scale = config_.range_view.fusion_rv_boost_scale;
            geo_cfg.max_lines_per_cluster = std::max(1, std::min(8, config_.max_lines_per_cluster));
            geo_cfg.log_level = config_.geometric_log_level;
            geo_cfg.log_detail = config_.geometric_log_detail;
            geometric_processor_ = std::make_unique<GeometricProcessor>(geo_cfg);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
            "[SEMANTIC][Processor][INIT] step=ok backend=%s model_type=%s fov=[%.1f,%.1f] img=%dx%d input_channels=%d num_classes=%d tree_class_id=%d",
            segmentor_->name(), config.model_type.c_str(), config.fov_up, config.fov_down, config.img_w, config.img_h,
            config.input_channels, config.num_classes, config.tree_class_id);
        
        if (config.model_type == "lsk3dnet_hybrid" && config.mode != "geometric_only") {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INIT_HYBRID] checkpoint=%s classifier=%s python=%s worker=%s",
                config.lsk3dnet_checkpoint.c_str(), config.lsk3dnet_classifier_torchscript.c_str(),
                config.lsk3dnet_python_exe.c_str(), config.lsk3dnet_worker_script.c_str());
            const bool official_profile =
                std::abs(config.lsk3dnet_normal_fov_up_deg - 3.0f) < 1e-3f &&
                std::abs(config.lsk3dnet_normal_fov_down_deg + 25.0f) < 1e-3f &&
                config.lsk3dnet_normal_proj_h == 64 &&
                config.lsk3dnet_normal_proj_w == 900;
            const bool equals_sensor_fov =
                std::abs(config.lsk3dnet_normal_fov_up_deg - config.fov_up) < 1e-3f &&
                std::abs(config.lsk3dnet_normal_fov_down_deg - config.fov_down) < 1e-3f;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][NORMAL_PROFILE_AUDIT] source_keys={semantic.lsk3dnet.normal_fov_up_deg,semantic.lsk3dnet.normal_fov_down_deg,semantic.lsk3dnet.normal_proj_h,semantic.lsk3dnet.normal_proj_w} "
                "effective=[up=%.1f,down=%.1f,h=%d,w=%d] official=[up=3.0,down=-25.0,h=64,w=900] "
                "sensor_fov=[up=%.1f,down=%.1f] official_match=%d sensor_fov_match=%d note='not from loop_closure.overlap_transformer.*'",
                config.lsk3dnet_normal_fov_up_deg,
                config.lsk3dnet_normal_fov_down_deg,
                config.lsk3dnet_normal_proj_h,
                config.lsk3dnet_normal_proj_w,
                config.fov_up,
                config.fov_down,
                official_profile ? 1 : 0,
                equals_sensor_fov ? 1 : 0);
        }

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT_DIAG] cluster(thr=%.3f max_dist=%.3f min_vertex=%d min_landmark_size=%d min_height=%.2f profile=%s input_mode=%s) fit(max_radius=%.2f max_tilt=%.1f truncation=%.2f) trellis(min_cluster_points=%d min_tree_vertices=%d) diag(detailed=%d class_hist=%d topk=%d interval=%d override_tree_class_id=%d)",
            config_.beam_cluster_threshold, config_.max_dist_to_centroid, config_.min_vertex_size,
            config_.min_landmark_size, config_.min_landmark_height, config_.diag_cluster_profile.c_str(),
            config_.diag_cluster_input_mode.c_str(),
            config_.max_tree_radius, config_.max_axis_theta, config_.tree_truncation_height,
            params.min_cluster_points, params.min_tree_vertices,
            config_.diag_enable_detailed_stats ? 1 : 0, config_.diag_log_class_histogram ? 1 : 0,
            config_.diag_class_hist_top_k, config_.diag_class_hist_interval_frames, config_.diag_override_tree_class_id);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT_DIAG] cylinder_fit method=%s rel_h_above_ground=[%.2f,%.2f] ground_xy_r=%.2f gnd_min_n=%d gnd_pct=%.2f "
            "cyl_cv_max=%.3f axial_min=%.3f pcl_sac_dist=%.3f pcl_norm_r=%.3f min_tree_conf=%.2f trunk_ego_xy_warn=%.2f reject=%.2f",
            config_.cylinder_fit_method.c_str(),
            config_.cylinder_fit_rel_z_min, config_.cylinder_fit_rel_z_max,
            config_.cylinder_fit_ground_search_radius_m,
            config_.cylinder_fit_ground_min_samples,
            config_.cylinder_fit_ground_percentile,
            config_.cylinder_radial_cv_max, config_.cylinder_min_axial_extent_m,
            config_.cylinder_pcl_sac_distance, config_.cylinder_pcl_normal_radius,
            config_.min_tree_confidence,
            config_.trunk_ego_xy_clearance_warn_m,
            config_.trunk_ego_xy_clearance_reject_m);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT_DIAG] sparse_trunk_column enable=%d r=%.2f extent_min=%.2f upper_z=[%.2f,%.2f] "
            "min_up=%d min_col=%d max_fit_pts=%d structural=%d s_ext_min=%.2f s_col_min=%d s_fol_min=%d r_fol=%.2f "
            "taper_ratio=%.2f taper_max_z=%.2f structural_direct_cyl=%d s_ambient=%d amb_in_m+%.2f amb_R=%.2f amb_max_pts=%d "
            "fallback_enable=%d fallback_min_conf=%.2f "
            "fallback_pca_after_merge=%d r_med_slack=%.2f pca_min_up_cos=%.2f pca_extra_tilt_deg=%.1f diag_trunk_chain_log=%d",
            config_.sparse_trunk_column_enable ? 1 : 0,
            config_.sparse_trunk_column_radius_m,
            config_.sparse_trunk_min_vertical_extent_m,
            config_.sparse_trunk_upper_rel_z_min_m,
            config_.sparse_trunk_upper_rel_z_max_m,
            config_.sparse_trunk_min_upper_points,
            config_.sparse_trunk_min_column_points,
            config_.sparse_trunk_max_fit_points,
            config_.sparse_trunk_structural_enable ? 1 : 0,
            config_.sparse_trunk_structural_min_extent_m,
            config_.sparse_trunk_structural_min_column_points,
            config_.sparse_trunk_structural_min_foliage_points,
            config_.sparse_trunk_structural_foliage_radius_m,
            config_.sparse_trunk_structural_taper_min_ratio,
            config_.sparse_trunk_structural_taper_max_rel_z_m,
            config_.sparse_trunk_structural_direct_cylinder ? 1 : 0,
            config_.sparse_trunk_structural_ambient_check_enable ? 1 : 0,
            config_.sparse_trunk_structural_ambient_inner_margin_m,
            config_.sparse_trunk_structural_ambient_outer_radius_m,
            config_.sparse_trunk_structural_ambient_max_points,
            config_.sparse_trunk_fallback_enable ? 1 : 0,
            config_.sparse_trunk_fallback_min_confidence,
            config_.sparse_trunk_fallback_pca_after_merge ? 1 : 0,
            config_.sparse_trunk_fallback_r_med_slack,
            config_.sparse_trunk_fallback_pca_min_up_cos,
            config_.sparse_trunk_fallback_pca_extra_tilt_deg,
            config_.diag_trunk_chain_log ? 1 : 0);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT_DIAG] geometric_tree_recall euclidean_cluster(tol=%.3f min_pts=%d max_pts=%d) "
            "max_lines_per_cluster=%d skip_trunk_rv_wall_line=%d geometric_only_frame_merge(xy=%.3f use_explicit_xy=%d z=%.3f use_explicit_z=%d "
            "axis_deg=%.1f use_explicit_axis=%d) note='xy/z/axis<=0 uses legacy auto'",
            config_.euclidean_cluster.tolerance_m, config_.euclidean_cluster.min_points, config_.euclidean_cluster.max_points,
            config_.max_lines_per_cluster,
            config_.trunk_chain_skip_rv_wall_label ? 1 : 0,
            config_.geometric_only_frame_merge.max_xy_m, config_.geometric_only_frame_merge.max_xy_m > 0.f ? 1 : 0,
            config_.geometric_only_frame_merge.max_z_m, config_.geometric_only_frame_merge.max_z_m > 0.f ? 1 : 0,
            config_.geometric_only_frame_merge.max_axis_angle_deg,
            config_.geometric_only_frame_merge.max_axis_angle_deg > 0.f ? 1 : 0);
    } catch (const std::exception& e) {
        const std::string full_err = describeExceptionFull(e);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), 
            "[SEMANTIC][Processor][INIT] step=FAILED model_type=%s error=%s → abort startup",
            config.model_type.c_str(), e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT] FULL_EXCEPTION: %s",
            full_err.c_str());
        throw std::runtime_error(std::string("SemanticProcessor init failed: ") + full_err);
    } catch (...) {
        const std::string full_err = describeCurrentExceptionFull();
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT] UNKNOWN_EXCEPTION: %s",
            full_err.c_str());
        throw std::runtime_error(std::string("SemanticProcessor init failed: ") + full_err);
    }
}

SemanticProcessor::~SemanticProcessor() = default;

bool SemanticProcessor::trunkPassesEgoClearance_(const Eigen::Vector3d& root_body,
                                                 uint64_t landmark_id,
                                                 const char* stage_tag) const {
    const double warn_m = static_cast<double>(config_.trunk_ego_xy_clearance_warn_m);
    const double reject_m = static_cast<double>(config_.trunk_ego_xy_clearance_reject_m);
    if (warn_m <= 0.0 && reject_m <= 0.0) {
        return true;
    }
    const double dxy = root_body.head<2>().norm();
    if (reject_m > 0.0 && dxy < reject_m) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_EGO_CLEARANCE] stage=%s action=reject id=%lu d_xy=%.3f reject_m=%.3f "
            "(trunk too close to vehicle in body XY; unlikely roadside tree)",
            stage_tag, static_cast<unsigned long>(landmark_id), dxy, reject_m);
        return false;
    }
    if (warn_m > 0.0 && dxy < warn_m) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_EGO_CLEARANCE] stage=%s action=warn id=%lu d_xy=%.3f warn_m=%.3f "
            "(investigate false fit or anchor; map-frame trajectory distance is separate)",
            stage_tag, static_cast<unsigned long>(landmark_id), dxy, warn_m);
    }
    return true;
}


CylinderLandmark::Ptr SemanticProcessor::geometricOnlyTryTrunkChain_(
    const GeometricResult::Primitive& prim,
    const std::vector<pcl::PointXYZI>& raw_line_points,
    float band_rel_h_lo,
    float band_rel_h_hi,
    const GeometricResult& geo_result,
    const CloudXYZIConstPtr& cloud,
    double ts,
    int trunk_line_idx,
    size_t& rejected_line_cylinder_fit,
    size_t& rejected_line_tilt,
    size_t& rejected_line_low_confidence,
    size_t& rejected_line_trunk_near_ego,
    size_t& sparse_trunk_column_gate_pass,
    size_t& sparse_trunk_structural_gate_pass,
    size_t& sparse_trunk_structural_direct_accepted,
    size_t& sparse_trunk_fit_recovered,
    size_t& sparse_trunk_fallback_accepted) {
    bool from_sparse_fallback = false;
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_for_line =
        (geo_result.ground_cloud && !geo_result.ground_cloud->empty()) ? geo_result.ground_cloud : nullptr;

    const bool diag_line = config_.diag_trunk_chain_log || geoDebugEnabled(config_);
    if (diag_line) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=line_prim residual=%.3f linearity=%.3f "
            "coeff_n=%zu raw_line_pts=%zu ground_cloud=%d",
            ts,
            trunk_line_idx,
            prim.residual,
            prim.linearity,
            prim.model_coeffs.size(),
            raw_line_points.size(),
            ground_for_line ? 1 : 0);
    }

    std::vector<pcl::PointXYZI> line_fit_pts = trunkBandPointsBodyFrame(
        raw_line_points,
        ground_for_line,
        vertical_ref_body_,
        band_rel_h_lo,
        band_rel_h_hi,
        config_.cylinder_fit_ground_search_radius_m,
        config_.cylinder_fit_ground_min_samples,
        config_.cylinder_fit_ground_percentile);

    if (diag_line) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=height_band "
            "rel_z=[%.2f,%.2f] gnd_r=%.2f gnd_min_n=%d band_pts=%zu raw_line_pts=%zu",
            ts,
            trunk_line_idx,
            band_rel_h_lo,
            band_rel_h_hi,
            config_.cylinder_fit_ground_search_radius_m,
            config_.cylinder_fit_ground_min_samples,
            line_fit_pts.size(),
            raw_line_points.size());
    }

    std::vector<pcl::PointXYZI> pts_for_fit = line_fit_pts;
    FitRejectReason fit_reason = FitRejectReason::kRejectHeightWindow;
    double fit_tilt_deg = -1.0;
    auto try_fit_cylinder = [&](const std::vector<pcl::PointXYZI>& pts, const char* pass_tag) -> CylinderLandmark::Ptr {
        if (pts.size() < 10) {
            fit_reason = FitRejectReason::kRejectHeightWindow;
            fit_tilt_deg = -1.0;
            if (diag_line) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=fit_try pass=%s skip=pts_lt_10 pts=%zu",
                    ts, trunk_line_idx, pass_tag, pts.size());
            }
            return nullptr;
        }
        CylinderFitDiagFrame cyl_diag;
        cyl_diag.ts = ts;
        cyl_diag.context_idx = trunk_line_idx;
        cyl_diag.pass_tag = pass_tag;
        const CylinderFitDiagFrame* cyl_diag_ptr =
            (config_.diag_trunk_chain_log || geoDebugEnabled(config_)) ? &cyl_diag : nullptr;
        FitRejectReason fr = FitRejectReason::kSuccess;
        double tilt = -1.0;
        CylinderLandmark::Ptr lm = fitCylinder(pts, &fr, &tilt, cyl_diag_ptr);
        if (diag_line) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=fit_try pass=%s pts=%zu ok=%d "
                "reject=%s tilt=%.1f cyl_method=%s max_r=%.2f min_conf=%.2f",
                ts,
                trunk_line_idx,
                pass_tag,
                pts.size(),
                lm ? 1 : 0,
                trunkFitRejectCStr(static_cast<int>(fr)),
                tilt,
                config_.cylinder_fit_method.c_str(),
                config_.max_tree_radius,
                config_.min_tree_confidence);
        }
        fit_reason = fr;
        fit_tilt_deg = tilt;
        return lm;
    };

    CylinderLandmark::Ptr line = try_fit_cylinder(pts_for_fit, "narrow_band");

    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr ng =
        (geo_result.nonground_cloud && !geo_result.nonground_cloud->empty()) ? geo_result.nonground_cloud : nullptr;

    if (!line && diag_line && config_.sparse_trunk_column_enable && !ng) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=sparse_column_skip reason=no_nonground_cloud",
            ts,
            trunk_line_idx);
    }

    if (!line && config_.sparse_trunk_column_enable && ng) {
        const SparseTrunkColumnEval col = evalSparseTrunkColumnForLine(
            prim,
            ng,
            ground_for_line,
            vertical_ref_body_,
            config_,
            diag_line,
            ts,
            trunk_line_idx);
        if (col.gate_ok) {
            ++sparse_trunk_column_gate_pass;
            if (col.used_structural_evidence) {
                ++sparse_trunk_structural_gate_pass;
            }
        }
        if (diag_line) {
            RCLCPP_INFO(
                rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=sparse_column "
                "gate_ok=%d structural=%d n_col=%zu n_up=%zu extent_m=%.2f h[min,max]=[%.2f,%.2f] "
                "r_col=%.2f need(ext>=%.2f,up>=%d,col>=%d) reject=%s",
                ts,
                trunk_line_idx,
                col.gate_ok ? 1 : 0,
                col.used_structural_evidence ? 1 : 0,
                col.n_column,
                col.n_upper,
                col.vertical_extent_m,
                col.h_min,
                col.h_max,
                config_.sparse_trunk_column_radius_m,
                config_.sparse_trunk_min_vertical_extent_m,
                config_.sparse_trunk_min_upper_points,
                config_.sparse_trunk_min_column_points,
                col.reject_reason.c_str());
        }
        if (col.gate_ok && !col.samples.empty()) {
            pts_for_fit = mergeTrunkBandAndColumnDedup(
                line_fit_pts,
                col.samples,
                0.06f,
                std::max(200, config_.sparse_trunk_max_fit_points));
            if (diag_line) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=merge_band_column "
                    "band_pts=%zu col_pts=%zu merged_pts=%zu voxel_m=0.06 max_pts=%d",
                    ts,
                    trunk_line_idx,
                    line_fit_pts.size(),
                    col.samples.size(),
                    pts_for_fit.size(),
                    std::max(200, config_.sparse_trunk_max_fit_points));
            }
            const bool structural_direct_only =
                col.used_structural_evidence && config_.sparse_trunk_structural_direct_cylinder;
            if (structural_direct_only) {
                line = buildStructuralTrunkCylinderFromSamples(
                    prim,
                    pts_for_fit,
                    vertical_ref_body_,
                    config_,
                    diag_line,
                    ts,
                    trunk_line_idx);
                if (line) {
                    from_sparse_fallback = true;
                    ++sparse_trunk_structural_direct_accepted;
                    if (diag_line) {
                        RCLCPP_INFO(
                            rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=structural_skip_fit "
                            "direct_cylinder r=%.3f conf=%.3f pts=%zu",
                            ts,
                            trunk_line_idx,
                            line->radius,
                            line->confidence,
                            pts_for_fit.size());
                    }
                }
            } else {
                line = try_fit_cylinder(pts_for_fit, "band_plus_column");
                if (line) {
                    ++sparse_trunk_fit_recovered;
                }
            }
            if (!line && !structural_direct_only && config_.sparse_trunk_fallback_enable) {
                std::vector<pcl::PointXYZI> pts_fb = pts_for_fit;
                if (pts_fb.size() < 10) {
                    pts_fb = col.samples;
                }
                if (pts_fb.size() >= 10) {
                    std::string fb_fail;
                    line = buildSparseTrunkFallbackLandmark(
                        prim,
                        pts_fb,
                        vertical_ref_body_,
                        config_,
                        diag_line,
                        ts,
                        trunk_line_idx,
                        diag_line ? &fb_fail : nullptr,
                        nullptr,
                        0.0);
                    if (!line && config_.sparse_trunk_fallback_pca_after_merge) {
                        std::vector<pcl::PointXYZI> pts_pca =
                            (pts_for_fit.size() >= 10) ? pts_for_fit : col.samples;
                        if (pts_pca.size() >= 10) {
                            Eigen::Vector3d pca_axis;
                            if (computeSparseTrunkDominantAxisFromPoints(
                                    pts_pca,
                                    vertical_ref_body_,
                                    static_cast<double>(config_.sparse_trunk_fallback_pca_min_up_cos),
                                    &pca_axis)) {
                                std::string fb2;
                                line = buildSparseTrunkFallbackLandmark(
                                    prim,
                                    pts_pca,
                                    vertical_ref_body_,
                                    config_,
                                    diag_line,
                                    ts,
                                    trunk_line_idx,
                                    diag_line ? &fb2 : nullptr,
                                    &pca_axis,
                                    static_cast<double>(config_.sparse_trunk_fallback_r_med_slack));
                                if (line) {
                                    pts_fb = std::move(pts_pca);
                                } else if (diag_line) {
                                    RCLCPP_INFO(
                                        rclcpp::get_logger("automap_system"),
                                        "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=fallback_pca_fail "
                                        "pts=%zu detail=%s",
                                        ts,
                                        trunk_line_idx,
                                        pts_pca.size(),
                                        fb2.empty() ? "unset" : fb2.c_str());
                                }
                            }
                        }
                    }
                    if (line) {
                        from_sparse_fallback = true;
                        ++sparse_trunk_fallback_accepted;
                        pts_for_fit = std::move(pts_fb);
                        if (diag_line) {
                            RCLCPP_INFO(
                                rclcpp::get_logger("automap_system"),
                                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=fallback_trunk_accept "
                                "pts=%zu r=%.3f conf=%.3f",
                                ts,
                                trunk_line_idx,
                                pts_for_fit.size(),
                                line->radius,
                                line->confidence);
                        }
                    } else if (diag_line) {
                        RCLCPP_INFO(
                            rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=fallback_trunk_fail "
                            "pts=%zu detail=%s",
                            ts,
                            trunk_line_idx,
                            pts_fb.size(),
                            fb_fail.empty() ? "unset" : fb_fail.c_str());
                    }
                } else if (diag_line) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=fallback_trunk_skip "
                        "reason=pts_fb_lt_10 merged=%zu col_only=%zu",
                        ts,
                        trunk_line_idx,
                        pts_for_fit.size(),
                        col.samples.size());
                }
            }
        }
    }

    if (!line) {
        ++rejected_line_cylinder_fit;
        if (fit_reason == FitRejectReason::kRejectTilt) {
            ++rejected_line_tilt;
        }
        if (diag_line) {
            RCLCPP_INFO(
                rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=reject_line_final reason=%s tilt=%.1f "
                "band_pts=%zu fit_pts=%zu sparse_fallback_attempted=%d",
                ts,
                trunk_line_idx,
                trunkFitRejectCStr(static_cast<int>(fit_reason)),
                fit_tilt_deg,
                line_fit_pts.size(),
                pts_for_fit.size(),
                (config_.sparse_trunk_column_enable && ng) ? 1 : 0);
        } else if (geoDebugEnabled(config_)) {
            RCLCPP_DEBUG(
                rclcpp::get_logger("automap_system"),
                "[GEOMETRIC][SEM_PROCESSOR][DETAIL] step=reject_line_final reason=%s tilt=%.1f band_pts=%zu fit_pts=%zu",
                trunkFitRejectCStr(static_cast<int>(fit_reason)),
                fit_tilt_deg,
                line_fit_pts.size(),
                pts_for_fit.size());
        }
        return nullptr;
    }

    const double tilt_deg =
        std::acos(std::clamp(std::abs(line->ray.dot(vertical_ref_body_)), 0.0, 1.0)) * 180.0 / M_PI;
    if (tilt_deg > config_.max_axis_theta) {
        ++rejected_line_tilt;
        if (diag_line) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=reject_axis_tilt tilt=%.1f max_axis_theta=%.1f",
                ts, trunk_line_idx, tilt_deg, config_.max_axis_theta);
        } else if (geoDebugEnabled(config_)) {
            RCLCPP_DEBUG(
                rclcpp::get_logger("automap_system"),
                "[GEOMETRIC][SEM_PROCESSOR][DETAIL] step=reject_line_tilt tilt=%.1f max=%.1f",
                tilt_deg, config_.max_axis_theta);
        }
        return nullptr;
    }
    line->confidence = std::max(line->confidence, prim.residual);
    const double conf_min_accept =
        from_sparse_fallback ? static_cast<double>(config_.sparse_trunk_fallback_min_confidence)
                             : static_cast<double>(config_.min_tree_confidence);
    if (line->confidence < conf_min_accept) {
        ++rejected_line_low_confidence;
        if (diag_line) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=reject_low_confidence conf=%.3f "
                "min_accept=%.3f sparse_fallback=%d",
                ts,
                trunk_line_idx,
                line->confidence,
                conf_min_accept,
                from_sparse_fallback ? 1 : 0);
        }
        return nullptr;
    }
    if (!trunkPassesEgoClearance_(line->root, line->id, "geometric_only_line")) {
        ++rejected_line_trunk_near_ego;
        if (diag_line) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=reject_trunk_ego_clearance root=(%.2f,%.2f,%.2f)",
                ts, trunk_line_idx, line->root.x(), line->root.y(), line->root.z());
        }
        return nullptr;
    }
    line->primitive_linearity = static_cast<double>(prim.linearity);
    line->detection_point_count = static_cast<int>(pts_for_fit.size());
    line->points = CloudXYZIPtr(new CloudXYZI());
    if (prim.points) {
        line->points->header = prim.points->header;
    } else {
        line->points->header = cloud->header;
    }
    line->points->points.assign(pts_for_fit.begin(), pts_for_fit.end());
    if (diag_line) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=accept_tree r=%.3f conf=%.3f det_pts=%d "
            "sparse_fallback=%d",
            ts,
            trunk_line_idx,
            line->radius,
            line->confidence,
            line->detection_point_count,
            from_sparse_fallback ? 1 : 0);
    }
    return line;
}


SemanticProcessor::ProcessResult SemanticProcessor::process(const CloudXYZIConstPtr& cloud,
                                                             double ts,
                                                             const Eigen::Isometry3d& T_odom_b,
                                                             CloudXYZIPtr* out_labeled_cloud,
                                                             CloudXYZIPtr* out_trunk_pre_cluster_body,
                                                             CloudXYZIPtr* out_trunk_post_cluster_body) {
    ProcessResult result;
    const uint64_t frame_seq_hint = processed_frames_.load(std::memory_order_relaxed) + 1;
    if (runtime_disabled_.load(std::memory_order_relaxed)) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=runtime_disabled_after_repeated_exceptions frame=%lu",
            static_cast<unsigned long>(frame_seq_hint));
        return result;
    }
    const bool run_model = (config_.mode != "geometric_only");
    if (run_model && (!segmentor_ || !segmentor_->isReady())) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=segmentor_unavailable frame=%lu (feature disabled)",
            static_cast<unsigned long>(frame_seq_hint));
        return result;
    }
    if (!cloud || cloud->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=cloud_null_or_empty frame=%lu",
            static_cast<unsigned long>(frame_seq_hint));
        return result;
    }

    std::lock_guard<std::recursive_mutex> lock(resource_mutex_);

        try {
        vertical_ref_body_ = Eigen::Vector3d::UnitZ();
        const auto t0 = std::chrono::steady_clock::now();
        const auto processed_idx = processed_frames_.load(std::memory_order_relaxed);
        const auto should_log_checks = (processed_idx <= 5 || (processed_idx % 50 == 0));
        if (processed_idx == 0 && config_.model_type == "lsk3dnet_hybrid") {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][FIRST_FRAME_SNAPSHOT] source={SemanticProcessor::Config} "
                "effective={normal_mode=%s normal_profile=[up=%.1f,down=%.1f,h=%d,w=%d] "
                "sensor_fov=[up=%.1f,down=%.1f] img=[%dx%d] tree_class=%d model_type=%s}",
                config_.lsk3dnet_hybrid_normal_mode.c_str(),
                config_.lsk3dnet_normal_fov_up_deg,
                config_.lsk3dnet_normal_fov_down_deg,
                config_.lsk3dnet_normal_proj_h,
                config_.lsk3dnet_normal_proj_w,
                config_.fov_up,
                config_.fov_down,
                config_.img_w,
                config_.img_h,
                config_.tree_class_id,
                config_.model_type.c_str());
        }
        auto log_consistency_check = [&](const char* step, bool ok, const std::string& detail) {
            if (ok) {
                if (should_log_checks) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][ConsistencyCheck] step=%s status=PASS detail=%s",
                        step, detail.c_str());
                }
                return;
            }
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][ConsistencyCheck] step=%s status=FAIL detail=%s",
                step, detail.c_str());
        };
        
        if (processed_idx <= 5 || (processed_idx % 100 == 0)) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=input_start frame=%lu cloud_size=%zu cloud_wh=%ux%u expected_mask_wh=%dx%d",
                static_cast<unsigned long>(processed_idx),
                cloud->size(), cloud->width, cloud->height, config_.img_w, config_.img_h);
        }

        if (config_.diag_enable_detailed_stats || processed_idx <= 2) {
            double x_min = std::numeric_limits<double>::infinity();
            double x_max = -std::numeric_limits<double>::infinity();
            double y_min = std::numeric_limits<double>::infinity();
            double y_max = -std::numeric_limits<double>::infinity();
            double z_min = std::numeric_limits<double>::infinity();
            double z_max = -std::numeric_limits<double>::infinity();
            for (const auto& p : cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                x_min = std::min(x_min, static_cast<double>(p.x));
                x_max = std::max(x_max, static_cast<double>(p.x));
                y_min = std::min(y_min, static_cast<double>(p.y));
                y_max = std::max(y_max, static_cast<double>(p.y));
                z_min = std::min(z_min, static_cast<double>(p.z));
                z_max = std::max(z_max, static_cast<double>(p.z));
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INPUT_DIAG] raw_pts=%zu raw_wh=%ux%u xyz_range=[x:(%.2f,%.2f) y:(%.2f,%.2f) z:(%.2f,%.2f)]",
                cloud->size(), cloud->width, cloud->height, x_min, x_max, y_min, y_max, z_min, z_max);
        }

        // Patchwork++/聚类/RANSAC 与分割共用同一套有效点过滤：禁止把 NaN/Inf/原点 送入几何，
        // 否则易出现 ground=0、立面基元为空（与 flipped_cloud 语义不一致）。
        const bool run_geometric =
            geometric_processor_ && (config_.mode == "hybrid" || config_.mode == "geometric_only");
        CloudXYZIPtr geo_body_cloud;
        if (run_geometric) {
            geo_body_cloud.reset(new CloudXYZI());
            geo_body_cloud->header = cloud->header;
            geo_body_cloud->points.reserve(cloud->size());
        }

        // 🏛️ [坐标系对齐] sloam_rec 内部使用 yaw = -atan2(y, x)
        // 标准 ROS 使用 atan2(y, x)。为了对齐，我们将 y 轴取反后再送入投影。
        CloudXYZIPtr flipped_cloud(new CloudXYZI());
        flipped_cloud->header = cloud->header;
        flipped_cloud->points.reserve(cloud->size());
        std::vector<size_t> flipped_source_idx;
        flipped_source_idx.reserve(cloud->size());
        size_t dropped_invalid_points = 0;
        for (size_t orig_i = 0; orig_i < cloud->points.size(); ++orig_i) {
            const auto& p = cloud->points[orig_i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity)) {
                ++dropped_invalid_points;
                continue;
            }
            const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
            if (!(r2 > 1e-12f)) {
                ++dropped_invalid_points;
                continue;
            }
            if (run_geometric) {
                geo_body_cloud->push_back(p);
            }
            auto p_f = p;
            p_f.y = -p.y;
            flipped_source_idx.push_back(orig_i);
            flipped_cloud->push_back(p_f);
        }
        flipped_cloud->width = static_cast<uint32_t>(flipped_cloud->points.size());
        flipped_cloud->height = 1;
        if (run_geometric && geo_body_cloud) {
            geo_body_cloud->width = static_cast<uint32_t>(geo_body_cloud->points.size());
            geo_body_cloud->height = 1;
        }
        // grep GEO_PIPELINE：语义入口与几何共用「有效点」云一致；对齐 mapping 的 span 与 Patchwork 输入
        if (run_geometric && geo_body_cloud && !geo_body_cloud->empty() && geoInfoEnabled(config_)) {
            const uint64_t pf = static_cast<uint64_t>(processed_idx) + 1u;
            if (pf <= 25u || (pf % 200u) == 0u) {
                double gx_min = std::numeric_limits<double>::infinity();
                double gx_max = -std::numeric_limits<double>::infinity();
                double gy_min = std::numeric_limits<double>::infinity();
                double gy_max = -std::numeric_limits<double>::infinity();
                double gz_min = std::numeric_limits<double>::infinity();
                double gz_max = -std::numeric_limits<double>::infinity();
                for (const auto& p : geo_body_cloud->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                        continue;
                    }
                    gx_min = std::min(gx_min, static_cast<double>(p.x));
                    gx_max = std::max(gx_max, static_cast<double>(p.x));
                    gy_min = std::min(gy_min, static_cast<double>(p.y));
                    gy_max = std::max(gy_max, static_cast<double>(p.y));
                    gz_min = std::min(gz_min, static_cast<double>(p.z));
                    gz_max = std::max(gz_max, static_cast<double>(p.z));
                }
                const double g_xy = std::max(gx_max - gx_min, gy_max - gy_min);
                const Eigen::Vector3d t_odom = T_odom_b.translation();
                const std::string frame_str = cloud->header.frame_id.empty() ? "(empty)" : std::string(cloud->header.frame_id);
                RCLCPP_INFO(
                    rclcpp::get_logger("automap_system"),
                    "[GEO_PIPELINE][SEM_IN] frame=%lu ts=%.3f raw_pts=%zu geo_pts=%zu dropped_invalid=%zu "
                    "span_xy=%.2f z[%.2f,%.2f] T_odom_t=[%.2f,%.2f,%.2f] cloud_header_frame=%s",
                    static_cast<unsigned long>(pf),
                    ts,
                    cloud->size(),
                    geo_body_cloud->size(),
                    dropped_invalid_points,
                    g_xy,
                    gz_min,
                    gz_max,
                    t_odom.x(),
                    t_odom.y(),
                    t_odom.z(),
                    frame_str.c_str());
            }
        }
        if (dropped_invalid_points > 0) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=input_filter dropped_invalid=%zu kept=%zu",
                dropped_invalid_points, flipped_cloud->size());
        }
        if (config_.diag_enable_detailed_stats) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INPUT_FILTER_DIAG] kept=%zu dropped=%zu keep_ratio=%.2f%%",
                flipped_cloud->size(),
                dropped_invalid_points,
                100.0 * static_cast<double>(flipped_cloud->size()) /
                    static_cast<double>(std::max<size_t>(1, cloud->size())));
        }
        if (flipped_cloud->empty()) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=skip reason=all_points_invalid_after_filter frame=%lu raw_size=%zu",
                static_cast<unsigned long>(processed_idx + 1),
                cloud->size());
            return result;
        }
        const auto t1 = std::chrono::steady_clock::now();

        cv::Mat mask = cv::Mat::zeros(config_.img_h, config_.img_w, CV_8U);
        SemanticSegResult seg_result;
        GeometricResult geo_result;
        size_t geo_ground_pts = 0;
        size_t geo_nonground_pts = 0;
        size_t geo_line_cnt = 0;
        size_t geo_plane_cnt = 0;

        if (run_model) {
            // 1. Semantic Segmentation
            segmentor_->run(flipped_cloud, mask, &seg_result);
        } else {
            seg_result.success = true;
            seg_result.backend_name = "geometric_only";
            seg_result.message = "model inference skipped";
        }

        // 🏛️ [GeoSemantic-Fusion] Hybrid / geometric_only 模式下进行几何语义修正
        if (run_geometric) {
            geo_result = geometric_processor_->process(ts, geo_body_cloud, T_odom_b);
            vertical_ref_body_ = geo_result.gravity_up_body;
            if (vertical_ref_body_.norm() < 1e-9) {
                vertical_ref_body_ = Eigen::Vector3d::UnitZ();
            } else {
                vertical_ref_body_.normalize();
            }
            geo_ground_pts = geo_result.ground_cloud ? geo_result.ground_cloud->size() : 0;
            geo_nonground_pts = geo_result.nonground_cloud ? geo_result.nonground_cloud->size() : 0;
            geo_line_cnt = 0;
            geo_plane_cnt = 0;
            for (const auto& prim : geo_result.primitives) {
                if (prim.type == GeometricResult::Primitive::Type::LINE) {
                    ++geo_line_cnt;
                } else if (prim.type == GeometricResult::Primitive::Type::PLANE) {
                    ++geo_plane_cnt;
                }
            }
            if (geoInfoEnabled(config_)) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[GEOMETRIC][SEM_PROCESSOR] step=geo_result_ready ts=%.3f mode=%s cloud_in=%zu ground=%zu nonground=%zu "
                    "primitives(lines/planes/total)=(%zu/%zu/%zu) (lines=trees/ poles, planes=vertical walls after gates)",
                    ts, config_.mode.c_str(), geo_body_cloud ? geo_body_cloud->size() : 0u, geo_ground_pts, geo_nonground_pts,
                    geo_line_cnt, geo_plane_cnt, geo_result.primitives.size());
            }
            if (geoDebugEnabled(config_)) {
                const size_t gsz = geo_body_cloud ? geo_body_cloud->size() : 0u;
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[GEOMETRIC][SEM_PROCESSOR][DETAIL] ts=%.3f ground_ratio=%.3f nonground_ratio=%.3f",
                    ts,
                    gsz > 0 ? static_cast<double>(geo_ground_pts) / static_cast<double>(gsz) : 0.0,
                    gsz > 0 ? static_cast<double>(geo_nonground_pts) / static_cast<double>(gsz) : 0.0);
            }
            
            // 修正地面标签: Patchwork 地面点投回 range mask，写入 learning-label id（默认 9=SemanticKITTI-sub road）。
            // 勿与旧注释 NuScenes 混用: 在本仓库 LSK semantic-kitti-sub 中 13 对应 building，不是地面。
            if (config_.mode == "hybrid" || config_.mode == "geometric_only") {
                const double fov_up_rad = static_cast<double>(config_.fov_up) * M_PI / 180.0;
                const double fov_down_rad = static_cast<double>(config_.fov_down) * M_PI / 180.0;
                const double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
                const uint8_t ground_paint_u8 = static_cast<uint8_t>(
                    std::clamp(config_.geometric_ground_paint_class_id, 0, 255));

                size_t ground_paint_samples = 0;
                size_t ground_proj_oob = 0;
                double pitch_min_rad = std::numeric_limits<double>::infinity();
                double pitch_max_rad = -std::numeric_limits<double>::infinity();
                if (geo_result.ground_cloud) {
                    for (const auto& p : geo_result.ground_cloud->points) {
                        const double r = std::sqrt(static_cast<double>(p.x) * p.x +
                                                 static_cast<double>(p.y) * p.y +
                                                 static_cast<double>(p.z) * p.z);
                        if (r < 1e-6) {
                            continue;
                        }
                        // 对齐 flipped_cloud 的坐标变换 (y -> -y)
                        const double yaw = -std::atan2(static_cast<double>(-p.y), static_cast<double>(p.x));
                        const double sin_pitch = std::clamp(static_cast<double>(p.z) / r, -1.0, 1.0);
                        const double pitch = std::asin(sin_pitch);
                        pitch_min_rad = std::min(pitch_min_rad, pitch);
                        pitch_max_rad = std::max(pitch_max_rad, pitch);

                        const int px_raw = static_cast<int>(std::floor(
                            0.5 * (yaw / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                        const int py_raw = static_cast<int>(std::floor(
                            (1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov_rad)) *
                            static_cast<double>(config_.img_h)));
                        if (px_raw < 0 || px_raw >= config_.img_w || py_raw < 0 || py_raw >= config_.img_h) {
                            ++ground_proj_oob;
                        }
                        ++ground_paint_samples;
                        const int px = std::clamp(px_raw, 0, config_.img_w - 1);
                        const int py = std::clamp(py_raw, 0, config_.img_h - 1);
                        mask.at<uint8_t>(py, px) = ground_paint_u8;
                    }
                }

                size_t mask_ground_px = 0;
                for (int r = 0; r < mask.rows; ++r) {
                    const auto* row = mask.ptr<uint8_t>(r);
                    for (int c = 0; c < mask.cols; ++c) {
                        if (row[c] == ground_paint_u8) {
                            ++mask_ground_px;
                        }
                    }
                }
                const uint64_t frame_hint = processed_frames_.load(std::memory_order_relaxed) + 1;
                const double pitch_min_deg =
                    (ground_paint_samples > 0 && std::isfinite(pitch_min_rad)) ? pitch_min_rad * 180.0 / M_PI : 0.0;
                const double pitch_max_deg =
                    (ground_paint_samples > 0 && std::isfinite(pitch_max_rad)) ? pitch_max_rad * 180.0 / M_PI : 0.0;
                if (geoInfoEnabled(config_) && (frame_hint <= 8 || (frame_hint % 50 == 0))) {
                    RCLCPP_INFO(
                        rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][GROUND_CHAIN] frame=%lu ts=%.3f mode=%s geo_ground_pts=%zu mask_ground_px=%zu ground_class=%d mask_wh=%dx%d "
                        "ground_paint_samples=%zu proj_oob_before_clamp=%zu pitch_deg[min,max]=[%.2f,%.2f] fov_up/down_deg=(%.1f,%.1f) "
                        "(geo_ground>0 & mask_ground==0 & oob>0 => widen semantic.fov_* or fix img_w/h; geo_ground==0 => patchwork/sensor_height)",
                        static_cast<unsigned long>(frame_hint),
                        ts,
                        config_.mode.c_str(),
                        geo_ground_pts,
                        mask_ground_px,
                        static_cast<int>(ground_paint_u8),
                        mask.cols,
                        mask.rows,
                        ground_paint_samples,
                        ground_proj_oob,
                        pitch_min_deg,
                        pitch_max_deg,
                        config_.fov_up,
                        config_.fov_down);
                }
                if (geo_ground_pts > 0 && mask_ground_px == 0 && ground_proj_oob == ground_paint_samples &&
                    ground_paint_samples > 0) {
                    RCLCPP_WARN_THROTTLE(
                        rclcpp::get_logger("automap_system"),
                        semanticDiagThrottleClock(),
                        5000,
                        "[SEMANTIC][FOV_PROJ_DIAG] frame=%lu ts=%.3f ALL ground samples projected OOB before clamp "
                        "(%zu pts) — ground class=%d will stack on border pixels only; align fov_up/fov_down/img with sensor",
                        static_cast<unsigned long>(frame_hint),
                        ts,
                        ground_paint_samples,
                        static_cast<int>(ground_paint_u8));
                }
            }
        }
        {
            const uint64_t pf = static_cast<uint64_t>(processed_idx) + 1u;
            if (pf <= 10u || (pf % 200u) == 0u) {
                if (!run_geometric) {
                    const char* reason =
                        geometric_processor_ ? "mode_not_hybrid_or_geometric_only" : "geometric_processor_null";
                    RCLCPP_INFO(
                        rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][GEO_CHAIN] frame=%lu ts=%.3f mode=%s run_geometric=0 reason=%s",
                        static_cast<unsigned long>(pf),
                        ts,
                        config_.mode.c_str(),
                        reason);
                } else {
                    RCLCPP_INFO(
                        rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][GEO_CHAIN] frame=%lu ts=%.3f mode=%s run_geometric=1 cloud_in=%zu ground=%zu "
                        "nonground=%zu lines=%zu planes=%zu | full chain: semantic.geometric.log_level=info "
                        "-> [GEOMETRIC][CHAIN]/[PRIMITIVE]/[EVIDENCE]",
                        static_cast<unsigned long>(pf),
                        ts,
                        config_.mode.c_str(),
                        geo_body_cloud ? geo_body_cloud->size() : 0u,
                        geo_ground_pts,
                        geo_nonground_pts,
                        geo_line_cnt,
                        geo_plane_cnt);
                }
            }
        }
        if (mask.rows != config_.img_h || mask.cols != config_.img_w) {
            std::ostringstream os;
            os << "mask geometry mismatch mask=" << mask.cols << "x" << mask.rows
               << " cfg=" << config_.img_w << "x" << config_.img_h;
            RCLCPP_FATAL(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][ConsistencyCheck] step=mask_geometry status=FAIL detail=%s",
                os.str().c_str());
            throw std::runtime_error(os.str());
        }
        log_consistency_check("mask_geometry", true, "mask shape equals configured image shape");
        const auto t2 = std::chrono::steady_clock::now();
        const auto processed = processed_frames_.load(std::memory_order_relaxed) + 1;

        // High-signal segmentation log: first few frames + periodic + always on failure
        if (!seg_result.success || processed <= 5 || (processed % 50 == 0)) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][SEG] frame=%lu backend=%s success=%d infer_ms=%.2f msg=%s mask_wh=%dx%d cloud_pts=%zu",
                static_cast<unsigned long>(processed),
                seg_result.backend_name.empty() ? "(unknown)" : seg_result.backend_name.c_str(),
                seg_result.success ? 1 : 0,
                seg_result.inference_ms,
                seg_result.message.empty() ? "(none)" : seg_result.message.c_str(),
                mask.cols, mask.rows,
                flipped_cloud->size());
        }
        if (run_model && !seg_result.success) {
            std::ostringstream os;
            os << "segmentation_inference_failed"
               << " frame=" << static_cast<unsigned long>(processed)
               << " backend=" << (seg_result.backend_name.empty() ? "(unknown)" : seg_result.backend_name)
               << " infer_ms=" << std::fixed << std::setprecision(2) << seg_result.inference_ms
               << " msg=" << (seg_result.message.empty() ? "(none)" : seg_result.message)
               << " cloud_pts=" << flipped_cloud->size()
               << " mask_wh=" << mask.cols << "x" << mask.rows;
            RCLCPP_FATAL(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INFER_CHAIN_ERROR] %s",
                os.str().c_str());
            throw std::runtime_error(os.str());
        }

        // Build labeled cloud immediately after segmentation succeeds.
        // This guarantees RViz semantic cloud can still be published even when
        // downstream tree extraction returns early (e.g. no tree points/clusters).
        if (run_model && out_labeled_cloud) {
            *out_labeled_cloud = CloudXYZIPtr(new CloudXYZI());
            (*out_labeled_cloud)->header = cloud->header;
            (*out_labeled_cloud)->points.reserve(cloud->size());

            const bool use_pointwise =
                !seg_result.per_point_labels.empty() &&
                seg_result.per_point_labels.size() == flipped_cloud->size() &&
                flipped_source_idx.size() == flipped_cloud->size();

            if (use_pointwise) {
                std::vector<uint8_t> body_label(static_cast<size_t>(cloud->size()), 0);
                for (size_t fi = 0; fi < flipped_cloud->size(); ++fi) {
                    const size_t oi = flipped_source_idx[fi];
                    if (oi < body_label.size()) {
                        body_label[oi] = seg_result.per_point_labels[fi];
                    }
                }
                for (size_t oi = 0; oi < cloud->points.size(); ++oi) {
                    auto p_labeled = cloud->points[oi];
                    if (!std::isfinite(p_labeled.x) || !std::isfinite(p_labeled.y) || !std::isfinite(p_labeled.z)) {
                        p_labeled.intensity = 0.0f;
                        (*out_labeled_cloud)->push_back(p_labeled);
                        continue;
                    }
                    p_labeled.intensity = static_cast<float>(body_label[oi]);
                    (*out_labeled_cloud)->push_back(p_labeled);
                }
                log_consistency_check(
                    "labeled_cloud_source",
                    true,
                    "per_point_labels (dense hybrid); avoids range-mask bin collapse");
            } else {
                const double fov_up_rad = static_cast<double>(config_.fov_up) * M_PI / 180.0;
                const double fov_down_rad = static_cast<double>(config_.fov_down) * M_PI / 180.0;
                const double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);

                size_t projection_equivalence_checked = 0;
                size_t projection_equivalence_fail = 0;
                for (const auto& p : cloud->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                    const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                                   static_cast<double>(p.y) * p.y +
                                                   static_cast<double>(p.z) * p.z);
                    if (!(range > 1e-6)) {
                        auto p_labeled = p;
                        p_labeled.intensity = 0.0f;
                        (*out_labeled_cloud)->push_back(p_labeled);
                        continue;
                    }

                    // Use the same projection logic as the segmentor.
                    const double yaw = -std::atan2(static_cast<double>(-p.y), static_cast<double>(p.x));
                    const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
                    const double pitch = std::asin(sin_pitch);

                    int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                    int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov_rad)) *
                                                             static_cast<double>(config_.img_h)));
                    proj_x = std::clamp(proj_x, 0, config_.img_w - 1);
                    proj_y = std::clamp(proj_y, 0, config_.img_h - 1);

                    auto p_labeled = p;
                    p_labeled.intensity = static_cast<float>(mask.at<uint8_t>(proj_y, proj_x));
                    (*out_labeled_cloud)->push_back(p_labeled);

                    if (projection_equivalence_checked < 128) {
                        const double yaw_via_flipped = -std::atan2(static_cast<double>(-p.y), static_cast<double>(p.x));
                        int proj_x2 = static_cast<int>(std::floor(0.5 * (yaw_via_flipped / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                        proj_x2 = std::clamp(proj_x2, 0, config_.img_w - 1);
                        if (proj_x2 != proj_x) {
                            ++projection_equivalence_fail;
                        }
                        ++projection_equivalence_checked;
                    }
                }
                if (projection_equivalence_checked > 0) {
                    std::ostringstream os;
                    os << "checked=" << projection_equivalence_checked
                       << " fail=" << projection_equivalence_fail;
                    log_consistency_check("projection_formula_equivalence", projection_equivalence_fail == 0, os.str());
                }
                log_consistency_check("labeled_cloud_source", true, "mask_backprojection (legacy)");
            }
        }

        const bool log_class_hist = config_.diag_log_class_histogram;
        const bool dump_all_classes = config_.diag_dump_all_classes;
        if ((log_class_hist || dump_all_classes) &&
            (processed <= 10 || (processed % static_cast<uint64_t>(config_.diag_class_hist_interval_frames) == 0))) {
            std::array<uint64_t, 256> hist{};
            for (int r = 0; r < mask.rows; ++r) {
                const auto* row = mask.ptr<uint8_t>(r);
                for (int c = 0; c < mask.cols; ++c) {
                    ++hist[row[c]];
                }
            }
            if (log_class_hist) {
                const int topk = std::max(1, std::min(config_.diag_class_hist_top_k, 32));
                std::ostringstream os;
                std::array<bool, 256> used{};
                for (int i = 0; i < topk; ++i) {
                    int best_idx = -1;
                    uint64_t best_val = 0;
                    for (int k = 0; k < 256; ++k) {
                        if (used[k]) continue;
                        if (hist[k] > best_val) {
                            best_val = hist[k];
                            best_idx = k;
                        }
                    }
                    if (best_idx < 0 || best_val == 0) break;
                    used[best_idx] = true;
                    if (i > 0) os << ", ";
                    os << best_idx << ":" << best_val;
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][MASK_HIST] frame=%lu topk=%s non_zero_classes=%d class255=%lu override_tree_class_id=%d cfg_tree_class_id=%d",
                    static_cast<unsigned long>(processed), os.str().c_str(),
                    static_cast<int>(std::count_if(hist.begin(), hist.end(), [](uint64_t v) { return v > 0; })),
                    static_cast<unsigned long>(hist[255]),
                    config_.diag_override_tree_class_id, config_.tree_class_id);
            }
            if (config_.diag_enable_detailed_stats) {
                const uint64_t total_px = static_cast<uint64_t>(mask.rows) * static_cast<uint64_t>(mask.cols);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][MASK_DIAG] frame=%lu mask_wh=%dx%d class255_ratio=%.3f%%",
                    static_cast<unsigned long>(processed), mask.cols, mask.rows,
                    100.0 * static_cast<double>(hist[255]) / static_cast<double>(std::max<uint64_t>(1, total_px)));
            }

            if (dump_all_classes) {
                struct ClassPointStats {
                    uint64_t points = 0;
                    double z_min = std::numeric_limits<double>::infinity();
                    double z_max = -std::numeric_limits<double>::infinity();
                    double z_sum = 0.0;
                    std::vector<double> ranges;
                    std::vector<pcl::PointXYZI> samples;
                };
                std::unordered_map<int, ClassPointStats> class_point_stats;
                class_point_stats.reserve(64);

                const double fov_up_rad = static_cast<double>(config_.fov_up) * M_PI / 180.0;
                const double fov_down_rad = static_cast<double>(config_.fov_down) * M_PI / 180.0;
                const double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
                const int sample_limit = config_.diag_dump_points_per_class_limit;
                const bool dump_all_points = (sample_limit < 0);

                for (const auto& p : flipped_cloud->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                    const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                                   static_cast<double>(p.y) * p.y +
                                                   static_cast<double>(p.z) * p.z);
                    if (!(range > 1e-6)) continue;

                    const double yaw = -std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
                    const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
                    const double pitch = std::asin(sin_pitch);

                    int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                    int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov_rad)) *
                                                             static_cast<double>(config_.img_h)));
                    proj_x = std::clamp(proj_x, 0, config_.img_w - 1);
                    proj_y = std::clamp(proj_y, 0, config_.img_h - 1);

                    const int cls = static_cast<int>(mask.at<uint8_t>(proj_y, proj_x));
                    auto& stats = class_point_stats[cls];
                    ++stats.points;
                    stats.z_min = std::min(stats.z_min, static_cast<double>(p.z));
                    stats.z_max = std::max(stats.z_max, static_cast<double>(p.z));
                    stats.z_sum += static_cast<double>(p.z);
                    stats.ranges.push_back(range);
                    if (dump_all_points ||
                        (sample_limit > 0 && static_cast<int>(stats.samples.size()) < sample_limit)) {
                        stats.samples.push_back(p);
                    }
                }

                std::ostringstream classes_os;
                bool first = true;
                for (int cls = 0; cls < 256; ++cls) {
                    if (hist[cls] == 0) continue;
                    if (!first) classes_os << ", ";
                    first = false;
                    const auto it = class_point_stats.find(cls);
                    const uint64_t pts = (it == class_point_stats.end()) ? 0 : it->second.points;
                    classes_os << cls << "(px=" << hist[cls] << ",pts=" << pts << ")";
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CLASS_DUMP] frame=%lu classes=%s",
                    static_cast<unsigned long>(processed),
                    classes_os.str().c_str());

                const uint64_t total_classified_points = [&class_point_stats]() {
                    uint64_t sum = 0;
                    for (const auto& kv : class_point_stats) sum += kv.second.points;
                    return sum;
                }();
                auto get_quantile = [](std::vector<double> values, double q) -> double {
                    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
                    q = std::clamp(q, 0.0, 1.0);
                    const size_t idx = static_cast<size_t>(std::floor(q * static_cast<double>(values.size() - 1)));
                    std::nth_element(values.begin(), values.begin() + static_cast<long>(idx), values.end());
                    return values[idx];
                };
                for (const auto& kv : class_point_stats) {
                    const int cls = kv.first;
                    const auto& s = kv.second;
                    if (s.points == 0) continue;
                    const double z_mean = s.z_sum / static_cast<double>(s.points);
                    const double ratio = 100.0 * static_cast<double>(s.points) /
                                         static_cast<double>(std::max<uint64_t>(1, total_classified_points));
                    const double r_p25 = get_quantile(s.ranges, 0.25);
                    const double r_p50 = get_quantile(s.ranges, 0.50);
                    const double r_p75 = get_quantile(s.ranges, 0.75);
                    const double r_p95 = get_quantile(s.ranges, 0.95);
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][Processor][CLASS_SUMMARY] frame=%lu class=%d pts=%lu ratio=%.2f%% z[min/mean/max]=[%.2f/%.2f/%.2f] r[p25/p50/p75/p95]=[%.2f/%.2f/%.2f/%.2f]",
                        static_cast<unsigned long>(processed),
                        cls,
                        static_cast<unsigned long>(s.points),
                        ratio,
                        s.z_min, z_mean, s.z_max,
                        r_p25, r_p50, r_p75, r_p95);
                }

                if (sample_limit != 0) {
                    for (const auto& kv : class_point_stats) {
                        if (kv.second.samples.empty()) continue;
                        std::ostringstream sample_os;
                        for (size_t i = 0; i < kv.second.samples.size(); ++i) {
                            const auto& sp = kv.second.samples[i];
                            if (i > 0) sample_os << " ";
                            sample_os << "("
                                      << std::fixed << std::setprecision(2)
                                      << sp.x << "," << sp.y << "," << sp.z << ")";
                        }
                        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][Processor][CLASS_POINTS] frame=%lu class=%d samples=%s",
                            static_cast<unsigned long>(processed), kv.first, sample_os.str().c_str());
                    }
                }
            }
        }

        if (!run_model) {
            if (out_labeled_cloud) {
                *out_labeled_cloud = CloudXYZIPtr(new CloudXYZI());
                (*out_labeled_cloud)->header = cloud->header;
                (*out_labeled_cloud)->points = cloud->points;
                const double fov_up_rad = static_cast<double>(config_.fov_up) * M_PI / 180.0;
                const double fov_down_rad = static_cast<double>(config_.fov_down) * M_PI / 180.0;
                const double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
                for (auto& p : (*out_labeled_cloud)->points) {
                    const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                                   static_cast<double>(p.y) * p.y +
                                                   static_cast<double>(p.z) * p.z);
                    if (!(range > 1e-6)) {
                        p.intensity = 0.0f;
                        continue;
                    }
                    const double yaw = -std::atan2(static_cast<double>(-p.y), static_cast<double>(p.x));
                    const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
                    const double pitch = std::asin(sin_pitch);
                    int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                    int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov_rad)) *
                                                             static_cast<double>(config_.img_h)));
                    proj_x = std::clamp(proj_x, 0, config_.img_w - 1);
                    proj_y = std::clamp(proj_y, 0, config_.img_h - 1);
                    p.intensity = static_cast<float>(mask.at<uint8_t>(proj_y, proj_x));
                }
            }
            size_t rejected_line_tilt = 0;
            size_t rejected_line_cylinder_fit = 0;
            size_t rejected_line_low_confidence = 0;
            size_t rejected_line_trunk_near_ego = 0;
            size_t merged_duplicate_trees = 0;
            size_t sparse_trunk_column_gate_pass = 0;
            size_t sparse_trunk_structural_gate_pass = 0;
            size_t sparse_trunk_structural_direct_accepted = 0;
            size_t sparse_trunk_fit_recovered = 0;
            size_t sparse_trunk_fallback_accepted = 0;
            size_t skipped_trunk_rv_wall_line = 0;
            int trunk_line_idx = -1;
            for (const auto& prim : geo_result.primitives) {
                if (prim.type == GeometricResult::Primitive::Type::PLANE) {
                    auto plane = std::make_shared<PlaneLandmark>();
                    plane->normal = prim.model_coeffs.head<3>();
                    if (plane->normal.norm() > 1e-6) plane->normal.normalize();
                    plane->distance = prim.model_coeffs(3);
                    plane->confidence = fuseGeoPrimitiveConfidence(
                        prim.residual, prim.range_view_score, config_.range_view.fusion_rv_boost_scale);
                    plane->points = prim.points;
                    plane->detection_point_count =
                        (prim.points && !prim.points->empty()) ? static_cast<int>(prim.points->size()) : 0;

                    // Estimate geometric extents in body frame: vertical span (along up) and
                    // dominant in-plane tangent span. These are later used by backend gating
                    // to filter out short/low planes such as car bodies / guardrails.
                    if (prim.points && !prim.points->empty()) {
                        Eigen::Vector3d up = vertical_ref_body_;
                        if (up.norm() < 1e-9) {
                            up = Eigen::Vector3d::UnitZ();
                        } else {
                            up.normalize();
                        }
                        double min_z = std::numeric_limits<double>::infinity();
                        double max_z = -std::numeric_limits<double>::infinity();

                        // Tangent direction: roughly horizontal along the wall,
                        // orthogonal to both up and plane normal.
                        Eigen::Vector3d n = plane->normal;
                        if (n.norm() < 1e-6) {
                            n = Eigen::Vector3d::UnitX();
                        } else {
                            n.normalize();
                        }
                        Eigen::Vector3d t = up.cross(n);
                        if (t.norm() < 1e-6) {
                            // Fallback when up ‖ normal: pick any orthogonal direction.
                            t = n.unitOrthogonal();
                        }
                        t.normalize();
                        double min_t = std::numeric_limits<double>::infinity();
                        double max_t = -std::numeric_limits<double>::infinity();

                        for (const auto& pt : prim.points->points) {
                            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                                continue;
                            }
                            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
                            const double z = p.dot(up);
                            min_z = std::min(min_z, z);
                            max_z = std::max(max_z, z);
                            const double s = p.dot(t);
                            min_t = std::min(min_t, s);
                            max_t = std::max(max_t, s);
                        }
                        if (std::isfinite(min_z) && std::isfinite(max_z)) {
                            plane->vertical_span_m = std::max(0.0, max_z - min_z);
                        }
                        if (std::isfinite(min_t) && std::isfinite(max_t)) {
                            plane->tangent_span_m = std::max(0.0, max_t - min_t);
                        }
                    }

                    plane->id = allocateLandmarkId();
                    result.plane_landmarks.push_back(plane);
                } else if (prim.type == GeometricResult::Primitive::Type::LINE) {
                    // 立面启发（与 geometric_processor.h 注释一致）：不将此类 LINE 当作 tree 候选。
                    if (config_.trunk_chain_skip_rv_wall_label &&
                        (prim.range_view_label == 1u || prim.range_view_label == 3u)) {
                        ++skipped_trunk_rv_wall_line;
                        if (config_.diag_trunk_chain_log || geoDebugEnabled(config_)) {
                            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                                "[SEMANTIC][TRUNK_CHAIN] ts=%.3f line_idx=%d step=skip_wall_label label=%u",
                                ts, trunk_line_idx + 1, prim.range_view_label);
                        }
                        continue;
                    }
                    ++trunk_line_idx;
                    std::vector<pcl::PointXYZI> line_points;
                    if (prim.points) {
                        line_points.assign(prim.points->points.begin(), prim.points->points.end());
                    }
                    const float rel_h_lo = std::min(config_.cylinder_fit_rel_z_min, config_.cylinder_fit_rel_z_max);
                    const float rel_h_hi = std::max(config_.cylinder_fit_rel_z_min, config_.cylinder_fit_rel_z_max);
                    if (CylinderLandmark::Ptr lm = geometricOnlyTryTrunkChain_(
                            prim,
                            line_points,
                            rel_h_lo,
                            rel_h_hi,
                            geo_result,
                            cloud,
                            ts,
                            trunk_line_idx,
                            rejected_line_cylinder_fit,
                            rejected_line_tilt,
                            rejected_line_low_confidence,
                            rejected_line_trunk_near_ego,
                            sparse_trunk_column_gate_pass,
                            sparse_trunk_structural_gate_pass,
                            sparse_trunk_structural_direct_accepted,
                            sparse_trunk_fit_recovered,
                            sparse_trunk_fallback_accepted)) {
                        result.tree_landmarks.push_back(lm);
                    }
                }
            }

            size_t shaft_candidate_count = 0;
            size_t shaft_trunks_added = 0;
            if (config_.geometric_only_shaft_enable && geo_result.nonground_cloud &&
                !geo_result.nonground_cloud->empty()) {
                const auto shaft_cols = collectGeometricOnlyShaftRefinedColumns(
                    geo_result.nonground_cloud,
                    (geo_result.ground_cloud && !geo_result.ground_cloud->empty()) ? geo_result.ground_cloud : nullptr,
                    vertical_ref_body_,
                    config_,
                    geo_result.primitives);
                shaft_candidate_count = shaft_cols.size();
                Eigen::Vector3d up_s = vertical_ref_body_;
                if (up_s.norm() < 1e-9) {
                    up_s = Eigen::Vector3d::UnitZ();
                } else {
                    up_s.normalize();
                }
                const double skip_tree_xy = static_cast<double>(config_.geometric_only_shaft_skip_near_tree_xy_m);
                const float shaft_z_hi = std::max(8.f, config_.geometric_only_shaft_rel_z_max_m);

                for (auto& col_vec : shaft_cols) {
                    if (col_vec.size() < static_cast<size_t>(std::max(6, config_.geometric_only_shaft_min_points))) {
                        continue;
                    }
                    Eigen::Vector3d csum(0.0, 0.0, 0.0);
                    for (const auto& p : col_vec) {
                        csum += Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
                    }
                    csum /= static_cast<double>(col_vec.size());
                    const Eigen::Vector2d cxy = csum.head<2>();
                    if (skip_tree_xy > 0.0) {
                        bool near_tree = false;
                        for (const auto& tr : result.tree_landmarks) {
                            if (!tr) {
                                continue;
                            }
                            if ((tr->root.head<2>() - cxy).norm() < skip_tree_xy) {
                                near_tree = true;
                                break;
                            }
                        }
                        if (near_tree) {
                            continue;
                        }
                    }

                    pcl::PointCloud<pcl::PointXYZI>::Ptr col_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    // 注意：pcl::PointCloud 使用 Eigen 对齐分配器，不能直接用 std::move 赋值非对齐 std::vector
                    // 这里用 assign 保持对齐安全，同时复用几何链上的原始列点集 col_vec 作为后续 raw_line_points 输入
                    col_cloud->points.assign(col_vec.begin(), col_vec.end());
                    col_cloud->width = static_cast<uint32_t>(col_cloud->points.size());
                    col_cloud->height = 1;
                    col_cloud->header = geo_result.nonground_cloud->header;

                    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_s =
                        (geo_result.ground_cloud && !geo_result.ground_cloud->empty()) ? geo_result.ground_cloud
                                                                                        : nullptr;
                    const std::optional<double> s_opt = estimateLocalGroundSupportAlongUp(
                        ground_s,
                        csum.x(),
                        csum.y(),
                        up_s,
                        config_.cylinder_fit_ground_search_radius_m,
                        config_.cylinder_fit_ground_min_samples,
                        config_.cylinder_fit_ground_percentile);
                    const double s_g = s_opt.value_or(csum.dot(up_s) - 0.5);

                    GeometricResult::Primitive synth_prim =
                        makeShaftSyntheticLinePrimitive(col_cloud, up_s, s_g);

                    ++trunk_line_idx;
                    if (CylinderLandmark::Ptr lm = geometricOnlyTryTrunkChain_(
                            synth_prim,
                            col_vec,
                            0.f,
                            shaft_z_hi,
                            geo_result,
                            cloud,
                            ts,
                            trunk_line_idx,
                            rejected_line_cylinder_fit,
                            rejected_line_tilt,
                            rejected_line_low_confidence,
                            rejected_line_trunk_near_ego,
                            sparse_trunk_column_gate_pass,
                            sparse_trunk_structural_gate_pass,
                            sparse_trunk_structural_direct_accepted,
                            sparse_trunk_fit_recovered,
                            sparse_trunk_fallback_accepted)) {
                        result.tree_landmarks.push_back(lm);
                        ++shaft_trunks_added;
                    }
                }
            }

            // Deduplicate nearby tree landmarks in geometric_only mode:
            // one local area should not output many trees from fragmented line clusters.
            if (result.tree_landmarks.size() > 1) {
                std::vector<CylinderLandmark::Ptr> deduped;
                deduped.reserve(result.tree_landmarks.size());
                const double max_xy_dist =
                    (config_.geometric_only_frame_merge.max_xy_m > 0.f)
                        ? static_cast<double>(config_.geometric_only_frame_merge.max_xy_m)
                        : std::max(0.4, static_cast<double>(config_.max_tree_radius) * 2.0 + 0.2);
                const double max_z_dist =
                    (config_.geometric_only_frame_merge.max_z_m > 0.f)
                        ? static_cast<double>(config_.geometric_only_frame_merge.max_z_m)
                        : 2.0;
                const double max_axis_angle_deg =
                    (config_.geometric_only_frame_merge.max_axis_angle_deg > 0.f)
                        ? static_cast<double>(config_.geometric_only_frame_merge.max_axis_angle_deg)
                        : std::max(5.0, static_cast<double>(config_.max_axis_theta));
                const double cos_axis_th = std::cos(max_axis_angle_deg * M_PI / 180.0);

                for (const auto& cand : result.tree_landmarks) {
                    bool merged = false;
                    for (auto& kept : deduped) {
                        const Eigen::Vector2d dxy =
                            cand->root.head<2>() - kept->root.head<2>();
                        const double dz = std::abs(cand->root.z() - kept->root.z());
                        const double axis_cos =
                            std::abs(cand->ray.normalized().dot(kept->ray.normalized()));
                        if (dxy.norm() <= max_xy_dist && dz <= max_z_dist && axis_cos >= cos_axis_th) {
                            ++merged_duplicate_trees;
                            // Keep the higher-confidence fit to avoid drift from weak local fragments.
                            if (cand->confidence > kept->confidence) {
                                kept = cand;
                            }
                            merged = true;
                            break;
                        }
                    }
                    if (!merged) {
                        deduped.push_back(cand);
                    }
                }
                result.tree_landmarks.swap(deduped);
            }
            fit_reject_trunk_near_ego_total_.fetch_add(rejected_line_trunk_near_ego, std::memory_order_relaxed);
            processed_frames_.fetch_add(1, std::memory_order_relaxed);
            consecutive_failures_.store(0, std::memory_order_relaxed);
            const size_t geo_ground_n = geo_result.ground_cloud ? geo_result.ground_cloud->size() : 0;
            const uint8_t ground_paint_u8_done = static_cast<uint8_t>(
                std::clamp(config_.geometric_ground_paint_class_id, 0, 255));
            size_t mask_ground_paint_px = 0;
            for (int r = 0; r < mask.rows; ++r) {
                const auto* row = mask.ptr<uint8_t>(r);
                for (int c = 0; c < mask.cols; ++c) {
                    if (row[c] == ground_paint_u8_done) {
                        ++mask_ground_paint_px;
                    }
                }
            }
            if (geoInfoEnabled(config_)) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[GEOMETRIC][SEM_PROCESSOR] step=geometric_only_done ts=%.3f trees=%zu planes=%zu geo_ground_pts=%zu "
                    "mask_ground_px=%zu ground_class=%d "
                    "merged_duplicates=%zu skipped_trunk_rv_wall_line=%zu rejected_line_fit=%zu rejected_line_tilt=%zu rejected_line_low_conf=%zu rejected_line_trunk_near_ego=%zu "
                    "sparse_col_gate=%zu sparse_structural=%zu sparse_structural_direct=%zu "
                    "sparse_col_recovered=%zu sparse_col_fallback=%zu shaft_candidates=%zu shaft_accepted=%zu labeled_pts=%zu",
                    ts,
                    result.tree_landmarks.size(),
                    result.plane_landmarks.size(),
                    geo_ground_n,
                    mask_ground_paint_px,
                    static_cast<int>(ground_paint_u8_done),
                    merged_duplicate_trees,
                    skipped_trunk_rv_wall_line,
                    rejected_line_cylinder_fit,
                    rejected_line_tilt,
                    rejected_line_low_confidence,
                    rejected_line_trunk_near_ego,
                    sparse_trunk_column_gate_pass,
                    sparse_trunk_structural_gate_pass,
                    sparse_trunk_structural_direct_accepted,
                    sparse_trunk_fit_recovered,
                    sparse_trunk_fallback_accepted,
                    shaft_candidate_count,
                    shaft_trunks_added,
                    (out_labeled_cloud && *out_labeled_cloud) ? (*out_labeled_cloud)->size() : 0);
            }
            return result;
        }

        // 2. Mask Tree Points
        CloudXYZIPtr tree_cloud(new CloudXYZI());
        const int tree_label = resolveTreeLabel(config_);
        const bool dense_for_clustering = (config_.diag_cluster_input_mode == "dense_for_clustering");
        if (config_.model_type == "lsk3dnet_hybrid") {
            const bool official_normal_profile =
                std::abs(config_.lsk3dnet_normal_fov_up_deg - 3.0f) < 1e-3f &&
                std::abs(config_.lsk3dnet_normal_fov_down_deg + 25.0f) < 1e-3f &&
                config_.lsk3dnet_normal_proj_h == 64 &&
                config_.lsk3dnet_normal_proj_w == 900;
            const bool equals_sensor_fov =
                std::abs(config_.lsk3dnet_normal_fov_up_deg - config_.fov_up) < 1e-3f &&
                std::abs(config_.lsk3dnet_normal_fov_down_deg - config_.fov_down) < 1e-3f;
            std::ostringstream os;
            os << "normal_profile=[up=" << config_.lsk3dnet_normal_fov_up_deg
               << ",down=" << config_.lsk3dnet_normal_fov_down_deg
               << ",h=" << config_.lsk3dnet_normal_proj_h
               << ",w=" << config_.lsk3dnet_normal_proj_w << "]"
               << " official=[3,-25,64,900]"
               << " sensor_fov=[" << config_.fov_up << "," << config_.fov_down << "]"
               << " official_match=" << (official_normal_profile ? 1 : 0)
               << " sensor_fov_match=" << (equals_sensor_fov ? 1 : 0)
               << " source=semantic.lsk3dnet.normal_*"
               << " note=not_loop_closure.overlap_transformer";
            log_consistency_check("official_normal_profile", official_normal_profile, os.str());
        }

        if (should_log_checks) {
            std::array<uint64_t, 256> pre_mask_hist{};
            for (int r = 0; r < mask.rows; ++r) {
                const auto* row = mask.ptr<uint8_t>(r);
                for (int c = 0; c < mask.cols; ++c) {
                    ++pre_mask_hist[row[c]];
                }
            }
            std::ostringstream os;
            os << "tree_label=" << tree_label
               << " tree_px=" << pre_mask_hist[static_cast<uint8_t>(std::clamp(tree_label, 0, 255))]
               << " class0_px=" << pre_mask_hist[0];
            log_consistency_check("tree_label_presence", pre_mask_hist[static_cast<uint8_t>(std::clamp(tree_label, 0, 255))] > 0, os.str());
        }
        segmentor_->maskCloud(flipped_cloud, mask, tree_cloud, tree_label, dense_for_clustering);
        auto is_organized_cloud = [](const CloudXYZIPtr& c) -> bool {
            return c && c->height > 1 && c->width > 1 &&
                   c->size() == static_cast<size_t>(c->height) * static_cast<size_t>(c->width);
        };
        // Trellis in sloam_rec uses 2D indexing internally; sparse cloud can trigger
        // "Can't use 2D indexing with an unorganized point cloud". Auto-fallback
        // to dense layout only for clustering input to keep chain alive.
        bool forced_dense_fallback = false;
        if (!dense_for_clustering && !is_organized_cloud(tree_cloud)) {
            CloudXYZIPtr dense_tree_cloud(new CloudXYZI());
            segmentor_->maskCloud(flipped_cloud, mask, dense_tree_cloud, tree_label, true);
            if (is_organized_cloud(dense_tree_cloud)) {
                tree_cloud = dense_tree_cloud;
                forced_dense_fallback = true;
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CLUSTER_INPUT] sparse input is unorganized; fallback to dense_for_clustering for Trellis compatibility");
            }
        }
        const bool organized_after_mask = is_organized_cloud(tree_cloud);
        const size_t input_cloud_size = flipped_cloud ? flipped_cloud->size() : 0;
        CloudXYZIPtr valid_tree_cloud(new CloudXYZI());
        const bool tree_from_pointwise =
            !seg_result.per_point_labels.empty() &&
            seg_result.per_point_labels.size() == flipped_cloud->size() &&
            flipped_source_idx.size() == flipped_cloud->size();
        if (tree_from_pointwise) {
            const uint8_t tl_u8 = static_cast<uint8_t>(std::clamp(tree_label, 0, 255));
            valid_tree_cloud->points.reserve(flipped_cloud->size());
            for (size_t fi = 0; fi < flipped_cloud->size(); ++fi) {
                if (seg_result.per_point_labels[fi] != tl_u8) continue;
                const auto& p = flipped_cloud->points[fi];
                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    valid_tree_cloud->points.push_back(p);
                }
            }
        } else if (tree_cloud) {
            valid_tree_cloud->points.reserve(tree_cloud->size());
            for (const auto& p : tree_cloud->points) {
                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    valid_tree_cloud->points.push_back(p);
                }
            }
        }
        valid_tree_cloud->width = static_cast<uint32_t>(valid_tree_cloud->points.size());
        valid_tree_cloud->height = 1;
        const size_t valid_tree_points = valid_tree_cloud->size();
        const double tree_cloud_ratio = input_cloud_size > 0
            ? (100.0 * static_cast<double>(valid_tree_points) / static_cast<double>(input_cloud_size))
            : 0.0;
        const int tree_label_u8 = std::clamp(tree_label, 0, 255);
        const int mask_tree_pixels = cv::countNonZero(mask == static_cast<uint8_t>(tree_label_u8));
        const int mask_total_pixels = mask.rows * mask.cols;
        const double mask_tree_ratio = mask_total_pixels > 0
            ? (100.0 * static_cast<double>(mask_tree_pixels) / static_cast<double>(mask_total_pixels))
            : 0.0;
        {
            std::ostringstream os;
            os << "organized=" << (organized_after_mask ? 1 : 0)
               << " width=" << (tree_cloud ? tree_cloud->width : 0)
               << " height=" << (tree_cloud ? tree_cloud->height : 0)
               << " size=" << (tree_cloud ? tree_cloud->size() : 0)
               << " expected=" << static_cast<size_t>(config_.img_w) * static_cast<size_t>(config_.img_h);
            log_consistency_check("tree_cloud_organized_contract", organized_after_mask, os.str());
        }
        if (tree_cloud_ratio >= 80.0 || mask_tree_ratio >= 80.0) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][RISK] unusually_high_tree_coverage frame=%lu tree_cloud_ratio=%.1f%% mask_tree_ratio=%.1f%% "
                "input_cloud=%zu tree_cloud_valid=%zu tree_cloud_total=%zu mask_tree_px=%d/%d tree_label=%d mode=%s normal_profile=[up=%.1f,down=%.1f,h=%d,w=%d]",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed) + 1),
                tree_cloud_ratio,
                mask_tree_ratio,
                input_cloud_size,
                valid_tree_points,
                tree_cloud ? tree_cloud->size() : 0,
                mask_tree_pixels,
                mask_total_pixels,
                tree_label,
                config_.model_type.c_str(),
                config_.lsk3dnet_normal_fov_up_deg,
                config_.lsk3dnet_normal_fov_down_deg,
                config_.lsk3dnet_normal_proj_h,
                config_.lsk3dnet_normal_proj_w);
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][TREE_VALIDITY] frame=%lu tree_cloud_total=%zu tree_cloud_valid=%zu valid_ratio=%.2f%%",
            static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed) + 1),
            tree_cloud ? tree_cloud->size() : 0,
            valid_tree_points,
            (tree_cloud && !tree_cloud->empty())
                ? (100.0 * static_cast<double>(valid_tree_points) / static_cast<double>(tree_cloud->size()))
                : 0.0);
        const auto t3 = std::chrono::steady_clock::now();
        ++processed_frames_;
        // Inference explainability log: class -> point count and sampled points.
        // This helps diagnose "segmentation has classes but downstream has no landmarks".
        if (config_.diag_enable_detailed_stats &&
            (processed_frames_.load(std::memory_order_relaxed) <= 10 ||
             (processed_frames_.load(std::memory_order_relaxed) % static_cast<uint64_t>(config_.diag_class_hist_interval_frames) == 0))) {
            const double fov_up_rad = static_cast<double>(config_.fov_up) * M_PI / 180.0;
            const double fov_down_rad = static_cast<double>(config_.fov_down) * M_PI / 180.0;
            const double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
            const int configured_sample_limit = config_.diag_dump_points_per_class_limit;
            const int sample_limit = (configured_sample_limit < 0)
                ? 8  // guardrail: avoid huge per-frame logs when configured as dump-all
                : std::max(0, std::min(configured_sample_limit, 16));

            struct PerClassStats {
                uint64_t point_count{0};
                std::vector<pcl::PointXYZI> samples;
            };
            std::unordered_map<int, PerClassStats> class_stats;
            class_stats.reserve(32);

            for (const auto& p : flipped_cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                               static_cast<double>(p.y) * p.y +
                                               static_cast<double>(p.z) * p.z);
                if (!(range > 1e-6)) continue;

                const double yaw = -std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
                const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
                const double pitch = std::asin(sin_pitch);

                int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov_rad)) *
                                                         static_cast<double>(config_.img_h)));
                proj_x = std::clamp(proj_x, 0, config_.img_w - 1);
                proj_y = std::clamp(proj_y, 0, config_.img_h - 1);

                const int cls = static_cast<int>(mask.at<uint8_t>(proj_y, proj_x));
                auto& st = class_stats[cls];
                ++st.point_count;
                if (sample_limit > 0 && static_cast<int>(st.samples.size()) < sample_limit) {
                    st.samples.push_back(p);
                }
            }

            std::vector<std::pair<int, PerClassStats>> sorted_stats;
            sorted_stats.reserve(class_stats.size());
            for (const auto& kv : class_stats) sorted_stats.push_back(kv);
            std::sort(sorted_stats.begin(), sorted_stats.end(),
                      [](const auto& a, const auto& b) { return a.second.point_count > b.second.point_count; });

            std::ostringstream class_count_os;
            const size_t max_classes_to_log = std::min<size_t>(sorted_stats.size(), 12);
            for (size_t i = 0; i < max_classes_to_log; ++i) {
                if (i > 0) class_count_os << ", ";
                class_count_os << sorted_stats[i].first << ":" << sorted_stats[i].second.point_count;
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INFER_RESULT] frame=%lu classes=%zu top_class_points=[%s] tree_label=%d sample_limit=%d",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                sorted_stats.size(),
                class_count_os.str().c_str(),
                tree_label,
                sample_limit);

            for (size_t i = 0; i < max_classes_to_log; ++i) {
                const int cls = sorted_stats[i].first;
                const auto& st = sorted_stats[i].second;
                std::ostringstream pts_os;
                for (size_t j = 0; j < st.samples.size(); ++j) {
                    if (j > 0) pts_os << " ";
                    const auto& sp = st.samples[j];
                    pts_os << "(" << std::fixed << std::setprecision(2)
                           << sp.x << "," << sp.y << "," << sp.z << ",i=" << sp.intensity << ")";
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CLASS_POINTS_SAMPLED] frame=%lu class=%d point_count=%lu samples=%zu pts=%s",
                    static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                    cls,
                    static_cast<unsigned long>(st.point_count),
                    st.samples.size(),
                    st.samples.empty() ? "(none)" : pts_os.str().c_str());
            }
        }
        if (config_.diag_enable_detailed_stats || dense_for_clustering) {
            const bool organized = (tree_cloud->height > 1 && tree_cloud->width > 1 &&
                                    tree_cloud->size() == static_cast<size_t>(tree_cloud->height) * static_cast<size_t>(tree_cloud->width));
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][CLUSTER_INPUT] frame=%lu mode=%s tree_pts=%zu wh=%ux%u organized=%d",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                (dense_for_clustering || forced_dense_fallback) ? "dense_for_clustering" : "sparse",
                tree_cloud->size(),
                tree_cloud->width, tree_cloud->height,
                organized ? 1 : 0);
            if (dense_for_clustering && !organized) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CLUSTER_INPUT] dense_for_clustering requested but tree_cloud is still non-organized (likely DENSE_LAYOUT_MISMATCH in segmentator)");
            }
            double t_x_min = std::numeric_limits<double>::infinity();
            double t_x_max = -std::numeric_limits<double>::infinity();
            double t_y_min = std::numeric_limits<double>::infinity();
            double t_y_max = -std::numeric_limits<double>::infinity();
            double t_z_min = std::numeric_limits<double>::infinity();
            double t_z_max = -std::numeric_limits<double>::infinity();
            for (const auto& p : tree_cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                t_x_min = std::min(t_x_min, static_cast<double>(p.x));
                t_x_max = std::max(t_x_max, static_cast<double>(p.x));
                t_y_min = std::min(t_y_min, static_cast<double>(p.y));
                t_y_max = std::max(t_y_max, static_cast<double>(p.y));
                t_z_min = std::min(t_z_min, static_cast<double>(p.z));
                t_z_max = std::max(t_z_max, static_cast<double>(p.z));
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][TREE_CLOUD_DIAG] frame=%lu pts=%zu xyz_range=[x:(%.2f,%.2f) y:(%.2f,%.2f) z:(%.2f,%.2f)]",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                tree_cloud->size(),
                t_x_min, t_x_max, t_y_min, t_y_max, t_z_min, t_z_max);
        }

        if (valid_tree_cloud->empty()) {
            ++empty_tree_frames_;
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=mask_tree frame=%lu pts=%zu tree_pts_valid=0 -> no tree points",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                cloud->size());
            return result;
        }

        if (out_trunk_pre_cluster_body) {
            *out_trunk_pre_cluster_body = CloudXYZIPtr(new CloudXYZI());
            (*out_trunk_pre_cluster_body)->header = cloud->header;
            (*out_trunk_pre_cluster_body)->height = 1;
            (*out_trunk_pre_cluster_body)->is_dense = false;
            const int tl_u8 = std::clamp(resolveTreeLabel(config_), 0, 255);
            const float tl_f = static_cast<float>(tl_u8);
            (*out_trunk_pre_cluster_body)->points.reserve(valid_tree_cloud->size());
            for (const auto& p : valid_tree_cloud->points) {
                pcl::PointXYZI q;
                q.x = p.x;
                q.y = -p.y;
                q.z = p.z;
                q.intensity = tl_f;
                (*out_trunk_pre_cluster_body)->points.push_back(q);
            }
            (*out_trunk_pre_cluster_body)->width = static_cast<uint32_t>((*out_trunk_pre_cluster_body)->points.size());
        }

        // 3. Instance Segmentation (Clustering)
        // Trellis expects organized tree cloud for 2D indexing. Use organized layout
        // (with NaN placeholders) when available, while keeping valid_tree_cloud for metrics.
        CloudXYZIPtr cluster_input_cloud = is_organized_cloud(tree_cloud) ? tree_cloud : valid_tree_cloud;
        std::vector<std::vector<TreeVertex>> tree_clusters;
        instance_detector_->computeGraph(flipped_cloud, cluster_input_cloud, tree_clusters);
        const auto t4 = std::chrono::steady_clock::now();

        if (out_trunk_post_cluster_body) {
            *out_trunk_post_cluster_body = CloudXYZIPtr(new CloudXYZI());
            (*out_trunk_post_cluster_body)->header = cloud->header;
            (*out_trunk_post_cluster_body)->height = 1;
            (*out_trunk_post_cluster_body)->is_dense = false;
            for (size_t ci = 0; ci < tree_clusters.size(); ++ci) {
                const float id_f = static_cast<float>(ci + 1);
                for (const auto& v : tree_clusters[ci]) {
                    for (const auto& p : v.points) {
                        pcl::PointXYZI q;
                        q.x = p.x;
                        q.y = -p.y;
                        q.z = p.z;
                        q.intensity = id_f;
                        (*out_trunk_post_cluster_body)->points.push_back(q);
                    }
                }
            }
            (*out_trunk_post_cluster_body)->width =
                static_cast<uint32_t>((*out_trunk_post_cluster_body)->points.size());
        }

        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=instance tree_pts_valid=%zu cluster_input_wh=%ux%u clusters=%zu",
            valid_tree_cloud->size(),
            cluster_input_cloud ? cluster_input_cloud->width : 0,
            cluster_input_cloud ? cluster_input_cloud->height : 0,
            tree_clusters.size());
        if (config_.diag_enable_detailed_stats && !tree_clusters.empty()) {
            size_t min_cluster_pts = std::numeric_limits<size_t>::max();
            size_t max_cluster_pts = 0;
            size_t sum_cluster_pts = 0;
            std::ostringstream top_clusters;
            for (size_t i = 0; i < tree_clusters.size(); ++i) {
                size_t pts = 0;
                for (const auto& v : tree_clusters[i]) pts += v.points.size();
                min_cluster_pts = std::min(min_cluster_pts, pts);
                max_cluster_pts = std::max(max_cluster_pts, pts);
                sum_cluster_pts += pts;
                if (i < 10) {
                    if (i > 0) top_clusters << ", ";
                    top_clusters << "#" << i << "(v=" << tree_clusters[i].size() << ",pts=" << pts << ")";
                }
            }
            const double avg_cluster_pts = static_cast<double>(sum_cluster_pts) /
                                           static_cast<double>(std::max<size_t>(1, tree_clusters.size()));
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][CLUSTER_DIAG] frame=%lu clusters=%zu pts[min/avg/max]=[%zu/%.1f/%zu] top=%s",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                tree_clusters.size(),
                min_cluster_pts == std::numeric_limits<size_t>::max() ? 0 : min_cluster_pts,
                avg_cluster_pts,
                max_cluster_pts,
                top_clusters.str().c_str());
        }
        if (tree_clusters.empty()) {
            ++empty_cluster_frames_;
            double z_min = std::numeric_limits<double>::infinity();
            double z_max = -std::numeric_limits<double>::infinity();
            for (const auto& p : tree_cloud->points) {
                z_min = std::min(z_min, static_cast<double>(p.z));
                z_max = std::max(z_max, static_cast<double>(p.z));
            }
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][EMPTY_CLUSTER_DIAG] frame=%lu tree_pts=%zu tree_cloud_ratio=%.1f%% mask_tree_ratio=%.1f%% "
                "z_span=%.3f params(thr=%.3f max_dist=%.3f min_vertex=%d min_landmark_size=%d min_height=%.2f profile=%s tree_label=%d)",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                valid_tree_cloud->size(),
                tree_cloud_ratio,
                mask_tree_ratio,
                (std::isfinite(z_min) && std::isfinite(z_max)) ? (z_max - z_min) : -1.0,
                config_.beam_cluster_threshold,
                config_.max_dist_to_centroid,
                config_.min_vertex_size,
                config_.min_landmark_size,
                config_.min_landmark_height,
                config_.diag_cluster_profile.c_str(),
                tree_label);
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][STAGE_RESULT] frame=%lu stage=cluster result=empty_clusters action=continue_with_zero_fit",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)));
        }

        // 4. Cylinder Fitting for each cluster
        size_t fit_input_clusters = 0;
        size_t fit_small_cluster_skips = 0;
        size_t fit_success = 0;
        size_t fit_ransac_reject = 0;
        size_t fit_ceres_fail = 0;
        size_t fit_reject_radius = 0;
        size_t fit_reject_tilt = 0;
        size_t fit_reject_height_window = 0;
        size_t fit_reject_cylinder_radial_cv = 0;
        size_t fit_reject_cylinder_axial_span = 0;
        size_t fit_reject_low_confidence = 0;
        size_t fit_reject_trunk_near_ego = 0;
        size_t fit_reject_hybrid_trunk_near_ego = 0;
        double fit_reject_tilt_deg_sum = 0.0;
        double fit_reject_tilt_deg_max = 0.0;
        size_t fit_reject_tilt_with_value = 0;
        const float rel_h_window_min = std::min(config_.cylinder_fit_rel_z_min, config_.cylinder_fit_rel_z_max);
        const float rel_h_window_max = std::max(config_.cylinder_fit_rel_z_min, config_.cylinder_fit_rel_z_max);
        Eigen::Vector3d up_fit = vertical_ref_body_;
        if (up_fit.norm() < 1e-9) {
            up_fit = Eigen::Vector3d::UnitZ();
        } else {
            up_fit.normalize();
        }
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_for_trunk =
            (geo_result.ground_cloud && !geo_result.ground_cloud->empty()) ? geo_result.ground_cloud : nullptr;

        const bool cluster_cylinder_diag =
            config_.diag_trunk_chain_log || geoDebugEnabled(config_);

        for (size_t cluster_idx = 0; cluster_idx < tree_clusters.size(); ++cluster_idx) {
            const auto& cluster = tree_clusters[cluster_idx];
            std::vector<PointT> points;
            double sum_x = 0.0;
            double sum_y_flipped = 0.0;
            for (const auto& v : cluster) {
                for (const auto& p : v.points) {
                    points.push_back(p);
                    sum_x += static_cast<double>(p.x);
                    sum_y_flipped += static_cast<double>(p.y);
                }
            }
            ++fit_input_clusters;

            if (points.size() < 10) {
                ++fit_small_cluster_skips;
                if (cluster_cylinder_diag) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][TRUNK_CLUSTER_FIT] ts=%.3f cluster_idx=%zu step=skip_cluster reason=pts_lt_10 n=%zu",
                        ts,
                        cluster_idx,
                        points.size());
                }
                continue;
            }

            const double inv_n = 1.0 / static_cast<double>(points.size());
            const double cx_body = sum_x * inv_n;
            const double cy_body = -sum_y_flipped * inv_n;

            double support_min_along_up = std::numeric_limits<double>::infinity();
            for (const auto& p : points) {
                support_min_along_up = std::min(support_min_along_up, flippedTreePointToBody(p).dot(up_fit));
            }

            const std::optional<double> s_ground_opt = estimateLocalGroundSupportAlongUp(
                ground_for_trunk,
                cx_body,
                cy_body,
                up_fit,
                config_.cylinder_fit_ground_search_radius_m,
                config_.cylinder_fit_ground_min_samples,
                config_.cylinder_fit_ground_percentile);
            const double s_ground = s_ground_opt.value_or(support_min_along_up);

            // 🌳 [Truncation] If tree_truncation_height is set, keep only the bottom part (above local ground along up).
            std::vector<PointT> fit_points;
            if (config_.tree_truncation_height > 0.001f) {
                const double h_cutoff = static_cast<double>(config_.tree_truncation_height);
                fit_points.reserve(points.size());
                for (const auto& p : points) {
                    const double h = flippedTreePointToBody(p).dot(up_fit) - s_ground;
                    if (h <= h_cutoff) {
                        fit_points.push_back(p);
                    }
                }
                if (fit_points.size() < 10) {
                    fit_points = points;
                }
            } else {
                fit_points = points;
            }

            // 仅保留相对「局部地面」沿竖直方向 [rel_h_min, rel_h_max] 内的点（不用车体 z=0 或簇内最低点当地面，避免树冠-only 簇）。
            std::vector<PointT> fit_points_windowed;
            fit_points_windowed.reserve(fit_points.size());
            for (const auto& p : fit_points) {
                const double h_along_up = flippedTreePointToBody(p).dot(up_fit) - s_ground;
                if (h_along_up >= static_cast<double>(rel_h_window_min) && h_along_up <= static_cast<double>(rel_h_window_max)) {
                    fit_points_windowed.push_back(p);
                }
            }

            FitRejectReason reject_reason = FitRejectReason::kSuccess;
            double reject_tilt_deg = -1.0;
            CylinderLandmark::Ptr landmark = nullptr;
            CylinderFitDiagFrame cluster_cyl_diag;
            cluster_cyl_diag.ts = ts;
            cluster_cyl_diag.context_idx = static_cast<int>(cluster_idx);
            cluster_cyl_diag.pass_tag = "trellis_cluster";
            const CylinderFitDiagFrame* cluster_cyl_ptr = cluster_cylinder_diag ? &cluster_cyl_diag : nullptr;

            if (cluster_cylinder_diag) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][TRUNK_CLUSTER_FIT] ts=%.3f cluster_idx=%zu step=height_pipeline "
                    "pts_raw=%zu fit_after_trunc=%zu fit_windowed=%zu rel_h=[%.2f,%.2f] s_ground=%s",
                    ts,
                    cluster_idx,
                    points.size(),
                    fit_points.size(),
                    fit_points_windowed.size(),
                    rel_h_window_min,
                    rel_h_window_max,
                    s_ground_opt.has_value() ? "local_est" : "cluster_min");
            }

            if (fit_points_windowed.size() >= 10) {
                landmark = fitCylinder(fit_points_windowed, &reject_reason, &reject_tilt_deg, cluster_cyl_ptr);
            } else {
                reject_reason = FitRejectReason::kRejectHeightWindow;
                if (cluster_cylinder_diag) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][TRUNK_CLUSTER_FIT] ts=%.3f cluster_idx=%zu step=reject_height_window "
                        "windowed_pts=%zu need>=10 (fit_points=%zu)",
                        ts,
                        cluster_idx,
                        fit_points_windowed.size(),
                        fit_points.size());
                }
            }
            if (landmark) {
                landmark->detection_point_count = static_cast<int>(fit_points_windowed.size());
                // 🏛️ [坐标系对齐] 将拟合结果从翻转坐标系转回 body 系
                landmark->root.y() = -landmark->root.y();
                landmark->ray.y() = -landmark->ray.y();
                if (landmark->confidence < static_cast<double>(config_.min_tree_confidence)) {
                    ++fit_reject_low_confidence;
                    if (cluster_cylinder_diag) {
                        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][TRUNK_CLUSTER_FIT] ts=%.3f cluster_idx=%zu step=reject_low_confidence "
                            "conf=%.3f min=%.3f",
                            ts,
                            cluster_idx,
                            landmark->confidence,
                            config_.min_tree_confidence);
                    }
                } else if (!trunkPassesEgoClearance_(landmark->root, landmark->id, "model_cluster_cylinder")) {
                    ++fit_reject_trunk_near_ego;
                    if (cluster_cylinder_diag) {
                        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][TRUNK_CLUSTER_FIT] ts=%.3f cluster_idx=%zu step=reject_trunk_ego_clearance "
                            "root=(%.2f,%.2f,%.2f)",
                            ts,
                            cluster_idx,
                            landmark->root.x(),
                            landmark->root.y(),
                            landmark->root.z());
                    }
                } else {
                    if (cluster_cylinder_diag) {
                        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][TRUNK_CLUSTER_FIT] ts=%.3f cluster_idx=%zu step=accept_tree "
                            "r=%.3f conf=%.3f det_pts=%d",
                            ts,
                            cluster_idx,
                            landmark->radius,
                            landmark->confidence,
                            landmark->detection_point_count);
                    }
                    result.tree_landmarks.push_back(landmark);
                    ++fit_success;
                }
            } else {
                switch (reject_reason) {
                    case FitRejectReason::kRansacReject:
                        ++fit_ransac_reject;
                        break;
                    case FitRejectReason::kCeresFail:
                        ++fit_ceres_fail;
                        break;
                    case FitRejectReason::kRejectRadius:
                        ++fit_reject_radius;
                        break;
                    case FitRejectReason::kRejectTilt:
                        ++fit_reject_tilt;
                        if (reject_tilt_deg >= 0.0) {
                            fit_reject_tilt_deg_sum += reject_tilt_deg;
                            fit_reject_tilt_deg_max = std::max(fit_reject_tilt_deg_max, reject_tilt_deg);
                            ++fit_reject_tilt_with_value;
                        }
                        break;
                    case FitRejectReason::kRejectHeightWindow:
                        ++fit_reject_height_window;
                        break;
                    case FitRejectReason::kSuccess:
                        break;
                }
            }
        }
        const auto t5 = std::chrono::steady_clock::now();

        // 🏛️ [GeoSemantic-Fusion] Fusion for Wall and Trunk (Phase 3 Integration)
        if (!geo_result.primitives.empty()) {
            for (const auto& prim : geo_result.primitives) {
                if (prim.type == GeometricResult::Primitive::Type::PLANE) {
                    auto plane = std::make_shared<PlaneLandmark>();
                    plane->normal = prim.model_coeffs.head<3>();
                    if (plane->normal.norm() > 1e-6) plane->normal.normalize();
                    plane->distance = prim.model_coeffs(3);
                    plane->confidence = fuseGeoPrimitiveConfidence(
                        prim.residual, prim.range_view_score, config_.range_view.fusion_rv_boost_scale);
                    plane->points = prim.points;
                    plane->detection_point_count =
                        (prim.points && !prim.points->empty()) ? static_cast<int>(prim.points->size()) : 0;

                    if (prim.points && !prim.points->empty()) {
                        Eigen::Vector3d up = vertical_ref_body_;
                        if (up.norm() < 1e-9) {
                            up = Eigen::Vector3d::UnitZ();
                        } else {
                            up.normalize();
                        }
                        double min_z = std::numeric_limits<double>::infinity();
                        double max_z = -std::numeric_limits<double>::infinity();

                        Eigen::Vector3d n = plane->normal;
                        if (n.norm() < 1e-6) {
                            n = Eigen::Vector3d::UnitX();
                        } else {
                            n.normalize();
                        }
                        Eigen::Vector3d t = up.cross(n);
                        if (t.norm() < 1e-6) {
                            t = n.unitOrthogonal();
                        }
                        t.normalize();
                        double min_t = std::numeric_limits<double>::infinity();
                        double max_t = -std::numeric_limits<double>::infinity();

                        for (const auto& pt : prim.points->points) {
                            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                                continue;
                            }
                            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
                            const double z = p.dot(up);
                            min_z = std::min(min_z, z);
                            max_z = std::max(max_z, z);
                            const double s = p.dot(t);
                            min_t = std::min(min_t, s);
                            max_t = std::max(max_t, s);
                        }
                        if (std::isfinite(min_z) && std::isfinite(max_z)) {
                            plane->vertical_span_m = std::max(0.0, max_z - min_z);
                        }
                        if (std::isfinite(min_t) && std::isfinite(max_t)) {
                            plane->tangent_span_m = std::max(0.0, max_t - min_t);
                        }
                    }

                    plane->id = allocateLandmarkId();
                    result.plane_landmarks.push_back(plane);
                } else if (prim.type == GeometricResult::Primitive::Type::LINE) {
                    // Check if this line matches any model-based tree
                    auto line_root = prim.model_coeffs.head<3>();
                    auto line_ray = prim.model_coeffs.segment<3>(3);
                    
                    bool overlap = false;
                    for (const auto& tree : result.tree_landmarks) {
                        if ((tree->root - line_root).norm() < 1.0) {
                            overlap = true;
                            break;
                        }
                    }
                    if (!overlap) {
                        if (prim.residual < static_cast<double>(config_.min_tree_confidence)) {
                            continue;
                        }
                        auto line = std::make_shared<CylinderLandmark>();
                        line->root = line_root;
                        line->ray = line_ray;
                        canonicalizeCylinderAxisUp(&line->ray, vertical_ref_body_);
                        line->radius = 0.1; // Default
                        line->confidence = prim.residual;
                        if (prim.points && !prim.points->empty()) {
                            anchorCylinderRootOnAxisFootprint(&line->root, line->ray, prim.points->points);
                        }
                        line->points = prim.points;
                        line->primitive_linearity = static_cast<double>(prim.linearity);
                        line->detection_point_count =
                            (prim.points && !prim.points->empty()) ? static_cast<int>(prim.points->size()) : 0;
                        line->id = allocateLandmarkId();
                        if (!trunkPassesEgoClearance_(line->root, line->id, "hybrid_geo_line")) {
                            ++fit_reject_hybrid_trunk_near_ego;
                            continue;
                        }
                        result.tree_landmarks.push_back(line);
                    }
                }
            }
        }

        fit_reject_trunk_near_ego += fit_reject_hybrid_trunk_near_ego;

        if (config_.diag_enable_detailed_stats) {
            const double tilt_reject_mean_deg = fit_reject_tilt_with_value > 0
                ? (fit_reject_tilt_deg_sum / static_cast<double>(fit_reject_tilt_with_value))
                : -1.0;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][FIT_DIAG] frame=%lu fit_input_clusters=%zu small_cluster_skip=%zu fit_success=%zu fit_fail=%zu "
                "reject[ransac=%zu ceres=%zu radius=%zu tilt=%zu height_window=%zu cyl_cv=%zu cyl_axial=%zu low_conf=%zu trunk_near_ego=%zu] "
                "tilt_deg(mean=%.1f max=%.1f samples=%zu threshold=%.1f) height_window=[%.2f,%.2f]m cyl_thresh(cv_max=%.3f axial_min=%.3f)",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                fit_input_clusters,
                fit_small_cluster_skips,
                fit_success,
                fit_input_clusters >= fit_small_cluster_skips + fit_success
                    ? (fit_input_clusters - fit_small_cluster_skips - fit_success)
                    : 0,
                fit_ransac_reject,
                fit_ceres_fail,
                fit_reject_radius,
                fit_reject_tilt,
                fit_reject_height_window,
                fit_reject_cylinder_radial_cv,
                fit_reject_cylinder_axial_span,
                fit_reject_low_confidence,
                fit_reject_trunk_near_ego,
                tilt_reject_mean_deg,
                fit_reject_tilt_deg_max,
                fit_reject_tilt_with_value,
                static_cast<double>(config_.max_axis_theta),
                static_cast<double>(rel_h_window_min),
                static_cast<double>(rel_h_window_max),
                static_cast<double>(config_.cylinder_radial_cv_max),
                static_cast<double>(config_.cylinder_min_axial_extent_m));
        }
        fit_input_clusters_total_.fetch_add(fit_input_clusters, std::memory_order_relaxed);
        fit_success_total_.fetch_add(fit_success, std::memory_order_relaxed);
        fit_reject_ransac_total_.fetch_add(fit_ransac_reject, std::memory_order_relaxed);
        fit_reject_ceres_total_.fetch_add(fit_ceres_fail, std::memory_order_relaxed);
        fit_reject_radius_total_.fetch_add(fit_reject_radius, std::memory_order_relaxed);
        fit_reject_tilt_total_.fetch_add(fit_reject_tilt, std::memory_order_relaxed);
        fit_reject_trunk_near_ego_total_.fetch_add(fit_reject_trunk_near_ego, std::memory_order_relaxed);

        if (result.tree_landmarks.empty() && result.plane_landmarks.empty()) {
            ++empty_fit_frames_;
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=done clusters=%zu fitted=0 (all rejected)",
                tree_clusters.size());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=done clusters=%zu trees=%zu planes=%zu",
                tree_clusters.size(), result.tree_landmarks.size(), result.plane_landmarks.size());
        }
        const auto processed_total = processed_frames_.load();
        const auto now = std::chrono::steady_clock::now();
        const auto secs_since_stats = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log_tp_).count();
        if (secs_since_stats >= 10) {
            last_stats_log_tp_ = now;
            const auto empty_tree = empty_tree_frames_.load();
            const auto empty_cluster = empty_cluster_frames_.load();
            const auto empty_fit = empty_fit_frames_.load();
            const auto fit_input_total = fit_input_clusters_total_.load(std::memory_order_relaxed);
            const auto fit_success_total = fit_success_total_.load(std::memory_order_relaxed);
            const auto fit_ransac_total = fit_reject_ransac_total_.load(std::memory_order_relaxed);
            const auto fit_ceres_total = fit_reject_ceres_total_.load(std::memory_order_relaxed);
            const auto fit_radius_total = fit_reject_radius_total_.load(std::memory_order_relaxed);
            const auto fit_tilt_total = fit_reject_tilt_total_.load(std::memory_order_relaxed);
            const auto fit_trunk_near_ego_total = fit_reject_trunk_near_ego_total_.load(std::memory_order_relaxed);
            const uint64_t fit_fail_total = fit_input_total >= fit_success_total
                ? (fit_input_total - fit_success_total)
                : 0;
            const double empty_tree_ratio = processed_total > 0 ? (100.0 * static_cast<double>(empty_tree) / static_cast<double>(processed_total)) : 0.0;
            const double empty_cluster_ratio = processed_total > 0 ? (100.0 * static_cast<double>(empty_cluster) / static_cast<double>(processed_total)) : 0.0;
            const double empty_fit_ratio = processed_total > 0 ? (100.0 * static_cast<double>(empty_fit) / static_cast<double>(processed_total)) : 0.0;
            const double fit_success_ratio = fit_input_total > 0
                ? (100.0 * static_cast<double>(fit_success_total) / static_cast<double>(fit_input_total))
                : 0.0;
            const auto input_filter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const auto seg_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            const auto mask_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
            const auto cluster_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
            const auto fit_ms = std::chrono::duration<double, std::milli>(t5 - t4).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][STATS] processed=%lu empty_tree=%lu(%.1f%%) empty_cluster=%lu(%.1f%%) empty_fit=%lu(%.1f%%) "
                "latest(tree_pts=%zu clusters=%zu trees=%zu planes=%zu) "
                "fit_totals(input=%lu success=%lu fail=%lu success_ratio=%.1f%% reject[ransac=%lu ceres=%lu radius=%lu tilt=%lu trunk_near_ego=%lu]) "
                "stage_ms(filter=%.1f seg=%.1f mask=%.1f cluster=%.1f fit=%.1f)",
                static_cast<unsigned long>(processed_total),
                static_cast<unsigned long>(empty_tree), empty_tree_ratio,
                static_cast<unsigned long>(empty_cluster), empty_cluster_ratio,
                static_cast<unsigned long>(empty_fit), empty_fit_ratio,
                tree_cloud->size(), tree_clusters.size(), result.tree_landmarks.size(), result.plane_landmarks.size(),
                static_cast<unsigned long>(fit_input_total),
                static_cast<unsigned long>(fit_success_total),
                static_cast<unsigned long>(fit_fail_total),
                fit_success_ratio,
                static_cast<unsigned long>(fit_ransac_total),
                static_cast<unsigned long>(fit_ceres_total),
                static_cast<unsigned long>(fit_radius_total),
                static_cast<unsigned long>(fit_tilt_total),
                static_cast<unsigned long>(fit_trunk_near_ego_total),
                input_filter_ms, seg_ms, mask_ms, cluster_ms, fit_ms);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][FLOW_RESULT] frame=%lu result=%s tree_pts=%zu clusters=%zu trees=%zu planes=%zu "
                "timing_ms(filter=%.1f seg=%.1f mask=%.1f cluster=%.1f fit=%.1f)",
                static_cast<unsigned long>(processed_total),
                (result.tree_landmarks.empty() && result.plane_landmarks.empty()) ? "no_landmark" : "landmark_ready",
                tree_cloud->size(),
                tree_clusters.size(),
                result.tree_landmarks.size(),
                result.plane_landmarks.size(),
                input_filter_ms, seg_ms, mask_ms, cluster_ms, fit_ms);
            // Root-cause oriented anomaly summary for full semantic chain.
            if (processed_total >= 20 &&
                (empty_fit_ratio >= 90.0 || empty_cluster_ratio >= 70.0 || empty_tree_ratio >= 70.0)) {
                const char* dominant_reason = "fit_failed";
                double dominant_ratio = empty_fit_ratio;
                if (empty_cluster_ratio > dominant_ratio) {
                    dominant_reason = "cluster_empty";
                    dominant_ratio = empty_cluster_ratio;
                }
                if (empty_tree_ratio > dominant_ratio) {
                    dominant_reason = "tree_points_empty";
                    dominant_ratio = empty_tree_ratio;
                }
                const char* dominant_fit_cause = "none";
                uint64_t dominant_fit_count = 0;
                if (fit_ransac_total > dominant_fit_count) {
                    dominant_fit_cause = "ransac_reject";
                    dominant_fit_count = fit_ransac_total;
                }
                if (fit_ceres_total > dominant_fit_count) {
                    dominant_fit_cause = "ceres_fail";
                    dominant_fit_count = fit_ceres_total;
                }
                if (fit_radius_total > dominant_fit_count) {
                    dominant_fit_cause = "radius_reject";
                    dominant_fit_count = fit_radius_total;
                }
                if (fit_tilt_total > dominant_fit_count) {
                    dominant_fit_cause = "tilt_reject";
                    dominant_fit_count = fit_tilt_total;
                }

                const double total_ms = input_filter_ms + seg_ms + mask_ms + cluster_ms + fit_ms;
                const char* bottleneck_stage = "seg";
                double bottleneck_ms = seg_ms;
                if (cluster_ms > bottleneck_ms) {
                    bottleneck_stage = "cluster";
                    bottleneck_ms = cluster_ms;
                }
                if (fit_ms > bottleneck_ms) {
                    bottleneck_stage = "fit";
                    bottleneck_ms = fit_ms;
                }
                if (mask_ms > bottleneck_ms) {
                    bottleneck_stage = "mask";
                    bottleneck_ms = mask_ms;
                }
                if (input_filter_ms > bottleneck_ms) {
                    bottleneck_stage = "filter";
                    bottleneck_ms = input_filter_ms;
                }

                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CHAIN_ANOMALY] processed=%lu dominant_reason=%s ratio=%.1f%% "
                    "ratios(empty_tree=%.1f%% empty_cluster=%.1f%% empty_fit=%.1f%%) "
                    "                    latest(tree_pts=%zu clusters=%zu trees=%zu planes=%zu tree_label=%d) "
                    "fit_root_cause(dominant=%s count=%lu totals:input=%lu success=%lu ransac=%lu ceres=%lu radius=%lu tilt=%lu) "
                    "bottleneck=%s(%.1fms/%.1fms total) "
                    "reason_code=E_SEMANTIC_CHAIN_ANOMALY",
                    static_cast<unsigned long>(processed_total),
                    dominant_reason,
                    dominant_ratio,
                    empty_tree_ratio,
                    empty_cluster_ratio,
                    empty_fit_ratio,
                    tree_cloud->size(),
                    tree_clusters.size(),
                    result.tree_landmarks.size(),
                    result.plane_landmarks.size(),
                    tree_label,
                    dominant_fit_cause,
                    static_cast<unsigned long>(dominant_fit_count),
                    static_cast<unsigned long>(fit_input_total),
                    static_cast<unsigned long>(fit_success_total),
                    static_cast<unsigned long>(fit_ransac_total),
                    static_cast<unsigned long>(fit_ceres_total),
                    static_cast<unsigned long>(fit_radius_total),
                    static_cast<unsigned long>(fit_tilt_total),
                    bottleneck_stage,
                    bottleneck_ms,
                    total_ms);
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][TRIGGER][E_SEMANTIC_CHAIN_ANOMALY] model_type=%s normal_mode=%s "
                    "normal_profile=[up=%.1f,down=%.1f,h=%d,w=%d] sensor_fov=[up=%.1f,down=%.1f] "
                    "tree_label=%d cloud_in=%zu tree_cloud_total=%zu tree_cloud_valid=%zu tree_cloud_ratio=%.1f%% "
                    "mask_tree_px=%d mask_wh=%dx%d",
                    config_.model_type.c_str(),
                    config_.lsk3dnet_hybrid_normal_mode.c_str(),
                    config_.lsk3dnet_normal_fov_up_deg,
                    config_.lsk3dnet_normal_fov_down_deg,
                    config_.lsk3dnet_normal_proj_h,
                    config_.lsk3dnet_normal_proj_w,
                    config_.fov_up,
                    config_.fov_down,
                    tree_label,
                    flipped_cloud ? flipped_cloud->size() : 0,
                    tree_cloud ? tree_cloud->size() : 0,
                    valid_tree_points,
                    tree_cloud_ratio,
                    mask_tree_pixels,
                    mask.cols,
                    mask.rows);
                std::ostringstream os;
                os << "semantic_chain_anomaly"
                   << " dominant_reason=" << dominant_reason
                   << " dominant_ratio=" << std::fixed << std::setprecision(1) << dominant_ratio << "%"
                   << " empty_tree_ratio=" << empty_tree_ratio << "%"
                   << " empty_cluster_ratio=" << empty_cluster_ratio << "%"
                   << " empty_fit_ratio=" << empty_fit_ratio << "%"
                   << " counters(processed=" << processed_total
                   << ",empty_tree=" << empty_tree
                   << ",empty_cluster=" << empty_cluster
                   << ",empty_fit=" << empty_fit
                   << ",fit_input=" << fit_input_total
                   << ",fit_success=" << fit_success_total << ")"
                   << " fit_dominant_cause=" << dominant_fit_cause
                   << " reason_code=E_SEMANTIC_CHAIN_ANOMALY";
                throw std::runtime_error(os.str());
            }
            if (config_.diag_enable_detailed_stats) {
                const double total_ms = input_filter_ms + seg_ms + mask_ms + cluster_ms + fit_ms;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][STAGE_RATIO] total=%.1fms filter=%.1f%% seg=%.1f%% mask=%.1f%% cluster=%.1f%% fit=%.1f%%",
                    total_ms,
                    100.0 * input_filter_ms / std::max(1e-6, total_ms),
                    100.0 * seg_ms / std::max(1e-6, total_ms),
                    100.0 * mask_ms / std::max(1e-6, total_ms),
                    100.0 * cluster_ms / std::max(1e-6, total_ms),
                    100.0 * fit_ms / std::max(1e-6, total_ms));
            }
        }
        consecutive_failures_.store(0, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        const size_t failures = consecutive_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
        const std::string full_err = describeExceptionFull(e);
        RCLCPP_FATAL(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] FATAL EXCEPTION error=%s cloud_size=%zu cloud_wh=%ux%u cfg(mask_wh=%dx%d input_ch=%d classes=%d tree_class=%d) consecutive_failures=%zu",
            e.what(), cloud ? cloud->size() : 0, cloud ? cloud->width : 0, cloud ? cloud->height : 0,
            config_.img_w, config_.img_h, config_.input_channels, config_.num_classes, config_.tree_class_id, failures);
        RCLCPP_FATAL(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] FULL_EXCEPTION: %s",
            full_err.c_str());
        
        // 🏛️ [架构加固] 满足用户契约：重新抛出异常，让上层 SemanticModule 触发进程退出
        throw;
    } catch (...) {
        const size_t failures = consecutive_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
        const std::string full_err = describeCurrentExceptionFull();
        RCLCPP_FATAL(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] FATAL UNKNOWN_EXCEPTION cloud_size=%zu cloud_wh=%ux%u cfg(mask_wh=%dx%d input_ch=%d classes=%d tree_class=%d) consecutive_failures=%zu detail=%s",
            cloud ? cloud->size() : 0, cloud ? cloud->width : 0, cloud ? cloud->height : 0,
            config_.img_w, config_.img_h, config_.input_channels, config_.num_classes, config_.tree_class_id, failures,
            full_err.c_str());
        throw std::runtime_error(std::string("SemanticProcessor process unknown exception: ") + full_err);
    }

    return result;
}

bool SemanticProcessor::hasRuntimeCapability() const {
    if (runtime_disabled_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (config_.mode == "geometric_only") {
        return geometric_processor_ != nullptr;
    }
    return segmentor_ != nullptr && segmentor_->isReady();
}

bool SemanticProcessor::fitCylinderTryPclSac(const CloudXYZIConstPtr& cloud,
                                               double root[3],
                                               double ray[3],
                                               double* radius,
                                               size_t* inlier_count,
                                               FitRejectReason* reason,
                                               bool log_stages) {
    if (reason) {
        *reason = FitRejectReason::kSuccess;
    }
    if (!cloud || cloud->size() < 10 || !radius || !inlier_count) {
        if (log_stages) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_FIT] step=pcl_sac_pre reject=input cloud_ok=%d n=%zu",
                cloud ? 1 : 0,
                cloud ? cloud->size() : 0U);
        }
        if (reason) {
            *reason = FitRejectReason::kRansacReject;
        }
        return false;
    }
    const size_t n = cloud->size();
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdt(new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setSearchMethod(kdt);
    if (n >= 20) {
        ne.setRadiusSearch(static_cast<double>(config_.cylinder_pcl_normal_radius));
    } else {
        const int knn = static_cast<int>(std::min<std::size_t>(std::max<std::size_t>(n, 2) - 1, 12ULL));
        ne.setKSearch(std::max(3, knn));
    }
    ne.compute(*normals);
    if (normals->size() != n) {
        if (log_stages) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_FIT] step=pcl_normals reject=size_mismatch n_pts=%zu n_normals=%zu",
                n,
                normals->size());
        }
        if (reason) {
            *reason = FitRejectReason::kRansacReject;
        }
        return false;
    }

    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(static_cast<double>(config_.cylinder_pcl_sac_distance));
    seg.setRadiusLimits(0.02, static_cast<double>(config_.max_tree_radius));
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    {
        Eigen::Vector3f ax = vertical_ref_body_.cast<float>();
        if (ax.norm() < 1e-6f) {
            ax = Eigen::Vector3f(0.f, 0.f, 1.f);
        } else {
            ax.normalize();
        }
        seg.setAxis(ax);
    }
    seg.setEpsAngle(static_cast<float>(config_.max_axis_theta * static_cast<float>(M_PI) / 180.f));

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    const size_t min_inl = std::max<size_t>(6, std::min(n / 5 + 1, n));
    if (inliers->indices.size() < min_inl || coefficients->values.size() < 7) {
        if (log_stages) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_FIT] step=pcl_sac_segment reject=inliers_or_coeffs inliers=%zu n=%zu coeff_n=%zu min_inl=%zu",
                inliers->indices.size(),
                n,
                coefficients->values.size(),
                min_inl);
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][fitCylinderTryPclSac] reject inliers=%zu n=%zu coeff_n=%zu min_inl=%zu",
                inliers->indices.size(), n, coefficients->values.size(), min_inl);
        }
        if (reason) {
            *reason = FitRejectReason::kRansacReject;
        }
        return false;
    }

    const auto& v = coefficients->values;
    root[0] = v[0];
    root[1] = v[1];
    root[2] = v[2];
    ray[0] = v[3];
    ray[1] = v[4];
    ray[2] = v[5];
    *radius = static_cast<double>(v[6]);
    *inlier_count = inliers->indices.size();

    if (!std::isfinite(*radius) || *radius <= 0.0 || !std::isfinite(root[0]) || !std::isfinite(ray[0])) {
        if (log_stages) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][TRUNK_FIT] step=pcl_sac_coeffs reject=nonfinite r=%.4f",
                *radius);
        }
        if (reason) {
            *reason = FitRejectReason::kRansacReject;
        }
        return false;
    }
    if (log_stages) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_FIT] step=pcl_sac_ok inliers=%zu r_init=%.4f",
            *inlier_count,
            *radius);
    }
    return true;
}

CylinderLandmark::Ptr SemanticProcessor::fitCylinder(const std::vector<pcl::PointXYZI>& points,
                                                     FitRejectReason* reason,
                                                     double* out_tilt_deg,
                                                     const CylinderFitDiagFrame* diag_frame) {
    if (reason) {
        *reason = FitRejectReason::kSuccess;
    }
    if (out_tilt_deg) {
        *out_tilt_deg = -1.0;
    }
    const bool log_stages =
        diag_frame != nullptr && (config_.diag_trunk_chain_log || geoDebugEnabled(config_));
    auto log_fit = [&](const char* step, const char* fmt, ...) {
        if (!log_stages) {
            return;
        }
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][TRUNK_FIT] ts=%.3f ctx=%d pass=%s step=%s %s",
            diag_frame->ts,
            diag_frame->context_idx,
            diag_frame->pass_tag ? diag_frame->pass_tag : "",
            step,
            buf);
    };

    CloudXYZIPtr tree(new CloudXYZI());
    for (const auto& p : points) {
        tree->push_back(p);
    }

    std::string method = toLowerCopy(config_.cylinder_fit_method);
    if (method != "line_ceres" && method != "pcl_sac" && method != "pcl_sac_ceres") {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] unknown cylinder_fit_method='%s' → using line_ceres",
            config_.cylinder_fit_method.c_str());
        method = "line_ceres";
    }

    log_fit("enter", "pts=%zu method=%s", points.size(), method.c_str());

    double root[3] = {0.0, 0.0, 0.0};
    double ray[3] = {0.0, 0.0, 1.0};
    double radius = 0.05;
    size_t conf_inliers = 0;

    const bool use_pcl_init = (method == "pcl_sac" || method == "pcl_sac_ceres");
    const bool run_ceres = (method == "line_ceres" || method == "pcl_sac_ceres");

    if (use_pcl_init) {
        if (!fitCylinderTryPclSac(tree, root, ray, &radius, &conf_inliers, reason, log_stages)) {
            log_fit("exit", "reject=pcl_sac_fail");
            return nullptr;
        }
        Eigen::Vector3d ax(ray[0], ray[1], ray[2]);
        if (ax.norm() < 1e-6) {
            if (reason) {
                *reason = FitRejectReason::kRansacReject;
            }
            log_fit("exit", "reject=pcl_axis_zero");
            return nullptr;
        }
        ax.normalize();
        ray[0] = ax.x();
        ray[1] = ax.y();
        ray[2] = ax.z();
    } else {
        log_fit("line_ransac", "dist_th=0.10");
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(tree);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][fitCylinder] step=ransac_reject pts=%zu inliers=0", points.size());
            log_fit("exit", "reject=line_ransac_empty");
            if (reason) {
                *reason = FitRejectReason::kRansacReject;
            }
            return nullptr;
        }

        root[0] = coefficients->values[0];
        root[1] = coefficients->values[1];
        root[2] = coefficients->values[2];
        ray[0] = coefficients->values[3];
        ray[1] = coefficients->values[4];
        ray[2] = coefficients->values[5];
        conf_inliers = inliers->indices.size();

        std::vector<double> distances;
        Eigen::Vector3d r(root[0], root[1], root[2]);
        Eigen::Vector3d v(ray[0], ray[1], ray[2]);
        v.normalize();
        for (const auto& p : points) {
            Eigen::Vector3d pt(p.x, p.y, p.z);
            double d = (pt - r).cross(v).norm();
            distances.push_back(d);
        }
        std::sort(distances.begin(), distances.end());
        radius = distances[distances.size() / 2];
        log_fit("line_ransac_ok", "inliers=%zu r_med=%.4f", conf_inliers, radius);
    }

    if (run_ceres) {
        log_fit("ceres", "blocks=%zu max_iter=20", points.size());
        ceres::Problem problem;
        for (const auto& p : points) {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CylinderFittingCost, 1, 3, 3, 1>(
                    new CylinderFittingCost(Eigen::Vector3d(p.x, p.y, p.z))),
                nullptr,
                root, ray, &radius);
        }

        problem.SetManifold(ray, new ceres::SphereManifold<3>());
        problem.SetParameterLowerBound(&radius, 0, 0.005);
        problem.SetParameterUpperBound(&radius, 0, config_.max_tree_radius);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 20;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (summary.termination_type != ceres::CONVERGENCE && summary.termination_type != ceres::USER_SUCCESS) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][fitCylinder] step=ceres_fail pts=%zu term=%d iters=%d",
                points.size(), static_cast<int>(summary.termination_type), summary.iterations.size());
            log_fit("exit",
                "reject=ceres term=%d iters=%zu initial_cost=%.4f final_cost=%.4f",
                static_cast<int>(summary.termination_type),
                summary.iterations.size(),
                summary.initial_cost,
                summary.final_cost);
            if (reason) {
                *reason = FitRejectReason::kCeresFail;
            }
            return nullptr;
        }
        log_fit("ceres_ok",
            "iters=%zu final_cost=%.4f r=%.4f",
            summary.iterations.size(),
            summary.final_cost,
            radius);
    } else {
        log_fit("ceres", "skipped run_ceres=0");
    }

    auto landmark = std::make_shared<CylinderLandmark>();
    landmark->root = Eigen::Vector3d(root[0], root[1], root[2]);
    landmark->ray = Eigen::Vector3d(ray[0], ray[1], ray[2]);
    canonicalizeCylinderAxisUp(&landmark->ray, vertical_ref_body_);
    landmark->radius = radius;
    landmark->confidence = static_cast<double>(conf_inliers) / std::max<size_t>(1, points.size());

    // Sanity checks
    if (landmark->radius > config_.max_tree_radius || landmark->radius < 0.01) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=reject_radius radius=%.3f max=%.2f",
            landmark->radius, config_.max_tree_radius);
        log_fit("exit", "reject=radius r=%.4f max_r=%.2f min_r=0.01", landmark->radius, config_.max_tree_radius);
        if (reason) {
            *reason = FitRejectReason::kRejectRadius;
        }
        return nullptr;
    }

    // Check axis orientation (tilt from vertical in body, aligned with odom gravity when geometric ran)
    double tilt =
        std::acos(std::clamp(std::abs(landmark->ray.dot(vertical_ref_body_)), 0.0, 1.0)) * 180.0 / M_PI;
    if (out_tilt_deg) {
        *out_tilt_deg = tilt;
    }
    log_fit("sanity_precheck",
        "r=%.4f conf_ratio=%.3f tilt=%.2f max_tilt=%.2f cyl_cv_max=%.3f axial_min=%.3f",
        landmark->radius,
        landmark->confidence,
        tilt,
        config_.max_axis_theta,
        static_cast<double>(config_.cylinder_radial_cv_max),
        static_cast<double>(config_.cylinder_min_axial_extent_m));
    if (tilt > config_.max_axis_theta) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=reject_tilt tilt=%.1f deg max=%.1f",
            tilt, config_.max_axis_theta);
        log_fit("exit", "reject=tilt tilt=%.2f max=%.2f", tilt, config_.max_axis_theta);
        if (reason) {
            *reason = FitRejectReason::kRejectTilt;
        }
        return nullptr;
    }

    // Cylinder-shape sanity: reject blobby / non-circular cross-section (e.g. pedestrians).
    const Eigen::Vector3d& r_axis = landmark->root;
    const Eigen::Vector3d& ax_dir = landmark->ray;
    if (config_.cylinder_radial_cv_max > 0.0f) {
        double mean_rad = 0.0;
        for (const auto& p : points) {
            Eigen::Vector3d pt(p.x, p.y, p.z);
            mean_rad += (pt - r_axis).cross(ax_dir).norm();
        }
        mean_rad /= static_cast<double>(points.size());
        double var_rad = 0.0;
        for (const auto& p : points) {
            Eigen::Vector3d pt(p.x, p.y, p.z);
            const double d = (pt - r_axis).cross(ax_dir).norm();
            const double e = d - mean_rad;
            var_rad += e * e;
        }
        var_rad /= static_cast<double>(points.size());
        const double std_rad = std::sqrt(var_rad);
        const double cv = (mean_rad > 1e-4) ? (std_rad / mean_rad) : 99.0;
        if (cv > static_cast<double>(config_.cylinder_radial_cv_max)) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][fitCylinder] step=reject_cyl_cv cv=%.3f max=%.3f pts=%zu",
                cv, static_cast<double>(config_.cylinder_radial_cv_max), points.size());
            log_fit("exit",
                "reject=radial_cv cv=%.4f max=%.4f mean_rad=%.4f std_rad=%.4f",
                cv,
                static_cast<double>(config_.cylinder_radial_cv_max),
                mean_rad,
                std_rad);
            if (reason) {
                *reason = FitRejectReason::kRejectCylinderRadialCv;
            }
            return nullptr;
        }
        log_fit("radial_cv_ok", "cv=%.4f mean_rad=%.4f", cv, mean_rad);
    }
    if (config_.cylinder_min_axial_extent_m > 0.0f && points.size() >= 2U) {
        double t_min = std::numeric_limits<double>::infinity();
        double t_max = -std::numeric_limits<double>::infinity();
        for (const auto& p : points) {
            Eigen::Vector3d pt(p.x, p.y, p.z);
            const double t = (pt - r_axis).dot(ax_dir);
            t_min = std::min(t_min, t);
            t_max = std::max(t_max, t);
        }
        const double axial_span = t_max - t_min;
        if (axial_span < static_cast<double>(config_.cylinder_min_axial_extent_m)) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][fitCylinder] step=reject_cyl_axial span=%.3f min=%.3f pts=%zu",
                axial_span, static_cast<double>(config_.cylinder_min_axial_extent_m), points.size());
            log_fit("exit",
                "reject=axial_span span=%.4f min=%.4f t_min=%.4f t_max=%.4f",
                axial_span,
                static_cast<double>(config_.cylinder_min_axial_extent_m),
                t_min,
                t_max);
            if (reason) {
                *reason = FitRejectReason::kRejectCylinderAxialSpan;
            }
            return nullptr;
        }
        log_fit("axial_ok", "span=%.4f t[min,max]=[%.4f,%.4f]", axial_span, t_min, t_max);
    }

    anchorCylinderRootOnAxisFootprint(&landmark->root, landmark->ray, points);

    landmark->id = allocateLandmarkId();
    log_fit("done",
        "id=%" PRIu64 " root=(%.3f,%.3f,%.3f) r=%.4f conf=%.3f",
        static_cast<unsigned long long>(landmark->id),
        landmark->root.x(),
        landmark->root.y(),
        landmark->root.z(),
        landmark->radius,
        landmark->confidence);
    return landmark;
}

} // namespace automap_pro::v3
