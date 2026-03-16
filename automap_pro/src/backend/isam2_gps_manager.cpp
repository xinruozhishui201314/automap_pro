#include "automap_pro/backend/isam2_gps_manager.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/backend/isam2_factor_types.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

#include <rclcpp/rclcpp.hpp>

#define MOD "ISAM2_GPS"

namespace automap_pro {

// GTSAM Symbol 约定：s(sm_id) = Symbol('s', sm_id)
static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }

ISAM2GPSManager::ISAM2GPSManager() {
}

ISAM2GPSManager::~ISAM2GPSManager() {
}

void ISAM2GPSManager::addGPSFactor(
    int sm_id,
    const Eigen::Vector3d& pos_map,
    const Eigen::Matrix3d& cov3x3,
    gtsam::NonlinearFactorGraph& pending_graph,
    int& factor_count,
    const gtsam::Values& current_estimate,
    const std::unordered_map<int, bool>& node_exists)
{
    // 检查节点是否存在
    if (node_exists.find(sm_id) == node_exists.end()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2GPSManager] sm_id=%d not exists, skip", sm_id);
        return;
    }

    // 检查节点是否在 current_estimate 中
    if (!current_estimate.exists(SM(sm_id))) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2GPSManager] sm_id=%d not in current_estimate, skip", sm_id);
        return;
    }

    // 约束合理性检查
    if (!pos_map.allFinite() || !cov3x3.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[ISAM2GPSManager] addGPSFactor sm_id=%d non-finite input, skip", sm_id);
        return;
    }

    // 应用动态协方差调整
    Eigen::Matrix3d final_cov = applyDynamicCovariance(pos_map, cov3x3);

    // GPS 异常值检测
    auto outlier_result = detectOutlier(sm_id, pos_map, cov3x3, current_estimate);
    if (outlier_result.is_outlier) {
        final_cov *= outlier_result.scale;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2GPSManager] GPS outlier detected: sm_id=%d residual=%.3f scale=%.1f",
            sm_id, outlier_result.residual, outlier_result.scale);
    }

    // 添加 GPS 因子到因子图
    addGPSToGraph(sm_id, pos_map, final_cov, pending_graph, factor_count);
}

int ISAM2GPSManager::addGPSFactorsBatch(
    const std::vector<GPSFactorItem>& factors,
    gtsam::NonlinearFactorGraph& pending_graph,
    int& factor_count,
    const std::unordered_map<int, bool>& node_exists)
{
    int added = 0;
    for (const auto& f : factors) {
        if (node_exists.find(f.sm_id) == node_exists.end()) {
            continue;
        }
        try {
            addGPSToGraph(f.sm_id, f.pos, f.cov, pending_graph, factor_count);
            added++;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[ISAM2GPSManager] addGPSFactorsBatch failed for sm_id=%d: %s",
                f.sm_id, e.what());
        }
    }
    return added;
}

int ISAM2GPSManager::flushPendingGPSFactors(
    std::vector<GPSFactorItem>& pending_gps,
    gtsam::NonlinearFactorGraph& pending_graph,
    int& factor_count,
    const gtsam::Values& current_estimate,
    const std::unordered_map<int, bool>& node_exists)
{
    int added = 0;
    std::vector<GPSFactorItem> still_pending;

    for (auto& f : pending_gps) {
        if (node_exists.find(f.sm_id) == node_exists.end() ||
            !current_estimate.exists(SM(f.sm_id))) {
            still_pending.push_back(std::move(f));
            continue;
        }

        if (!f.pos.allFinite() || !f.cov.allFinite()) {
            continue;
        }

        addGPSToGraph(f.sm_id, f.pos, f.cov, pending_graph, factor_count);
        added++;
    }

    pending_gps = std::move(still_pending);
    return added;
}

GPSOutlierResult ISAM2GPSManager::detectOutlier(
    int sm_id,
    const Eigen::Vector3d& pos_map,
    const Eigen::Matrix3d& cov3x3,
    const gtsam::Values& current_estimate) const
{
    GPSOutlierResult result;

    if (!current_estimate.exists(SM(sm_id))) {
        return result;
    }

    const auto& cfg = ConfigManager::instance();
    if (!cfg.gpsEnableOutlierDetection()) {
        return result;
    }

    try {
        auto current_est = current_estimate.at<gtsam::Pose3>(SM(sm_id));
        Eigen::Vector3d current_pos = current_est.translation();

        double residual = (pos_map - current_pos).norm();
        double residual_baseline = cfg.gpsResidualBaseline();

        if (residual > residual_baseline) {
            result.is_outlier = true;
            result.residual = residual;
            result.scale = cfg.gpsOutlierCovScale();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2GPSManager] detectOutlier failed: %s", e.what());
    }

    return result;
}

Eigen::Matrix3d ISAM2GPSManager::applyDynamicCovariance(
    const Eigen::Vector3d& pos_map,
    const Eigen::Matrix3d& cov3x3) const
{
    const auto& cfg = ConfigManager::instance();

    if (!cfg.gpsEnableDynamicCov()) {
        return cov3x3;
    }

    // 基于卫星数调整协方差
    double sat_scale = 1.0;
    int min_sats = cfg.gpsMinSatellites();
    sat_scale = std::min(1.0, 6.0 / static_cast<double>(std::max(min_sats, 4)));

    // 基于高度调整协方差
    double alt_scale = 1.0;
    double high_alt_thresh = cfg.gpsHighAltitudeThreshold();
    double high_alt_scale = cfg.gpsHighAltitudeScale();
    if (std::abs(pos_map.z()) > high_alt_thresh) {
        alt_scale = high_alt_scale;
    }

    return cov3x3 * sat_scale * alt_scale;
}

void ISAM2GPSManager::addGPSToGraph(
    int sm_id,
    const Eigen::Vector3d& pos,
    const Eigen::Matrix3d& cov,
    gtsam::NonlinearFactorGraph& pending_graph,
    int& factor_count)
{
    gtsam::Point3 gps_point(pos.x(), pos.y(), pos.z());
    gtsam::Vector3 vars;
    vars << std::max(1e-6, cov(0, 0)),
            std::max(1e-6, cov(1, 1)),
            std::max(1e-6, cov(2, 2));
    auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
    pending_graph.add(gtsam::GPSFactor(SM(sm_id), gps_point, noise));
    factor_count++;
}

}  // namespace automap_pro
