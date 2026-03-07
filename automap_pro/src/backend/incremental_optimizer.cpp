#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/error_code.h"
#define MOD "iSAM2"

#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Matrix.h>
#include <Eigen/SVD>
#include <chrono>

namespace automap_pro {

// GTSAM Symbol 约定：s(sm_id) = Symbol('s', sm_id)
static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }

IncrementalOptimizer::IncrementalOptimizer() {
    const auto& cfg = ConfigManager::instance();

    gtsam::ISAM2Params params;
    // 重线性化阈值：误差变化超过阈值才重线性化（控制计算量）
    params.relinearizeThreshold = cfg.isam2RelinThresh();
    // 每隔几次 update 才检查重线性化（越小越精确，越慢）
    params.relinearizeSkip      = cfg.isam2RelinSkip();
    // 启用/禁用重线性化
    params.enableRelinearization = cfg.isam2EnableRelin();
    // 使用 QR 分解（更稳健，但比 Cholesky 慢）
    params.factorization = gtsam::ISAM2Params::QR;
    // 缓存线性结果（加速 getLinearizationPoint）
    params.cacheLinearizedFactors = true;

    isam2_ = gtsam::ISAM2(params);
}

void IncrementalOptimizer::addSubMapNode(int sm_id, const Pose3d& init_pose, bool fixed) {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);

    if (node_exists_.count(sm_id)) return;
    node_exists_[sm_id] = true;

    pending_values_.insert(SM(sm_id), toPose3(init_pose));
    node_count_++;

    if (fixed || !has_prior_) {
        // 添加先验因子（固定节点或第一个节点）
        auto noise = gtsam::noiseModel::Constrained::All(6);
        pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
            SM(sm_id), toPose3(init_pose), noise));
        has_prior_ = true;
        factor_count_++;
    }
}

void IncrementalOptimizer::addOdomFactor(
    int from, int to,
    const Pose3d& rel,
    const Mat66d& info_matrix)
{
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    if (!node_exists_.count(from) || !node_exists_.count(to)) return;

    auto noise = infoToNoise(info_matrix);
    pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        SM(from), SM(to), toPose3(rel), noise));
    factor_count_++;

    // 里程计因子不立即 update（累积到回环或 GPS 时再提交，减少计算量）
}

OptimizationResult IncrementalOptimizer::addLoopFactor(
    int from, int to,
    const Pose3d& rel,
    const Mat66d& info_matrix)
{
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    if (!node_exists_.count(from) || !node_exists_.count(to)) {
        return OptimizationResult{};
    }

    // 回环约束使用 Huber 鲁棒核（抑制错误回环的影响）
    auto base_noise = infoToNoise(info_matrix);
    auto robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),  // 标准 Huber 参数
        base_noise);

    pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        SM(from), SM(to), toPose3(rel), robust_noise));
    factor_count_++;

    // 回环因子立即触发 iSAM2 update
    return commitAndUpdate();
}

void IncrementalOptimizer::addGPSFactor(
    int sm_id,
    const Eigen::Vector3d& pos_map,
    const Eigen::Matrix3d& cov3x3)
{
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    if (!node_exists_.count(sm_id)) return;

    // GTSAM GPSFactor: 仅约束位置 (X,Y,Z)，不约束姿态
    gtsam::Point3 gps_point(pos_map.x(), pos_map.y(), pos_map.z());
    auto noise = gtsam::noiseModel::Gaussian::Covariance(
        gtsam::Matrix33(cov3x3.data()));
    pending_graph_.add(gtsam::GPSFactor(SM(sm_id), gps_point, noise));
    factor_count_++;

    // GPS 因子立即 update（全局约束，立刻生效）
    commitAndUpdate();
}

OptimizationResult IncrementalOptimizer::forceUpdate() {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    return commitAndUpdate();
}

OptimizationResult IncrementalOptimizer::commitAndUpdate() {
    // 注意：调用时已持有写锁
    if (pending_graph_.empty() && pending_values_.empty()) {
        return OptimizationResult{};
    }

    auto t0 = std::chrono::steady_clock::now();

    ALOG_DEBUG(MOD, "iSAM2 update: pending_factors={} pending_nodes={}",
               pending_graph_.size(), pending_values_.size());
    AUTOMAP_TIMED_SCOPE(MOD, "iSAM2::update", 200.0);

    try {
        isam2_.update(pending_graph_, pending_values_);
        isam2_.update();
        isam2_.update();
        current_estimate_ = isam2_.calculateEstimate();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "iSAM2 update FAILED: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] iSAM2 update FAILED: %s", e.what());
        pending_graph_.resize(0);
        pending_values_.clear();
        return OptimizationResult{false};
    }

    pending_graph_.resize(0);
    pending_values_.clear();

    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 提取所有位姿
    std::unordered_map<int, Pose3d> poses;
    for (const auto& kv : node_exists_) {
        int id = kv.first;
        try {
            auto p = current_estimate_.at<gtsam::Pose3>(SM(id));
            poses[id] = fromPose3(p);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "iSAM2 extract pose sm_id={} exception: {}", id, e.what());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] extract pose sm_id=%d: %s", id, e.what());
        } catch (...) {
            ALOG_ERROR(MOD, "iSAM2 extract pose sm_id={} unknown exception (not in estimate?)", id);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] extract pose sm_id=%d: unknown exception", id);
        }
    }

    OptimizationResult res;
    res.success       = true;
    res.nodes_updated = (int)poses.size();
    res.elapsed_ms    = elapsed;
    res.submap_poses  = poses;

    ALOG_INFO(MOD, "iSAM2 update done: nodes={} elapsed={:.1f}ms total_factors={}",
              res.nodes_updated, res.elapsed_ms, factor_count_);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[PRECISION][OPT] iSAM2_update nodes_updated=%d elapsed_ms=%.1f factor_count=%d",
        res.nodes_updated, res.elapsed_ms, factor_count_);

    notifyPoseUpdate(poses);
    return res;
}

Pose3d IncrementalOptimizer::getPose(int sm_id) const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    try {
        if (!current_estimate_.exists(SM(sm_id)))
            return Pose3d::Identity();
        auto p = current_estimate_.at<gtsam::Pose3>(SM(sm_id));
        return fromPose3(p);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "getPose(sm_id={}) exception: {}", sm_id, e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] getPose sm_id=%d: %s", sm_id, e.what());
        return Pose3d::Identity();
    } catch (...) {
        ALOG_ERROR(MOD, "getPose(sm_id={}) unknown exception", sm_id);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] getPose sm_id=%d: unknown exception", sm_id);
        return Pose3d::Identity();
    }
}

std::unordered_map<int, Pose3d> IncrementalOptimizer::getAllPoses() const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    std::unordered_map<int, Pose3d> out;
    for (const auto& kv : node_exists_) {
        try {
            auto p = current_estimate_.at<gtsam::Pose3>(SM(kv.first));
            out[kv.first] = fromPose3(p);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "getAllPoses sm_id={} exception: {}", kv.first, e.what());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] getAllPoses sm_id=%d: %s", kv.first, e.what());
        } catch (...) {
            ALOG_ERROR(MOD, "getAllPoses sm_id={} unknown exception", kv.first);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] getAllPoses sm_id=%d: unknown exception", kv.first);
        }
    }
    return out;
}

void IncrementalOptimizer::reset() {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = ConfigManager::instance().isam2RelinThresh();
    params.relinearizeSkip      = ConfigManager::instance().isam2RelinSkip();
    isam2_ = gtsam::ISAM2(params);
    pending_graph_.resize(0);
    pending_values_.clear();
    current_estimate_.clear();
    node_exists_.clear();
    node_count_ = 0;
    factor_count_ = 0;
    has_prior_ = false;
}

int IncrementalOptimizer::nodeCount()   const { return node_count_; }
int IncrementalOptimizer::factorCount() const { return factor_count_; }

// ─────────────────────────────────────────────────────────────────────────────
// 类型转换工具
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Pose3 IncrementalOptimizer::toPose3(const Pose3d& T) const {
    Eigen::Quaterniond q(T.rotation());
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
    gtsam::Point3 pos(T.translation().x(), T.translation().y(), T.translation().z());
    return gtsam::Pose3(rot, pos);
}

Pose3d IncrementalOptimizer::fromPose3(const gtsam::Pose3& p) const {
    Pose3d T = Pose3d::Identity();
    T.translation() = Eigen::Vector3d(
        p.translation().x(), p.translation().y(), p.translation().z());
    gtsam::Quaternion gq = p.rotation().toQuaternion();
    T.linear() = Eigen::Quaterniond(gq.w(), gq.x(), gq.y(), gq.z()).toRotationMatrix();
    return T;
}

gtsam::noiseModel::Gaussian::shared_ptr
IncrementalOptimizer::infoToNoise(const Mat66d& info) const {
    // 检查奇异性：使用 SVD 分解判断秩和条件数
    Eigen::JacobiSVD<Mat66d> svd(info, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // 检查秩是否满秩（rank < 6 表示奇异）
    const int rank = svd.rank();
    if (rank < 6) {
        ALOG_WARN("IncrementalOptimizer", 
                  "Information matrix rank-deficient (rank={}/6), using conservative covariance", rank);
        gtsam::Matrix66 cov = gtsam::Matrix66::Identity() * 1.0;
        return gtsam::noiseModel::Gaussian::Covariance(cov);
    }

    // ✅ 修复：降低阈值，更早触发正则化（从 1e10 降到 1e6）
    // ✅ 修复：对所有情况使用相对正则化（避免数值不稳定）
    const double max_sv = svd.singularValues()(0);
    const double min_sv = svd.singularValues()(5);
    const double cond = (max_sv > 1e-12) ? (max_sv / min_sv) : 1e12;

    // 相对正则化：info + (max_sv * 1e-3) * I
    Mat66d info_reg = info + Mat66d::Identity() * std::max(1e-6, min_sv * 1e-3);
    
    gtsam::Matrix66 cov = info_reg.inverse().cast<double>();
    cov = 0.5 * (cov + cov.transpose());  // 确保对称

    // ✅ 修复：检查求逆结果是否有效
    if (!cov.allFinite()) {
        ALOG_ERROR(MOD, "Regularized inverse contains NaN/Inf, using fallback covariance");
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] Regularized inverse contains NaN/Inf, using fallback covariance");
        cov = gtsam::Matrix66::Identity() * 1.0;
        return gtsam::noiseModel::Gaussian::Covariance(cov);
    }

    ALOG_DEBUG(MOD, "InfoToNoise: cond={:.2e} reg_factor={:.2e}", cond, min_sv * 1e-3);
    return gtsam::noiseModel::Gaussian::Covariance(cov);
}

void IncrementalOptimizer::notifyPoseUpdate(const std::unordered_map<int, Pose3d>& poses) {
    for (auto& cb : pose_update_cbs_) {
        cb(poses);
    }
}

} // namespace automap_pro
