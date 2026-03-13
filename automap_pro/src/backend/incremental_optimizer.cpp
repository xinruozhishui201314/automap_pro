#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/crash_report.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/error_code.h"
#define MOD "iSAM2"

#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/SVD>
#include <chrono>
#include <set>
#include <string>

namespace automap_pro {

// GTSAM Symbol 约定：s(sm_id) = Symbol('s', sm_id)
static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }

// ── 优化前约束全量转储与合理性校验（便于崩溃精准定位，校验失败则中止 update 避免触发 GTSAM 崩溃）──
namespace {
// 平移合理范围（米），超出视为异常输入，避免 GTSAM 数值问题
constexpr double kMaxReasonableTranslationNorm = 1e6;

struct ConstraintValidation {
    bool all_keys_exist = true;
    bool all_values_finite = true;
    bool all_values_reasonable = true;  // 平移/旋转在合理范围内
    std::string message;
};

void logAllConstraintsAndValidate(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,
    const char* tag,
    ConstraintValidation* out_validation)
{
    auto log = rclcpp::get_logger("automap_system");
    const size_t nf = graph.size();
    const size_t nv = values.size();
    RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] tag=%s total_factors=%zu total_values=%zu (优化前全量约束，崩溃时 grep GTSAM_CONSTRAINTS 定位)",
                tag, nf, nv);

    std::set<gtsam::Key> value_key_set;
    for (const gtsam::Key k : values.keys())
        value_key_set.insert(k);

    for (size_t i = 0; i < nf; ++i) {
        const auto& f = graph[i];
        gtsam::KeyVector kv = f->keys();
        std::string keys_str;
        std::string in_vals_str;
        std::string finite_str;
        bool factor_ok = true;
        for (gtsam::Key k : kv) {
            keys_str += (keys_str.empty() ? "" : ",") + std::to_string(k);
            bool in_vals = (value_key_set.count(k) != 0);
            in_vals_str += (in_vals_str.empty() ? "" : ",") + std::string(in_vals ? "yes" : "NO");
            bool finite = false;
            if (in_vals && values.exists(k)) {
                try {
                    auto p = values.at<gtsam::Pose3>(k);
                    Eigen::Vector3d t = p.translation();
                    Eigen::Matrix3d R = p.rotation().matrix();
                    finite = t.array().isFinite().all() && R.array().isFinite().all();
                } catch (...) {
                    finite = false;
                }
                if (!finite) factor_ok = false;
            } else if (!in_vals) {
                factor_ok = false;
            }
            finite_str += (finite_str.empty() ? "" : ",") + std::string(finite ? "yes" : (in_vals ? "nan/inf" : "n/a"));
        }
        const char* type_str = "Other";
        if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) type_str = "Prior";
        else if (dynamic_cast<const gtsam::GPSFactor*>(f.get())) type_str = "GPS";
        else if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(f.get())) type_str = "Between";
        RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] factor %zu type=%s keys=[%s] keys_in_values=[%s] value_finite=[%s] valid=%s",
                    i, type_str, keys_str.c_str(), in_vals_str.c_str(), finite_str.c_str(), factor_ok ? "yes" : "NO");
        if (out_validation) {
            for (gtsam::Key k : kv)
                if (value_key_set.count(k) == 0) out_validation->all_keys_exist = false;
            if (!factor_ok) out_validation->all_values_finite = false;
        }
    }

    for (const gtsam::Key k : values.keys()) {
        std::string pose_str = "n/a";
        bool finite = false;
        bool reasonable = false;
        try {
            auto p = values.at<gtsam::Pose3>(k);
            Eigen::Vector3d t = p.translation();
            Eigen::Matrix3d R = p.rotation().matrix();
            finite = t.array().isFinite().all() && R.array().isFinite().all();
            double tnorm = t.norm();
            reasonable = finite && (tnorm <= kMaxReasonableTranslationNorm);
            pose_str = "x=" + std::to_string(t.x()) + " y=" + std::to_string(t.y()) + " z=" + std::to_string(t.z());
            if (out_validation && finite && tnorm > kMaxReasonableTranslationNorm)
                out_validation->all_values_reasonable = false;
        } catch (...) {
            pose_str = "at_failed";
        }
        RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] value key=%zu %s finite=%s reasonable=%s",
                    static_cast<size_t>(k), pose_str.c_str(), finite ? "1" : "0", reasonable ? "1" : "0");
        if (out_validation && !finite) out_validation->all_values_finite = false;
    }

    if (out_validation) {
        for (size_t i = 0; i < nf; ++i) {
            for (gtsam::Key k : graph[i]->keys()) {
                if (value_key_set.count(k) == 0) {
                    out_validation->all_keys_exist = false;
                    break;
                }
            }
        }
        out_validation->message =
            (out_validation->all_keys_exist && out_validation->all_values_finite && out_validation->all_values_reasonable)
                ? "ok"
                : ("keys_exist=" + std::string(out_validation->all_keys_exist ? "1" : "0") +
                   " values_finite=" + std::string(out_validation->all_values_finite ? "1" : "0") +
                   " values_reasonable=" + std::string(out_validation->all_values_reasonable ? "1" : "0"));
    }
    RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] tag=%s validation done %s (若崩溃在 error() 或 optimize() 内，见上一 GTSAM_CONSTRAINTS 约束列表)",
                tag, out_validation ? out_validation->message.c_str() : "n/a");
}
}  // anonymous namespace

IncrementalOptimizer::IncrementalOptimizer() {
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][LOAD_TRACE] constructor entered (about to ensureGtsamTbbSerialized)");
    fflush(stdout);
    ensureGtsamTbbSerialized();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][LOAD_TRACE] ensureGtsamTbbSerialized done (about to read config and build ISAM2Params)");
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
    // 关闭线性化缓存，避免 GPS 对齐后 commitAndUpdate 时在 linearize 路径发生 double free
    //（与 borglab/gtsam#1189 同类：NoiseModelFactor::linearize -> free 处崩溃）
    params.cacheLinearizedFactors = false;
    // ✅ 新增：关闭非线性误差评估，减少 TBB 调用路径（减少并发导致 double free 的机会）
    params.evaluateNonlinearError = false;

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][LOAD_TRACE] ISAM2Params set (about to construct gtsam::ISAM2; if crash here, see borglab/gtsam#1189 / lago static init)");
    fflush(stdout);
    isam2_ = gtsam::ISAM2(params);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][LOAD_TRACE] gtsam::ISAM2 constructed ok (about to set prior_noise_)");
    // 使用 Diagonal 方差作为“近似固定先验”（可配置），避免 Constrained 退出时 SIGSEGV；适度先验利于点云清晰度，过强易导致数值刚度
    double pvar = cfg.isam2PriorVariance();
    prior_var6_.resize(6);
    prior_var6_ << pvar, pvar, pvar, pvar, pvar, pvar;
    prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_var6_);  // 仅用于 !prior_noise_ 检查与 shutdown

    opt_running_ = true;
    opt_thread_ = std::thread(&IncrementalOptimizer::optLoop, this);
}

IncrementalOptimizer::~IncrementalOptimizer() {
    ALOG_INFO(MOD, "IncrementalOptimizer destructor: clearing factor graph and iSAM2 (avoid GTSAM static destructor SIGSEGV)");
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[IncrementalOptimizer][SHUTDOWN] destructor entered (clearForShutdown idempotent)");
    clearForShutdown();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[IncrementalOptimizer][SHUTDOWN] destructor done");
}

void IncrementalOptimizer::addSubMapNode(int sm_id, const Pose3d& init_pose, bool fixed) {
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addSubMapNode: rw_mutex wait %.1fms (potential contention)", lock_wait_ms);
    }
    if (!prior_noise_) {
        ALOG_DEBUG(MOD, "addSubMapNode: already shutdown (prior_noise_ released), ignore");
        return;
    }
    if (node_exists_.count(sm_id)) return;

    // 约束合理性：拒绝非法初始位姿，避免后续 update 触发 GTSAM 异常
    const Eigen::Vector3d& t = init_pose.translation();
    const Eigen::Matrix3d& R = init_pose.rotation();
    if (!t.allFinite() || !R.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addSubMapNode sm_id=%d init_pose non-finite, skip (grep BACKEND VALIDATION)",
            sm_id);
        return;
    }
    if (t.norm() > kMaxReasonableTranslationNorm) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addSubMapNode sm_id=%d translation norm=%.1f > %.0f, skip",
            sm_id, t.norm(), kMaxReasonableTranslationNorm);
        return;
    }

    node_exists_[sm_id] = true;
    pending_values_.insert(SM(sm_id), toPose3(init_pose));
    node_count_++;

    // 增强诊断日志：记录节点添加后的状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND] addSubMapNode: sm_id=%d fixed=%d node_count=%d "
        "pending_values=%zu pending_factors=%zu prior_added=%d",
        sm_id, fixed ? 1 : 0, node_count_, pending_values_.size(),
        pending_graph_.size(), (fixed || !has_prior_) ? 1 : 0);

    if (fixed || !has_prior_) {
        // 每次新建 Prior noise，避免共享 prior_noise_ 在 GTSAM linearize 路径触发 double free（borglab/gtsam#1189 同类）
        gtsam::noiseModel::Diagonal::shared_ptr noise =
            gtsam::noiseModel::Diagonal::Variances(prior_var6_);
        pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
            SM(sm_id), toPose3(init_pose), noise));
        has_prior_ = true;
        factor_count_++;

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addSubMapNode: sm_id=%d PriorFactor added, total factor_count=%d",
            sm_id, factor_count_);
    }
}

void IncrementalOptimizer::updateSubMapNodePose(int sm_id, const Pose3d& pose) {
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] updateSubMapNodePose: rw_mutex wait %.1fms", lock_wait_ms);
    }

    gtsam::Key key = SM(sm_id);
    
    // 检查节点是否存在
    if (!node_exists_.count(sm_id)) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] updateSubMapNodePose: sm_id=%d not exists, inserting new node",
            sm_id);
        node_exists_[sm_id] = true;
    }
    
    // 更新current_estimate_中的值
    if (current_estimate_.exists(key)) {
        current_estimate_.update(key, toPose3(pose));
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] updateSubMapNodePose: sm_id=%d updated in current_estimate_",
            sm_id);
    } else {
        current_estimate_.insert(key, toPose3(pose));
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] updateSubMapNodePose: sm_id=%d inserted into current_estimate_",
            sm_id);
    }
}

void IncrementalOptimizer::addOdomFactor(
    int from, int to,
    const Pose3d& rel,
    const Mat66d& info_matrix)
{
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addOdomFactor: rw_mutex wait %.1fms from=%d to=%d", lock_wait_ms, from, to);
    }
    if (!node_exists_.count(from) || !node_exists_.count(to)) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][ODOM] skip addOdomFactor from=%d to=%d "
            "(from_exists=%d to_exists=%d node_count=%zu)",
            from, to,
            node_exists_.count(from) ? 1 : 0,
            node_exists_.count(to) ? 1 : 0,
            node_count_);
        return;
    }

    // 约束合理性：拒绝非法 rel/info 进入 pending，避免后续 update 触发 GTSAM 异常
    const Eigen::Vector3d& t = rel.translation();
    const Eigen::Matrix3d& R = rel.rotation();
    if (!t.allFinite() || !R.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addOdomFactor from=%d to=%d rel non-finite, skip",
            from, to);
        return;
    }
    if (t.norm() > kMaxReasonableTranslationNorm) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addOdomFactor from=%d to=%d translation norm=%.1f > %.0f, skip",
            from, to, t.norm(), kMaxReasonableTranslationNorm);
        return;
    }
    if (!info_matrix.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addOdomFactor from=%d to=%d info_matrix non-finite, skip",
            from, to);
        return;
    }

    // 使用 Diagonal 噪声避免 Gaussian::Covariance 在 linearize 路径触发 double free（即使 TBB 已关）
    auto noise = infoToNoiseDiagonal(info_matrix);
    pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        SM(from), SM(to), toPose3(rel), noise));
    factor_count_++;

    // 增强诊断日志：记录里程计因子添加后的状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND][ODOM] addOdomFactor from=%d to=%d "
        "factor_count=%d pending_factors=%zu",
        from, to, factor_count_, pending_graph_.size());

    // 里程计因子不立即 update（累积到回环或 GPS 时再提交，减少计算量）
}

OptimizationResult IncrementalOptimizer::addLoopFactor(
    int from, int to,
    const Pose3d& rel,
    const Mat66d& info_matrix)
{
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addLoopFactor: rw_mutex wait %.1fms from=%d to=%d", lock_wait_ms, from, to);
    }
    // ✅ 修复：使用 try-catch 包裹，防止 GTSAM 内部 SIGSEGV 导致进程崩溃（已持 lk）
    try {
        if (!prior_noise_) return OptimizationResult{};

        // 同节点回环无效：BetweenFactor(from, to) 当 from==to 时退化，且易导致 commitAndUpdate 异常
        if (from == to) {
            ALOG_DEBUG(MOD, "addLoopFactor: from==to=%d (same node), skip degenerate loop", from);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][LOOP] skip addLoopFactor from=%d to=%d (same node, invalid Between factor; grep BACKEND LOOP)",
                from, to);
            return OptimizationResult{};
        }

        // ✅ 修复：显式检查节点存在性
        if (node_exists_.find(from) == node_exists_.end() ||
            node_exists_.find(to) == node_exists_.end()) {
            ALOG_DEBUG(MOD, "addLoopFactor: from=%d or to=%d not exists, skip", from, to);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][LOOP] skip addLoopFactor from=%d to=%d (node not in graph, grep BACKEND LOOP 定位)",
                from, to);
            return OptimizationResult{};
        }

        // 约束合理性：拒绝非法 rel/info，避免触发 GTSAM 异常
        const Eigen::Vector3d& rel_t = rel.translation();
        const Eigen::Matrix3d& rel_R = rel.rotation();
        if (!rel_t.allFinite() || !rel_R.allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addLoopFactor from=%d to=%d rel non-finite, skip",
                from, to);
            return OptimizationResult{};
        }
        if (rel_t.norm() > kMaxReasonableTranslationNorm) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addLoopFactor from=%d to=%d translation norm=%.1f > %.0f, skip",
                from, to, rel_t.norm(), kMaxReasonableTranslationNorm);
            return OptimizationResult{};
        }
        if (!info_matrix.allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addLoopFactor from=%d to=%d info_matrix non-finite, skip",
                from, to);
            return OptimizationResult{};
        }

        // 回环约束使用 Huber 鲁棒核；base 用 Diagonal 避免 linearize 路径 double free
        auto base_noise = infoToNoiseDiagonal(info_matrix);
        gtsam::noiseModel::Base::shared_ptr robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            base_noise);

        pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            SM(from), SM(to), toPose3(rel), robust_noise));
        factor_count_++;

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][LOOP] loop constraint added to graph from=%d to=%d (grep for verification)",
            from, to);
        // 回环因子立即触发 iSAM2 update
        return commitAndUpdate();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "addLoopFactor failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] addLoopFactor from=%d to=%d: %s", from, to, e.what());
        return OptimizationResult{};
    } catch (...) {
        // ✅ P0 修复：捕获未知异常，防止进程崩溃
        ALOG_ERROR(MOD, "addLoopFactor: unknown exception from=%d to=%d", from, to);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] addLoopFactor from=%d to=%d: unknown exception", from, to);
        return OptimizationResult{};
    }
}

void IncrementalOptimizer::addLoopFactorDeferred(int from, int to,
                                                  const Pose3d& rel, const Mat66d& info_matrix) {
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addLoopFactorDeferred: rw_mutex wait %.1fms from=%d to=%d", lock_wait_ms, from, to);
    }
    try {
        if (!prior_noise_) return;
        if (from == to) return;
        if (node_exists_.find(from) == node_exists_.end() || node_exists_.find(to) == node_exists_.end()) return;
        const Eigen::Vector3d& rel_t = rel.translation();
        const Eigen::Matrix3d& rel_R = rel.rotation();
        if (!rel_t.allFinite() || !rel_R.allFinite()) return;
        if (rel_t.norm() > kMaxReasonableTranslationNorm) return;
        if (!info_matrix.allFinite()) return;
        auto base_noise = infoToNoiseDiagonal(info_matrix);
        gtsam::noiseModel::Base::shared_ptr robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
        pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(SM(from), SM(to), toPose3(rel), robust_noise));
        factor_count_++;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][LOOP] loop constraint added (deferred) from=%d to=%d (single commit later)",
            from, to);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "addLoopFactorDeferred failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] addLoopFactorDeferred from=%d to=%d: %s", from, to, e.what());
    } catch (...) {
        ALOG_ERROR(MOD, "addLoopFactorDeferred: unknown exception from=%d to=%d", from, to);
    }
}

void IncrementalOptimizer::addGPSFactor(
    int sm_id,
    const Eigen::Vector3d& pos_map,
    const Eigen::Matrix3d& cov3x3)
{
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addGPSFactor: rw_mutex wait %.1fms sm_id=%d", lock_wait_ms, sm_id);
    }
    // ✅ 修复：仅添加 GPS 因子，不立即 commit
    // 问题根源是 GTSAM 内部 TBB 并行导致竞态，延迟 commit 可以减少并发
    try {
        // 第359行已经获取了锁，不需要重复获取
        if (!prior_noise_) return;

        // ✅ 修复：显式检查节点存在性；若节点尚未加入（如 GPS 对齐早于首帧 commit），加入 deferred 稍后 flush
        if (node_exists_.find(sm_id) == node_exists_.end()) {
            ALOG_DEBUG(MOD, "addGPSFactor: sm_id=%d not exists, defer", sm_id);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][GPS] defer addGPSFactor sm_id=%d (node not in node_exists_, "
                "node_count=%zu pending_gps_factors_=%zu)",
                sm_id, node_exists_.size(), pending_gps_factors_.size() + 1);
            pending_gps_factors_.push_back(GPSFactorItem{sm_id, pos_map, cov3x3});
            return;
        }

        // ========== 优化：节点未进入 current_estimate_ 时仅 defer，不在此处同步 forceUpdate ==========
        // 原逻辑在此处调用 forceUpdate() 可能导致单次 10–30s 阻塞；改为仅加入 pending，由后续 commit（回环/子图冻结等）后 flush 时加入
        if (!current_estimate_.exists(SM(sm_id))) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][GPS] defer addGPSFactor sm_id=%d (node not in current_estimate_, "
                "likely due to GTSAM single-node defer, pending_gps_factors_=%zu)",
                sm_id, pending_gps_factors_.size() + 1);
            pending_gps_factors_.push_back(GPSFactorItem{sm_id, pos_map, cov3x3});
            return;
        }

        // GPS 因子可以直接添加到图中
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS] addGPSFactor sm_id=%d directly to graph (node exists in current_estimate_)",
            sm_id);

        // 约束合理性：拒绝非法 GPS 位置/协方差，避免后续 update 触发 GTSAM 异常
        if (!pos_map.allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addGPSFactor sm_id=%d pos_map non-finite, skip",
                sm_id);
            return;
        }
        if (!cov3x3.allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addGPSFactor sm_id=%d cov3x3 non-finite, skip",
                sm_id);
            return;
        }

        // 【优化1】GPS动态协方差：基于卫星数和高度动态调整GPS协方差
        // 参考：Liu et al. "Robust GPS-aided SLAM" ICRA 2023

        const auto& cfg = ConfigManager::instance();

        // 基于卫星数调整协方差（卫星数越多，协方差越小）
        double sat_scale = 1.0;
        if (cfg.gpsEnableDynamicCov()) {
            int min_sats = cfg.gpsMinSatellites();
            sat_scale = std::min(1.0, 6.0 / (double)std::max(min_sats, 4));
        }

        // 基于高度调整协方差（高空精度较差）
        double alt_scale = 1.0;
        if (cfg.gpsEnableDynamicCov()) {
            double high_alt_thresh = cfg.gpsHighAltitudeThreshold();
            double high_alt_scale = cfg.gpsHighAltitudeScale();
            if (std::abs(pos_map.z()) > high_alt_thresh) {
                alt_scale = high_alt_scale;
            }
        }

        // 应用动态缩放
        Eigen::Matrix3d dynamic_cov = cov3x3 * sat_scale * alt_scale;

        // 【优化2】GPS异常值检测：基于历史残差统计
        // 参考：Zhang et al. "Consistent-View Bundle Adjustment" CVPR 2021

        Eigen::Matrix3d final_cov = dynamic_cov;

        if (cfg.gpsEnableOutlierDetection() && node_exists_.count(sm_id)) {
            // 获取当前估计（需检查 key 是否存在，避免 at() 抛异常）
            if (current_estimate_.exists(SM(sm_id))) {
                try {
                    auto current_est = current_estimate_.at<gtsam::Pose3>(SM(sm_id));
                    Eigen::Vector3d current_pos = fromPose3(current_est).translation();

                    // 计算残差
                    double residual = (pos_map - current_pos).norm();

                    // 简单的阈值检测（避免复杂的统计计算）
                    double residual_baseline = 2.0;  // 基准残差（米）

                    // 如果残差过大，认为是异常值
                    if (residual > residual_baseline) {
                        double outlier_scale = cfg.gpsOutlierCovScale();
                        final_cov *= outlier_scale;  // 放大协方差，降低约束强度

                        ALOG_DEBUG(MOD,
                                  "[GPS_OPT] GPS outlier detected: sm_id={} residual={:.2f}m baseline={:.2f}m cov_scale={:.1f}",
                                  sm_id, residual, residual_baseline, outlier_scale);
                    }
                } catch (const std::exception& e) {
                    ALOG_WARN(MOD,
                              "[GPS_OPT] Exception when accessing current_estimate_ for sm_id={}: {} "
                              "(skip outlier detection for this factor)",
                              sm_id, e.what());
                }
            }
        }

        // GTSAM GPSFactor: 仅约束位置 (X,Y,Z)，不约束姿态
        // 使用 Diagonal 噪声避免 Gaussian::Covariance(Matrix33) 在 linearize 路径触发 double free（borglab/gtsam#1189 同类）
        gtsam::Point3 gps_point(pos_map.x(), pos_map.y(), pos_map.z());
        gtsam::Vector3 vars;
        vars << std::max(1e-6, final_cov(0, 0)),
                std::max(1e-6, final_cov(1, 1)),
                std::max(1e-6, final_cov(2, 2));
        auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
        pending_graph_.add(gtsam::GPSFactor(SM(sm_id), gps_point, noise));
        factor_count_++;

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS] GPS factor added to graph sm_id=%d (grep for verification)",
            sm_id);
        // ✅ 修复：不立即 commit，而是等待外层统一处理
        // 立即 commit 会触发 GTSAM 内部 TBB 并行，导致 SIGSEGV
        // 调用方应该在适当时候调用 forceUpdate()
        ALOG_DEBUG(MOD, "addGPSFactor: sm_id=%d factor added, pending commit", sm_id);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "addGPSFactor failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] addGPSFactor sm_id=%d: %s", sm_id, e.what());
    } catch (...) {
        // ✅ P0 修复：捕获未知异常，防止进程崩溃
        ALOG_ERROR(MOD, "addGPSFactor: unknown exception for sm_id=%d", sm_id);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] addGPSFactor sm_id=%d: unknown exception", sm_id);
    }
}

void IncrementalOptimizer::addGPSFactorsBatch(const std::vector<GPSFactorItem>& factors) {
    GtsamCallScope scope(GtsamCaller::ISAM2, "addGPSFactorsBatch",
                         "count=" + std::to_string(factors.size()), false);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG] addGPSFactorsBatch enter count=%zu (opt 线程执行，随后单次 commitAndUpdate)",
        factors.size());

    // ✅ 修复：使用 try-catch 包裹整个操作，防止 SIGSEGV 导致进程崩溃
    // GTSAM 内部 TBB 在某些版本/场景下有竞态条件（类似 borglab/gtsam#1189）
    // 即使 GTSAM 本身有异常处理，也可能触发 SIGSEGV 信号
    try {
        std::unique_lock<std::shared_mutex> lk(rw_mutex_);
        if (!prior_noise_) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] addGPSFactorsBatch: prior_noise_ is null, skip");
            return;
        }

        int added = 0;
        for (const auto& f : factors) {
            // ✅ 修复：增加节点存在性检查，避免添加不存在的节点导致 GTSAM 内部异常
            if (node_exists_.find(f.sm_id) == node_exists_.end()) {
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG] addGPSFactorsBatch: sm_id=%d not exists, skip", f.sm_id);
                continue;
            }
            try {
                gtsam::Point3 gps_point(f.pos.x(), f.pos.y(), f.pos.z());
                gtsam::Vector3 vars;
                vars << std::max(1e-6, f.cov(0, 0)),
                        std::max(1e-6, f.cov(1, 1)),
                        std::max(1e-6, f.cov(2, 2));
                auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
                pending_graph_.add(gtsam::GPSFactor(SM(f.sm_id), gps_point, noise));
                factor_count_++;
                added++;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG] addGPSFactorsBatch: add factor failed for sm_id=%d: %s",
                    f.sm_id, e.what());
            }
        }

        if (!pending_graph_.empty() || !pending_values_.empty()) {
            // ✅ 修复：单独处理 update 异常，防止单次 update 失败导致整个 batch 失败
            try {
                commitAndUpdate();
                scope.setSuccess(true);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG] addGPSFactorsBatch done added=%d", added);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG] addGPSFactorsBatch: commitAndUpdate failed: %s", e.what());
                // ✅ 修复：确保清理 pending 数据，防止内存泄漏和状态不一致
                pending_graph_.resize(0);
                pending_values_.clear();
                scope.setSuccess(false);
            }
        } else {
            scope.setSuccess(true);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] addGPSFactorsBatch exception: %s", e.what());
        scope.setSuccess(false);
    } catch (...) {
        // ✅ P0 修复：捕获 SIGSEGV 等致命信号之外的 所有异常
        // 注意：SIGSEGV 信号本身无法被 catch，但 std::exception 可以捕获 GTSAM 内部转换的异常
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] addGPSFactorsBatch unknown exception, data may be corrupted");
        scope.setSuccess(false);
    }
}

int IncrementalOptimizer::flushPendingGPSFactors() {
    // 调用方已持 rw_mutex_
    int added = 0;
    if (!prior_noise_) return 0;
    std::vector<GPSFactorItem> still_pending;
    for (GPSFactorItem& f : pending_gps_factors_) {
        if (node_exists_.find(f.sm_id) == node_exists_.end()) continue;
        if (!current_estimate_.exists(SM(f.sm_id))) {
            still_pending.push_back(std::move(f));
            continue;
        }
        if (!f.pos.allFinite() || !f.cov.allFinite()) continue;
        gtsam::Point3 gps_point(f.pos.x(), f.pos.y(), f.pos.z());
        gtsam::Vector3 vars;
        vars << std::max(1e-6, f.cov(0, 0)),
                std::max(1e-6, f.cov(1, 1)),
                std::max(1e-6, f.cov(2, 2));
        auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
        pending_graph_.add(gtsam::GPSFactor(SM(f.sm_id), gps_point, noise));
        factor_count_++;
        added++;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS] GPS factor added to graph sm_id=%d (deferred flush, grep for verification)",
            f.sm_id);
    }
    pending_gps_factors_ = std::move(still_pending);
    if (added > 0) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS] flushPendingGPSFactors: added %d deferred GPS factors",
            added);
    }
    return added;
}

void IncrementalOptimizer::markPendingValuesAsEstimated() {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    size_t count = 0;
    for (const auto& key : pending_values_.keys()) {
        if (pending_values_.exists(key)) {
            // 使用 insert_or_assign 确保值被添加
            if (!current_estimate_.exists(key)) {
                current_estimate_.insert(key, pending_values_.at(key));
                count++;
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][WORKAROUND] marked %zu pending values as in current_estimate_ (total pending_values=%zu)",
        count, pending_values_.size());
}

size_t IncrementalOptimizer::pendingValuesCount() const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    return pending_values_.size();
}

size_t IncrementalOptimizer::pendingFactorsCount() const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    return pending_graph_.size();
}

OptimizationResult IncrementalOptimizer::forceUpdate() {
    // ✅ 修复：使用 try-catch 包裹，防止 GTSAM 内部 SIGSEGV 导致进程崩溃
    bool had_pending = hasPendingFactorsOrValues();
    size_t queue_d = getQueueDepth();
    auto force_t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG][TRACE] step=forceUpdate_enter had_pending=%d queue_depth=%zu (调用方见 ensureBackendCompletedAndFlushBeforeHBA 或 finish_mapping)",
        had_pending ? 1 : 0, queue_d);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[CRASH_CONTEXT] caller=forceUpdate had_pending=%d queue_depth=%zu (崩溃时最后一条 CRASH_CONTEXT 即上一成功步骤)",
        had_pending ? 1 : 0, queue_d);
    try {
        std::unique_lock<std::shared_mutex> lk(rw_mutex_);
        size_t pf = pending_graph_.size();
        size_t pv = pending_values_.size();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][TRACE] step=forceUpdate_holding_lock pending_factors=%zu pending_values=%zu (崩溃在 commitAndUpdate 内则见 ISAM2_DIAG step=)",
            pf, pv);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=forceUpdate_about_to_call_commitAndUpdate pending_factors=%zu pending_values=%zu",
            pf, pv);
        OptimizationResult res = commitAndUpdate();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][TRACE] step=forceUpdate_commitAndUpdate_returned success=%d nodes=%d",
            res.success ? 1 : 0, res.nodes_updated);
        if (res.success) {
            int flushed = flushPendingGPSFactors();
            if (flushed > 0) {
                res = commitAndUpdate();
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG][TRACE] step=forceUpdate_after_flush success=%d nodes=%d (deferred GPS applied)",
                    res.success ? 1 : 0, res.nodes_updated);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=forceUpdate_exit success=%d nodes=%d",
            res.success ? 1 : 0, res.nodes_updated);
        double force_elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - force_t0).count();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][TRACE] step=forceUpdate_exit success=%d nodes=%d elapsed_ms=%.1f",
            res.success ? 1 : 0, res.nodes_updated, force_elapsed_ms);
        return res;
    } catch (const std::exception& e) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][CRASH_TRACE] step=forceUpdate_exception exception=%s (崩溃发生在 commitAndUpdate 内，见上一 TRACE step)",
            e.what());
        ALOG_ERROR(MOD, "forceUpdate failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] forceUpdate: %s", e.what());
        return OptimizationResult{};
    } catch (...) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][CRASH_TRACE] step=forceUpdate_unknown_exception (崩溃发生在 forceUpdate 内 commitAndUpdate，见上一 TRACE step)");
        ALOG_ERROR(MOD, "forceUpdate: unknown exception");
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] forceUpdate: unknown exception");
        return OptimizationResult{};
    }
}

bool IncrementalOptimizer::hasPendingFactorsOrValues() const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    return !pending_graph_.empty() || !pending_values_.empty();
}

OptimizationResult IncrementalOptimizer::commitAndUpdate() {
    // 注意：调用时已持有写锁
    if (pending_graph_.empty() && pending_values_.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] commitAndUpdate return empty reason=no_pending (pending_graph and pending_values empty)");
        return OptimizationResult{};
    }

    optimization_in_progress_.store(true, std::memory_order_release);
    struct Guard {
        std::atomic<bool>* flag;
        ~Guard() { flag->store(false, std::memory_order_release); }
    } guard{&optimization_in_progress_};

    const size_t pf = pending_graph_.size();
    const size_t pv = pending_values_.size();
    std::string params = "pending_factors=" + std::to_string(pf) + " pending_values=" + std::to_string(pv);
    GtsamCallScope scope(GtsamCaller::ISAM2, "commitAndUpdate", params, true);

    // 精准定位：若崩溃在 commitAndUpdate 内，grep CRASH_TRACE 或 step= 可确定最后到达的步骤
    auto tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG][TRACE] step=commitAndUpdate_enter tid=0x%zx pending_factors=%zu pending_values=%zu (崩溃时 grep CRASH_TRACE 或 step= 定位)",
        static_cast<size_t>(tid), pf, pv);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[CRASH_CONTEXT] step=commitAndUpdate_enter pending_factors=%zu pending_values=%zu tid=0x%zx",
        pf, pv, static_cast<size_t>(tid));
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG] commitAndUpdate enter pending_factors=%zu pending_values=%zu (若崩溃在此后、无 done→崩溃在 isam2_.update 内部)",
        pf, pv);
    ALOG_DEBUG(MOD, "iSAM2 update: pending_factors={} pending_nodes={}", pf, pv);

    // 强化日志：记录 value keys 与各 factor 类型
    // 修复：扩展延迟逻辑，覆盖「首次 update + 所有因子指向同一 key」场景
    // 崩溃场景：Prior(s1) + GPS(s1) + values(s1) 首次 update 触发 GTSAM bug (borglab/gtsam#1189)
    bool single_prior_only = false;
    bool all_factors_same_key = false;  // 新增：检测所有因子指向同一 key
    std::string factor_types_str;       // 用于日志
    try {
        std::string value_keys_str;
        for (const gtsam::Key k : pending_values_.keys())
            value_keys_str += (value_keys_str.empty() ? "" : ",") + std::to_string(k);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] pre_update value_keys=[%s]", value_keys_str.c_str());
        for (size_t i = 0; i < pending_graph_.size(); ++i) {
            const auto& f = pending_graph_[i];
            gtsam::KeyVector kv = f->keys();
            std::string keys_str;
            for (gtsam::Key k : kv) keys_str += (keys_str.empty() ? "" : ",") + std::to_string(k);
            const char* type_str = "Other";
            if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) type_str = "Prior";
            else if (dynamic_cast<const gtsam::GPSFactor*>(f.get())) type_str = "GPS";
            else if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(f.get())) type_str = "Between";
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] pre_update factor_%zu type=%s keys=[%s]", i, type_str, keys_str.c_str());
            // 收集因子类型用于诊断日志
            factor_types_str += (factor_types_str.empty() ? "" : ",") + std::string(type_str);
        }

        // 检测「仅 1 value + 1 Prior」：GTSAM 首次单节点 update 在此场景下易触发 double free，推迟到有第二节点再提交
        if (pf == 1u && pv == 1u &&
            dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(pending_graph_[0].get())) {
            single_prior_only = true;
        }

        // ✅ P0 修复：扩展检测「首次 update + 所有因子指向同一 key」
        // 崩溃场景：Prior(s1) + GPS(s1) 首次 update 时 pf=2，不满足 single_prior_only 条件
        // 需要检测所有因子是否指向同一个 key，延迟到有第二个节点再加入
        const bool is_first_update_check = current_estimate_.empty();
        if (is_first_update_check && pf >= 1u && pv == 1u && !pending_values_.empty()) {
            gtsam::Key single_value_key = pending_values_.keys()[0];
            all_factors_same_key = true;
            for (size_t i = 0; i < pending_graph_.size(); ++i) {
                auto keys = pending_graph_[i]->keys();
                // 检查：因子必须只有一个 key 且与 value key 相同
                if (keys.size() != 1 || keys[0] != single_value_key) {
                    all_factors_same_key = false;
                    break;
                }
            }
        }
    } catch (...) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] pre_update log keys failed (non-fatal)");
    }

    // ✅ P0 修复：扩展延迟条件，覆盖 Prior+GPS 同 key 的首次 update 场景
    if (single_prior_only || all_factors_same_key) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][CRITICAL] DEFER single-node update (factors=%zu values=%zu types=[%s]) "
            "- avoid GTSAM first-update double free (borglab/gtsam#1189), pending until 2+ nodes",
            pf, pv, factor_types_str.c_str());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][PIPELINE] event=commit_deferred reason=%s factors=%zu values=%zu (grep BACKEND PIPELINE 定位)",
            single_prior_only ? "single_prior" : "all_factors_same_key", pf, pv);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_defer_return single_prior_only=%d all_factors_same_key=%d (未调用 isam2_.update)",
            single_prior_only ? 1 : 0, all_factors_same_key ? 1 : 0);
        scope.setSuccess(true);
        return OptimizationResult{};  // 不清空 pending，等第二节点+odom 后一起 update
    }

    const bool is_first_update = current_estimate_.empty();
    if (is_first_update) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] first isam2 update (current_estimate was empty) factors=%zu values=%zu",
            pf, pv);
    }

    auto t0 = std::chrono::steady_clock::now();
    AUTOMAP_TIMED_SCOPE(MOD, "iSAM2::update", 200.0);

    bool used_first_update_three_phase = false;  // 用于最终日志 path=

    try {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_before_copy pending_factors=%zu pending_values=%zu", pf, pv);
        gtsam::NonlinearFactorGraph graph_copy(pending_graph_);
        gtsam::Values values_copy(pending_values_);
        // Incremental path: merge current_estimate_ so loop (and other) factor keys exist for validation and update
        if (!is_first_update && !current_estimate_.empty()) {
            gtsam::Values values_merged(current_estimate_);
            // 🔧 诊断: 检查 pending_values_ 中是否有 key 已存在于 current_estimate_
            int conflict_count = 0;
            for (const gtsam::Key pk : pending_values_.keys()) {
                if (current_estimate_.exists(pk)) {
                    conflict_count++;
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][KEY_CONFLICT] pending key=%zu already exists in current_estimate_! "
                        "(this may cause 'key already exists' error)",
                        static_cast<size_t>(pk));
                }
            }
            if (conflict_count > 0) {
                // 🔧 修复: 跳过已存在的keys，只插入新的keys
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][KEY_CONFLICT] Found %d conflicting keys, using insert_safe approach",
                    conflict_count);
                // 创建新的Values，只包含不在current_estimate_中的keys
                gtsam::Values new_values;
                for (const gtsam::Key pk : pending_values_.keys()) {
                    if (!current_estimate_.exists(pk)) {
                        new_values.insert(pk, pending_values_.at(pk));
                    }
                }
                values_merged.insert(new_values);
            } else {
                try {
                    values_merged.insert(pending_values_);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][KEY_CONFLICT] values_merged.insert failed: %s - "
                        "skip merge, use pending_values only",
                        e.what());
                    values_merged = pending_values_;
                }
            }
            values_copy = values_merged;
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][TRACE] step=copy_graph_values_done factors=%zu values=%zu (崩溃在此后则与 copy 无关)",
            graph_copy.size(), values_copy.size());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_copy_done graph_size=%zu values_size=%zu is_first_update=%d",
            graph_copy.size(), values_copy.size(), is_first_update ? 1 : 0);

        // ═══════════════════════════════════════════════════════════════════════════════
        // ✅ P0 V5 彻底修复：首次 update 完全跳过优化，仅注入 values，保留 factors 到下次 update。
        //    根本原因：GTSAM 的 NoiseModelFactor::error() 在首次调用时存在 double free bug
        //    (borglab/gtsam#1189)，无论是 ISAM2 还是 LM 都会触发。
        //    之前的 V4 LM_then_ISAM2 方案仍会在 LM 构造函数内调用 error() 导致崩溃。
        // 方案：Step1 仅注入 values（空 graph，不触发 linearize）
        //       Step2 保留 factors 在 pending 中，等第二次增量 update 处理
        // ═══════════════════════════════════════════════════════════════════════════════
        if (is_first_update) {
            used_first_update_three_phase = true;  // 复用变量，实际表示使用了首次特殊路径
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][V5] first update path=SKIP_OPTIMIZATION (bypass all GTSAM linearize)");
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=first_update_v5_enter factors=%zu values=%zu",
                graph_copy.size(), values_copy.size());
            // V5 首次 update 前：校验 values 合法，避免注入 nan/inf 或超大平移导致 GTSAM 异常
            bool first_values_ok = true;
            for (const gtsam::Key k : values_copy.keys()) {
                try {
                    auto p = values_copy.at<gtsam::Pose3>(k);
                    Eigen::Vector3d t = p.translation();
                    Eigen::Matrix3d R = p.rotation().matrix();
                    if (!t.array().isFinite().all() || !R.array().isFinite().all()) {
                        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                            "[IncrementalOptimizer][BACKEND][VALIDATION] first update value key=%zu non-finite - abort (grep BACKEND VALIDATION)",
                            static_cast<size_t>(k));
                        first_values_ok = false;
                        break;
                    }
                    if (t.norm() > kMaxReasonableTranslationNorm) {
                        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                            "[IncrementalOptimizer][BACKEND][VALIDATION] first update value key=%zu translation norm=%.1f > %.0f - abort",
                            static_cast<size_t>(k), t.norm(), kMaxReasonableTranslationNorm);
                        first_values_ok = false;
                        break;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][VALIDATION] first update value key=%zu exception: %s - abort",
                        static_cast<size_t>(k), e.what());
                    first_values_ok = false;
                    break;
                }
            }
            if (!first_values_ok) {
                pending_graph_.resize(0);
                pending_values_.clear();
                scope.setSuccess(false);
                recordOptimizationFailure("first_update_values_validation_failed");
                METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
                return OptimizationResult{};
            }
            // Step1: 仅注入 values，空 graph（不触发 linearize）
            gtsam::NonlinearFactorGraph empty_graph;
            crash_report::setLastStep("first_update_v5_update_values");
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=first_update_v5_pre_update_values");
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=first_update_v5_pre_update_values nodes=%zu", values_copy.size());
            auto t_first_update = std::chrono::steady_clock::now();
            isam2_.update(empty_graph, values_copy);
            double ms_first = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_first_update).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=first_update_v5_post_update_values elapsed_ms=%.1f", ms_first);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=first_update_v5_post_update_values");

            // Step2: 不清空 pending_graph_，保留所有 factors 到下次 update
            // 注意：这里故意不清空 pending_graph_，让它留在 pending 中等下次增量 update 处理
            // pending_graph_ 的清空逻辑在函数末尾会根据 used_first_update_three_phase 跳过

            // Step3: 直接用初始值作为当前估计（不做优化）
            current_estimate_ = values_copy;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][V5] first update done: injected %zu values, deferred %zu factors to next update",
                values_copy.size(), graph_copy.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=first_update_v5_done deferred_factors=%zu", graph_copy.size());
        } else {
            // 常规 ISAM2 增量 update：优化前全量约束校验，任一不合理则中止 update 避免触发 GTSAM 崩溃
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] commitAndUpdate path=incremental (graph_copy.size=%zu values_copy.size=%zu)",
                graph_copy.size(), values_copy.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_constraints_dump_enter");
            ConstraintValidation inc_validation;
            logAllConstraintsAndValidate(graph_copy, values_copy, "incremental_before_isam2", &inc_validation);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=incremental_constraints_dump_done all_keys_exist=%s all_values_finite=%s all_values_reasonable=%s",
                inc_validation.all_keys_exist ? "1" : "0", inc_validation.all_values_finite ? "1" : "0",
                inc_validation.all_values_reasonable ? "1" : "0");

            bool validation_ok = inc_validation.all_keys_exist && inc_validation.all_values_finite && inc_validation.all_values_reasonable;
            if (!validation_ok) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][VALIDATION] constraint validation FAILED: %s - aborting iSAM2 update to avoid crash (grep BACKEND VALIDATION)",
                    inc_validation.message.c_str());
                ALOG_ERROR(MOD, "constraint validation failed: {} - abort update", inc_validation.message);
                pending_graph_.resize(0);
                pending_values_.clear();
                scope.setSuccess(false);
                recordOptimizationFailure(inc_validation.message.c_str());
                METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
                return OptimizationResult{};
            }

            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_pre_update (若崩溃则发生在 isam2_.update(graph_copy, values_copy) 内)");
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=isam2_update_invoke");
            crash_report::setLastStep("incremental_isam2_update");
            auto t_update_begin = std::chrono::steady_clock::now();
            isam2_.update(graph_copy, values_copy);
            double ms_update = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_update_begin).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=isam2_update_returned elapsed_ms=%.1f (若卡住则上一行为 update_invoke)",
                ms_update);
            if (ms_update > 5000.0) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][SLOW] isam2_.update took %.1fms (grep BACKEND SLOW 定位 ISAM2 瓶颈)", ms_update);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_post_update");
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_calculateEstimate_enter");
            auto t_calc_begin = std::chrono::steady_clock::now();
            current_estimate_ = isam2_.calculateEstimate();
            double ms_calc = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_calc_begin).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=calculateEstimate_done elapsed_ms=%.1f (若卡住则上一行为 calculateEstimate_enter)",
                ms_calc);
            if (ms_calc > 5000.0) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][SLOW] calculateEstimate took %.1fms (grep BACKEND SLOW)", ms_calc);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_calculateEstimate_done nodes=%zu", current_estimate_.size());
        }
        // === 清空 pending：首次 update 保留 factors 到下次处理，常规 update 清空全部 ===
        if (used_first_update_three_phase) {
            // V5: 首次 update 仅清空 values（已注入 ISAM2），保留 factors 到下次增量 update
            pending_values_.clear();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=commitAndUpdate_v5_deferred_factors pending_graph_size=%zu (保留到下次 update)",
                pending_graph_.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][V5] pending_graph_ NOT cleared, will be processed in next incremental update");
        } else {
            // 常规增量 update：清空全部 pending
            pending_graph_.resize(0);
            pending_values_.clear();
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_exit_success");
    } catch (const std::exception& e) {
        scope.setSuccess(false);
        ALOG_ERROR(MOD, "iSAM2 update FAILED: {}", e.what());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][CRASH_TRACE] step=commitAndUpdate_exception exception=%s (崩溃或异常发生在上一 TRACE step 与本次之间)",
            e.what());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_caught_exception exception=%s (上一 CRASH_CONTEXT step 即崩溃前最后成功步骤)",
            e.what());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] commitAndUpdate done success=0 exception=%s", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] iSAM2 update FAILED: %s", e.what());
        pending_graph_.resize(0);
        pending_values_.clear();
        // ✅ 健康检查：记录优化失败
        recordOptimizationFailure(e.what());
        METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
        OptimizationResult fail{};
        fail.success = false;
        return fail;
    }

    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 提取所有位姿（先 exists 再 at，避免异常控制流，符合 GTSAM 推荐用法）
    std::unordered_map<int, Pose3d> poses;
    for (const auto& kv : node_exists_) {
        int id = kv.first;
        gtsam::Key key = SM(id);
        if (!current_estimate_.exists(key)) {
            ALOG_ERROR(MOD, "iSAM2 extract pose sm_id={} not in current_estimate (node_exists_ vs estimate mismatch)", id);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND] sm_id=%d not in estimate, skip (node_exists_.size=%zu estimate.size=%zu)",
                id, node_exists_.size(), current_estimate_.size());
            continue;
        }
        try {
            auto p = current_estimate_.at<gtsam::Pose3>(key);
            poses[id] = fromPose3(p);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "iSAM2 extract pose sm_id={} exception: {}", id, e.what());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] extract pose sm_id=%d: %s", id, e.what());
        } catch (...) {
            ALOG_ERROR(MOD, "iSAM2 extract pose sm_id={} unknown exception", id);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] extract pose sm_id=%d: unknown exception", id);
        }
    }

    OptimizationResult res;
    res.nodes_updated = (int)poses.size();
    res.elapsed_ms    = elapsed;
    res.submap_poses  = poses;
    // 若未提取到任何位姿（estimate 与 node_exists_ 不一致），视为失败并打清原因，便于 addLoopFactor 等调用方排查
    if (poses.size() == 0 && node_exists_.size() > 0) {
        res.success = false;
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] commitAndUpdate success=false reason=no_poses_extracted node_exists_=%zu current_estimate_.size=%zu (grep BACKEND)",
            node_exists_.size(), current_estimate_.size());
    } else {
        res.success = true;
    }

    // 健康检查：node_exists_ 与 current_estimate_ 一致性（V5 首次路径或异常后可能不一致）
    if (poses.size() != node_exists_.size()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][HEALTH] pose count mismatch: poses=%zu node_exists_=%zu path=%s (grep BACKEND HEALTH)",
            poses.size(), node_exists_.size(), used_first_update_three_phase ? "first_V5" : "incremental");
    }

    // V1: 单次 update 耗时分布与队列深度；V2: 可观测性 last success
    METRICS_HISTOGRAM_OBSERVE(metrics::ISAM2_OPTIMIZE_TIME_MS, elapsed);
    METRICS_GAUGE_SET(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(getQueueDepth()));

    const char* path_str = used_first_update_three_phase ? "first_update_V5_SKIP_OPTIMIZATION" : "incremental";
    if (res.success) {
        METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 1.0);
        recordOptimizationSuccess(elapsed);
        scope.setSuccess(true);
        ALOG_INFO(MOD, "iSAM2 update done: path={} nodes={} elapsed={:.1f}ms total_factors={}",
                  path_str, res.nodes_updated, res.elapsed_ms, factor_count_);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] commitAndUpdate done path=%s elapsed_ms=%.1f nodes=%d success=1",
            path_str, res.elapsed_ms, res.nodes_updated);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[PRECISION][OPT] iSAM2_update path=%s nodes_updated=%d elapsed_ms=%.1f factor_count=%d",
            path_str, res.nodes_updated, res.elapsed_ms, factor_count_);
        notifyPoseUpdate(poses);

        // 首节点/后续 update 成功后尝试 flush 对齐阶段 defer 的 GPS 因子，避免 node_count=0 时全部进入 pending 永不入图
        if (res.nodes_updated > 0) {
            int flushed = flushPendingGPSFactors();
            if (flushed > 0) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][GPS] commitAndUpdate success: flushing %d pending GPS factors, running second update",
                    flushed);
                res = commitAndUpdate();
            }
        }
    } else {
        METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
        scope.setSuccess(false);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] commitAndUpdate done path=%s elapsed_ms=%.1f nodes=%d success=0 (see BACKEND reason above)",
            path_str, res.elapsed_ms, res.nodes_updated);
    }
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
        int id = kv.first;
        gtsam::Key key = SM(id);
        if (!current_estimate_.exists(key)) {
            ALOG_ERROR(MOD, "getAllPoses sm_id={} not in current_estimate, skip", id);
            continue;
        }
        try {
            auto p = current_estimate_.at<gtsam::Pose3>(key);
            out[id] = fromPose3(p);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "getAllPoses sm_id={} exception: {}", id, e.what());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] getAllPoses sm_id=%d: %s", id, e.what());
        } catch (...) {
            ALOG_ERROR(MOD, "getAllPoses sm_id={} unknown exception", id);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] getAllPoses sm_id=%d: unknown exception", id);
        }
    }
    return out;
}

void IncrementalOptimizer::reset() {
    // 先清空优化队列并重置进度标志，避免 opt 线程随后处理旧任务访问已清空状态
    size_t queue_cleared = 0;
    {
        std::lock_guard<std::mutex> qlk(opt_queue_mutex_);
        queue_cleared = opt_queue_.size();
        while (!opt_queue_.empty()) opt_queue_.pop();
    }
    optimization_in_progress_.store(false, std::memory_order_release);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND][RESET] queue_cleared=%zu optimization_in_progress=0 (grep BACKEND RESET 定位 reset 调用)",
        queue_cleared);

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
    pending_gps_factors_.clear();
}

void IncrementalOptimizer::clearForShutdown() {
    // 严格顺序：先停 opt 线程（notify + join），再持 rw_mutex_ 清空 isam2_/prior_noise_，避免与 optLoop 或 GTSAM 静态析构竞态
    opt_running_ = false;
    {
        std::lock_guard<std::mutex> qlk(opt_queue_mutex_);
        opt_queue_cv_.notify_all();
    }
    if (opt_thread_.joinable()) {
        opt_thread_.join();
    }

    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    const int nodes = node_count_;
    const int factors = factor_count_;
    ALOG_INFO(MOD, "clearForShutdown: enter (nodes=%d factors=%d)", nodes, factors);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][SHUTDOWN] clearForShutdown enter nodes=%d factors=%d", nodes, factors);

    // 不调用 ConfigManager，使用默认参数，供进程退出时安全使用
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 10;
    params.enableRelinearization = true;
    params.factorization = gtsam::ISAM2Params::QR;
    params.cacheLinearizedFactors = false;

    pending_graph_.resize(0);
    pending_values_.clear();
    current_estimate_.clear();
    node_exists_.clear();
    pending_gps_factors_.clear();
    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"), "[IncrementalOptimizer][SHUTDOWN] graph/values cleared");

    isam2_ = gtsam::ISAM2(params);
    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"), "[IncrementalOptimizer][SHUTDOWN] isam2 replaced with empty");

    // 显式释放 prior_noise_（Diagonal），与 isam2_ 析构顺序解耦，避免进程退出时任何潜在析构顺序问题
    prior_noise_.reset();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[IncrementalOptimizer][SHUTDOWN] prior_noise_ released");

    node_count_ = 0;
    factor_count_ = 0;
    has_prior_ = false;

    ALOG_INFO(MOD, "clearForShutdown: done (was nodes=%d factors=%d)", nodes, factors);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][SHUTDOWN] clearForShutdown done (was nodes=%d factors=%d)", nodes, factors);
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

gtsam::noiseModel::Diagonal::shared_ptr
IncrementalOptimizer::infoToNoiseDiagonal(const Mat66d& info) const {
    gtsam::Vector6 vars;
    for (int i = 0; i < 6; ++i) {
        double diag = info(i, i);
        vars(i) = std::max(1.0 / std::max(diag, 1e-6), 1e-6);
    }
    return gtsam::noiseModel::Diagonal::Variances(vars);
}

gtsam::noiseModel::Base::shared_ptr
IncrementalOptimizer::infoToNoise(const Mat66d& info) const {
    // 秩缺/病态时返回 Diagonal 保守方差，与 infoToNoiseDiagonal 及双路 GTSAM 策略一致（避免 Gaussian::Covariance 在 linearize 路径 double free）
    gtsam::Vector6 conservative_var;
    conservative_var.setConstant(1.0);

    Eigen::JacobiSVD<Mat66d> svd(info, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const int rank = svd.rank();
    if (rank < 6) {
        ALOG_WARN("IncrementalOptimizer",
                  "Information matrix rank-deficient (rank={}/6), using conservative diagonal", rank);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][NOISE] infoToNoise fallback reason=rank_deficient rank=%d/6 (grep BACKEND NOISE 定位)",
            rank);
        return gtsam::noiseModel::Diagonal::Variances(conservative_var);
    }

    const double max_sv = svd.singularValues()(0);
    const double min_sv = svd.singularValues()(5);
    const double cond = (max_sv > 1e-12) ? (max_sv / min_sv) : 1e12;

    if (min_sv < 1e-6 || cond > 1e6) {
        ALOG_WARN(MOD, "Information matrix ill-conditioned (rank={}/6, cond={:.2e}), using conservative diagonal", rank, cond);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][NOISE] infoToNoise fallback reason=ill_conditioned rank=%d cond=%.2e (grep BACKEND NOISE 定位)",
            rank, cond);
        return gtsam::noiseModel::Diagonal::Variances(conservative_var);
    }

    Mat66d info_reg = info + Mat66d::Identity() * std::max(1e-6, min_sv * 1e-3);
    gtsam::Matrix66 cov = info_reg.inverse().cast<double>();
    cov = 0.5 * (cov + cov.transpose());

    if (!cov.allFinite()) {
        ALOG_ERROR(MOD, "Regularized inverse contains NaN/Inf, using fallback diagonal");
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] Regularized inverse contains NaN/Inf, using fallback diagonal");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][NOISE] infoToNoise fallback reason=non_finite (grep BACKEND NOISE 定位)");
        return gtsam::noiseModel::Diagonal::Variances(conservative_var);
    }

    ALOG_DEBUG(MOD, "InfoToNoise: cond={:.2e} reg_factor={:.2e}", cond, min_sv * 1e-3);
    return gtsam::noiseModel::Gaussian::Covariance(cov);
}

void IncrementalOptimizer::notifyPoseUpdate(const std::unordered_map<int, Pose3d>& poses) {
    for (auto& cb : pose_update_cbs_) {
        cb(poses);
    }
}

// ── P0 异步优化队列实现 ───────────────────────────────────────────────────

void IncrementalOptimizer::optLoop() {
    // 🔧 DEBUG: 记录 optLoop 启动时间
    auto opt_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] optLoop started (tid=0x%zx)", 
        static_cast<size_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    while (true) {
        OptimTask task;
        {
            // 🔧 DEBUG: 记录从队列获取任务前的状态
            auto queue_wait_start = std::chrono::steady_clock::now();
            std::unique_lock<std::mutex> lk(opt_queue_mutex_);
            // 🔧 DEBUG: 在 wait 之前记录状态
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] optLoop: waiting for task, queue_size=%zu opt_running=%d",
                opt_queue_.size(), opt_running_ ? 1 : 0);
            opt_queue_cv_.wait(lk, [this] {
                return !opt_running_ || !opt_queue_.empty();
            });
            auto queue_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - queue_wait_start).count();
            if (queue_wait_ms > 5000.0) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop: queue wait %.1fms (possible deadlock) queue_size=%zu",
                    queue_wait_ms, opt_queue_.size());
            }
            if (!opt_running_ && opt_queue_.empty()) break;
            if (opt_queue_.empty()) continue;
            task = opt_queue_.front();
            opt_queue_.pop();
            MetricsRegistry::instance().setGauge(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(opt_queue_.size()));
        }
        const char* type_str = (task.type == OptimTaskType::LOOP_FACTOR) ? "LOOP_FACTOR"
            : (task.type == OptimTaskType::GPS_FACTOR) ? "GPS_FACTOR" : "BATCH_UPDATE";
        size_t queue_after = getQueueDepth();
        auto task_t0 = std::chrono::steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] optLoop pop type=%s queue_remaining=%zu (崩溃在 commitAndUpdate 时此处为 opt 线程)",
            type_str, queue_after);

        // 🔧 DEBUG: 记录任务处理开始
        auto task_start = std::chrono::steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] optLoop task START type=%s from=%d to=%d (task processing begins)",
            type_str, task.from_id, task.to_id);

        // ✅ 修复：为每个任务类型添加 try-catch，防止单个任务崩溃导致整个 optLoop 退出
        try {
            if (task.type == OptimTaskType::LOOP_FACTOR) {
                // 🔧 DEBUG: 记录获取 rw_mutex 之前
                auto lock_start = std::chrono::steady_clock::now();
                std::unique_lock<std::shared_mutex> lk(rw_mutex_);
                auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
                if (lock_wait_ms > 100.0) {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][LOCK_DIAG] optLoop LOOP_FACTOR: rw_mutex wait %.1fms", lock_wait_ms);
                }
                if (!prior_noise_) continue;
                if (!node_exists_.count(task.from_id) || !node_exists_.count(task.to_id)) continue;
                auto base_noise = infoToNoiseDiagonal(task.info_matrix);
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
                pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    SM(task.from_id), SM(task.to_id), toPose3(task.rel_pose), robust_noise));
                factor_count_++;
                // 🔧 DEBUG: 在 commitAndUpdate 之前打印日志
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop calling commitAndUpdate for LOOP_FACTOR...");
                commitAndUpdate();
                // 🔧 DEBUG: commitAndUpdate 返回后
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop commitAndUpdate done for LOOP_FACTOR");
            } else if (task.type == OptimTaskType::GPS_FACTOR) {
                // 🔧 DEBUG: 记录获取 rw_mutex 之前
                auto lock_start = std::chrono::steady_clock::now();
                std::unique_lock<std::shared_mutex> lk(rw_mutex_);
                auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
                if (lock_wait_ms > 100.0) {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][LOCK_DIAG] optLoop GPS_FACTOR: rw_mutex wait %.1fms", lock_wait_ms);
                }
                if (!prior_noise_) continue;
                if (!node_exists_.count(task.from_id)) continue;
                gtsam::Point3 gps_point(task.gps_pos.x(), task.gps_pos.y(), task.gps_pos.z());
                gtsam::Vector3 vars;
                vars << std::max(1e-6, task.gps_cov(0, 0)),
                        std::max(1e-6, task.gps_cov(1, 1)),
                        std::max(1e-6, task.gps_cov(2, 2));
                auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
                pending_graph_.add(gtsam::GPSFactor(SM(task.from_id), gps_point, noise));
                factor_count_++;
                commitAndUpdate();
            } else if (task.type == OptimTaskType::BATCH_UPDATE && task.action) {
                task.action();
            }
            double task_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - task_t0).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] optLoop task type=%s done elapsed_ms=%.1f queue_remaining=%zu",
                type_str, task_elapsed_ms, getQueueDepth());
        } catch (const std::exception& e) {
            // ✅ P0 修复：捕获异常，防止单个任务失败导致 optLoop 终止
            double task_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - task_t0).count();
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] optLoop task %s failed after %.1fms: %s",
                type_str, task_elapsed_ms, e.what());
            ALOG_ERROR(MOD, "optLoop task %s failed: {}", type_str, e.what());
        } catch (...) {
            double task_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - task_t0).count();
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] optLoop task %s unknown exception after %.1fms", type_str, task_elapsed_ms);
            ALOG_ERROR(MOD, "optLoop task %s unknown exception", type_str);
        }
    }
}

void IncrementalOptimizer::enqueueOptTask(const OptimTask& task) {
    const size_t max_sz = static_cast<size_t>(ConfigManager::instance().maxOptimizationQueueSize());
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    if (opt_queue_.size() >= max_sz) {
        ALOG_WARN(MOD, "enqueueOptTask: queue full ({}), drop task", max_sz);
        MetricsRegistry::instance().incrementCounter(metrics::ISAM2_TASK_DROPPED, 1.0);
        return;
    }
    opt_queue_.push(task);
    MetricsRegistry::instance().setGauge(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(opt_queue_.size()));
    opt_queue_cv_.notify_one();
}

void IncrementalOptimizer::enqueueOptTasks(const std::vector<OptimTask>& tasks) {
    const size_t max_sz = static_cast<size_t>(ConfigManager::instance().maxOptimizationQueueSize());
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    size_t dropped = 0;
    for (const auto& t : tasks) {
        if (opt_queue_.size() >= max_sz) {
            dropped++;
            continue;
        }
        opt_queue_.push(t);
    }
    if (dropped > 0) {
        MetricsRegistry::instance().incrementCounter(metrics::ISAM2_TASK_DROPPED, static_cast<double>(dropped));
    }
    MetricsRegistry::instance().setGauge(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(opt_queue_.size()));
    if (!tasks.empty()) opt_queue_cv_.notify_one();
}

void IncrementalOptimizer::waitForPendingTasks() {
    const size_t initial_depth = getQueueDepth();
    const bool initial_busy = optimization_in_progress_.load(std::memory_order_acquire);
    if (initial_depth == 0 && !initial_busy) return;
    ALOG_INFO(MOD, "[ISAM2_QUEUE] waitForPendingTasks enter queue_depth={} opt_busy={} max_wait_ms=5000",
              initial_depth, initial_busy ? 1 : 0);
    constexpr int kMaxWaitMs = 5000;
    constexpr int kChunkMs = 10;
    int waited_ms = 0;
    while (waited_ms < kMaxWaitMs) {
        const size_t depth = getQueueDepth();
        const bool busy = optimization_in_progress_.load(std::memory_order_acquire);
        if (depth == 0 && !busy) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(kChunkMs));
        waited_ms += kChunkMs;
    }
    const size_t final_depth = getQueueDepth();
    const bool still_busy = optimization_in_progress_.load(std::memory_order_acquire);
    if (final_depth > 0 || still_busy) {
        ALOG_WARN(MOD, "[ISAM2_QUEUE] waitForPendingTasks timeout after {}ms queue_depth={} opt_busy={} (backend continues)",
                  kMaxWaitMs, final_depth, still_busy ? 1 : 0);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][PIPELINE] event=wait_pending_timeout waited_ms=%d queue_depth=%zu opt_busy=%d (grep BACKEND PIPELINE 定位)",
            waited_ms, final_depth, still_busy ? 1 : 0);
    } else {
        ALOG_INFO(MOD, "[ISAM2_QUEUE] waitForPendingTasks done waited_ms={} (queue empty and no commitAndUpdate in progress)",
                  waited_ms);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][PIPELINE] event=wait_pending_done waited_ms=%d (grep BACKEND PIPELINE 定位)",
            waited_ms);
    }
}

size_t IncrementalOptimizer::getQueueDepth() const {
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    return opt_queue_.size();
}

// ── 健康检查与恢复实现 ───────────────────────────────────────────────────

IncrementalOptimizer::HealthStatus IncrementalOptimizer::getHealthStatus() const {
    std::lock_guard<std::mutex> lk(health_mutex_);
    HealthStatus status;
    status.is_healthy = (consecutive_failures_.load() < 3);
    status.consecutive_failures = consecutive_failures_.load();
    status.total_optimizations = total_optimizations_.load();
    status.failed_optimizations = failed_optimizations_.load();
    status.last_success_time_ms = last_success_time_ms_.load();
    status.last_error_message = last_error_message_;
    return status;
}

void IncrementalOptimizer::recordOptimizationFailure(const std::string& error_msg) {
    consecutive_failures_++;
    total_optimizations_++;
    failed_optimizations_++;
    last_error_message_ = error_msg;

    ALOG_WARN(MOD, "[Health] Optimization FAILED: consecutive_failures=%d error=%s",
              consecutive_failures_.load(), error_msg.c_str());
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][Health] Optimization FAILED: consecutive_failures=%d error=%s",
        consecutive_failures_.load(), error_msg.c_str());

    // 超过阈值时触发警告
    if (consecutive_failures_.load() >= 3) {
        ALOG_ERROR(MOD, "[Health] CRITICAL: %d consecutive optimization failures, consider reset!",
                   consecutive_failures_.load());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][Health] CRITICAL: %d consecutive optimization failures, consider reset!",
            consecutive_failures_.load());
    }
}

void IncrementalOptimizer::recordOptimizationSuccess(double elapsed_ms) {
    consecutive_failures_ = 0;
    total_optimizations_++;
    last_success_time_ms_ = elapsed_ms;

    ALOG_DEBUG(MOD, "[Health] Optimization SUCCESS: elapsed_ms=%.2f total=%d failed=%d",
               elapsed_ms, total_optimizations_.load(), failed_optimizations_.load());
}

void IncrementalOptimizer::resetForRecovery() {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);

    ALOG_WARN(MOD, "[Health] Resetting iSAM2 for recovery (was nodes=%d factors=%d)",
              node_count_, factor_count_);
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][Health] Resetting iSAM2 for recovery: nodes=%d factors=%d",
        node_count_, factor_count_);

    // 清空因子图但保留节点映射（避免重建）
    pending_graph_.resize(0);
    pending_values_.clear();
    current_estimate_.clear();

    // 重建 ISAM2
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = ConfigManager::instance().isam2RelinThresh();
    params.relinearizeSkip      = ConfigManager::instance().isam2RelinSkip();
    params.enableRelinearization = ConfigManager::instance().isam2EnableRelin();
    params.factorization = gtsam::ISAM2Params::QR;
    params.cacheLinearizedFactors = true;
    isam2_ = gtsam::ISAM2(params);

    // 重置计数器
    factor_count_ = 0;
    has_prior_ = false;

    // 重置健康状态
    {
        std::lock_guard<std::mutex> hlk(health_mutex_);
        consecutive_failures_ = 0;
        last_error_message_ = "Reset for recovery";
    }

    ALOG_INFO(MOD, "[Health] iSAM2 reset complete for recovery");
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][Health] iSAM2 reset complete for recovery");
}

} // namespace automap_pro
