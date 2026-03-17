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
/** 建图后端每步计算日志：grep BACKEND_STEP 可追踪每步输入/输出/结果，便于定位计算错误与逻辑漏洞 */
#define BACKEND_STEP(fmt, ...) \
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[BACKEND_STEP] " fmt, ##__VA_ARGS__)
// 约束汇总：grep [CONSTRAINT] 可快速定位 先验/里程计/回环/GPS 的添加结果（ok/skip/defer）
#define CONSTRAINT_LOG(fmt, ...) \
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[CONSTRAINT] " fmt, ##__VA_ARGS__)

#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <set>
#include <string>

namespace automap_pro {

// GTSAM Symbol 约定：s(sm_id) = Symbol('s', sm_id)
static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }
// GTSAM Symbol 约定：x(kf_id) = Symbol('x', kf_id) - keyframe级别节点
static gtsam::Symbol KF(int id) { return gtsam::Symbol('x', id); }

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
    // ===== 修复2: 初始化单节点pending开始时间 =====
    single_node_pending_start_ = std::chrono::steady_clock::now();

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
    BACKEND_STEP("step=addSubMapNode_enter sm_id=%d fixed=%d nodes_before=%d factor_count=%d",
        sm_id, fixed ? 1 : 0, node_count_, factor_count_);
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addSubMapNode: rw_mutex wait %.1fms (potential contention)", lock_wait_ms);
    }

    // 🔧 诊断: 记录当前优化器状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] addSubMapNode ENTER: sm_id=%d fixed=%d "
        "node_exists_ size=%zu pending_values=%zu pending_factors=%zu current_estimate_=%zu "
        "consecutive_failures=%d",
        sm_id, fixed, node_exists_.size(), pending_values_.size(),
        pending_graph_.size(), current_estimate_.size(), consecutive_failures_.load());

    if (!prior_noise_) {
        ALOG_DEBUG(MOD, "addSubMapNode: already shutdown (prior_noise_ released), ignore");
        return;
    }
    if (node_exists_.count(sm_id)) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] addSubMapNode: sm_id=%d already exists in node_exists_",
            sm_id);

        // 🔧 修复: 添加节点前检查是否需要恢复
        if (consecutive_failures_.load() >= 3) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND] addSubMapNode: %d consecutive failures, triggering recovery before adding sm_id=%d",
                consecutive_failures_.load(), sm_id);
            // 重建 ISAM2
            pending_graph_.resize(0);
            pending_values_.clear();
            current_estimate_.clear();
            gtsam::ISAM2Params params;
            params.relinearizeThreshold = ConfigManager::instance().isam2RelinThresh();
            params.relinearizeSkip = ConfigManager::instance().isam2RelinSkip();
            params.enableRelinearization = ConfigManager::instance().isam2EnableRelin();
            params.factorization = gtsam::ISAM2Params::QR;
            params.cacheLinearizedFactors = true;
            isam2_ = gtsam::ISAM2(params);
            node_exists_.clear();
            node_count_ = 0;
            factor_count_ = 0;
            has_prior_ = false;
            consecutive_failures_ = 0;
            ALOG_WARN(MOD, "[Health] Auto-recovery triggered before addSubMapNode: reset iSAM2");
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] addSubMapNode: after recovery - node_exists_ size=%zu",
                node_exists_.size());
        }
        return;
    }

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

    // ===== 修复1: 添加子图节点后立即尝试刷新pending的GPS因子 =====
    // 解决GPS因子延迟累积问题：在子图节点添加到优化器后，立即检查并刷新pending的GPS因子
    if (!pending_gps_factors_.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS] addSubMapNode: sm_id=%d, checking pending GPS factors, count=%zu",
            sm_id, pending_gps_factors_.size());

        // 刷新该sm_id对应的GPS因子
        int flushed = 0;
        std::vector<GPSFactorItem> still_pending;
        for (GPSFactorItem& f : pending_gps_factors_) {
            if (f.sm_id != sm_id) {
                still_pending.push_back(std::move(f));
                continue;
            }
            if (!f.pos.allFinite() || !f.cov.allFinite()) {
                still_pending.push_back(std::move(f));
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
                flushed++;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][GPS] addSubMapNode: flushed GPS factor for sm_id=%d",
                    f.sm_id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][GPS] addSubMapNode: failed to flush GPS factor for sm_id=%d: %s",
                    f.sm_id, e.what());
                still_pending.push_back(std::move(f));
            }
        }
        pending_gps_factors_ = std::move(still_pending);

        if (flushed > 0) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][GPS] addSubMapNode: sm_id=%d flushed %d GPS factors, triggering optimization",
                sm_id, flushed);
            // 注意：这里不直接调用commitAndUpdate，因为addSubMapNode是在持有锁的状态
            // 调用方会在适当时候触发forceUpdate
        }
    }
    BACKEND_STEP("step=addSubMapNode_done sm_id=%d nodes_after=%d pending_factors=%zu pending_values=%zu result=ok",
        sm_id, node_count_, pending_graph_.size(), pending_values_.size());
    CONSTRAINT_LOG("step=submap_node sm_id=%d result=ok has_prior=%d factor_count=%d (grep CONSTRAINT 定位约束)",
        sm_id, has_prior_ ? 1 : 0, factor_count_);
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

    BACKEND_STEP("step=addOdomFactor_enter from=%d to=%d rel_norm=%.4f node_count=%zu estimate_size=%zu",
        from, to, rel.translation().norm(), node_exists_.size(), current_estimate_.size());
    // 🔧 诊断: 记录当前状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] addOdomFactor ENTER: from=%d to=%d "
        "node_exists_ size=%zu current_estimate_=%zu consecutive_failures=%d",
        from, to, node_exists_.size(), current_estimate_.size(), consecutive_failures_.load());

    if (!node_exists_.count(from) || !node_exists_.count(to)) {
        BACKEND_STEP("step=addOdomFactor_skip from=%d to=%d reason=node_not_exists", from, to);
        CONSTRAINT_LOG("step=odom from=%d to=%d result=skip reason=node_not_exists (grep CONSTRAINT 定位)", from, to);
        // 🔧 诊断: 详细记录节点不存在的情况
        bool from_in_estimate = current_estimate_.exists(SM(from));
        bool to_in_estimate = current_estimate_.exists(SM(to));
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][ODOM] skip addOdomFactor from=%d to=%d "
            "(from_exists=%d to_exists=%d node_count=%zu) "
            "from_in_estimate=%d to_in_estimate=%d",
            from, to,
            node_exists_.count(from) ? 1 : 0,
            node_exists_.count(to) ? 1 : 0,
            node_count_,
            from_in_estimate ? 1 : 0,
            to_in_estimate ? 1 : 0);
        return;
    }

    // 约束合理性：拒绝非法 rel/info 进入 pending，避免后续 update 触发 GTSAM 异常
    const Eigen::Vector3d& t = rel.translation();
    const Eigen::Matrix3d& R = rel.rotation();
    if (!t.allFinite() || !R.allFinite()) {
        BACKEND_STEP("step=addOdomFactor_skip from=%d to=%d reason=rel_non_finite", from, to);
        CONSTRAINT_LOG("step=odom from=%d to=%d result=skip reason=rel_non_finite", from, to);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addOdomFactor from=%d to=%d rel non-finite, skip",
            from, to);
        return;
    }
    if (t.norm() > kMaxReasonableTranslationNorm) {
        BACKEND_STEP("step=addOdomFactor_skip from=%d to=%d reason=translation_too_large norm=%.1f", from, to, t.norm());
        CONSTRAINT_LOG("step=odom from=%d to=%d result=skip reason=translation_too_large norm=%.1f", from, to, t.norm());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][VALIDATION] addOdomFactor from=%d to=%d translation norm=%.1f > %.0f, skip",
            from, to, t.norm(), kMaxReasonableTranslationNorm);
        return;
    }
    if (!info_matrix.allFinite()) {
        BACKEND_STEP("step=addOdomFactor_skip from=%d to=%d reason=info_matrix_non_finite", from, to);
        CONSTRAINT_LOG("step=odom from=%d to=%d result=skip reason=info_matrix_non_finite", from, to);
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

    BACKEND_STEP("step=addOdomFactor_added from=%d to=%d pending_factors=%zu result=ok", from, to, pending_graph_.size());
    CONSTRAINT_LOG("step=odom from=%d to=%d result=ok factor_count=%d (子图间里程计约束)", from, to, factor_count_);
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
    BACKEND_STEP("step=addLoopFactor_enter from=%d to=%d rel_norm=%.4f node_count=%zu",
        from, to, rel.translation().norm(), node_count_);
    // ✅ 修复：使用 try-catch 包裹，防止 GTSAM 内部 SIGSEGV 导致进程崩溃（已持 lk）
    try {
        if (!prior_noise_) return OptimizationResult{};

        // 同节点回环无效：BetweenFactor(from, to) 当 from==to 时退化，且易导致 commitAndUpdate 异常
        if (from == to) {
            BACKEND_STEP("step=addLoopFactor_skip from=%d to=%d reason=same_node", from, to);
            CONSTRAINT_LOG("step=loop_inter from=%d to=%d result=skip reason=same_node (grep CONSTRAINT 定位)", from, to);
            ALOG_DEBUG(MOD, "addLoopFactor: from==to=%d (same node), skip degenerate loop", from);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][LOOP] skip addLoopFactor from=%d to=%d (same node, invalid Between factor; grep BACKEND LOOP)",
                from, to);
            return OptimizationResult{};
        }

        // ✅ 修复：显式检查节点存在性
        if (node_exists_.find(from) == node_exists_.end() ||
            node_exists_.find(to) == node_exists_.end()) {
            BACKEND_STEP("step=addLoopFactor_skip from=%d to=%d reason=node_not_in_graph", from, to);
            CONSTRAINT_LOG("step=loop_inter from=%d to=%d result=skip reason=node_not_in_graph", from, to);
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
            CONSTRAINT_LOG("step=loop_inter from=%d to=%d result=skip reason=rel_non_finite", from, to);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addLoopFactor from=%d to=%d rel non-finite, skip",
                from, to);
            return OptimizationResult{};
        }
        if (rel_t.norm() > kMaxReasonableTranslationNorm) {
            CONSTRAINT_LOG("step=loop_inter from=%d to=%d result=skip reason=translation_too_large norm=%.1f", from, to, rel_t.norm());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addLoopFactor from=%d to=%d translation norm=%.1f > %.0f, skip",
                from, to, rel_t.norm(), kMaxReasonableTranslationNorm);
            return OptimizationResult{};
        }
        if (!info_matrix.allFinite()) {
            CONSTRAINT_LOG("step=loop_inter from=%d to=%d result=skip reason=info_non_finite", from, to);
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
        CONSTRAINT_LOG("step=loop_inter from=%d to=%d result=ok factor_count=%d (子图间回环，即将 commit)", from, to, factor_count_);
        BACKEND_STEP("step=addLoopFactor_added from=%d to=%d pending_factors=%zu invoking_commit", from, to, pending_graph_.size());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][LOOP] loop constraint added to graph from=%d to=%d (grep for verification)",
            from, to);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[LOOP_ACCEPTED] addLoopFactor success from=%d to=%d (点云重影排查：无此条=零回环=结构重影；grep LOOP_ACCEPTED)",
            from, to);
        // 回环因子立即触发 iSAM2 update
        OptimizationResult res = commitAndUpdate();
        BACKEND_STEP("step=addLoopFactor_done from=%d to=%d commit_success=%d nodes_updated=%d elapsed_ms=%.1f",
            from, to, res.success ? 1 : 0, res.nodes_updated, res.elapsed_ms);
        return res;
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
    BACKEND_STEP("step=addLoopFactorDeferred_enter from=%d to=%d (子图内回环 deferred)", from, to);
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addLoopFactorDeferred: rw_mutex wait %.1fms from=%d to=%d", lock_wait_ms, from, to);
    }
    try {
        if (!prior_noise_) {
            CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=prior_noise_null", from, to);
            return;
        }
        if (from == to) {
            CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=same_node", from, to);
            return;
        }
        if (node_exists_.find(from) == node_exists_.end() || node_exists_.find(to) == node_exists_.end()) {
            CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=node_not_in_graph", from, to);
            return;
        }
        const Eigen::Vector3d& rel_t = rel.translation();
        const Eigen::Matrix3d& rel_R = rel.rotation();
        if (!rel_t.allFinite() || !rel_R.allFinite()) {
            CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=rel_non_finite", from, to);
            return;
        }
        if (rel_t.norm() > kMaxReasonableTranslationNorm) {
            CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=translation_too_large", from, to);
            return;
        }
        if (!info_matrix.allFinite()) {
            CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=info_non_finite", from, to);
            return;
        }
        auto base_noise = infoToNoiseDiagonal(info_matrix);
        gtsam::noiseModel::Base::shared_ptr robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
        pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(SM(from), SM(to), toPose3(rel), robust_noise));
        factor_count_++;
        BACKEND_STEP("step=addLoopFactorDeferred_added from=%d to=%d pending_factors=%zu result=ok", from, to, pending_graph_.size());
        CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=ok factor_count=%d (子图内回环 deferred)", from, to, factor_count_);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][LOOP] loop constraint added (deferred) from=%d to=%d (single commit later)",
            from, to);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[LOOP_ACCEPTED] addLoopFactorDeferred success from=%d to=%d (点云重影排查：无此条=零子图内回环；grep LOOP_ACCEPTED)",
            from, to);
    } catch (const std::exception& e) {
        CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=exception %s", from, to, e.what());
        ALOG_ERROR(MOD, "addLoopFactorDeferred failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] addLoopFactorDeferred from=%d to=%d: %s", from, to, e.what());
    } catch (...) {
        CONSTRAINT_LOG("step=loop_intra from=%d to=%d result=skip reason=unknown_exception", from, to);
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
    BACKEND_STEP("step=addGPSFactor_enter sm_id=%d node_count=%zu estimate_size=%zu",
        sm_id, node_exists_.size(), current_estimate_.size());
    // ✅ 修复：仅添加 GPS 因子，不立即 commit
    // 问题根源是 GTSAM 内部 TBB 并行导致竞态，延迟 commit 可以减少并发
    try {
        // 第359行已经获取了锁，不需要重复获取
        if (!prior_noise_) return;

        // ✅ 修复：显式检查节点存在性；若节点尚未加入（如 GPS 对齐早于首帧 commit），加入 deferred 稍后 flush
        if (node_exists_.find(sm_id) == node_exists_.end()) {
            BACKEND_STEP("step=addGPSFactor_defer sm_id=%d reason=node_not_exists pending_gps=%zu", sm_id, pending_gps_factors_.size() + 1);
            CONSTRAINT_LOG("step=gps_submap sm_id=%d result=defer reason=node_not_exists pending=%zu", sm_id, pending_gps_factors_.size() + 1);
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
            BACKEND_STEP("step=addGPSFactor_defer sm_id=%d reason=not_in_estimate pending_gps=%zu", sm_id, pending_gps_factors_.size() + 1);
            CONSTRAINT_LOG("step=gps_submap sm_id=%d result=defer reason=not_in_estimate pending=%zu", sm_id, pending_gps_factors_.size() + 1);
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
            CONSTRAINT_LOG("step=gps_submap sm_id=%d result=skip reason=pos_non_finite", sm_id);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][VALIDATION] addGPSFactor sm_id=%d pos_map non-finite, skip",
                sm_id);
            return;
        }
        if (!cov3x3.allFinite()) {
            CONSTRAINT_LOG("step=gps_submap sm_id=%d result=skip reason=cov_non_finite", sm_id);
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

        BACKEND_STEP("step=addGPSFactor_added sm_id=%d pending_factors=%zu result=ok", sm_id, pending_graph_.size());
        CONSTRAINT_LOG("step=gps_submap sm_id=%d result=ok factor_count=%d (子图级 GPS 约束)", sm_id, factor_count_);
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
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    return flushPendingGPSFactorsInternal();
}

// 内部版本：假设锁已由调用者持有
int IncrementalOptimizer::flushPendingGPSFactorsInternal() {
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

// ── KeyFrame 级别因子实现 ───────────────────────────────────────────────

void IncrementalOptimizer::addKeyFrameNode(int kf_id, const Pose3d& init_pose, bool fixed, bool is_first_kf_of_submap) {
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addKeyFrameNode: rw_mutex wait %.1fms kf_id=%d", lock_wait_ms, kf_id);
    }

    // 🔧 P0 防崩溃: 不再迭代 current_estimate_.keys()，避免 GTSAM Values 内部状态异常时 SIGSEGV
    // （run_20260317_115934 崩溃: addGPSFactor_defer 后进入 addKeyFrameNode 时在 keys() 相关路径段错误）
    size_t estimate_size_safe = 0;
    try {
        estimate_size_safe = current_estimate_.size();
    } catch (...) {
        estimate_size_safe = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] addKeyFrameNode ENTER: kf_id=%d fixed=%d first_of_submap=%d "
        "keyframe_node_exists_ size=%zu current_estimate_ size=%zu "
        "keyframe_count=%d has_prior=%d consecutive_failures=%d",
        kf_id, fixed ? 1 : 0, is_first_kf_of_submap ? 1 : 0,
        keyframe_node_exists_.size(), estimate_size_safe,
        keyframe_count_,
        has_prior_ ? 1 : 0,
        consecutive_failures_.load());

    if (!prior_noise_) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d prior_noise_ is null, skip (optimizer shutting down?)",
            kf_id);
        return;
    }

    // 检查节点是否已存在
    auto it = keyframe_node_exists_.find(kf_id);
    if (it != keyframe_node_exists_.end()) {
        // 🔧 诊断: 节点已存在，检查是否在 current_estimate_ 中
        bool in_estimate = current_estimate_.exists(KF(kf_id));
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d already exists in keyframe_node_exists_ "
            "in_current_estimate_=%d keyframe_count=%d, skip",
            kf_id, in_estimate ? 1 : 0, keyframe_count_);
        return;
    }

    // 验证位姿有效性
    const Eigen::Vector3d& t = init_pose.translation();
    const Eigen::Matrix3d& R = init_pose.rotation();
    if (!t.allFinite() || !R.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d init_pose non-finite, skip. "
            "t=[%f,%f,%f] R_finite=%d",
            kf_id, t.x(), t.y(), t.z(), R.allFinite() ? 1 : 0);
        return;
    }

    keyframe_node_exists_[kf_id] = true;
    keyframe_count_++;
    pending_values_.insert(KF(kf_id), toPose3(init_pose));

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d fixed=%d keyframe_count=%d "
        "pending_values=%zu pending_factors=%zu",
        kf_id, fixed ? 1 : 0, keyframe_count_,
        pending_values_.size(), pending_graph_.size());

    // 🔧 诊断: 在添加 Prior 之前，记录当前约束状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d check Prior condition: "
        "fixed=%d keyframe_count=%d has_prior_=%d (Prior added if fixed=true OR keyframe_count==1)",
        kf_id, fixed ? 1 : 0, keyframe_count_, has_prior_ ? 1 : 0);

    // 🔧 修复: 第一个 keyframe 或新子图首帧必须添加 Prior 因子，避免孤立节点引发 IndeterminantLinearSystemException
    // - keyframe_count_ == 1: 全 session 首帧
    // - is_first_kf_of_submap: 新子图首帧（上一子图冻结后首帧），否则无 odom 约束
    bool need_prior = fixed || keyframe_count_ == 1 || is_first_kf_of_submap;
    if (need_prior) {
        if (prior_var6_.size() != 6) {
            CONSTRAINT_LOG("step=prior_kf kf_id=%d result=skip reason=prior_var6_size=%zd", kf_id, prior_var6_.size());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d prior_var6_.size()=%zd != 6, skip Prior",
                kf_id, prior_var6_.size());
        } else {
            gtsam::noiseModel::Diagonal::shared_ptr noise =
                gtsam::noiseModel::Diagonal::Variances(prior_var6_);
            pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                KF(kf_id), toPose3(init_pose), noise));
            has_prior_ = true;
            factor_count_++;
            CONSTRAINT_LOG("step=prior_kf kf_id=%d result=ok factor_count=%d (首帧/新子图首帧先验)", kf_id, factor_count_);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d ADDED PriorFactor "
                "(fixed=%d first_of_submap=%d keyframe_count=%d) factor_count=%d pending_graph_size=%zu",
                kf_id, fixed ? 1 : 0, is_first_kf_of_submap ? 1 : 0, keyframe_count_, factor_count_, pending_graph_.size());
        }
    } else {
        CONSTRAINT_LOG("step=prior_kf kf_id=%d result=skip reason=has_odom_constraints keyframe_count=%d", kf_id, keyframe_count_);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d SKIPPED PriorFactor "
            "(fixed=%d keyframe_count=%d has_prior_=%d) - will rely on odometry constraints",
            kf_id, fixed ? 1 : 0, keyframe_count_, has_prior_ ? 1 : 0);
    }

    // 🔧 修复欠定：添加相邻 keyframe 间 Between 因子，使 KF 链满秩
    if (keyframe_count_ >= 2 && !is_first_kf_of_submap && last_keyframe_id_ >= 0) {
        Pose3d rel = last_keyframe_pose_.inverse() * init_pose;
        const Eigen::Vector3d& t = rel.translation();
        const Eigen::Matrix3d& R = rel.rotation();
        if (t.allFinite() && R.allFinite() && t.norm() <= kMaxReasonableTranslationNorm) {
            gtsam::Vector6 var6;
            var6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;  // 平移/旋转方差，约 1cm / ~0.01rad
            auto noise = gtsam::noiseModel::Diagonal::Variances(var6);
            pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                KF(last_keyframe_id_), KF(kf_id), toPose3(rel), noise));
            factor_count_++;
            CONSTRAINT_LOG("step=between_kf from=%d to=%d result=ok factor_count=%d (关键帧间里程计)", last_keyframe_id_, kf_id, factor_count_);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][KF_ODOM] addKeyFrameNode: added Between(KF%d, KF%d) factor_count=%d",
                last_keyframe_id_, kf_id, factor_count_);
        } else {
            CONSTRAINT_LOG("step=between_kf from=%d to=%d result=skip reason=rel_invalid_or_too_large", last_keyframe_id_, kf_id);
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][KF_ODOM] addKeyFrameNode: skip Between(KF%d, KF%d) rel invalid or too large",
                last_keyframe_id_, kf_id);
        }
    }
    last_keyframe_id_ = kf_id;
    last_keyframe_pose_ = init_pose;

    // 🔧 修复2: 添加keyframe节点后，立即刷新pending的GPS因子
    // 这样GPS约束可以在keyframe级别生效，而不是等到submap冻结
    int flushed_kf = flushPendingGPSFactorsForKeyFramesInternal();
    if (flushed_kf > 0) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d flushed %d pending GPS factors (keyframe level)",
            kf_id, flushed_kf);
    }
    // 🔧 修复3: 同时刷新submap级别的GPS因子（V5首次更新后pending的GPS因子）
    // 注意：这里调用Internal版本，因为addKeyFrameNode已经持有锁
    int flushed_sm = flushPendingGPSFactorsInternal();
    if (flushed_sm > 0) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] addKeyFrameNode: kf_id=%d flushed %d pending GPS factors (submap level)",
            kf_id, flushed_sm);
    }

    // 🔧 诊断: 添加完成后的状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] addKeyFrameNode EXIT: kf_id=%d keyframe_count=%d "
        "pending_values=%zu pending_factors=%zu factor_count=%d",
        kf_id, keyframe_count_, pending_values_.size(), pending_graph_.size(), factor_count_);
}

void IncrementalOptimizer::addGPSFactorForKeyFrame(int kf_id, const Eigen::Vector3d& pos_map,
                                                   const Eigen::Matrix3d& cov3x3) {
    BACKEND_STEP("step=addGPSFactorForKeyFrame_enter kf_id=%d (关键帧级 GPS)", kf_id);
    // 🔧 DEBUG: 记录锁等待开始
    auto lock_start = std::chrono::steady_clock::now();
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    auto lock_wait_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start).count();
    if (lock_wait_ms > 100.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][LOCK_DIAG] addGPSFactorForKeyFrame: rw_mutex wait %.1fms kf_id=%d", lock_wait_ms, kf_id);
    }

    // 🔧 P0 防崩溃: 不迭代 current_estimate_.keys()，仅记录 size（与 addKeyFrameNode 一致）
    size_t estimate_size_safe = 0;
    try { estimate_size_safe = current_estimate_.size(); } catch (...) { estimate_size_safe = 0; }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] addGPSFactorForKeyFrame ENTER: kf_id=%d pos=[%.2f,%.2f,%.2f] "
        "keyframe_node_exists_ size=%zu current_estimate_ size=%zu pending_gps_factors_kf_ size=%zu",
        kf_id, pos_map.x(), pos_map.y(), pos_map.z(),
        keyframe_node_exists_.size(), estimate_size_safe, pending_gps_factors_kf_.size());

    if (!prior_noise_) {
        CONSTRAINT_LOG("step=gps_kf kf_id=%d result=skip reason=prior_noise_null", kf_id);
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] addGPSFactorForKeyFrame: kf_id=%d prior_noise_ is null, skip",
            kf_id);
        return;
    }

    // 检查节点是否存在
    auto it = keyframe_node_exists_.find(kf_id);
    if (it == keyframe_node_exists_.end()) {
        BACKEND_STEP("step=addGPSFactorForKeyFrame_defer kf_id=%d reason=kf_not_in_graph pending_kf=%zu", kf_id, pending_gps_factors_kf_.size() + 1);
        CONSTRAINT_LOG("step=gps_kf kf_id=%d result=defer reason=kf_not_in_graph pending=%zu", kf_id, pending_gps_factors_kf_.size() + 1);
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] addGPSFactorForKeyFrame: kf_id=%d not in keyframe_node_exists_, defer",
            kf_id);
        // 1.2.1: pending_gps_factors_kf_ 上限，超过时 FIFO 丢弃最旧并打 WARN
        const int max_pending = ConfigManager::instance().backendMaxPendingGpsKeyframeFactors();
        while (static_cast<int>(pending_gps_factors_kf_.size()) >= max_pending && !pending_gps_factors_kf_.empty()) {
            pending_gps_factors_kf_.erase(pending_gps_factors_kf_.begin());
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][GPS_KF] pending_gps_factors_kf_ at cap=%d, dropped oldest (grep BACKEND GPS_KF)",
                max_pending);
        }
        pending_gps_factors_kf_.push_back({kf_id, pos_map, cov3x3});
        MetricsRegistry::instance().setGauge(metrics::ISAM2_PENDING_GPS_KF, static_cast<double>(pending_gps_factors_kf_.size()));
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] addGPSFactorForKeyFrame: kf_id=%d deferred to pending_gps_factors_kf_ (size now=%zu)",
            kf_id, pending_gps_factors_kf_.size());
        return;
    }

    // 检查是否已在 current_estimate_ 中
    if (!current_estimate_.exists(KF(kf_id))) {
        BACKEND_STEP("step=addGPSFactorForKeyFrame_defer kf_id=%d reason=not_in_estimate pending_kf=%zu", kf_id, pending_gps_factors_kf_.size() + 1);
        CONSTRAINT_LOG("step=gps_kf kf_id=%d result=defer reason=not_in_estimate pending=%zu", kf_id, pending_gps_factors_kf_.size() + 1);
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] addGPSFactorForKeyFrame: kf_id=%d not in current_estimate_, defer",
            kf_id);
        const int max_pending = ConfigManager::instance().backendMaxPendingGpsKeyframeFactors();
        while (static_cast<int>(pending_gps_factors_kf_.size()) >= max_pending && !pending_gps_factors_kf_.empty()) {
            pending_gps_factors_kf_.erase(pending_gps_factors_kf_.begin());
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][GPS_KF] pending_gps_factors_kf_ at cap=%d, dropped oldest (grep BACKEND GPS_KF)",
                max_pending);
        }
        pending_gps_factors_kf_.push_back({kf_id, pos_map, cov3x3});
        MetricsRegistry::instance().setGauge(metrics::ISAM2_PENDING_GPS_KF, static_cast<double>(pending_gps_factors_kf_.size()));
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] addGPSFactorForKeyFrame: kf_id=%d deferred to pending_gps_factors_kf_ (size now=%zu)",
            kf_id, pending_gps_factors_kf_.size());
        return;
    }

    // 验证 GPS 数据有效性
    if (!pos_map.allFinite() || !cov3x3.allFinite()) {
        CONSTRAINT_LOG("step=gps_kf kf_id=%d result=skip reason=pos_or_cov_non_finite", kf_id);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] addGPSFactorForKeyFrame: kf_id=%d non-finite input, skip. "
            "pos_finite=%d cov_finite=%d",
            kf_id, pos_map.allFinite() ? 1 : 0, cov3x3.allFinite() ? 1 : 0);
        return;
    }

    // 添加 GPS 因子到 keyframe 节点
    gtsam::Point3 gps_point(pos_map.x(), pos_map.y(), pos_map.z());
    gtsam::Vector3 vars;
    vars << std::max(1e-6, cov3x3(0, 0)),
            std::max(1e-6, cov3x3(1, 1)),
            std::max(1e-6, cov3x3(2, 2));
    auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
    pending_graph_.add(gtsam::GPSFactor(KF(kf_id), gps_point, noise));
    factor_count_++;
    BACKEND_STEP("step=addGPSFactorForKeyFrame_added kf_id=%d pending_factors=%zu result=ok", kf_id, pending_graph_.size());
    CONSTRAINT_LOG("step=gps_kf kf_id=%d result=ok factor_count=%d (关键帧级 GPS 约束)", kf_id, factor_count_);

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND][GPS_KF] GPS factor added to graph kf_id=%d "
        "pos=[%.2f,%.2f,%.2f] vars=[%.2e,%.2e,%.2e] factor_count=%d pending_graph_size=%zu",
        kf_id, pos_map.x(), pos_map.y(), pos_map.z(),
        vars(0), vars(1), vars(2), factor_count_, pending_graph_.size());
}

Pose3d IncrementalOptimizer::getKeyFramePose(int kf_id) const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] getKeyFramePose kf_id=%d keyframe_node_exists_ size=%zu current_estimate_ size=%zu",
        kf_id, keyframe_node_exists_.size(), current_estimate_.size());

    if (current_estimate_.exists(KF(kf_id))) {
        return fromPose3(current_estimate_.at<gtsam::Pose3>(KF(kf_id)));
    }

    // 🔧 增强: 诊断 keyframe 不存在的情况
    bool exists_in_map = keyframe_node_exists_.find(kf_id) != keyframe_node_exists_.end();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] getKeyFramePose kf_id=%d: exists_in_map=%d exists_in_estimate=0 (returning Identity)",
        kf_id, exists_in_map ? 1 : 0);

    return Pose3d::Identity();
}

bool IncrementalOptimizer::keyFrameExists(int kf_id) const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    bool exists = keyframe_node_exists_.find(kf_id) != keyframe_node_exists_.end();
    bool in_estimate = current_estimate_.exists(KF(kf_id));

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] keyFrameExists kf_id=%d: exists_in_map=%d in_estimate=%d",
        kf_id, exists ? 1 : 0, in_estimate ? 1 : 0);

    return exists;
}

void IncrementalOptimizer::addGPSFactorsForKeyFramesBatch(const std::vector<GPSFactorItemKF>& factors) {
    GtsamCallScope scope(GtsamCaller::ISAM2, "addGPSFactorsForKeyFramesBatch",
                         "count=" + std::to_string(factors.size()), false);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG] addGPSFactorsForKeyFramesBatch enter count=%zu",
        factors.size());

    try {
        std::unique_lock<std::shared_mutex> lk(rw_mutex_);
        if (!prior_noise_) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] addGPSFactorsForKeyFramesBatch: prior_noise_ is null, skip");
            return;
        }

        int added = 0;
        for (const auto& f : factors) {
            if (keyframe_node_exists_.find(f.kf_id) == keyframe_node_exists_.end()) {
                continue;
            }
            if (!f.pos.allFinite() || !f.cov.allFinite()) continue;

            gtsam::Point3 gps_point(f.pos.x(), f.pos.y(), f.pos.z());
            gtsam::Vector3 vars;
            vars << std::max(1e-6, f.cov(0, 0)),
                    std::max(1e-6, f.cov(1, 1)),
                    std::max(1e-6, f.cov(2, 2));
            auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
            pending_graph_.add(gtsam::GPSFactor(KF(f.kf_id), gps_point, noise));
            factor_count_++;
            added++;
        }

        if (added > 0) {
            commitAndUpdate();
            scope.setSuccess(true);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] addGPSFactorsForKeyFramesBatch done added=%d", added);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] addGPSFactorsForKeyFramesBatch exception: %s", e.what());
        scope.setSuccess(false);
    }
}

int IncrementalOptimizer::flushPendingGPSFactorsForKeyFrames() {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    return flushPendingGPSFactorsForKeyFramesInternal();
}

// 🔧 修复2: 添加内部版本，假设锁已由调用者持有
int IncrementalOptimizer::flushPendingGPSFactorsForKeyFramesInternal() {
    int added = 0;
    if (!prior_noise_) return 0;

    std::vector<GPSFactorItemKF> still_pending;
    for (auto& f : pending_gps_factors_kf_) {
        if (keyframe_node_exists_.find(f.kf_id) == keyframe_node_exists_.end()) {
            still_pending.push_back(std::move(f));
            continue;
        }
        if (!current_estimate_.exists(KF(f.kf_id))) {
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
        pending_graph_.add(gtsam::GPSFactor(KF(f.kf_id), gps_point, noise));
        factor_count_++;
        added++;
    }
    if (added > 0) {
        pending_gps_factors_kf_ = std::move(still_pending);
        MetricsRegistry::instance().setGauge(metrics::ISAM2_PENDING_GPS_KF, static_cast<double>(pending_gps_factors_kf_.size()));
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][GPS_KF] flushPendingGPSFactorsForKeyFramesInternal: added %d",
            added);
    }
    return added;
}

void IncrementalOptimizer::rollbackKeyframeStateForPendingKeys(const std::vector<int>& kf_ids_in_pending) {
    if (kf_ids_in_pending.empty()) return;
    for (int id : kf_ids_in_pending) {
        keyframe_node_exists_.erase(id);
    }
    keyframe_count_ = static_cast<int>(keyframe_node_exists_.size());
    bool removed_last = (std::find(kf_ids_in_pending.begin(), kf_ids_in_pending.end(), last_keyframe_id_) != kf_ids_in_pending.end());
    if (removed_last) last_keyframe_id_ = -1;
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][BACKEND][VALIDATION] rollbackKeyframeStateForPendingKeys: removed %zu KF ids, keyframe_count_=%d last_keyframe_id_=%d (grep BACKEND VALIDATION)",
        kf_ids_in_pending.size(), keyframe_count_, last_keyframe_id_);
}

// 旧版本已删除，功能合并到 flushPendingGPSFactorsForKeyFramesInternal

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
            // 🔧 修复: V5首次更新成功后跳过GPS因子刷新，避免pending_graph_刚被清空后又添加因子导致状态不一致
            // 判断方法：如果pending_values_为空且刚完成首次更新，说明是V5首次更新后的状态
            // 此时不应该立即刷新GPS因子，因为因子和values不在同一事务中
            bool is_first_update_aftermath = (pending_values_.empty() && pending_graph_.size() > 0);
            if (!is_first_update_aftermath) {
                // 注意：这里调用Internal版本，因为forceUpdate已经持有锁
                int flushed = flushPendingGPSFactorsInternal();
                if (flushed > 0) {
                    res = commitAndUpdate();
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[ISAM2_DIAG][TRACE] step=forceUpdate_after_flush success=%d nodes=%d (deferred GPS applied)",
                        res.success ? 1 : 0, res.nodes_updated);
                }
            } else {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG][TRACE] step=forceUpdate_after_flush: first update aftermath, defer GPS factor flush");
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
        // 🔧 诊断: forceUpdate 异常时记录详细状态
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] forceUpdate EXCEPTION: "
            "node_exists_ size=%zu pending_values=%zu pending_factors=%zu current_estimate_=%zu "
            "exception='%s'",
            node_exists_.size(), pending_values_.size(), pending_graph_.size(),
            current_estimate_.size(), e.what());

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][CRASH_TRACE] step=forceUpdate_exception exception=%s (崩溃发生在 commitAndUpdate 内，见上一 TRACE step)",
            e.what());

        // 🔧 修复: 根据异常类型采用不同的恢复策略
        std::string exc_msg = e.what();
        if (exc_msg.find("IndeterminantLinearSystemException") != std::string::npos ||
            exc_msg.find("key already exists") != std::string::npos ||
            exc_msg.find("InvalidNoise") != std::string::npos ||
            exc_msg.find("Unable to factor") != std::string::npos) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] forceUpdate: unrecoverable exception, clearing pending state");
            pending_graph_.resize(0);
            pending_values_.clear();
        }

        ALOG_ERROR(MOD, "forceUpdate failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] forceUpdate: %s", e.what());
        return OptimizationResult{};
    } catch (...) {
        // 🔧 诊断: 未知异常
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] forceUpdate UNKNOWN EXCEPTION: "
            "node_exists_ size=%zu pending_values=%zu pending_factors=%zu",
            node_exists_.size(), pending_values_.size(), pending_graph_.size());

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

// 输出单子图位姿到 POSE_TRACE，便于定位后端/HBA 各环节偏差（grep POSE_TRACE 按 stage 对比）
static void logSubmapPoseTrace(rclcpp::Logger log, const char* stage, int sm_id, const Pose3d& pose) {
    Eigen::Quaterniond q(pose.rotation());
    RCLCPP_INFO(log,
        "[POSE_TRACE] stage=%s sm_id=%d x=%.6f y=%.6f z=%.6f qx=%.6f qy=%.6f qz=%.6f qw=%.6f",
        stage, sm_id,
        pose.translation().x(), pose.translation().y(), pose.translation().z(),
        q.x(), q.y(), q.z(), q.w());
}

OptimizationResult IncrementalOptimizer::commitAndUpdate() {
    // 注意：调用时已持有写锁
    if (pending_graph_.empty() && pending_values_.empty()) {
        BACKEND_STEP("step=commitAndUpdate_skip reason=no_pending");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] commitAndUpdate skip: no pending (first effective commit after first submap freeze or keyframe+Between batch; grep BACKEND)");
        return OptimizationResult{};
    }
    // 后端图优化前：记录当前图中所有子图位姿（便于与优化后、HBA 前后对比定位问题）
    {
        auto log = rclcpp::get_logger("automap_system");
        for (const auto& kv : node_exists_) {
            int id = kv.first;
            gtsam::Key key = SM(id);
            if (!current_estimate_.exists(key)) continue;
            try {
                auto p = current_estimate_.at<gtsam::Pose3>(key);
                logSubmapPoseTrace(log, "backend_before", id, fromPose3(p));
            } catch (...) {}
        }
    }
    BACKEND_STEP("step=commitAndUpdate_enter pending_factors=%zu pending_values=%zu", pending_graph_.size(), pending_values_.size());

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
        bool has_isolated_node = false;     // 存在 0 约束节点时置 true，用于提前中止避免 GTSAM 抛异常
        std::string factor_types_str;       // 用于日志
        // 🔧 增强: 统计 keyframe 节点和因子数量
        int kf_node_count = 0;
        int kf_factor_count = 0;
        int sm_node_count = 0;
        int sm_factor_count = 0;
        try {
            std::string value_keys_str;
            std::string kf_keys_str;
            std::string sm_keys_str;
            for (const gtsam::Key k : pending_values_.keys()) {
                gtsam::Symbol sym(k);
                value_keys_str += (value_keys_str.empty() ? "" : ",") + std::to_string(k);
                if (sym.chr() == 'x') {
                    kf_keys_str += (kf_keys_str.empty() ? "" : ",") + std::to_string(sym.index());
                    kf_node_count++;
                } else if (sym.chr() == 's') {
                    sm_keys_str += (sm_keys_str.empty() ? "" : ",") + std::to_string(sym.index());
                    sm_node_count++;
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] pre_update: TOTAL values=%zu (KF nodes=%d: [%s], SM nodes=%d: [%s])",
                pending_values_.size(), kf_node_count, kf_keys_str.c_str(), sm_node_count, sm_keys_str.c_str());

            // 🔧 增强: 诊断当前约束状态 - 记录每个节点有多少约束
            std::map<gtsam::Key, int> key_factor_count;
            for (size_t i = 0; i < pending_graph_.size(); ++i) {
                const auto& f = pending_graph_[i];
                gtsam::KeyVector kv = f->keys();
                std::string keys_str;
                for (gtsam::Key k : kv) {
                    keys_str += (keys_str.empty() ? "" : ",") + std::to_string(k);
                    key_factor_count[k]++;
                    // 统计因子类型
                    gtsam::Symbol sym(k);
                    if (sym.chr() == 'x') kf_factor_count++;
                    else if (sym.chr() == 's') sm_factor_count++;
                }
                const char* type_str = "Other";
                if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) type_str = "Prior";
                else if (dynamic_cast<const gtsam::GPSFactor*>(f.get())) type_str = "GPS";
                else if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(f.get())) type_str = "Between";
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG] pre_update factor_%zu type=%s keys=[%s]", i, type_str, keys_str.c_str());
                factor_types_str += (factor_types_str.empty() ? "" : ",") + std::string(type_str);
            }

            // 🔧 增强: 输出每个节点的约束数量（诊断孤立节点）
            for (const gtsam::Key k : pending_values_.keys()) {
                int fc = key_factor_count.count(k) ? key_factor_count[k] : 0;
                gtsam::Symbol sym(k);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG] pre_update node key=%zu (type=%c id=%d) has %d constraint(s) - %s",
                    static_cast<size_t>(k), static_cast<char>(sym.chr()), sym.index(), fc,
                    fc == 0 ? "WARNING: ISOLATED NODE!" : (fc == 1 ? "minimal" : "OK"));
            }

            // 🔧 增强: 记录约束不足的节点
            for (const gtsam::Key k : pending_values_.keys()) {
                int fc = key_factor_count.count(k) ? key_factor_count[k] : 0;
                if (fc == 0) {
                    has_isolated_node = true;
                    gtsam::Symbol sym(k);
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][CONSTRAINT] ISOLATED NODE: key=%zu type=%c id=%d has ZERO constraints! "
                        "- this WILL cause IndeterminantLinearSystemException!",
                        static_cast<size_t>(k), static_cast<char>(sym.chr()), sym.index());
                }
            }
            if (has_isolated_node) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] pre_update: FOUND ISOLATED NODE(S) - system is underconstrained! "
                    "total KF nodes=%d KF constraints=%d SM nodes=%d SM constraints=%d",
                    kf_node_count, kf_factor_count, sm_node_count, sm_factor_count);
            }

            // 🔧 增强: 输出总约束统计
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] pre_update summary: KF nodes=%d factors=%d, SM nodes=%d factors=%d, TOTAL constraints=%zu",
                kf_node_count, kf_factor_count, sm_node_count, sm_factor_count, pending_graph_.size());

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
    // ===== 修复2: 添加超时强制优化机制 =====
    auto now = std::chrono::steady_clock::now();
    auto pending_duration_ms = std::chrono::duration<double, std::milli>(now - single_node_pending_start_).count();
    bool force_update_due_to_timeout = false;

    if (single_prior_only || all_factors_same_key) {
        // 检查是否超时
        if (pending_duration_ms > kSingleNodePendingTimeoutMs) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][CRITICAL] FORCE UPDATE due to timeout: pending_duration=%.1fms > threshold=%dms (factors=%zu values=%zu types=[%s])",
                pending_duration_ms, kSingleNodePendingTimeoutMs, pf, pv, factor_types_str.c_str());
            force_update_due_to_timeout = true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][CRITICAL] DEFER single-node update (factors=%zu values=%zu types=[%s]) "
                "- avoid GTSAM first-update double free (borglab/gtsam#1189), pending until 2+ nodes or timeout (%.1fms/%.0fms)",
                pf, pv, factor_types_str.c_str(), pending_duration_ms, (double)kSingleNodePendingTimeoutMs);
            BACKEND_STEP("step=commitAndUpdate_defer reason=%s factors=%zu values=%zu result=deferred",
                single_prior_only ? "single_prior" : "all_factors_same_key", pf, pv);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][PIPELINE] event=commit_deferred reason=%s factors=%zu values=%zu (grep BACKEND PIPELINE 定位)",
                single_prior_only ? "single_prior" : "all_factors_same_key", pf, pv);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=commitAndUpdate_defer_return single_prior_only=%d all_factors_same_key=%d (未调用 isam2_.update)",
                single_prior_only ? 1 : 0, all_factors_same_key ? 1 : 0);
            scope.setSuccess(true);
            return OptimizationResult{};  // 不清空 pending，等第二节点+odom 后一起 update
        }
    }

    // ✅ 孤立节点提前中止：避免进入 isam2_.update 后 GTSAM 抛出 IndeterminantLinearSystemException
    if (has_isolated_node) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][CONSTRAINT] ABORT commitAndUpdate: isolated node(s) detected - clearing pending to avoid IndeterminantLinearSystemException (grep BACKEND CONSTRAINT)");
        std::vector<int> kf_ids_in_pending;
        for (const gtsam::Key k : pending_values_.keys()) {
            gtsam::Symbol sym(k);
            if (sym.chr() == 'x') kf_ids_in_pending.push_back(sym.index());
        }
        pending_graph_.resize(0);
        pending_values_.clear();
        rollbackKeyframeStateForPendingKeys(kf_ids_in_pending);
        scope.setSuccess(false);
        recordOptimizationFailure("isolated_node_abort");
        METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
        return OptimizationResult{};
    }

    // 如果是超时强制执行，跳过延迟检查，更新超时计时器
    if (force_update_due_to_timeout) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][TIMEOUT] Force update after timeout: pending_duration=%.1fms",
            pending_duration_ms);
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
        // ✅ IMPORTANT:
        // - values_for_update: ONLY contains newly added variables' initial values (pending_values_).
        //   Passing existing keys again to ISAM2::update triggers "key already exists" (e.g. s0).
        // - values_for_validation: may merge current_estimate_ so factor-key validation can access all keys.
        // 🔧 修复 KEY_CONFLICT：只将不在 current_estimate 中的 keys 用于 update
        gtsam::Values values_for_update;
        gtsam::Values values_for_validation(pending_values_);
        // Incremental path: merge current_estimate_ ONLY for validation (not for isam2_.update)
        if (!is_first_update && !current_estimate_.empty()) {
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
                // 🔧 修复 KEY_CONFLICT: 跳过已存在的keys，只插入新的keys到values_for_update
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][KEY_CONFLICT] Found %d conflicting keys, using insert_safe approach",
                    conflict_count);
                // 创建新的Values，只包含不在current_estimate_中的keys（用于update）
                for (const gtsam::Key pk : pending_values_.keys()) {
                    if (!current_estimate_.exists(pk)) {
                        values_for_update.insert(pk, pending_values_.at(pk));
                    }
                }
                // values_for_validation 仍包含所有 keys 用于验证
                gtsam::Values values_merged(current_estimate_);
                for (const gtsam::Key pk : pending_values_.keys()) {
                    if (!current_estimate_.exists(pk)) {
                        values_merged.insert(pk, pending_values_.at(pk));
                    }
                }
                values_for_validation = values_merged;
            } else {
                // 无冲突：所有 pending keys 都是新的，可以直接使用
                try {
                    values_for_update = pending_values_;
                    gtsam::Values values_merged(current_estimate_);
                    values_merged.insert(pending_values_);
                    values_for_validation = values_merged;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][KEY_CONFLICT] values_merged.insert failed: %s - "
                        "skip merge, use pending_values only",
                        e.what());
                    values_for_validation = pending_values_;
                }
            }
        } else {
            // 首次 update 或 current_estimate 为空：直接使用 pending_values
            values_for_update = pending_values_;
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][TRACE] step=copy_graph_values_done factors=%zu values=%zu (崩溃在此后则与 copy 无关)",
            graph_copy.size(), values_for_validation.size());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_copy_done graph_size=%zu values_size=%zu is_first_update=%d",
            graph_copy.size(), values_for_validation.size(), is_first_update ? 1 : 0);

        // ═══════════════════════════════════════════════════════════════════════════════
        // ✅ P0 V5 根本修复（BACKEND_ISOLATED_NODES_ROOT_CAUSE）：首次 update 分两阶段
        //    原问题：仅注入 values 后清空 pending，Prior/Between 从未入图 → 下次 update 仅 55 GPS → 奇异。
        // 方案：Step1 仅注入 values（空 graph，避免首次 linearize 触发 GTSAM bug）；
        //       Step2 同一调用内再 update(graph_copy, empty Values)，将结构因子加入 ISAM2；
        //       然后清空 pending。这样后续 flush 的 KF GPS 在已有拓扑的图上加入，不再奇异。
        // ═══════════════════════════════════════════════════════════════════════════════
        if (is_first_update) {
            used_first_update_three_phase = true;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][V5] first update path=values_then_factors (two-phase)");

            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=first_update_v5_enter factors=%zu values=%zu",
                graph_copy.size(), values_for_update.size());
            // V5 首次 update 前：校验 values 合法，避免注入 nan/inf 或超大平移导致 GTSAM 异常
            bool first_values_ok = true;
            for (const gtsam::Key k : values_for_update.keys()) {
                try {
                    auto p = values_for_update.at<gtsam::Pose3>(k);
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
                "[CRASH_CONTEXT] step=first_update_v5_pre_update_values nodes=%zu", values_for_update.size());
            auto t_first_update = std::chrono::steady_clock::now();
            isam2_.update(empty_graph, values_for_update);
            double ms_first = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_first_update).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=first_update_v5_post_update_values elapsed_ms=%.1f", ms_first);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=first_update_v5_post_update_values");

            current_estimate_ = values_for_update;

            // Step2（根本修复）：将结构因子（Prior + Between）加入 ISAM2，避免后续仅加 GPS 时图无拓扑导致奇异
            if (graph_copy.size() > 0) {
                gtsam::Values no_new_values;
                crash_report::setLastStep("first_update_v5_inject_factors");
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG][TRACE] step=first_update_v5_pre_inject_factors factors=%zu", graph_copy.size());
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[CRASH_CONTEXT] step=first_update_v5_pre_inject_factors");
                auto t_factors = std::chrono::steady_clock::now();
                isam2_.update(graph_copy, no_new_values);
                double ms_factors = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_factors).count();
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG][TRACE] step=first_update_v5_post_inject_factors elapsed_ms=%.1f", ms_factors);
                current_estimate_ = isam2_.calculateEstimate();
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ISAM2_DIAG][V5] first update: injected %zu structural factors, estimate_size=%zu",
                    graph_copy.size(), current_estimate_.size());
            }

            BACKEND_STEP("step=commitAndUpdate_first_update_done values_injected=%zu factors_injected=%zu result=ok",
                values_for_update.size(), graph_copy.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][V5] first update done: values=%zu factors=%zu (submap+keyframe structure in graph)",
                values_for_update.size(), graph_copy.size());
        } else {
            // 常规 ISAM2 增量 update：优化前全量约束校验，任一不合理则中止 update 避免触发 GTSAM 崩溃
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG] commitAndUpdate path=incremental (graph_copy.size=%zu values_for_update.size=%zu values_for_validation.size=%zu)",
                graph_copy.size(), values_for_update.size(), values_for_validation.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_constraints_dump_enter");
            ConstraintValidation inc_validation;
            logAllConstraintsAndValidate(graph_copy, values_for_validation, "incremental_before_isam2", &inc_validation);
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

                // 🔧 修复: 验证失败时同步 node_exists_ 与 current_estimate_ 状态
                // 问题: 失败后清空了 pending_graph_/pending_values_，但 node_exists_ 没有更新
                // 导致后续因子检查通过但实际 iSAM2 中没有这些节点
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][VALIDATION] Sync node_exists_ after validation failure: "
                    "node_exists_ has %zu keys, current_estimate_ has %zu values",
                    node_exists_.size(), current_estimate_.size());

                // 移除不在 current_estimate_ 中的节点
                std::vector<int> nodes_to_remove;
                for (const auto& kv : node_exists_) {
                    if (!current_estimate_.exists(SM(kv.first))) {
                        nodes_to_remove.push_back(kv.first);
                    }
                }
                for (int id : nodes_to_remove) {
                    node_exists_.erase(id);
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][VALIDATION] Removed stale node_exists_ entry: sm_id=%d", id);
                }

                // 如果连续失败太多，触发恢复机制
                if (consecutive_failures_.load() >= 3) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][BACKEND][VALIDATION] Too many consecutive failures (%d), triggering recovery reset",
                        consecutive_failures_.load());
                    // 不在验证失败路径调用 resetForRecovery（会死锁），而是标记需要在下一次调用时重置
                }

                // 1.3.1: 失败路径同步 keyframe 状态，避免图已清空但 keyframe_node_exists_ 仍含本批 id
                std::vector<int> kf_ids_in_pending;
                for (const gtsam::Key k : pending_values_.keys()) {
                    gtsam::Symbol sym(k);
                    if (sym.chr() == 'x') kf_ids_in_pending.push_back(sym.index());
                }
                pending_graph_.resize(0);
                pending_values_.clear();
                rollbackKeyframeStateForPendingKeys(kf_ids_in_pending);
                scope.setSuccess(false);
                recordOptimizationFailure(inc_validation.message.c_str());
                METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
                return OptimizationResult{};
            }

            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_pre_update (若崩溃则发生在 isam2_.update(graph_copy, values_for_update) 内)");
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_DIAG][TRACE] step=isam2_update_invoke");
            crash_report::setLastStep("incremental_isam2_update");
            auto t_update_begin = std::chrono::steady_clock::now();
            isam2_.update(graph_copy, values_for_update);
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
            BACKEND_STEP("step=commitAndUpdate_incremental_done estimate_size=%zu result=ok", current_estimate_.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CRASH_CONTEXT] step=incremental_calculateEstimate_done nodes=%zu", current_estimate_.size());
        }
        // === 清空 pending ===
        // 首次路径下结构因子已在 Step2 注入 ISAM2，此处清空避免重复入图；增量路径下本次已提交，清空待下一批
        pending_graph_.resize(0);
        pending_values_.clear();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG][V5] first update: cleared pending after values+factors injected");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_v5_exit_success");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[CRASH_CONTEXT] step=commitAndUpdate_exit_success");
    } catch (const std::exception& e) {
        scope.setSuccess(false);
        BACKEND_STEP("step=commitAndUpdate_done result=fail exception=%s", e.what());
        ALOG_ERROR(MOD, "iSAM2 update FAILED: {}", e.what());

        // 🔧 诊断: 异常时记录详细状态
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] EXCEPTION in commitAndUpdate: "
            "node_exists_ size=%zu pending_values=%zu pending_factors=%zu current_estimate_=%zu "
            "exception='%s'",
            node_exists_.size(), pending_values_.size(), pending_graph_.size(),
            current_estimate_.size(), e.what());

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

        // 🔧 修复: 根据异常类型采用不同的恢复策略
        // 不可恢复的异常：清理 pending 状态，避免死亡螺旋
        // 可恢复的异常：保留 pending 状态，下次重试
        std::string exc_msg = e.what();
        bool is_unrecoverable = false;
        if (exc_msg.find("IndeterminantLinearSystemException") != std::string::npos ||
            exc_msg.find("key already exists") != std::string::npos ||
            exc_msg.find("InvalidNoise") != std::string::npos ||
            exc_msg.find("Unable to factor") != std::string::npos) {
            is_unrecoverable = true;
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] Unrecoverable exception detected, clearing pending state to avoid death spiral");
            std::vector<int> kf_ids_in_pending;
            for (const gtsam::Key k : pending_values_.keys()) {
                gtsam::Symbol sym(k);
                if (sym.chr() == 'x') kf_ids_in_pending.push_back(sym.index());
            }
            pending_graph_.resize(0);
            pending_values_.clear();
            rollbackKeyframeStateForPendingKeys(kf_ids_in_pending);
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] After clear: pending_values=%zu pending_factors=%zu",
                pending_values_.size(), pending_graph_.size());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] Exception handled, preserving node_exists_ and pending state for recovery. "
                "node_exists_ size=%zu current_estimate_ size=%zu pending_values_ size=%zu pending_graph_ size=%zu",
                node_exists_.size(), current_estimate_.size(), pending_values_.size(), pending_graph_.size());
        }

        // 🔧 健康检查：记录优化失败并检查是否需要触发恢复
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

    BACKEND_STEP("step=commitAndUpdate_extract_poses node_exists=%zu estimate_size=%zu poses_extracted=%zu elapsed_ms=%.1f",
        node_exists_.size(), current_estimate_.size(), poses.size(), elapsed);

    // [ISAM2_GHOSTING_DIAG] 重影排查：本次 iSAM2 即将通知的位姿摘要（与 onPoseUpdated / map_publish 时序对照）
    {
        int sm_count = 0;
        std::string first_sm_str;
        std::string last_sm_str;
        int first_id = -1;
        int last_id = -1;
        for (const auto& [id, pose] : poses) {
            if (id < 100000) {  // 子图级节点 (MAX_KF_PER_SUBMAP)
                sm_count++;
                if (first_id < 0) { first_id = id; first_sm_str = "sm_id=" + std::to_string(id) + " pos=[" +
                    std::to_string(pose.translation().x()) + "," + std::to_string(pose.translation().y()) + "," + std::to_string(pose.translation().z()) + "]"; }
                last_id = id;
                last_sm_str = "sm_id=" + std::to_string(id) + " pos=[" +
                    std::to_string(pose.translation().x()) + "," + std::to_string(pose.translation().y()) + "," + std::to_string(pose.translation().z()) + "]";
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_GHOSTING_DIAG] commitAndUpdate_extract: total_poses=%zu sm_count=%d elapsed_ms=%.1f (即将 notifyPoseUpdate，重影排查见 onPoseUpdated 与 map_publish 时序)",
            poses.size(), sm_count, elapsed);
        if (!first_sm_str.empty())
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_GHOSTING_DIAG] commitAndUpdate_extract first %s", first_sm_str.c_str());
        if (last_id >= 0 && last_id != first_id)
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_GHOSTING_DIAG] commitAndUpdate_extract last %s", last_sm_str.c_str());
    }
    // 后端图优化后：记录本次提取的所有子图位姿（便于与 backend_before / hba_before / hba_after 对比）
    {
        auto log = rclcpp::get_logger("automap_system");
        for (const auto& [id, pose] : poses) {
            logSubmapPoseTrace(log, "backend_after", id, pose);
        }
    }

    OptimizationResult res;
    res.nodes_updated = (int)poses.size();
    res.elapsed_ms    = elapsed;
    res.submap_poses  = poses;
    // 若未提取到任何位姿（estimate 与 node_exists_ 不一致），视为失败并打清原因，便于 addLoopFactor 等调用方排查
    if (poses.size() == 0 && node_exists_.size() > 0) {
        res.success = false;
        BACKEND_STEP("step=commitAndUpdate_done result=fail reason=no_poses_extracted");
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] commitAndUpdate success=false reason=no_poses_extracted node_exists_=%zu current_estimate_.size=%zu (grep BACKEND)",
            node_exists_.size(), current_estimate_.size());
    } else {
        res.success = true;
        BACKEND_STEP("step=commitAndUpdate_done result=ok nodes_updated=%d elapsed_ms=%.1f", res.nodes_updated, elapsed);
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
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_GHOSTING_DIAG] notifyPoseUpdate_enter poses=%zu (回调 onPoseUpdated 将写回 SubMap/KF 位姿；与 map_publish/buildGlobalMap 时序对照可排查重影)",
            poses.size());
        notifyPoseUpdate(poses);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_GHOSTING_DIAG] notifyPoseUpdate_exit (onPoseUpdated 已返回)");

        // ===== 修复2: 优化成功后重置单节点pending计时器 =====
        single_node_pending_start_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][TIMEOUT] Reset single_node_pending timer after successful optimization");

        // 首节点/后续 update 成功后尝试 flush 对齐阶段 defer 的 GPS 因子，避免 node_count=0 时全部进入 pending 永不入图
        // 🔧 修复: V5首次更新成功后跳过GPS因子刷新，避免pending_graph_刚被清空后又添加因子导致状态不一致
        // GPS因子将在下一次addKeyFrameNode或addSubMapNode时刷新
        if (res.nodes_updated > 0 && !used_first_update_three_phase) {
            // 注意：这里调用Internal版本，因为commitAndUpdate是内部调用，已持有锁
            int flushed = flushPendingGPSFactorsInternal();
            if (flushed > 0) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][BACKEND][GPS] commitAndUpdate success: flushing %d pending GPS factors, running second update",
                    flushed);
                res = commitAndUpdate();
            }
        } else if (used_first_update_three_phase) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND][GPS] V5 first update done, defer GPS factor flush to next node add");
        }
    } else {
        METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 0.0);
        scope.setSuccess(false);
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND] commitAndUpdate FAILED: path=%s elapsed_ms=%.1f nodes=%zu (trying extract anyway)",
            path_str, elapsed, poses.size());

        // P0 修复：即使优化失败，也要通知位姿更新，避免子图位姿更新链中断导致重影
        if (!poses.empty()) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][BACKEND] notifying %zu poses despite failure", poses.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_GHOSTING_DIAG] notifyPoseUpdate_enter poses=%zu (despite_failure=1)", poses.size());
            notifyPoseUpdate(poses);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ISAM2_GHOSTING_DIAG] notifyPoseUpdate_exit (despite_failure=1)");
        }
    }
    return res;
}

std::optional<Pose3d> IncrementalOptimizer::getPoseOptional(int sm_id) const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    try {
        // 检查节点是否存在
        auto node_it = node_exists_.find(sm_id);
        bool node_exists = (node_it != node_exists_.end() && node_it->second);

        if (!node_exists) {
            // 节点完全不存在于因子图中
            BACKEND_STEP("step=getPoseOptional_miss sm_id=%d reason=node_not_exists", sm_id);
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer] getPoseOptional sm_id=%d: node does NOT exist in graph", sm_id);
            return std::nullopt;
        }

        if (!current_estimate_.exists(SM(sm_id))) {
            // 节点存在于因子图，但尚未被 forceUpdate 提交到 estimate
            BACKEND_STEP("step=getPoseOptional_miss sm_id=%d reason=not_in_estimate", sm_id);
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer] getPoseOptional sm_id=%d: node exists in graph but NOT in estimate (forceUpdate pending)", sm_id);
            return std::nullopt;
        }

        auto p = current_estimate_.at<gtsam::Pose3>(SM(sm_id));
        return fromPose3(p);
    } catch (const std::exception& e) {
        BACKEND_STEP("step=getPoseOptional_miss sm_id=%d reason=exception exception=%s", sm_id, e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] getPoseOptional sm_id=%d: %s", sm_id, e.what());
        return std::nullopt;
    }
}

Pose3d IncrementalOptimizer::getPose(int sm_id) const {
    auto result = getPoseOptional(sm_id);
    if (!result.has_value()) {
        // 🔧 保持原有 fallback 行为以避免破坏现有代码
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer] getPose sm_id=%d: returning Identity() due to missing node - prefer using getPoseOptional() instead", sm_id);
        return Pose3d::Identity();
    }
    return result.value();
}

bool IncrementalOptimizer::hasNodePendingEstimate(int sm_id) const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    // 节点存在于因子图，但不在 current_estimate_ 中（首次 forceUpdate 前）
    auto it = node_exists_.find(sm_id);
    if (it == node_exists_.end()) {
        return false;  // 节点完全不存在
    }
    if (!it->second) {
        return false;  // 节点已标记为不存在
    }
    // 节点存在，检查是否在 estimate 中
    gtsam::Key key = SM(sm_id);
    return !current_estimate_.exists(key);
}

std::unordered_map<int, Pose3d> IncrementalOptimizer::getAllPoses() const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    std::unordered_map<int, Pose3d> out;

    // 🔧 增强: 记录当前状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] getAllPoses: node_exists_ size=%zu current_estimate_ size=%zu",
        node_exists_.size(), current_estimate_.size());

    int found_count = 0;
    int not_in_estimate_count = 0;
    for (const auto& kv : node_exists_) {
        int id = kv.first;
        gtsam::Key key = SM(id);
        if (!current_estimate_.exists(key)) {
            not_in_estimate_count++;
            ALOG_ERROR(MOD, "getAllPoses sm_id={} not in current_estimate, skip", id);
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] getAllPoses: sm_id=%d not in current_estimate_ (node exists but estimate missing)",
                id);
            continue;
        }
        try {
            auto p = current_estimate_.at<gtsam::Pose3>(key);
            out[id] = fromPose3(p);
            found_count++;
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

    // 🔧 增强: 记录查询结果
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] getAllPoses: found=%d not_in_estimate=%d total_requested=%zu",
        found_count, not_in_estimate_count, node_exists_.size());

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
    keyframe_node_exists_.clear();
    keyframe_count_ = 0;
    last_keyframe_id_ = -1;
    pending_gps_factors_.clear();
    pending_gps_factors_kf_.clear();
    node_count_ = 0;
    factor_count_ = 0;
    has_prior_ = false;
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
    keyframe_node_exists_.clear();
    keyframe_count_ = 0;
    last_keyframe_id_ = -1;
    pending_gps_factors_.clear();
    pending_gps_factors_kf_.clear();
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
    // 条件触发式优化方案：
    // 1. 批量处理：累积一定数量的任务后再处理
    // 2. 超时触发：即使任务数量不够，超过一定时间也处理
    // 3. 避免阻塞：主线程添加任务时不等待结果

    // 🔧 DEBUG: 记录 optLoop 启动时间
    auto opt_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] optLoop started (tid=0x%zx)",
        static_cast<size_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())));

    // 批量处理参数
    static constexpr size_t kBatchSizeThreshold = 5;       // 累积5个任务后处理
    static constexpr int kBatchTimeoutMs = 2000;           // 或2秒超时

    while (true) {
        // ===== 条件触发：等待任务或超时 =====
        std::vector<OptimTask> batch_tasks;
        auto batch_start = std::chrono::steady_clock::now();

        {
            std::unique_lock<std::mutex> lk(opt_queue_mutex_);

            // 首先检查是否需要立即退出
            if (!opt_running_ && opt_queue_.empty()) {
                break;
            }

            // 如果队列为空，等待新任务
            if (opt_queue_.empty()) {
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop: waiting for task, queue_size=0");
                opt_queue_cv_.wait_for(lk, std::chrono::milliseconds(kBatchTimeoutMs), [this] {
                    return !opt_running_ || !opt_queue_.empty();
                });
            }

            // 检查是否需要退出
            if (!opt_running_ && opt_queue_.empty()) {
                break;
            }

            // 收集第一批任务
            while (!opt_queue_.empty() && batch_tasks.size() < kBatchSizeThreshold) {
                batch_tasks.push_back(opt_queue_.front());
                opt_queue_.pop();
            }

            // 如果只有1个任务且队列还有，尝试等待一小段时间看是否能聚合更多
            if (batch_tasks.size() == 1 && !opt_queue_.empty()) {
                auto wait_start = std::chrono::steady_clock::now();
                opt_queue_cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
                    return !opt_running_ || opt_queue_.empty();
                });
                auto wait_elapsed = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - wait_start).count();

                while (!opt_queue_.empty() && batch_tasks.size() < kBatchSizeThreshold) {
                    batch_tasks.push_back(opt_queue_.front());
                    opt_queue_.pop();
                }

                if (batch_tasks.size() > 1) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][DIAG] optLoop: batch coalesced %zu tasks after %.1fms wait",
                        batch_tasks.size(), wait_elapsed);
                }
            }

            MetricsRegistry::instance().setGauge(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(opt_queue_.size()));
        }

        if (batch_tasks.empty()) {
            continue;
        }

        auto batch_collect_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - batch_start).count();

        // ===== 处理批量任务 =====
        BACKEND_STEP("step=optLoop_batch_start tasks=%zu collected_ms=%.1f", batch_tasks.size(), batch_collect_ms);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] optLoop processing batch of %zu tasks, collected in %.1fms",
            batch_tasks.size(), batch_collect_ms);

        // 🔧 增强: 记录批量任务类型统计和详情
        int loop_count = 0, gps_count = 0, batch_count = 0;
        std::string loop_details;
        std::string gps_details;
        for (const auto& t : batch_tasks) {
            if (t.type == OptimTaskType::LOOP_FACTOR) {
                loop_count++;
                loop_details += (loop_details.empty() ? "" : ",") + std::to_string(t.from_id) + "->" + std::to_string(t.to_id);
            }
            else if (t.type == OptimTaskType::GPS_FACTOR) gps_count++;
            else if (t.type == OptimTaskType::BATCH_UPDATE) batch_count++;
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] optLoop batch: LOOP=%d (from->to: [%s]) GPS=%d BATCH=%d total=%zu",
            loop_count, loop_details.c_str(), gps_count, batch_count, batch_tasks.size());

        auto task_t0 = std::chrono::steady_clock::now();
        bool batch_ok = false;

        // 🔧 增强: 获取当前优化器状态
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][DIAG] optLoop batch START: node_count=%d factor_count=%d keyframe_count=%d "
            "pending_graph=%zu pending_values=%zu current_estimate_=%zu consecutive_failures=%d",
            node_count_, factor_count_, keyframe_count_,
            pending_graph_.size(), pending_values_.size(), current_estimate_.size(),
            consecutive_failures_.load());

        // ✅ 修复：为批量任务添加 try-catch，防止批量任务失败导致整个 optLoop 退出
        try {
            // 统一处理批量任务 - 获取一次锁
            auto lock_start = std::chrono::steady_clock::now();
            std::unique_lock<std::shared_mutex> lk(rw_mutex_);
            auto lock_wait_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - lock_start).count();

            if (lock_wait_ms > 100.0) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][LOCK_DIAG] optLoop batch: rw_mutex wait %.1fms (WARNING: high contention!)",
                    lock_wait_ms);
            }

            if (!prior_noise_) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop batch: prior_noise_ is null, optimizer shutting down");
                continue;
            }

            // 批量添加因子 - 增强日志
            int kf_factors_added = 0;
            int sm_factors_added = 0;
            for (const auto& task : batch_tasks) {
                if (task.type == OptimTaskType::LOOP_FACTOR) {
                    // 🔧 增强: 检查 submap 节点是否存在
                    auto from_it = node_exists_.find(task.from_id);
                    auto to_it = node_exists_.find(task.to_id);
                    bool from_exists = (from_it != node_exists_.end());
                    bool to_exists = (to_it != node_exists_.end());
                    bool from_in_estimate = current_estimate_.exists(SM(task.from_id));
                    bool to_in_estimate = current_estimate_.exists(SM(task.to_id));

                    if (!from_exists || !to_exists) {
                        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                            "[IncrementalOptimizer][DIAG] optLoop: SKIP LOOP from=%d to=%d "
                            "(from_exists=%d to_exists=%d from_in_estimate=%d to_in_estimate=%d)",
                            task.from_id, task.to_id,
                            from_exists ? 1 : 0, to_exists ? 1 : 0,
                            from_in_estimate ? 1 : 0, to_in_estimate ? 1 : 0);
                        continue;
                    }
                    auto base_noise = infoToNoiseDiagonal(task.info_matrix);
                    auto robust_noise = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
                    pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                        SM(task.from_id), SM(task.to_id), toPose3(task.rel_pose), robust_noise));
                    factor_count_++;
                    sm_factors_added++;
                } else if (task.type == OptimTaskType::GPS_FACTOR) {
                    if (!node_exists_.count(task.from_id)) {
                        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                            "[IncrementalOptimizer][DIAG] optLoop: SKIP GPS sm_id=%d (node not exists)",
                            task.from_id);
                        continue;
                    }
                    gtsam::Point3 gps_point(task.gps_pos.x(), task.gps_pos.y(), task.gps_pos.z());
                    gtsam::Vector3 vars;
                    vars << std::max(1e-6, task.gps_cov(0, 0)),
                            std::max(1e-6, task.gps_cov(1, 1)),
                            std::max(1e-6, task.gps_cov(2, 2));
                    auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
                    pending_graph_.add(gtsam::GPSFactor(SM(task.from_id), gps_point, noise));
                    factor_count_++;
                    sm_factors_added++;
                } else if (task.type == OptimTaskType::BATCH_UPDATE && task.action) {
                    // BATCH_UPDATE 在锁外执行
                }
            }

            // 🔧 增强: 记录因子添加结果
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][DIAG] optLoop: added %d SM factors, pending_graph now size=%zu",
                sm_factors_added, pending_graph_.size());

            // 释放锁后再执行 BATCH_UPDATE 和 commit
            lk.unlock();

            // 执行 BATCH_UPDATE 任务
            for (const auto& task : batch_tasks) {
                if (task.type == OptimTaskType::BATCH_UPDATE && task.action) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[IncrementalOptimizer][DIAG] optLoop: executing BATCH_UPDATE action");
                    task.action();
                }
            }

            // 批量完成后统一 commit 一次
            if (!pending_graph_.empty() || !pending_values_.empty()) {
                BACKEND_STEP("step=optLoop_batch_commit pending_factors=%zu pending_values=%zu",
                    pending_graph_.size(), pending_values_.size());
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop batch: committing %zu factors %zu values",
                    pending_graph_.size(), pending_values_.size());
                commitAndUpdate();
            } else {
                BACKEND_STEP("step=optLoop_batch_skip reason=no_pending");
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[IncrementalOptimizer][DIAG] optLoop batch: nothing to commit (pending empty)");
            }
            batch_ok = true;

        } catch (const std::exception& e) {
            double task_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - task_t0).count();
            BACKEND_STEP("step=optLoop_batch_done tasks=%zu elapsed_ms=%.1f result=fail exception=%s",
                batch_tasks.size(), task_elapsed_ms, e.what());
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] optLoop batch failed after %.1fms: %s (grep EXCEPTION)",
                task_elapsed_ms, e.what());
            ALOG_ERROR(MOD, "optLoop batch failed: {}", e.what());
        } catch (...) {
            double task_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - task_t0).count();
            BACKEND_STEP("step=optLoop_batch_done tasks=%zu elapsed_ms=%.1f result=fail exception=unknown",
                batch_tasks.size(), task_elapsed_ms);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] optLoop batch unknown exception after %.1fms",
                task_elapsed_ms);
            ALOG_ERROR(MOD, "optLoop batch unknown exception");
        }

        double batch_elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - task_t0).count();
        if (batch_ok) {
            BACKEND_STEP("step=optLoop_batch_done tasks=%zu elapsed_ms=%.1f queue_remaining=%zu result=ok",
                batch_tasks.size(), batch_elapsed_ms, getQueueDepth());
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] optLoop batch done: tasks=%zu elapsed_ms=%.1f queue_remaining=%zu",
            batch_tasks.size(), batch_elapsed_ms, getQueueDepth());
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] optLoop exited");
}

void IncrementalOptimizer::enqueueOptTask(const OptimTask& task) {
    const size_t max_sz = static_cast<size_t>(ConfigManager::instance().maxOptimizationQueueSize());
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    if (opt_queue_.size() >= max_sz) {
        ALOG_WARN(MOD, "enqueueOptTask: queue full ({}), drop task", max_sz);
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][QUEUE] opt queue full (max=%zu), task dropped (grep BACKEND QUEUE)",
            max_sz);
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
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][BACKEND][QUEUE] opt queue full (max=%zu), %zu task(s) dropped (grep BACKEND QUEUE)",
            max_sz, dropped);
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

    // 🔧 诊断: 记录失败时的详细状态
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] recordOptimizationFailure: "
        "consecutive_failures=%d total=%d failed=%d "
        "node_exists_=%zu current_estimate_=%zu pending_values=%zu pending_factors=%zu "
        "error='%s'",
        consecutive_failures_.load(), total_optimizations_.load(), failed_optimizations_.load(),
        node_exists_.size(), current_estimate_.size(), pending_values_.size(), pending_graph_.size(),
        error_msg.c_str());

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
            "[IncrementalOptimizer][Health] CRITICAL: %d consecutive optimization failures, consider reset! "
            "(node_exists_=%zu current_estimate_=%zu)",
            consecutive_failures_.load(), node_exists_.size(), current_estimate_.size());
    }
}

void IncrementalOptimizer::recordOptimizationSuccess(double elapsed_ms) {
    consecutive_failures_ = 0;
    total_optimizations_++;
    last_success_time_ms_ = elapsed_ms;

    // 🔧 诊断: 记录成功时的状态
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] Optimization SUCCESS: "
        "elapsed_ms=%.2f total=%d failed=%d "
        "node_exists_=%zu current_estimate_=%zu pending_values=%zu pending_factors=%zu",
        elapsed_ms, total_optimizations_.load(), failed_optimizations_.load(),
        node_exists_.size(), current_estimate_.size(),
        pending_values_.size(), pending_graph_.size());

    ALOG_DEBUG(MOD, "[Health] Optimization SUCCESS: elapsed_ms=%.2f total=%d failed=%d",
               elapsed_ms, total_optimizations_.load(), failed_optimizations_.load());
}

void IncrementalOptimizer::resetForRecovery() {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);

    // 🔧 诊断: 记录恢复前的状态
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[IncrementalOptimizer][DIAG] resetForRecovery ENTER: "
        "node_exists_ size=%zu current_estimate_=%zu pending_values=%zu pending_factors=%zu "
        "node_count=%d factor_count=%d consecutive_failures=%d",
        node_exists_.size(), current_estimate_.size(), pending_values_.size(), pending_graph_.size(),
        node_count_, factor_count_, consecutive_failures_.load());

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

    // 重置计数器；1.3.2: 恢复即从干净 keyframe 链开始，清空 KF 映射与 pending GPS
    factor_count_ = 0;
    has_prior_ = false;
    keyframe_node_exists_.clear();
    keyframe_count_ = 0;
    last_keyframe_id_ = -1;
    pending_gps_factors_kf_.clear();
    MetricsRegistry::instance().setGauge(metrics::ISAM2_PENDING_GPS_KF, 0.0);

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
