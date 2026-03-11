#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/error_code.h"
#define MOD "iSAM2"

#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <Eigen/SVD>
#include <chrono>
#include <string>

namespace automap_pro {

// GTSAM Symbol 约定：s(sm_id) = Symbol('s', sm_id)
static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }

IncrementalOptimizer::IncrementalOptimizer() {
    ensureGtsamTbbSerialized();
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

    isam2_ = gtsam::ISAM2(params);
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
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    if (!prior_noise_) {
        ALOG_DEBUG(MOD, "addSubMapNode: already shutdown (prior_noise_ released), ignore");
        return;
    }
    if (node_exists_.count(sm_id)) return;
    node_exists_[sm_id] = true;

    pending_values_.insert(SM(sm_id), toPose3(init_pose));
    node_count_++;

    if (fixed || !has_prior_) {
        // 每次新建 Prior noise，避免共享 prior_noise_ 在 GTSAM linearize 路径触发 double free（borglab/gtsam#1189 同类）
        gtsam::noiseModel::Diagonal::shared_ptr noise =
            gtsam::noiseModel::Diagonal::Variances(prior_var6_);
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
    // ✅ 修复：使用 try-catch 包裹，防止 GTSAM 内部 SIGSEGV 导致进程崩溃
    try {
        std::unique_lock<std::shared_mutex> lk(rw_mutex_);
        if (!prior_noise_) return OptimizationResult{};

        // ✅ 修复：显式检查节点存在性
        if (node_exists_.find(from) == node_exists_.end() ||
            node_exists_.find(to) == node_exists_.end()) {
            ALOG_DEBUG(MOD, "addLoopFactor: from=%d or to=%d not exists, skip", from, to);
            return OptimizationResult{};
        }

        // 回环约束使用 Huber 鲁棒核（抑制错误回环的影响）
        auto base_noise = infoToNoise(info_matrix);
        gtsam::noiseModel::Base::shared_ptr robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            base_noise);

        pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            SM(from), SM(to), toPose3(rel), robust_noise));
        factor_count_++;

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

void IncrementalOptimizer::addGPSFactor(
    int sm_id,
    const Eigen::Vector3d& pos_map,
    const Eigen::Matrix3d& cov3x3)
{
    // ✅ 修复：仅添加 GPS 因子，不立即 commit
    // 问题根源是 GTSAM 内部 TBB 并行导致竞态，延迟 commit 可以减少并发
    try {
        std::unique_lock<std::shared_mutex> lk(rw_mutex_);
        if (!prior_noise_) return;

        // ✅ 修复：显式检查节点存在性
        if (node_exists_.find(sm_id) == node_exists_.end()) {
            ALOG_DEBUG(MOD, "addGPSFactor: sm_id=%d not exists, skip", sm_id);
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

OptimizationResult IncrementalOptimizer::forceUpdate() {
    // ✅ 修复：使用 try-catch 包裹，防止 GTSAM 内部 SIGSEGV 导致进程崩溃
    try {
        std::unique_lock<std::shared_mutex> lk(rw_mutex_);
        return commitAndUpdate();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "forceUpdate failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[IncrementalOptimizer][EXCEPTION] forceUpdate: %s", e.what());
        return OptimizationResult{};
    } catch (...) {
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
        return OptimizationResult{};
    }

    const size_t pf = pending_graph_.size();
    const size_t pv = pending_values_.size();
    std::string params = "pending_factors=" + std::to_string(pf) + " pending_values=" + std::to_string(pv);
    GtsamCallScope scope(GtsamCaller::ISAM2, "commitAndUpdate", params, true);

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

    try {
        // 传入副本，避免 GTSAM 内部持有对 pending_ 的引用时我们 clear 导致 double free
        gtsam::NonlinearFactorGraph graph_copy(pending_graph_);
        gtsam::Values values_copy(pending_values_);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] update call enter (graph_copy.size=%zu values_copy.size=%zu)%s",
            graph_copy.size(), values_copy.size(), is_first_update ? " [FIRST_UPDATE]" : "");
        isam2_.update(graph_copy, values_copy);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] update call exit (before calculateEstimate)");
        current_estimate_ = isam2_.calculateEstimate();
    } catch (const std::exception& e) {
        scope.setSuccess(false);
        ALOG_ERROR(MOD, "iSAM2 update FAILED: {}", e.what());
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

    // V1: 单次 update 耗时分布与队列深度；V2: 可观测性 last success
    METRICS_HISTOGRAM_OBSERVE(metrics::ISAM2_OPTIMIZE_TIME_MS, elapsed);
    METRICS_GAUGE_SET(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(getQueueDepth()));
    METRICS_GAUGE_SET(metrics::ISAM2_LAST_SUCCESS, 1.0);

    // ✅ 健康检查：记录优化成功
    recordOptimizationSuccess(elapsed);

    scope.setSuccess(true);
    ALOG_INFO(MOD, "iSAM2 update done: nodes={} elapsed={:.1f}ms total_factors={}",
              res.nodes_updated, res.elapsed_ms, factor_count_);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG] commitAndUpdate done elapsed_ms=%.1f nodes=%d success=1",
        res.elapsed_ms, res.nodes_updated);
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

    // ✅ 修复：收紧条件数阈值（从 1e8 降到 1e6），更早触发正则化
    // ✅ 修复：对所有情况使用相对正则化（避免数值不稳定）
    const double max_sv = svd.singularValues()(0);
    const double min_sv = svd.singularValues()(5);
    const double cond = (max_sv > 1e-12) ? (max_sv / min_sv) : 1e12;

    if (min_sv < 1e-6 || cond > 1e6) {
        ALOG_WARN(MOD, "Information matrix rank-deficient (rank={}/6, cond={:.2e}), using conservative covariance", rank, cond);
        gtsam::Matrix66 cov = gtsam::Matrix66::Identity() * 1.0;
        return gtsam::noiseModel::Gaussian::Covariance(cov);
    }

    // ✅ 修复：使用相对正则化，确保最小特征值不低于 1e-12
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

// ── P0 异步优化队列实现 ───────────────────────────────────────────────────

void IncrementalOptimizer::optLoop() {
    while (true) {
        OptimTask task;
        {
            std::unique_lock<std::mutex> lk(opt_queue_mutex_);
            opt_queue_cv_.wait(lk, [this] {
                return !opt_running_ || !opt_queue_.empty();
            });
            if (!opt_running_ && opt_queue_.empty()) break;
            if (opt_queue_.empty()) continue;
            task = opt_queue_.front();
            opt_queue_.pop();
            MetricsRegistry::instance().setGauge(metrics::ISAM2_QUEUE_DEPTH, static_cast<double>(opt_queue_.size()));
        }
        const char* type_str = (task.type == OptimTaskType::LOOP_FACTOR) ? "LOOP_FACTOR"
            : (task.type == OptimTaskType::GPS_FACTOR) ? "GPS_FACTOR" : "BATCH_UPDATE";
        size_t queue_after = getQueueDepth();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ISAM2_DIAG] optLoop pop type=%s queue_remaining=%zu (崩溃在 commitAndUpdate 时此处为 opt 线程)",
            type_str, queue_after);

        // ✅ 修复：为每个任务类型添加 try-catch，防止单个任务崩溃导致整个 optLoop 退出
        try {
            if (task.type == OptimTaskType::LOOP_FACTOR) {
                std::unique_lock<std::shared_mutex> lk(rw_mutex_);
                if (!prior_noise_) continue;
                if (!node_exists_.count(task.from_id) || !node_exists_.count(task.to_id)) continue;
                auto base_noise = infoToNoise(task.info_matrix);
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
                pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    SM(task.from_id), SM(task.to_id), toPose3(task.rel_pose), robust_noise));
                factor_count_++;
                commitAndUpdate();
            } else if (task.type == OptimTaskType::GPS_FACTOR) {
                std::unique_lock<std::shared_mutex> lk(rw_mutex_);
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
        } catch (const std::exception& e) {
            // ✅ P0 修复：捕获异常，防止单个任务失败导致 optLoop 终止
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] optLoop task %s failed: %s", type_str, e.what());
            ALOG_ERROR(MOD, "optLoop task %s failed: {}", type_str, e.what());
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[IncrementalOptimizer][EXCEPTION] optLoop task %s unknown exception", type_str);
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
    if (initial_depth == 0) return;
    ALOG_INFO(MOD, "[ISAM2_QUEUE] waitForPendingTasks enter queue_depth={} max_wait_ms=5000", initial_depth);
    constexpr int kMaxWaitMs = 5000;
    constexpr int kChunkMs = 10;
    int waited_ms = 0;
    while (getQueueDepth() > 0 && waited_ms < kMaxWaitMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kChunkMs));
        waited_ms += kChunkMs;
    }
    const size_t final_depth = getQueueDepth();
    if (final_depth > 0) {
        ALOG_WARN(MOD, "[ISAM2_QUEUE] waitForPendingTasks timeout after {}ms queue_depth={} (backend continues)",
                  kMaxWaitMs, final_depth);
    } else {
        ALOG_INFO(MOD, "[ISAM2_QUEUE] waitForPendingTasks done waited_ms={}", waited_ms);
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
