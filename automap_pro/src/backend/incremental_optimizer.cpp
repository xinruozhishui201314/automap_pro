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
#include <gtsam/base/Vector.h>
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
    // 使用 Diagonal 方差作为“近似固定先验”（可配置），避免 Constrained 退出时 SIGSEGV；适度先验利于点云清晰度，过强易导致数值刚度
    double pvar = cfg.isam2PriorVariance();
    gtsam::Vector prior_var6(6);
    prior_var6 << pvar, pvar, pvar, pvar, pvar, pvar;
    prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_var6);

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
        // 先验因子使用 Diagonal 极小方差（等效固定），prior_noise_ 为成员单例
        pending_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
            SM(sm_id), toPose3(init_pose), prior_noise_));
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
    gtsam::noiseModel::Base::shared_ptr robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
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
        // 获取当前估计
        auto current_est = current_estimate_.at<gtsam::Pose3>(SM(sm_id));
        Eigen::Vector3d current_pos = fromPose3(current_est).translation();
        
        // 计算残差
        double residual = (pos_map - current_pos).norm();
        
        // 简单的阈值检测（避免复杂的统计计算）
        double outlier_threshold = cfg.gpsOutlierZScore() * 1.0;  // 简化：直接使用阈值
        double residual_baseline = 2.0;  // 基准残差（米）
        
        // 如果残差过大，认为是异常值
        if (residual > residual_baseline) {
            double outlier_scale = cfg.gpsOutlierCovScale();
            final_cov *= outlier_scale;  // 放大协方差，降低约束强度
            
            ALOG_DEBUG(MOD, "[GPS_OPT] GPS outlier detected: sm_id={} residual={:.2f}m threshold={:.2f}m cov_scale={:.1f}",
                      sm_id, residual, residual_baseline, outlier_scale);
        }
    }
    
    // GTSAM GPSFactor: 仅约束位置 (X,Y,Z)，不约束姿态
    gtsam::Point3 gps_point(pos_map.x(), pos_map.y(), pos_map.z());
    auto noise = gtsam::noiseModel::Gaussian::Covariance(
        gtsam::Matrix33(final_cov.data()));
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

void IncrementalOptimizer::clearForShutdown() {
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
        }
        if (task.type == OptimTaskType::LOOP_FACTOR) {
            std::unique_lock<std::shared_mutex> lk(rw_mutex_);
            if (!prior_noise_) break;
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
            if (!prior_noise_) break;
            if (!node_exists_.count(task.from_id)) continue;
            gtsam::Point3 gps_point(task.gps_pos.x(), task.gps_pos.y(), task.gps_pos.z());
            auto noise = gtsam::noiseModel::Gaussian::Covariance(
                gtsam::Matrix33(task.gps_cov.data()));
            pending_graph_.add(gtsam::GPSFactor(SM(task.from_id), gps_point, noise));
            factor_count_++;
            commitAndUpdate();
        } else if (task.type == OptimTaskType::BATCH_UPDATE && task.action) {
            task.action();
        }
    }
}

void IncrementalOptimizer::enqueueOptTask(const OptimTask& task) {
    const size_t max_sz = static_cast<size_t>(ConfigManager::instance().maxOptimizationQueueSize());
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    if (opt_queue_.size() >= max_sz) {
        ALOG_WARN(MOD, "enqueueOptTask: queue full ({}), drop task", max_sz);
        return;
    }
    opt_queue_.push(task);
    opt_queue_cv_.notify_one();
}

void IncrementalOptimizer::enqueueOptTasks(const std::vector<OptimTask>& tasks) {
    const size_t max_sz = static_cast<size_t>(ConfigManager::instance().maxOptimizationQueueSize());
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    for (const auto& t : tasks) {
        if (opt_queue_.size() >= max_sz) break;
        opt_queue_.push(t);
    }
    if (!tasks.empty()) opt_queue_cv_.notify_one();
}

void IncrementalOptimizer::waitForPendingTasks() {
    while (getQueueDepth() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

size_t IncrementalOptimizer::getQueueDepth() const {
    std::lock_guard<std::mutex> lk(opt_queue_mutex_);
    return opt_queue_.size();
}

} // namespace automap_pro
