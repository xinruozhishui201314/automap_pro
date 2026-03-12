#include "automap_pro/backend/hba_optimizer.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#define MOD "HBAOptimizer"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <string>
#include <cmath>
#include <cstdint>

#ifdef USE_HBA_API
#include <hba_api/hba_api.h>
#endif
#ifdef USE_GTSAM_FALLBACK
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#endif

namespace automap_pro {
#ifdef USE_GTSAM_FALLBACK
namespace {
constexpr double kMaxReasonableTranslationNorm = 1e6;

static gtsam::Pose3 toGtsamPose3(const Pose3d& T) {
    Eigen::Quaterniond q(T.rotation());
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
    gtsam::Point3 t(T.translation().x(), T.translation().y(), T.translation().z());
    return gtsam::Pose3(rot, t);
}
static Pose3d fromGtsamPose3(const gtsam::Pose3& p) {
    Pose3d T = Pose3d::Identity();
    T.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());
    const auto& q = p.rotation().toQuaternion();
    T.linear() = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
    return T;
}
static gtsam::Symbol KF(size_t i) { return gtsam::Symbol('x', static_cast<gtsam::Key>(i)); }
} // namespace
#endif

HBAOptimizer::HBAOptimizer() = default;

HBAOptimizer::~HBAOptimizer() { stop(); }

void HBAOptimizer::init() {}

void HBAOptimizer::start() {
    running_ = true;
    worker_thread_ = std::thread(&HBAOptimizer::workerLoop, this);
}

void HBAOptimizer::stop() {
    running_ = false;
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
}

void HBAOptimizer::onSubmapFrozen(const SubMap::Ptr& submap) {
    const auto& cfg = ConfigManager::instance();
    frozen_count_++;

    if (frozen_count_ % cfg.hbaTriggerSubmaps() != 0) return;
    if (!cfg.hbaOnLoop()) return;  // 周期性触发

    // 收集子图所有 KF（这里仅放入单个子图，实际由 AutoMapSystem 传入全量）
    std::lock_guard<std::mutex> lk(queue_mutex_);
    PendingTask task;
    for (const auto& kf : submap->keyframes) {
        task.keyframes.push_back(kf);
    }
    task.enable_gps = gps_aligned_;
    if (!task.keyframes.empty()) {
        size_t kf_count = task.keyframes.size();
        size_t qdepth = pending_queue_.size() + 1;
        pending_queue_.push(std::move(task));
        queue_cv_.notify_one();
        trigger_count_++;
        ALOG_INFO(MOD, "HBA triggered by frozen submap: kf_count={} queue_depth={}", kf_count, qdepth);
    }
}

void HBAOptimizer::triggerAsync(
    const std::vector<SubMap::Ptr>& all_submaps,
    bool wait)
{
    auto kfs = collectKeyFramesFromSubmaps(all_submaps);
    if (kfs.empty()) return;

    size_t kf_count = kfs.size();
    size_t sm_count = all_submaps.size();
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        PendingTask task;
        task.keyframes  = std::move(kfs);
        task.enable_gps = gps_aligned_;
        pending_queue_.push(std::move(task));
        queue_cv_.notify_one();
        trigger_count_++;
    }
    size_t queue_depth = 0;
    { std::lock_guard<std::mutex> lk(queue_mutex_); queue_depth = pending_queue_.size(); }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][STATE] enqueue submaps=%zu keyframes=%zu gps=%d trigger_count=%d queue_depth=%zu",
        sm_count, kf_count, gps_aligned_ ? 1 : 0, trigger_count_, queue_depth);
    ALOG_INFO(MOD, "HBA triggerAsync: submaps={} keyframes={} gps={} trigger_count={} queue_depth={}",
              sm_count, kf_count, gps_aligned_ ? 1 : 0, trigger_count_, queue_depth);

    if (wait) {
        // 设置合理超时：最多等待5分钟，避免永久阻塞导致析构卡死
        constexpr auto kMaxWaitTime = std::chrono::minutes(5);
        waitUntilIdleFor(kMaxWaitTime);
        if (!isIdle()) {
            ALOG_WARN(MOD, "HBA wait timeout after 5 minutes, forcing stop to avoid deadlock");
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[HBAOptimizer][TIMEOUT] HBA did not finish after 5 minutes, forcing stop");
        }
    }
}

bool HBAOptimizer::isIdle() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    return pending_queue_.empty() && !hba_running_.load();
}

size_t HBAOptimizer::queueDepth() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    size_t n = pending_queue_.size();
    if (hba_running_.load()) n += 1;
    return n;
}

void HBAOptimizer::waitUntilIdleFor(std::chrono::milliseconds timeout_ms) {
    // timeout_ms <= 0 视为使用最大合理等待时间，避免无限阻塞析构
    constexpr auto kMaxWaitTime = std::chrono::minutes(5);
    if (timeout_ms.count() <= 0) {
        timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(kMaxWaitTime);
    }

    auto deadline = std::chrono::steady_clock::now() + timeout_ms;
    while (std::chrono::steady_clock::now() < deadline && !isIdle()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void HBAOptimizer::onGPSAligned(
    const GPSAlignResult& align_result,
    const std::vector<SubMap::Ptr>& all_submaps)
{
    gps_aligned_     = true;
    gps_align_result_ = align_result;

    // GPS 对齐后立即触发一次全局优化（backend.hba.enabled=false 时跳过，仅 ISAM2+GPS）
    if (ConfigManager::instance().hbaEnabled())
        triggerAsync(all_submaps, false);
}

void HBAOptimizer::setGPSAlignedState(const GPSAlignResult& align_result) {
    gps_aligned_      = true;
    gps_align_result_ = align_result;
}

void HBAOptimizer::workerLoop() {
    while (running_) {
        PendingTask task;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] {
                return !pending_queue_.empty() || !running_;
            });
            if (!running_ && pending_queue_.empty()) break;

            // 取最新任务（丢弃旧任务，避免堆积）
            size_t old_size = pending_queue_.size();
            while (pending_queue_.size() > 1) pending_queue_.pop();
            task = std::move(pending_queue_.front());
            pending_queue_.pop();
            
            // 记录任务丢弃日志
            if (old_size > 1) {
                ALOG_WARN(MOD, "Dropped {} old HBA tasks to avoid backlog", old_size - 1);
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[HBAOptimizer][TASK_DROP] Dropped %zu old HBA tasks (queue size: %zu)",
                    static_cast<size_t>(old_size - 1), old_size);
            }
        }

        hba_running_ = true;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][STATE] optimization start keyframes=%zu gps=%d (running=1)",
            task.keyframes.size(), task.enable_gps ? 1 : 0);
        ALOG_INFO(MOD, "HBA optimization starting: kf_count={} gps={}",
                  task.keyframes.size(), task.enable_gps);
        AUTOMAP_TIMED_SCOPE(MOD, "HBA full optimize", 60000.0);
        HBAResult result = runHBA(task);
        hba_running_ = false;

        if (result.success) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][STATE] optimization done success=1 MME=%.4f elapsed=%.1fms poses=%zu (running=0)",
                result.final_mme, result.elapsed_ms, result.optimized_poses.size());
            ALOG_INFO(MOD, "HBA done: MME={:.4f} elapsed={:.1f}ms kf={}",
                      result.final_mme, result.elapsed_ms, result.optimized_poses.size());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[HBA][STATE] optimization done success=0 elapsed=%.1fms (running=0)", result.elapsed_ms);
            ALOG_ERROR(MOD, "HBA failed after {:.1f}ms", result.elapsed_ms);
        }
        for (auto& cb : done_cbs_) cb(result);
    }
}

HBAResult HBAOptimizer::runHBA(const PendingTask& task) {
    HBAResult result;
    result.success = false;

    // ========== 数据验证 ==========
    if (task.keyframes.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] No keyframes provided, skipping optimization");
        ALOG_WARN(MOD, "HBA aborted: no keyframes in task");
        return result;
    }

    // 检查关键帧数据完整性
    int valid_kf_count = 0;
    int empty_cloud_count = 0;
    int invalid_pose_count = 0;
    for (const auto& kf : task.keyframes) {
        if (!kf) { invalid_pose_count++; continue; }
        if (!kf->cloud_body || kf->cloud_body->empty()) {
            empty_cloud_count++;
            continue;
        }
        const auto& t = kf->T_w_b.translation();
        const auto& R = kf->T_w_b.rotation();
        if (!t.allFinite() || !R.allFinite()) {
            invalid_pose_count++;
            continue;
        }
        valid_kf_count++;
    }

    if (valid_kf_count == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] All keyframes invalid: empty=%d invalid_pose=%d total=%zu, skipping optimization",
            empty_cloud_count, invalid_pose_count, task.keyframes.size());
        ALOG_ERROR(MOD, "HBA aborted: all {} keyframes are invalid", task.keyframes.size());
        return result;
    }

    if (valid_kf_count < static_cast<int>(task.keyframes.size())) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] Some keyframes invalid: valid=%d empty_cloud=%d invalid_pose=%d total=%zu",
            valid_kf_count, empty_cloud_count, invalid_pose_count, task.keyframes.size());
    }

    // 检查时间跨度是否合理
    double time_span = task.keyframes.back()->timestamp - task.keyframes.front()->timestamp;
    if (time_span <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] Invalid time span: %.3f s, skipping optimization", time_span);
        return result;
    }

    // 时间跨度超过2小时视为异常
    if (time_span > 7200.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] Unusual time span: %.1f hours (>2h), proceeding anyway", time_span / 3600.0);
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][DATA] valid_keyframes=%d time_span=%.1fs gps=%d",
        valid_kf_count, time_span, task.enable_gps ? 1 : 0);
    ALOG_INFO(MOD, "HBA starting: valid_kf={} time_span={:.1f}s gps={}",
              valid_kf_count, time_span, task.enable_gps ? 1 : 0);

#ifdef USE_HBA_API
    const auto& cfg = ConfigManager::instance();
    hba_api::Config hba_cfg;
    hba_cfg.total_layer_num = cfg.hbaTotalLayers();
    hba_cfg.thread_num      = cfg.hbaThreadNum();
    hba_cfg.voxel_size      = 0.5;
    hba_cfg.enable_gps      = task.enable_gps;

    // 若启用 GPS，构建 GPS 条目（仅采纳 HIGH/EXCELLENT 质量，避免低质量拉偏）
    if (task.enable_gps && gps_aligned_) {
        for (const auto& kf : task.keyframes) {
            if (kf->has_valid_gps &&
                kf->gps.quality != GPSQuality::INVALID &&
                kf->gps.quality != GPSQuality::LOW) {
                hba_api::Config::GPSEntry entry;
                entry.timestamp = kf->gps.timestamp;
                entry.lat       = kf->gps.latitude;
                entry.lon       = kf->gps.longitude;
                entry.alt       = kf->gps.altitude;
                hba_cfg.gps_entries.push_back(entry);
            }
        }
    }

    hba_api::HBAOptimizer optimizer(hba_cfg);

    // 添加关键帧（按时间戳排序）
    std::vector<KeyFrame::Ptr> sorted_kfs = task.keyframes;
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    for (const auto& kf : sorted_kfs) {
        hba_api::KeyFrameInput input;
        input.timestamp   = kf->timestamp;
        input.rotation    = Eigen::Quaterniond(kf->T_w_b.rotation());
        input.translation = kf->T_w_b.translation();
        // 使用 cloud_body（body 系下），HBA 内部会用位姿变换到世界系
        if (kf->cloud_body && !kf->cloud_body->empty()) {
            // 转换 CloudXYZI → CloudXYZIN（HBA 需要 PointXYZINormal）
            auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
            for (const auto& pt : kf->cloud_body->points) {
                pcl::PointXYZINormal p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z;
                p.intensity = pt.intensity;
                cloud_in->push_back(p);
            }
            input.cloud = cloud_in;
        }
        optimizer.addKeyFrame(input);
    }

    // 执行 HBA 优化（进度输出到终端，与 [HBA][STATE] 一致）
    std::string params = "keyframes=" + std::to_string(task.keyframes.size()) + " gps=" + (task.enable_gps ? "1" : "0");
    GtsamCallScope scope(GtsamCaller::HBA, "PGO", params, true);
    auto api_result = optimizer.optimize([](int cur, int total, float pct) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][STATE] layer %d/%d %.0f%%", cur, total, pct);
    });
    scope.setSuccess(api_result.success);

    if (api_result.success) {
        // 将优化结果写回关键帧
        for (size_t i = 0; i < sorted_kfs.size() && i < api_result.optimized_poses.size(); ++i) {
            sorted_kfs[i]->T_w_b_optimized = api_result.optimized_poses[i];
        }
        result.success         = true;
        result.final_mme       = api_result.final_mme;
        result.elapsed_ms      = api_result.elapsed_ms;
        result.optimized_poses = api_result.optimized_poses;
    } else {
        result.success = false;
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] HBA API failed: %s (grep HBA BACKEND EXCEPTION)", api_result.error_msg.c_str());
        ALOG_ERROR(MOD, "HBA API failed: {}", api_result.error_msg);
        fprintf(stderr, "[HBAOptimizer] HBA failed: %s\n",
                api_result.error_msg.c_str());
    }

#elif defined(USE_GTSAM_FALLBACK)
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][CONFIG] backend.hba.enable_gtsam_fallback=%d (1=run GTSAM fallback, 0=skip; verify config file has enable_gtsam_fallback: true)",
        ConfigManager::instance().hbaGtsamFallbackEnabled() ? 1 : 0);
    if (!ConfigManager::instance().hbaGtsamFallbackEnabled()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] GTSAM fallback disabled (backend.hba.enable_gtsam_fallback=false), skipping HBA");
        ALOG_WARN(MOD, "HBA: GTSAM fallback disabled by config, skip");
        result.success = false;
    } else {
        result = runGTSAMFallback(task);
    }
#else
    result.success = false;
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[HBA][BACKEND] hba_api not available, skipping optimization (no changes applied)");
    fprintf(stderr, "[HBAOptimizer] hba_api not available, skipping optimization\n");
#endif

    return result;
}

#ifdef USE_GTSAM_FALLBACK
HBAResult HBAOptimizer::runGTSAMFallback(const PendingTask& task) {
    HBAResult result;
    result.success = false;
    if (task.keyframes.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][VALIDATION] GTSAM fallback: no keyframes, abort");
        return result;
    }

    std::vector<KeyFrame::Ptr> sorted_kfs = task.keyframes;
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    // 约束合理性：所有关键帧位姿有限且平移在合理范围内，否则不进入 GTSAM 避免崩溃
    auto poseForInitial = [](const KeyFrame::Ptr& kf) -> Pose3d {
        const Pose3d& o = kf->T_w_b_optimized;
        const Pose3d& t = kf->T_w_b;
        if ((o.translation() - t.translation()).norm() > 1e-9 || !o.rotation().isApprox(t.rotation()))
            return o;
        return t;
    };
    for (size_t i = 0; i < sorted_kfs.size(); ++i) {
        Pose3d p = poseForInitial(sorted_kfs[i]);
        const auto& t = p.translation();
        const auto& R = p.rotation();
        if (!t.allFinite() || !R.allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBA][BACKEND][VALIDATION] GTSAM fallback: keyframe[%zu] pose non-finite, abort (grep HBA VALIDATION)",
                i);
            return result;
        }
        if (t.norm() > kMaxReasonableTranslationNorm) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBA][BACKEND][VALIDATION] GTSAM fallback: keyframe[%zu] translation norm=%.1f > %.0f, abort",
                i, t.norm(), kMaxReasonableTranslationNorm);
            return result;
        }
    }

    ensureGtsamTbbSerialized();
    GtsamCallScope scope(GtsamCaller::HBA, "GTSAM_fallback",
                        "keyframes=" + std::to_string(sorted_kfs.size()) +
                        " gps=" + (task.enable_gps ? "1" : "0"), true);

    try {
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;
        // 记录每个因子的类型，用于崩溃时定位（与 ISAM2 双路 GTSAM 时 double free 诊断）
        std::vector<std::string> factor_type_log;

        // 每个因子使用独立噪声模型，避免多因子共享同一 shared_ptr 在 GTSAM 内触发 double free
        //（与 incremental_optimizer 及 FIX_GPS_BATCH_SIGSEGV 文档中 borglab/gtsam#1189 同类问题一致）
        const double prior_var = 1e-8;
        const double between_var = 0.01;

        // 后端优化后再 HBA：优先用 T_w_b_optimized 做初始值（当与 T_w_b 不同时表示已被后端/上一轮 HBA 更新）
        auto poseForInitial = [](const KeyFrame::Ptr& kf) -> Pose3d {
            const Pose3d& o = kf->T_w_b_optimized;
            const Pose3d& t = kf->T_w_b;
            if ((o.translation() - t.translation()).norm() > 1e-9 || !o.rotation().isApprox(t.rotation()))
                return o;
            return t;
        };

        // 使用命名变量作为方差向量，避免将 Eigen 临时量传入 Variances() 导致 GTSAM 内部悬垂引用
        //（LM 构造时 graph.error(initial) 会触发 NoiseModelFactor::error → double free，见 borglab/gtsam#1189 同类）
        gtsam::Vector6 prior_var6;
        prior_var6 << prior_var, prior_var, prior_var, prior_var, prior_var, prior_var;
        gtsam::Vector6 between_var6;
        between_var6 << between_var, between_var, between_var, between_var, between_var, between_var;

        for (size_t i = 0; i < sorted_kfs.size(); ++i) {
            Pose3d pose_i = poseForInitial(sorted_kfs[i]);
            initial.insert(KF(i), toGtsamPose3(pose_i));
            if (i == 0) {
                auto prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_var6);
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(KF(0), toGtsamPose3(pose_i), prior_noise));
                factor_type_log.push_back("Prior(k0)");
            } else {
                Pose3d pose_prev = poseForInitial(sorted_kfs[i - 1]);
                Pose3d rel = pose_prev.inverse() * pose_i;
                auto between_noise = gtsam::noiseModel::Diagonal::Variances(between_var6);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(KF(i - 1), KF(i), toGtsamPose3(rel), between_noise));
                factor_type_log.push_back("Between(k" + std::to_string(i - 1) + "-k" + std::to_string(i) + ")");
            }
        }

        size_t gps_factors_added = 0;
        if (task.enable_gps && gps_aligned_) {
            for (size_t i = 0; i < sorted_kfs.size(); ++i) {
                const auto& kf = sorted_kfs[i];
                if (!kf->has_valid_gps || kf->gps.quality == GPSQuality::INVALID || kf->gps.quality == GPSQuality::LOW)
                    continue;
                const auto& pos_enu = kf->gps.position_enu;
                if (!pos_enu.allFinite()) {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[HBA][GTSAM] skip GPS factor kf=%zu: non-finite position_enu", i);
                    ALOG_WARN(MOD, "HBA GTSAM: skip GPS kf={} non-finite position_enu", i);
                    continue;
                }
                // 与 iSAM2 一致：位姿在 map 系，GPS 观测也转换到 map 系（enu_to_map）
                Eigen::Vector3d pos_map = gps_align_result_.R_gps_lidar * pos_enu + gps_align_result_.t_gps_lidar;
                gtsam::Point3 pt(pos_map.x(), pos_map.y(), pos_map.z());
                Eigen::Matrix3d c = kf->gps.covariance;
                if (!c.allFinite()) {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[HBA][GTSAM] skip GPS factor kf=%zu: non-finite covariance", i);
                    ALOG_WARN(MOD, "HBA GTSAM: skip GPS kf={} non-finite covariance", i);
                    continue;
                }
                double v0 = std::max(1e-6, std::min(1e6, c(0, 0)));
                double v1 = std::max(1e-6, std::min(1e6, c(1, 1)));
                double v2 = std::max(1e-6, std::min(1e6, c(2, 2)));
                gtsam::Vector3 vars(v0, v1, v2);
                auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
                graph.add(gtsam::GPSFactor(KF(i), pt, noise));
                factor_type_log.push_back("GPS(k" + std::to_string(i) + ")");
                gps_factors_added++;
            }
            if (gps_factors_added > 0) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM][BACKEND] GPS positions in map frame (enu_to_map applied) gps_factors=%zu (grep BACKEND 定位坐标系)",
                    gps_factors_added);
            }
        }

        size_t n_factors = graph.size();
        size_t n_values = initial.size();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] graph built: factors=%zu values=%zu gps_factors=%zu (building LM optimizer...)",
            n_factors, n_values, gps_factors_added);
        ALOG_INFO(MOD, "HBA GTSAM: factors={} values={} gps_factors={}", n_factors, n_values, gps_factors_added);

        // 诊断：逐因子打印类型，便于崩溃时定位是哪一个 factor 在 error() 路径触发 double free
        for (size_t idx = 0; idx < graph.size(); ++idx) {
            const char* type_str = idx < factor_type_log.size() ? factor_type_log[idx].c_str() : "?";
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM] factor[%zu] type=%s", idx, type_str);
        }

        // 可选：在 LM 构造前逐因子计算 error，若崩溃则最后一条 factor_error 即肇事因子（与 LM 构造内 graph.error() 同路径）
        const char* diag_env = std::getenv("AUTOMAP_HBA_FACTOR_ERROR_DIAG");
        bool run_factor_error_diag = (diag_env && (std::string(diag_env) == "1" || std::string(diag_env) == "true"));
        if (run_factor_error_diag) {
            double total_err = 0;
            for (size_t idx = 0; idx < graph.size(); ++idx) {
                const char* type_str = idx < factor_type_log.size() ? factor_type_log[idx].c_str() : "?";
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM] factor_error idx=%zu type=%s (pre-call)", idx, type_str);
                fflush(stdout);
                double e = graph[idx]->error(initial);
                total_err += e * e;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM] factor_error idx=%zu type=%s err_sq=%.6g", idx, type_str, e * e);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM] factor_error total_sq=%.6g (LM constructor next)", total_err);
        }

        // 传入 graph/initial 的副本，避免 LM 内部持有对栈上对象的引用导致与 ISAM2 双路共用时的 double free（与 commitAndUpdate 中 graph_copy/values_copy 一致）
        gtsam::NonlinearFactorGraph graph_copy(graph);
        gtsam::Values initial_copy(initial);
        std::string key_sample;
        { size_t n = 0; for (gtsam::Key k : initial_copy.keys()) { if (n++) key_sample += ","; key_sample += std::to_string(k); if (n >= 5) { key_sample += ",..."; break; } } }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] pre-LM: graph_copy.size=%zu initial_copy.size=%zu initial_keys_sample=[%s]",
            graph_copy.size(), initial_copy.size(), key_sample.c_str());

        // 约束合理性：所有因子的 key 均在 initial 中，且所有 value 有限、平移在合理范围内
        bool key_ok = true;
        for (size_t idx = 0; idx < graph_copy.size() && key_ok; ++idx) {
            for (gtsam::Key k : graph_copy[idx]->keys()) {
                if (!initial_copy.exists(k)) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[HBA][BACKEND][VALIDATION] GTSAM fallback: factor[%zu] key %zu not in initial, abort",
                        idx, static_cast<size_t>(k));
                    key_ok = false;
                    break;
                }
            }
        }
        if (!key_ok) {
            ALOG_ERROR(MOD, "HBA GTSAM: factor key not in initial, skip");
            return result;
        }
        bool values_ok = true;
        for (gtsam::Key k : initial_copy.keys()) {
            try {
                auto p = initial_copy.at<gtsam::Pose3>(k);
                Eigen::Vector3d t = p.translation();
                Eigen::Matrix3d R = p.rotation().matrix();
                if (!t.array().isFinite().all() || !R.array().isFinite().all()) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[HBA][BACKEND][VALIDATION] GTSAM fallback: initial value key=%zu non-finite, abort",
                        static_cast<size_t>(k));
                    values_ok = false;
                    break;
                }
                if (t.norm() > kMaxReasonableTranslationNorm) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[HBA][BACKEND][VALIDATION] GTSAM fallback: initial value key=%zu translation norm=%.1f > %.0f, abort",
                        static_cast<size_t>(k), t.norm(), kMaxReasonableTranslationNorm);
                    values_ok = false;
                    break;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[HBA][BACKEND][VALIDATION] GTSAM fallback: initial value key=%zu exception: %s, abort",
                    static_cast<size_t>(k), e.what());
                values_ok = false;
                break;
            }
        }
        if (!values_ok) {
            ALOG_ERROR(MOD, "HBA GTSAM: initial values validation failed, skip");
            return result;
        }

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] LevenbergMarquardtOptimizer constructor enter (若崩溃在此后、无 exit→崩溃在 LM 构造/error 内)");
        fflush(stdout);

        gtsam::LevenbergMarquardtOptimizer opt(graph_copy, initial_copy);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] LevenbergMarquardtOptimizer constructor exit (LM built ok)");

        gtsam::Values optimized = opt.optimize();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] LM optimize() exit iterations done");

        for (size_t i = 0; i < sorted_kfs.size(); ++i) {
            if (optimized.exists(KF(i))) {
                sorted_kfs[i]->T_w_b_optimized = fromGtsamPose3(optimized.at<gtsam::Pose3>(KF(i)));
                result.optimized_poses.push_back(sorted_kfs[i]->T_w_b_optimized);
            }
        }
        result.success = true;
        result.elapsed_ms = 0.0;
        result.final_mme = 0.0;
        scope.setSuccess(true);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] GTSAM fallback done: poses=%zu", result.optimized_poses.size());
        ALOG_INFO(MOD, "HBA GTSAM fallback done: optimized_poses={}", result.optimized_poses.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] GTSAM fallback failed: %s (grep HBA BACKEND EXCEPTION)", e.what());
        ALOG_ERROR(MOD, "HBA GTSAM fallback exception: {}", e.what());
        fprintf(stderr, "[HBAOptimizer] GTSAM fallback exception: %s\n", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] GTSAM fallback unknown exception (若为 double free，检查 FIX_GPS_BATCH_SIGSEGV 与 pre-LM 日志，grep HBA BACKEND EXCEPTION)");
        ALOG_ERROR(MOD, "HBA GTSAM fallback: unknown exception");
        fprintf(stderr, "[HBAOptimizer] GTSAM fallback: unknown exception\n");
    }
    return result;
}
#endif

std::vector<KeyFrame::Ptr> HBAOptimizer::collectKeyFramesFromSubmaps(
    const std::vector<SubMap::Ptr>& submaps) const
{
    std::vector<KeyFrame::Ptr> kfs;
    for (const auto& sm : submaps) {
        if (!sm || sm->keyframes.empty()) continue;
        for (const auto& kf : sm->keyframes) {
            // 跳过无效关键帧：空点云或无效位姿
            if (!kf) continue;
            if (!kf->cloud_body || kf->cloud_body->empty()) continue;
            // 检查位姿有效性：平移和旋转必须是有限的
            const auto& t = kf->T_w_b.translation();
            const auto& R = kf->T_w_b.rotation();
            if (!t.allFinite() || !R.allFinite()) continue;
            kfs.push_back(kf);
        }
    }

    // 按时间戳排序
    std::sort(kfs.begin(), kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    // 去重：相同时间戳只保留一个
    if (kfs.size() > 1) {
        std::vector<KeyFrame::Ptr> unique_kfs;
        unique_kfs.push_back(kfs[0]);
        for (size_t i = 1; i < kfs.size(); ++i) {
            if (std::abs(kfs[i]->timestamp - unique_kfs.back()->timestamp) > 0.001) {
                unique_kfs.push_back(kfs[i]);
            }
        }
        kfs.swap(unique_kfs);
    }

    return kfs;
}

} // namespace automap_pro
