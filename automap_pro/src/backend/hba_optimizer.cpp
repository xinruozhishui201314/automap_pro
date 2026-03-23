#include "automap_pro/backend/hba_optimizer.h"
#include "automap_pro/backend/gps_constraint_policy.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#define MOD "HBAOptimizer"
/** 建图后端每步计算日志：与 incremental_optimizer 统一 tag [BACKEND_STEP]，便于 grep 定位 */
#define BACKEND_STEP(fmt, ...) \
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[BACKEND_STEP] " fmt, ##__VA_ARGS__)

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <string>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <vector>

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
#include <yaml-cpp/yaml.h>

namespace automap_pro {
namespace {
std::atomic<uint64_t> g_hba_loop_added_total{0};
std::atomic<uint64_t> g_hba_gps_added_total{0};

// 与 ConfigManager::gpsLeverArmImu 同源；主配置未给出非零杆臂时，按主 YAML 所在目录回退 sensor_config/gps_imu_extrinsic.yaml（避免依赖进程 CWD）
void resolveGpsLeverArmForHba(Eigen::Vector3d& lever_arm) {
    lever_arm = Eigen::Vector3d::Zero();
    try {
        const auto& cfg = ConfigManager::instance();
        if (cfg.isLoaded()) {
            lever_arm = cfg.gpsLeverArmImu();
            if (lever_arm.norm() > 1e-12) {
                return;
            }
        }
    } catch (const std::exception&) {
        // 回退到外参文件
    }

    std::vector<std::filesystem::path> candidates;
    try {
        const auto& cfg = ConfigManager::instance();
        if (cfg.isLoaded() && !cfg.configFilePath().empty()) {
            std::filesystem::path base(cfg.configFilePath());
            candidates.push_back(base.parent_path() / "sensor_config" / "gps_imu_extrinsic.yaml");
        }
    } catch (...) {
    }
    candidates.emplace_back("automap_pro/config/sensor_config/gps_imu_extrinsic.yaml");

    for (const auto& extrinsic_path : candidates) {
        try {
            if (extrinsic_path.empty() || !std::filesystem::exists(extrinsic_path)) {
                continue;
            }
            YAML::Node config = YAML::LoadFile(extrinsic_path.string());
            if (config["T_gps_imu"] && config["T_gps_imu"]["translation"]) {
                auto trans = config["T_gps_imu"]["translation"];
                lever_arm = Eigen::Vector3d(trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>());
                return;
            }
        } catch (const std::exception&) {
            continue;
        }
    }
}
} // namespace

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

void HBAOptimizer::init() {
    resolveGpsLeverArmForHba(lever_arm_);
    if (lever_arm_.norm() > 1e-12) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA] GPS lever arm (gps.lever_arm_imu or sensor_config/gps_imu_extrinsic.yaml next to config): [%.4f, %.4f, %.4f] m",
            lever_arm_.x(), lever_arm_.y(), lever_arm_.z());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA] GPS lever arm is zero (set gps.lever_arm_imu in main YAML or T_gps_imu.translation in config_dir/sensor_config/gps_imu_extrinsic.yaml)");
    }
}

void HBAOptimizer::start() {
    worker_thread_finished_.store(false, std::memory_order_release);
    running_ = true;
    worker_thread_ = std::thread(&HBAOptimizer::workerLoop, this);
}

void HBAOptimizer::stop() {
    running_ = false;
    queue_cv_.notify_all();
    // 🏛️ [修复] 使用带超时的 join，防止 HBA 优化任务（可能耗时较长）在程序退出时造成阻塞
    // 理由：AutoMapSystem 析构时会调用此函数。若 HBA 正在运行且 keyframe 数量巨大，
    // 原生的 join() 会导致整个进程卡死在此处，无法完成最后的日志落地。
    stopJoinWithTimeout(std::chrono::seconds(5));
}

void HBAOptimizer::stopJoinWithTimeout(std::chrono::milliseconds max_join) {
    running_ = false;
    queue_cv_.notify_all();
    if (!worker_thread_.joinable()) return;
    const auto deadline = std::chrono::steady_clock::now() + max_join;
    while (std::chrono::steady_clock::now() < deadline) {
        if (worker_thread_finished_.load(std::memory_order_acquire)) {
            worker_thread_.join();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
        "[HBAOptimizer][SHUTDOWN] stopJoinWithTimeout: worker did not finish within %lld ms (likely blocked inside runHBA). "
        "Detaching HBA worker thread — system state: HBA worker abandoned; process exit should follow immediately.",
        static_cast<long long>(max_join.count()));
    worker_thread_.detach();
}

void HBAOptimizer::onSubmapFrozen(const SubMap::Ptr& submap) {
    BACKEND_STEP("step=HBA_onSubmapFrozen_enter sm_id=%d frozen_count=%d kf_count=%zu",
        submap->id, frozen_count_ + 1, submap->keyframes.size());
    const auto& cfg = ConfigManager::instance();
    frozen_count_++;

    // 增强诊断日志：记录触发条件检查
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][CHECK] onSubmapFrozen: frozen_count=%d trigger_mod=%d hbaOnLoop=%d hbaEnabled=%d gps_aligned=%d",
        frozen_count_, cfg.hbaTriggerSubmaps(), cfg.hbaOnLoop() ? 1 : 0,
        ConfigManager::instance().hbaEnabled() ? 1 : 0, gps_aligned_ ? 1 : 0);

    if (frozen_count_ % cfg.hbaTriggerSubmaps() != 0) {
        BACKEND_STEP("step=HBA_onSubmapFrozen_skip sm_id=%d reason=trigger_mod frozen_count=%d mod=%d",
            submap->id, frozen_count_, cfg.hbaTriggerSubmaps());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][CHECK] skip: frozen_count=%d mod %d != 0",
            frozen_count_, cfg.hbaTriggerSubmaps());
        return;
    }

    if (!cfg.hbaOnLoop()) {
        BACKEND_STEP("step=HBA_onSubmapFrozen_skip sm_id=%d reason=hbaOnLoop_false", submap->id);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][CHECK] skip: hbaOnLoop=false");
        return;
    }

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
        BACKEND_STEP("step=HBA_onSubmapFrozen_trigger sm_id=%d kf_count=%zu queue_depth=%zu result=ok",
            submap->id, kf_count, qdepth);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][TRIGGER] onSubmapFrozen: sm_id=%d kf_count=%zu queue_depth=%zu trigger_count=%d gps_aligned=%d",
            submap->id, kf_count, qdepth, trigger_count_, gps_aligned_ ? 1 : 0);
        ALOG_INFO(MOD, "HBA triggered by frozen submap: kf_count={} queue_depth={}", kf_count, qdepth);
    }
}

void HBAOptimizer::triggerAsync(
    const std::vector<SubMap::Ptr>& all_submaps,
    const std::vector<LoopConstraint::Ptr>& loops,
    bool wait,
    const char* trigger_source)
{
    auto kfs = collectKeyFramesFromSubmaps(all_submaps);
    if (kfs.empty()) return;

    // 建立 SubMap ID 到其首个有效关键帧全局 ID 的映射，用于解析回环约束
    std::unordered_map<int, uint64_t> sm_id_to_anchor_kf_id;
    std::unordered_map<int, SubMap::Ptr> sm_id_to_ptr;
    for (const auto& sm : all_submaps) {
        if (!sm) continue;
        sm_id_to_ptr[sm->id] = sm;
        for (const auto& kf : sm->keyframes) {
            if (kf && kf->cloud_body && !kf->cloud_body->empty() && 
                kf->T_odom_b.translation().allFinite() && kf->T_odom_b.rotation().matrix().allFinite()) {
                sm_id_to_anchor_kf_id[sm->id] = kf->id;
                break; // 找到第一个有效的关键帧作为该子图在 HBA 中的锚点
            }
        }
    }

    // 解析回环约束中的局部索引为全局 ID
    std::vector<LoopConstraint::Ptr> resolved_loops;
    size_t loops_total = 0;
    size_t loops_already_kf = 0;
    size_t loops_intra_resolved = 0;
    size_t loops_inter_resolved = 0;
    size_t loops_unresolved = 0;
    size_t loops_degenerate_same_node = 0;
    for (const auto& lc : loops) {
        if (!lc) continue;
        loops_total++;
        // 创建副本，避免修改原始回环数据（虽然是 shared_ptr，但为了安全建议新建）
        auto resolved_lc = std::make_shared<LoopConstraint>(*lc);
        bool resolved = false;

        // 情况1：已经是关键帧级回环（已有全局 ID）
        if (lc->keyframe_global_id_i >= 0 && lc->keyframe_global_id_j >= 0) {
            resolved = true;
            loops_already_kf++;
        }
        // 情况2：子图内回环（submap_i == submap_j）
        else if (lc->submap_i == lc->submap_j) {
            auto it = sm_id_to_ptr.find(lc->submap_i);
            if (it != sm_id_to_ptr.end() && !it->second->keyframes.empty()) {
                const auto& sm_kfs = it->second->keyframes;
                if (lc->keyframe_i >= 0 && lc->keyframe_i < (int)sm_kfs.size() &&
                    lc->keyframe_j >= 0 && lc->keyframe_j < (int)sm_kfs.size()) {
                    resolved_lc->keyframe_global_id_i = sm_kfs[lc->keyframe_i]->id;
                    resolved_lc->keyframe_global_id_j = sm_kfs[lc->keyframe_j]->id;
                    resolved = true;
                    loops_intra_resolved++;
                }
            }
        }
        // 情况3：子图间回环（submap_i != submap_j）
        else {
            auto it_i = sm_id_to_anchor_kf_id.find(lc->submap_i);
            auto it_j = sm_id_to_anchor_kf_id.find(lc->submap_j);
            if (it_i != sm_id_to_anchor_kf_id.end() && it_j != sm_id_to_anchor_kf_id.end()) {
                resolved_lc->keyframe_global_id_i = it_i->second;
                resolved_lc->keyframe_global_id_j = it_j->second;
                resolved = true;
                loops_inter_resolved++;
            }
        }

        if (resolved && resolved_lc->keyframe_global_id_i == resolved_lc->keyframe_global_id_j) {
            loops_degenerate_same_node++;
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_RESOLVE] drop degenerate loop resolved to same keyframe id=%d (sm%d/kf%d <-> sm%d/kf%d)",
                resolved_lc->keyframe_global_id_i, lc->submap_i, lc->keyframe_i, lc->submap_j, lc->keyframe_j);
            continue;
        }
        if (resolved) {
            resolved_loops.push_back(resolved_lc);
        } else {
            loops_unresolved++;
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_RESOLVE] failed to resolve loop sm%d(kf%d) <-> sm%d(kf%d)",
                lc->submap_i, lc->keyframe_i, lc->submap_j, lc->keyframe_j);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][LOOP_RESOLVE] total=%zu resolved=%zu already_kf=%zu intra=%zu inter=%zu unresolved=%zu degenerate_same_node=%zu",
        loops_total, resolved_loops.size(), loops_already_kf, loops_intra_resolved, loops_inter_resolved, loops_unresolved, loops_degenerate_same_node);

    size_t kf_count = kfs.size();
    size_t sm_count = all_submaps.size();
    size_t loop_count = resolved_loops.size();
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        PendingTask task;
        task.keyframes  = std::move(kfs);
        task.loops      = std::move(resolved_loops);
        task.enable_gps = gps_aligned_;
        pending_queue_.push(std::move(task));
        queue_cv_.notify_one();
        trigger_count_++;
    }
    size_t queue_depth = 0;
    { std::lock_guard<std::mutex> lk(queue_mutex_); queue_depth = pending_queue_.size(); }
    const char* src = trigger_source ? trigger_source : "unknown";
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][STATE] enqueue source=%s submaps=%zu keyframes=%zu loops=%zu gps=%d trigger_count=%d queue_depth=%zu (重影诊断: 同一轮建图内 trigger_count>1 表示被多次触发)",
        src, sm_count, kf_count, loop_count, gps_aligned_ ? 1 : 0, trigger_count_, queue_depth);
    ALOG_INFO(MOD, "HBA triggerAsync: source={} submaps={} keyframes={} loops={} gps={} trigger_count={} queue_depth={}",
              src, sm_count, kf_count, loop_count, gps_aligned_ ? 1 : 0, trigger_count_, queue_depth);

    if (wait) {
        // 设置合理超时：最多等待5分钟，避免永久阻塞导致析构卡死
        constexpr auto kMaxWaitTime = std::chrono::minutes(5);
        waitUntilIdleFor(kMaxWaitTime);
        if (!isIdle()) {
            ALOG_ERROR(MOD, "HBA wait timeout after 5 minutes — save/stop may race with running HBA (use stopJoinWithTimeout on shutdown)");
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBAOptimizer][TIMEOUT] HBA did not become idle after 5 minutes (wait=true). "
                "saveMapToFiles / shutdown will use stopJoinWithTimeout to avoid unbounded join; expect possible abandoned HBA worker if still blocked in runHBA.");
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
    const std::vector<SubMap::Ptr>& all_submaps,
    const std::vector<LoopConstraint::Ptr>& loops)
{
    gps_aligned_     = true;
    gps_align_result_ = align_result;

    // GPS 对齐后立即触发一次全局优化（backend.hba.enabled=false 时跳过，仅 ISAM2+GPS）
    if (ConfigManager::instance().hbaEnabled())
        triggerAsync(all_submaps, loops, false, "onGPSAligned");
}

void HBAOptimizer::setGPSAlignedState(const GPSAlignResult& align_result) {
    gps_aligned_      = align_result.success;
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
        BACKEND_STEP("step=HBA_workerLoop_start keyframes=%zu gps=%d", task.keyframes.size(), task.enable_gps ? 1 : 0);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][STATE] optimization start keyframes=%zu gps=%d (running=1)",
            task.keyframes.size(), task.enable_gps ? 1 : 0);
        ALOG_INFO(MOD, "HBA optimization starting: kf_count={} gps={}",
                  task.keyframes.size(), task.enable_gps);
        AUTOMAP_TIMED_SCOPE(MOD, "HBA full optimize", 60000.0);
        HBAResult result = runHBA(task);

        if (result.success) {
            BACKEND_STEP("step=HBA_workerLoop_done success=1 MME=%.4f elapsed_ms=%.1f poses=%zu",
                result.final_mme, result.elapsed_ms, result.optimized_poses.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][STATE] optimization done success=1 MME=%.4f elapsed=%.1fms poses=%zu (running=1 until callbacks done)",
                result.final_mme, result.elapsed_ms, result.optimized_poses.size());
            ALOG_INFO(MOD, "HBA done: MME={:.4f} elapsed={:.1f}ms kf={}",
                      result.final_mme, result.elapsed_ms, result.optimized_poses.size());
        } else {
            BACKEND_STEP("step=HBA_workerLoop_done success=0 elapsed_ms=%.1f", result.elapsed_ms);
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[HBA][STATE] optimization done success=0 elapsed=%.1fms (running=1 until callbacks done)", result.elapsed_ms);
            ALOG_ERROR(MOD, "HBA failed after {:.1f}ms", result.elapsed_ms);
        }
        // 先执行回调（含 updateAllFromHBA + rebuildMergedCloudFromOptimizedPoses），再置 idle，
        // 保证 finish 线程的 waitUntilIdleFor() 在 rebuild 完成后才返回，避免 save 与 rebuild 竞态导致重影（见 docs/HBA_GHOSTING_ROOT_CAUSE_20260317.md）
        for (auto& cb : done_cbs_) cb(result);
        hba_running_ = false;
    }
    worker_thread_finished_.store(true, std::memory_order_release);
}

HBAResult HBAOptimizer::runHBA(const PendingTask& task) {
    BACKEND_STEP("step=HBA_runHBA_enter keyframes=%zu gps=%d", task.keyframes.size(), task.enable_gps ? 1 : 0);
    HBAResult result;
    result.success = false;

    // ========== 数据验证 ==========
    if (task.keyframes.empty()) {
        BACKEND_STEP("step=HBA_runHBA_skip reason=no_keyframes");
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
        const auto& t = kf->T_odom_b.translation();
        const auto& R = kf->T_odom_b.rotation();
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

    BACKEND_STEP("step=HBA_runHBA_validation_done valid_kf=%d time_span=%.1fs gps=%d",
        valid_kf_count, time_span, task.enable_gps ? 1 : 0);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][DATA] valid_keyframes=%d time_span=%.1fs gps=%d",
        valid_kf_count, time_span, task.enable_gps ? 1 : 0);
    ALOG_INFO(MOD, "HBA starting: valid_kf={} time_span={:.1f}s gps={}",
              valid_kf_count, time_span, task.enable_gps ? 1 : 0);

    // [PCD_GHOSTING_VERIFY] HBA 输入顺序：与 updateAllFromHBA 的 WRITEBACK_ORDER 应对齐（同为 collect 的 filter+sort+dedupe 顺序）；若 first/last kf_id+ts 不一致则写回错位→PCD 重影（grep HBA_INPUT_ORDER WRITEBACK_ORDER）
    {
        const auto& kfs = task.keyframes;
        if (!kfs.empty()) {
            const auto& f = kfs.front();
            const auto& b = kfs.back();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA_INPUT_ORDER] first kf_id=%lu sm_id=%d ts=%.3f last kf_id=%lu sm_id=%d ts=%.3f count=%zu (与 WRITEBACK_ORDER 应对齐，否则写回错位)",
                f->id, f->submap_id, f->timestamp, b->id, b->submap_id, b->timestamp, kfs.size());
        }
    }

    // 与 ConfigManager::gpsLeverArmImu 同源刷新，避免 HBA 侧杆臂与主配置脱节
    resolveGpsLeverArmForHba(lever_arm_);

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
                gpsQualityAcceptedByPolicy(kf->gps.quality, cfg)) {
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
        input.rotation    = Eigen::Quaterniond(kf->T_odom_b.rotation());
        input.translation = kf->T_odom_b.translation();
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
        BACKEND_STEP("step=HBA_runHBA_done backend=api success=1 MME=%.4f elapsed_ms=%.1f poses=%zu",
            api_result.final_mme, api_result.elapsed_ms, api_result.optimized_poses.size());
        // 不在此处写回 KF，由 updateAllFromHBA 持 SubMapManager::mutex_ 统一写回，避免与 buildGlobalMapAsync 取快照并发导致重影（见 GHOSTING 根因分析）
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GHOSTING_DIAG] runHBA API path no KF write here poses=%zu (writeback in updateAllFromHBA under mutex)",
            api_result.optimized_poses.size());
        result.success         = true;
        result.final_mme       = api_result.final_mme;
        result.elapsed_ms      = api_result.elapsed_ms;
        result.optimized_poses = api_result.optimized_poses;
        result.optimized_keyframe_ids.clear();
        result.optimized_keyframe_ids.reserve(sorted_kfs.size());
        for (const auto& kf : sorted_kfs) result.optimized_keyframe_ids.push_back(kf->id);
        result.pose_frame      = gps_aligned_ ? PoseFrame::MAP : PoseFrame::ODOM;
    } else {
        BACKEND_STEP("step=HBA_runHBA_done backend=api success=0 elapsed_ms=%.1f error=%s",
            api_result.elapsed_ms, api_result.error_msg.c_str());
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
        BACKEND_STEP("step=HBA_runHBA_skip reason=gtsam_fallback_disabled");
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] GTSAM fallback disabled (backend.hba.enable_gtsam_fallback=false), skipping HBA");
        ALOG_WARN(MOD, "HBA: GTSAM fallback disabled by config, skip");
        result.success = false;
    } else {
        result = runGTSAMFallback(task);
    }
#else
    BACKEND_STEP("step=HBA_runHBA_skip reason=hba_api_not_available");
    result.success = false;
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[HBA][BACKEND] hba_api not available, skipping optimization (no changes applied)");
    fprintf(stderr, "[HBAOptimizer] hba_api not available, skipping optimization\n");
#endif

    return result;
}

#ifdef USE_GTSAM_FALLBACK
HBAResult HBAOptimizer::runGTSAMFallback(const PendingTask& task) {
    auto t_start = std::chrono::steady_clock::now();
    BACKEND_STEP("step=HBA_runGTSAMFallback_enter keyframes=%zu loops=%zu gps=%d", 
                 task.keyframes.size(), task.loops.size(), task.enable_gps ? 1 : 0);
    HBAResult result;
    result.success = false;
    if (task.keyframes.empty()) {
        BACKEND_STEP("step=HBA_runGTSAMFallback_skip reason=no_keyframes");
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][VALIDATION] GTSAM fallback: no keyframes, abort");
        return result;
    }

    // 与主配置 gps.lever_arm_imu / 主 YAML 同目录下 sensor_config 外参一致（init 之后配置或工作目录可能才就绪）
    resolveGpsLeverArmForHba(lever_arm_);

    std::vector<KeyFrame::Ptr> sorted_kfs = task.keyframes;
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    // 建立 ID 到索引的映射，用于添加回环约束
    std::unordered_map<uint64_t, size_t> kf_id_to_idx;
    for (size_t i = 0; i < sorted_kfs.size(); ++i) {
        kf_id_to_idx[sorted_kfs[i]->id] = i;
    }

    // [PCD_GHOSTING_VERIFY] GTSAM 路径下 result.optimized_poses 顺序 = sorted_kfs（时间戳序），与 SubMapManager collectKeyframesInHBAOrder 一致；打条便于与 WRITEBACK_ORDER 对照
    if (!sorted_kfs.empty()) {
        const auto& f = sorted_kfs.front();
        const auto& b = sorted_kfs.back();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA_INPUT_ORDER] first kf_id=%lu sm_id=%d ts=%.3f last kf_id=%lu sm_id=%d ts=%.3f count=%zu (GTSAM_sorted，与 WRITEBACK_ORDER 应对齐)",
            f->id, f->submap_id, f->timestamp, b->id, b->submap_id, b->timestamp, sorted_kfs.size());
    }

    // HBA 输出位姿坐标系语义探测（用于上层写回时避免 map/odom 混淆）
    // 若输入里已有明显 map!=odom 偏移，则将输出视作 MAP；否则视作 ODOM。
    bool input_has_map_offset = false;
    double max_trans_diff = 0.0;
    double max_rot_diff_deg = 0.0;
    for (const auto& kf : sorted_kfs) {
        if (!kf) continue;
        const double trans_diff = (kf->T_map_b_optimized.translation() - kf->T_odom_b.translation()).norm();
        const double rot_diff = Eigen::AngleAxisd(
            kf->T_map_b_optimized.rotation().inverse() * kf->T_odom_b.rotation()).angle();
        max_trans_diff = std::max(max_trans_diff, trans_diff);
        max_rot_diff_deg = std::max(max_rot_diff_deg, rot_diff * 180.0 / M_PI);
        if (trans_diff > 0.20 || rot_diff > (5.0 * M_PI / 180.0)) {
            input_has_map_offset = true;
            break;
        }
    }
    result.pose_frame = input_has_map_offset ? PoseFrame::MAP : PoseFrame::ODOM;
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[V3][CONTRACT] HBA fallback frame inference: input_has_map_offset=%d max_trans_diff=%.3fm max_rot_diff=%.2fdeg => pose_frame=%d",
        input_has_map_offset ? 1 : 0, max_trans_diff, max_rot_diff_deg, static_cast<int>(result.pose_frame));

    // 约束合理性：所有关键帧位姿有限且平移在合理范围内，否则不进入 GTSAM 避免崩溃
    auto poseForInitial = [](const KeyFrame::Ptr& kf) -> Pose3d {
        const Pose3d& o = kf->T_map_b_optimized;
        const Pose3d& t = kf->T_odom_b;
        if ((o.translation() - t.translation()).norm() > 1e-9 || !o.rotation().isApprox(t.rotation()))
            return o;
        return t;
    };
    for (size_t i = 0; i < sorted_kfs.size(); ++i) {
        Pose3d p = poseForInitial(sorted_kfs[i]);
        const auto& t = p.translation();
        const auto& R = p.rotation();
        if (!t.allFinite() || !R.matrix().allFinite()) {
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
                        " loops=" + std::to_string(task.loops.size()) +
                        " gps=" + (task.enable_gps ? "1" : "0"), true);

    try {
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;
        // 记录每个因子的类型，用于崩溃时定位（与 ISAM2 双路 GTSAM 时 double free 诊断）
        std::vector<std::string> factor_type_log;

        // 每个因子使用独立噪声模型，避免多因子共享同一 shared_ptr 在 GTSAM 内触发 double free
        //（与 incremental_optimizer 及 FIX_GPS_BATCH_SIGSEGV 文档中 borglab/gtsam#1189 同类问题一致）
        // 🔧 [修复] 放宽首帧 Prior 约束：1e-9 -> 0.25 (0.5m std dev)
        // 原因：若初始对齐有微小偏差，过死的 Prior 会阻止轨迹向 GPS 整体偏移，导致拉花。
        const double prior_var = 0.25;
        const double between_rotate_var = 1e-6; // 降低旋转方差，增强轨迹刚度
        const double between_trans_var = 1e-4;  // 降低平移方差

        // 使用命名变量作为方差向量，避免将 Eigen 临时量传入 Variances() 导致 GTSAM 内部悬垂引用
        gtsam::Vector6 prior_var6;
        prior_var6 << prior_var, prior_var, prior_var, prior_var, prior_var, prior_var;
        
        gtsam::Vector6 between_var6;
        between_var6 << between_rotate_var, between_rotate_var, between_rotate_var, 
                        between_trans_var, between_trans_var, between_trans_var;

        for (size_t i = 0; i < sorted_kfs.size(); ++i) {
            Pose3d pose_i = poseForInitial(sorted_kfs[i]);
            initial.insert(KF(i), toGtsamPose3(pose_i));
            if (i == 0) {
                // 🔧 [修复] 为 PriorFactor 引入 Huber 鲁棒核函数
                // 原因：当 HBA 任务包含 GPS 约束或跨 Session 约束时，第一帧的 Prior 可能会与 GPS 冲突。
                // 如果 Prior 太死，LM 优化可能会为了满足 Prior 而牺牲 loop/gps 导致轨迹拉花。
                auto base_noise = gtsam::noiseModel::Diagonal::Variances(prior_var6);
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.0), base_noise);
                
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(KF(0), toGtsamPose3(pose_i), robust_noise));
                factor_type_log.push_back("Prior(k0)");
            } else {
                // 🔧 [修复] 关键点：BetweenFactor 的测量值必须始终使用原始里程计 (T_odom_b)，
                // 而非初始值 (poseForInitial 可能返回已优化的位姿)。
                // 否则会产生“自引用”逻辑错误：将“上次优化的结果”作为“本次优化的真实测量”，
                // 导致轨迹无法纠偏且误差不断累积（重影根因）。
                Pose3d rel = sorted_kfs[i - 1]->T_odom_b.inverse() * sorted_kfs[i]->T_odom_b;
                auto between_noise = gtsam::noiseModel::Diagonal::Variances(between_var6);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(KF(i - 1), KF(i), toGtsamPose3(rel), between_noise));
                factor_type_log.push_back("Between(k" + std::to_string(i - 1) + "-k" + std::to_string(i) + ")");
            }
        }

        // 添加回环约束
        size_t loop_factors_added = 0;
        double total_loop_info_norm = 0.0;
        for (const auto& lc : task.loops) {
            if (!lc || lc->keyframe_global_id_i < 0 || lc->keyframe_global_id_j < 0) continue;
            
            // triggerAsync 已解析好全局 ID，此处映射到 task 内的关键帧索引
            auto it_i = kf_id_to_idx.find((uint64_t)lc->keyframe_global_id_i);
            auto it_j = kf_id_to_idx.find((uint64_t)lc->keyframe_global_id_j);
            
            if (it_i != kf_id_to_idx.end() && it_j != kf_id_to_idx.end()) {
                // 详细记录回环因子的测量值 vs 当前位姿，识别潜在重影风险
                Pose3d pose_i = sorted_kfs[it_i->second]->T_odom_b;
                Pose3d pose_j = sorted_kfs[it_j->second]->T_odom_b;
                Pose3d current_rel = pose_i.inverse() * pose_j;
                double trans_diff = (current_rel.translation() - lc->delta_T.translation()).norm();
                double rot_diff = Eigen::AngleAxisd(current_rel.rotation().inverse() * lc->delta_T.rotation()).angle() * 180.0 / M_PI;
                double info_norm = lc->information.norm();
                total_loop_info_norm += info_norm;

                if (trans_diff > 5.0 || rot_diff > 20.0) {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[HBA][LOOP_CONFLICT] High conflict loop detected BEFORE OPT: k%zu-k%zu trans_diff=%.3fm rot_diff=%.2fdeg info_norm=%.2e (测量值与里程计差异过大，极可能导致重影)",
                        it_i->second, it_j->second, trans_diff, rot_diff, info_norm);
                }

                gtsam::Pose3 rel = toGtsamPose3(lc->delta_T);
                
                // 使用 Huber 鲁棒核函数，防止坏回环拉花地图
                auto base_noise = gtsam::noiseModel::Gaussian::Information(lc->information);
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);

                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(KF(it_i->second), KF(it_j->second), rel, robust_noise));
                factor_type_log.push_back("Loop(k" + std::to_string(it_i->second) + "-k" + std::to_string(it_j->second) + ")");
                loop_factors_added++;
            }
        }
        if (loop_factors_added > 0) {
            g_hba_loop_added_total.fetch_add(loop_factors_added, std::memory_order_relaxed);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][BACKEND] Loop factors added to HBA graph count=%zu avg_info_norm=%.2e", 
                loop_factors_added, total_loop_info_norm / loop_factors_added);
        }

        // [POSE_DIAG] HBA 初始值：首/中/尾关键帧记录使用的位姿来源及数值
        {
            const size_t n_kf_init = sorted_kfs.size();
            for (size_t idx : {static_cast<size_t>(0), n_kf_init / 2, n_kf_init - 1}) {
                if (idx >= n_kf_init) continue;
                const auto& kf = sorted_kfs[idx];
                Pose3d p_init = poseForInitial(kf);
                const auto& to = kf->T_map_b_optimized.translation();
                const auto& td = kf->T_odom_b.translation();
                bool used_opt = (p_init.translation() - to).norm() < 1e-9;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][POSE_DIAG] initial idx=%zu kf_id=%lu source=%s trans=[%.4f,%.4f,%.4f] | T_odom_b=[%.4f,%.4f,%.4f] T_map_b_opt=[%.4f,%.4f,%.4f]",
                    idx, kf->id, used_opt ? "T_map_b_optimized" : "T_odom_b",
                    p_init.translation().x(), p_init.translation().y(), p_init.translation().z(),
                    td.x(), td.y(), td.z(), to.x(), to.y(), to.z());
            }
        }

        size_t gps_factors_added = 0;
        size_t gps_candidates_total = sorted_kfs.size();
        size_t gps_reject_no_valid = 0;
        size_t gps_reject_quality = 0;
        size_t gps_reject_non_finite_pos = 0;
        size_t gps_reject_non_finite_cov = 0;
        if (task.enable_gps && gps_aligned_) {
            for (size_t i = 0; i < sorted_kfs.size(); ++i) {
                const auto& kf = sorted_kfs[i];
                if (!kf->has_valid_gps) {
                    gps_reject_no_valid++;
                    continue;
                }
                if (!gpsQualityAcceptedByPolicy(kf->gps.quality, ConfigManager::instance())) {
                    gps_reject_quality++;
                    continue;
                }
                const auto& pos_enu = kf->gps.position_enu;
                if (!pos_enu.allFinite()) {
                    gps_reject_non_finite_pos++;
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[HBA][GTSAM] skip GPS factor kf=%zu: non-finite position_enu", i);
                    continue;
                }
                // 与 iSAM2 一致：由于地图已整体变换到 ENU 系，GPS 观测直接使用 ENU 坐标即可
                // 🔧 [修复] GPS 杠臂补偿 (Lever-arm Compensation)
                // 原理：pos_imu = pos_gps_antenna - R_map_body * p_lever_arm
                // 注意：必须使用地图坐标系下的姿态 (T_map_b_optimized) 而非原始里程计 (T_odom_b)，
                // 否则当系统存在较大的对齐偏角时，补偿方向会完全错位导致重影。
                Eigen::Vector3d pos_map = pos_enu - kf->T_map_b_optimized.rotation() * lever_arm_;
                
                gtsam::Point3 pt(pos_map.x(), pos_map.y(), pos_map.z());
                Eigen::Matrix3d c = kf->gps.covariance;
                if (!c.allFinite()) {
                    gps_reject_non_finite_cov++;
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[HBA][GTSAM] skip GPS factor kf=%zu: non-finite covariance", i);
                    continue;
                }
                
                // 🔧 [修复] GPS Z轴降权：高度观测噪声通常远大于水平方向，放大 Z 轴方差
                double v0 = std::max(1e-6, std::min(1e6, c(0, 0)));
                double v1 = std::max(1e-6, std::min(1e6, c(1, 1)));
                double v2 = std::max(1e-6, std::min(1e6, c(2, 2))) * 20.0; // 放大20倍
                
                gtsam::Vector3 vars(v0, v1, v2);
                auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
                
                // 🔧 [修复] 引入 Huber 鲁棒核函数：抑制异常 GPS 点的影响
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);
                
                graph.add(gtsam::GPSFactor(KF(i), pt, robust_noise));
                factor_type_log.push_back("GPS(k" + std::to_string(i) + ")");
                gps_factors_added++;
            }
            if (gps_factors_added > 0) {
                g_hba_gps_added_total.fetch_add(gps_factors_added, std::memory_order_relaxed);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM][BACKEND] GPS positions in map frame (enu_to_map applied) gps_factors=%zu",
                    gps_factors_added);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][GPS_FILTER] candidates=%zu added=%zu reject_no_valid=%zu reject_quality=%zu reject_non_finite_pos=%zu reject_non_finite_cov=%zu",
                gps_candidates_total, gps_factors_added, gps_reject_no_valid, gps_reject_quality, gps_reject_non_finite_pos, gps_reject_non_finite_cov);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CONSTRAINT_KPI][HBA] loop_added=%zu gps_added=%zu gps_reject_quality=%zu gps_min_accepted_quality_level=%d total_loop_added=%lu total_gps_added=%lu",
                loop_factors_added, gps_factors_added, gps_reject_quality,
                ConfigManager::instance().gpsMinAcceptedQualityLevel(),
                static_cast<unsigned long>(g_hba_loop_added_total.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(g_hba_gps_added_total.load(std::memory_order_relaxed)));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][GPS_FILTER] skipped task_enable_gps=%d gps_aligned=%d candidates=%zu",
                task.enable_gps ? 1 : 0, gps_aligned_ ? 1 : 0, gps_candidates_total);
        }

        size_t n_factors = graph.size();
        size_t n_values = initial.size();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] graph built: factors=%zu values=%zu loop_factors=%zu gps_factors=%zu (building LM optimizer...)",
            n_factors, n_values, loop_factors_added, gps_factors_added);
        ALOG_INFO(MOD, "HBA GTSAM: factors={} values={} loops={} gps={}", n_factors, n_values, loop_factors_added, gps_factors_added);

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

        // [GHOSTING_DIAG] 记录优化前各类因子的残差
        auto logFactorErrors = [&](const gtsam::NonlinearFactorGraph& g, const gtsam::Values& v, const std::string& label) {
            double total_err = 0, prior_err = 0, odom_err = 0, gps_err = 0, loop_err = 0;
            size_t p_cnt = 0, o_cnt = 0, g_cnt = 0, l_cnt = 0;
            
            for (size_t idx = 0; idx < g.size(); ++idx) {
                if (!g[idx]) continue;
                double e = g[idx]->error(v);
                total_err += e;
                
                std::string type = (idx < factor_type_log.size()) ? factor_type_log[idx] : "unknown";
                if (type.find("Prior") != std::string::npos) { prior_err += e; p_cnt++; }
                else if (type.find("Between") != std::string::npos) { odom_err += e; o_cnt++; }
                else if (type.find("GPS") != std::string::npos) { gps_err += e; g_cnt++; }
                else if (type.find("Loop") != std::string::npos) { loop_err += e; l_cnt++; }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][ERROR_BREAKDOWN][%s] total=%.2f prior=%.2f(%zu) odom=%.2f(%zu) gps=%.2f(%zu) loop=%.2f(%zu)",
                label.c_str(), total_err, prior_err, p_cnt, odom_err, o_cnt, gps_err, g_cnt, loop_err, l_cnt);
        };

        logFactorErrors(graph_copy, initial_copy, "BEFORE");

        gtsam::Values optimized = opt.optimize();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] LM optimize() exit iterations done. final_error=%.6g", graph_copy.error(optimized));

        logFactorErrors(graph_copy, optimized, "AFTER");

        // [GHOSTING_DIAG] 写回前抽样记录 T_odom_b vs 即将写回的 T_map_b_optimized，便于定位重影是否来自写回不一致
        const size_t kSampleStep = std::max<size_t>(1, sorted_kfs.size() / 8);
        for (size_t i = 0; i < sorted_kfs.size(); ++i) {
            if (i % kSampleStep == 0 && optimized.exists(KF(i))) {
                const auto& kf = sorted_kfs[i];
                Pose3d new_pose = fromGtsamPose3(optimized.at<gtsam::Pose3>(KF(i)));
                double trans_diff = (new_pose.translation() - kf->T_odom_b.translation()).norm();
                double rot_diff = Eigen::AngleAxisd(new_pose.linear() * kf->T_odom_b.linear().transpose()).angle() * 180.0 / M_PI;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GHOSTING_DIAG] pre_writeback kf_id=%lu idx=%zu: T_odom_b=[%.2f,%.2f,%.2f] -> T_map_b_optimized=[%.2f,%.2f,%.2f] trans_diff=%.3fm rot_diff=%.2fdeg",
                    kf->id, i,
                    kf->T_odom_b.translation().x(), kf->T_odom_b.translation().y(), kf->T_odom_b.translation().z(),
                    new_pose.translation().x(), new_pose.translation().y(), new_pose.translation().z(),
                    trans_diff, rot_diff);
            }
        }

        // 详细记录回环因子的残差，识别哪些回环被优化器认为“不可信”
        if (loop_factors_added > 0) {
            double total_loop_error = 0.0;
            int high_residual_count = 0;
            for (size_t idx = 0; idx < graph_copy.size(); ++idx) {
                if (idx < factor_type_log.size() && factor_type_log[idx].find("Loop") != std::string::npos) {
                    double err = graph_copy[idx]->error(optimized);
                    total_loop_error += err;
                    if (err > 1.0) { // 残差较大
                        high_residual_count++;
                        
                        // [GHOSTING_DIAG] 提取该因子的节点 ID
                        auto factor = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(graph_copy[idx].get());
                        std::string nodes_str = "unknown";
                        if (factor) {
                            nodes_str = fmt::format("k{}-k{}", factor->key1(), factor->key2());
                        }

                        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                            "[HBA][LOOP_RESIDUAL] factor[%zu] nodes=%s residual=%.4f (High residual indicates conflict with odom/GPS/other loops)", 
                            idx, nodes_str.c_str(), err);
                        
                        // [GHOSTING_DIAG] 记录该坏因子的具体位姿偏移，便于量化对重影的贡献
                        if (err > 10.0) {
                            std::string info_str = "N/A";
                            if (factor) {
                                auto noise = dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor->noiseModel().get());
                                if (noise) {
                                    Eigen::MatrixXd info = noise->information();
                                    info_str = fmt::format("diag=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]", 
                                        info(0,0), info(1,1), info(2,2), info(3,3), info(4,4), info(5,5));
                                }
                            }
                            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                                "[HBA][CRITICAL_LOOP_ERROR] EXTREME residual=%.2f at %s. Info: %s. This loop is definitely conflicting and likely causing ghosting!",
                                err, nodes_str.c_str(), info_str.c_str());
                        }
                    }
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_SUMMARY] total_loops=%zu high_residual=%d avg_err=%.4f",
                loop_factors_added, high_residual_count, total_loop_error / loop_factors_added);
        }

        const size_t n_kf = sorted_kfs.size();
        for (size_t i = 0; i < n_kf; ++i) {
            if (optimized.exists(KF(i))) {
                Pose3d new_pose = fromGtsamPose3(optimized.at<gtsam::Pose3>(KF(i)));
                result.optimized_poses.push_back(new_pose);
                result.optimized_keyframe_ids.push_back(sorted_kfs[i]->id);
            }
        }
        // [POSE_DIAG] 写回由 updateAllFromHBA 持锁统一执行；此处用 result.optimized_poses 打首/中/尾与 T_odom_b 的差值
        for (size_t idx : {static_cast<size_t>(0), n_kf / 2, n_kf - 1}) {
            if (idx >= n_kf || idx >= result.optimized_poses.size()) continue;
            const auto& kf = sorted_kfs[idx];
            const auto& t_new = result.optimized_poses[idx].translation();
            const auto& t_odom = kf->T_odom_b.translation();
            double trans_diff = (t_new - t_odom).norm();
            double yaw_new = std::atan2(result.optimized_poses[idx].rotation()(1, 0), result.optimized_poses[idx].rotation()(0, 0)) * 180.0 / M_PI;
            double yaw_odom = std::atan2(kf->T_odom_b.rotation()(1, 0), kf->T_odom_b.rotation()(0, 0)) * 180.0 / M_PI;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][POSE_DIAG] writeback idx=%zu kf_id=%lu T_map_b_optimized=[%.4f,%.4f,%.4f] yaw=%.2fdeg | T_odom_b=[%.4f,%.4f,%.4f] yaw=%.2fdeg trans_diff=%.4fm",
                idx, kf->id, t_new.x(), t_new.y(), t_new.z(), yaw_new, t_odom.x(), t_odom.y(), t_odom.z(), yaw_odom, trans_diff);
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GHOSTING_DIAG] runHBA no KF write here poses=%zu (writeback in updateAllFromHBA under mutex；若重影请 grep GHOSTING_DIAG 核对 writeback_enter/done 与 pose_snapshot_taken 时间线)",
            result.optimized_poses.size());
        result.success = true;
        auto t_end = std::chrono::steady_clock::now();
        result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        result.final_mme = 0.0; // fallback 路径暂不计算 MME
        scope.setSuccess(true);
        BACKEND_STEP("step=HBA_runHBA_done backend=GTSAM_fallback success=1 MME=%.4f elapsed_ms=%.1f poses=%zu", 
                     result.final_mme, result.elapsed_ms, result.optimized_poses.size());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] GTSAM fallback done: poses=%zu elapsed=%.1fms", result.optimized_poses.size(), result.elapsed_ms);
        ALOG_INFO(MOD, "HBA GTSAM fallback done: optimized_poses={} elapsed={:.1f}ms", result.optimized_poses.size(), result.elapsed_ms);
    } catch (const std::exception& e) {
        BACKEND_STEP("step=HBA_runHBA_done backend=GTSAM_fallback success=0 exception=%s", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] GTSAM fallback failed: %s (grep HBA BACKEND EXCEPTION)", e.what());
        ALOG_ERROR(MOD, "HBA GTSAM fallback exception: {}", e.what());
        fprintf(stderr, "[HBAOptimizer] GTSAM fallback exception: %s\n", e.what());
    } catch (...) {
        BACKEND_STEP("step=HBA_runHBA_done backend=GTSAM_fallback success=0 exception=unknown");
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
            const auto& t = kf->T_odom_b.translation();
            const auto& R = kf->T_odom_b.rotation();
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
