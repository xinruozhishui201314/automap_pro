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

namespace automap_pro {

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

    // GPS 对齐后立即触发一次全局优化
    triggerAsync(all_submaps, false);
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
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] HBA failed: %s", api_result.error_msg.c_str());
        fprintf(stderr, "[HBAOptimizer] HBA failed: %s\n",
                api_result.error_msg.c_str());
    }

#else
    // fallback：无 HBA API，明确标记为失败，避免误以为已优化
    result.success = false;
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[HBA][BACKEND] hba_api not available, skipping optimization (no changes applied)");
    fprintf(stderr, "[HBAOptimizer] hba_api not available, skipping optimization\n");
#endif

    return result;
}

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
