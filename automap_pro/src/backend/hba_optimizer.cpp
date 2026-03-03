#include "automap_pro/backend/hba_optimizer.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "HBAOptimizer"

#include <chrono>
#include <algorithm>

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
    ALOG_INFO(MOD, "HBA triggerAsync: submaps={} keyframes={} gps={} trigger_count={}",
              sm_count, kf_count, gps_aligned_ ? 1 : 0, trigger_count_);

    if (wait) {
        // 等待队列清空（阻塞）
        while (!pending_queue_.empty() || hba_running_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
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
            while (pending_queue_.size() > 1) pending_queue_.pop();
            task = std::move(pending_queue_.front());
            pending_queue_.pop();
        }

        hba_running_ = true;
        ALOG_INFO(MOD, "HBA optimization starting: kf_count={} gps={}",
                  task.keyframes.size(), task.enable_gps);
        AUTOMAP_TIMED_SCOPE(MOD, "HBA full optimize", 60000.0);
        HBAResult result = runHBA(task);
        hba_running_ = false;

        if (result.success) {
            ALOG_INFO(MOD, "HBA done: MME={:.4f} elapsed={:.1f}ms kf={}",
                      result.final_mme, result.elapsed_ms, result.optimized_poses.size());
        } else {
            ALOG_ERROR(MOD, "HBA failed after {:.1f}ms", result.elapsed_ms);
        }
        for (auto& cb : done_cbs_) cb(result);
    }
}

HBAResult HBAOptimizer::runHBA(const PendingTask& task) {
    const auto& cfg = ConfigManager::instance();
    HBAResult result;

#ifdef USE_HBA_API
    hba_api::Config hba_cfg;
    hba_cfg.total_layer_num = cfg.hbaTotalLayers();
    hba_cfg.thread_num      = cfg.hbaThreadNum();
    hba_cfg.voxel_size      = 0.5;
    hba_cfg.enable_gps      = task.enable_gps;

    // 若启用 GPS，构建 GPS 条目
    if (task.enable_gps && gps_aligned_) {
        for (const auto& kf : task.keyframes) {
            if (kf->has_valid_gps) {
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

    // 执行 HBA 优化
    auto api_result = optimizer.optimize([](int cur, int total, float pct) {
        printf("[HBA] Layer %d/%d  %.0f%%\n", cur, total, pct);
    });

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
        fprintf(stderr, "[HBAOptimizer] HBA failed: %s\n",
                api_result.error_msg.c_str());
    }

#else
    // fallback：无 HBA API，直接复制当前位姿作为"优化结果"
    result.success = true;
    for (const auto& kf : task.keyframes) {
        result.optimized_poses.push_back(kf->T_w_b);
    }
    fprintf(stderr, "[HBAOptimizer] hba_api not available, skipping optimization\n");
#endif

    return result;
}

std::vector<KeyFrame::Ptr> HBAOptimizer::collectKeyFramesFromSubmaps(
    const std::vector<SubMap::Ptr>& submaps) const
{
    std::vector<KeyFrame::Ptr> kfs;
    for (const auto& sm : submaps) {
        for (const auto& kf : sm->keyframes) {
            kfs.push_back(kf);
        }
    }
    std::sort(kfs.begin(), kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });
    return kfs;
}

} // namespace automap_pro
