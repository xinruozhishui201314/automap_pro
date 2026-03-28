#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/isam2_factor_types.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>

#ifdef USE_HBA_API
#include <hba_api/hba_api.h>
#endif

namespace automap_pro {

/**
 * HBA 优化器（automap_pro 内部封装）
 *
 * 职责：
 *   1. 收集子图的关键帧数据
 *   2. 在触发条件满足时，异步调用 hba_api::HBAOptimizer::optimize()
 *   3. 将优化结果写回关键帧 T_w_b_optimized
 *   4. 通知 SubMapManager 更新子图锚定位姿
 *
 * 触发策略（三选一，可组合）：
 *   - 每 N 个子图冻结时触发（周期性）
 *   - 每次检测到回环时触发（可选，开销大）
 *   - 建图结束时触发（最终精化）
 *
 * 线程模型：
 *   - 主线程（建图线程）调用 onSubmapFrozen/triggerAsync
 *   - 优化线程（独立线程池）执行 HBA optimize，结果通过回调返回
 *   - 主线程和优化线程通过 pending_queue_ + condition_variable 通信
 */
class HBAOptimizer {
public:
    explicit HBAOptimizer();
    ~HBAOptimizer();

    void init();
    void start();  // 启动优化线程
    void stop();   // 等待优化完成并停止（析构/常规关闭：无限期 join）

    /**
     * 关闭路径专用：与 triggerAsync(..., wait=true) 的 5 分钟等待对齐，避免 runHBA 内部卡住时 stop() 无限 join。
     * 超时后打 ERROR 并 detach 工作线程（进程即将退出时仍可接受；见 service_handlers finish_mapping）。
     */
    void stopJoinWithTimeout(std::chrono::milliseconds max_join);

    // ── 数据输入 ──────────────────────────────────────────────────────────

    /** 子图冻结时调用（自动触发条件检查） */
    void onSubmapFrozen(const SubMap::Ptr& submap);

    /** 强制异步触发 HBA（建图结束时调用）。trigger_source 仅用于日志，便于定位重影时是否被多次触发（如 finish_mapping / frontend_idle）。 */
    void triggerAsync(const std::vector<SubMap::Ptr>& all_submaps,
                      const std::vector<LoopConstraint::Ptr>& loops = {},
                      bool wait = false,
                      const char* trigger_source = nullptr,
                      uint64_t alignment_epoch_snapshot = 0);

    /** 等待队列清空且当前无 HBA 运行，最多等待 timeout_ms 毫秒（用于关闭时限时等待） */
    void waitUntilIdleFor(std::chrono::milliseconds timeout_ms);

    /** GPS 对齐完成后，批量为已有关键帧添加 GPS 因子并立即触发 HBA（若仅建图结束时做一次 HBA，则改用 setGPSAlignedState） */
    void onGPSAligned(const GPSAlignResult& align_result,
                      const std::vector<SubMap::Ptr>& all_submaps,
                      const std::vector<LoopConstraint::Ptr>& loops = {});

    /** 仅设置 GPS 对齐状态，不触发 HBA（建图结束时的那次 HBA 会使用该状态添加 GPS 约束） */
    void setGPSAlignedState(const GPSAlignResult& align_result);

    // ── 状态查询 ──────────────────────────────────────────────────────────
    bool isRunning() const { return hba_running_.load(); }
    /** 队列空且当前无 HBA 运行（用于 map_publish 在 HBA 期间跳过发布，见 docs/GHOSTING_ROOT_CAUSE_HBA_VS_BACKEND_20260317.md） */
    bool isIdle() const;
    int  triggerCount() const { return trigger_count_; }
    /** 当前待处理任务数（含正在执行的一轮） */
    size_t queueDepth() const;

    // ── 回调注册 ──────────────────────────────────────────────────────────
    void registerDoneCallback(HBADoneCallback cb) {
        done_cbs_.push_back(std::move(cb));
    }

private:
    struct PendingTask {
        std::vector<KeyFrame::Ptr> keyframes;
        std::vector<LoopConstraint::Ptr> loops;
        // 🏛️ [架构增强] 语义因子列表，支持 HBA 语义优化
        std::vector<CylinderFactorItemKF> semantic_factors;
        std::vector<PlaneFactorItemKF> semantic_plane_factors;
        // 🏛️ [架构增强] 子图锚点快照，用于约束语义地标
        std::unordered_map<int, Pose3d> submap_anchor_poses;
        uint64_t alignment_epoch_snapshot = 0;
        bool enable_gps = false;
    };

    std::queue<PendingTask>     pending_queue_;
    mutable std::mutex          queue_mutex_;
    std::condition_variable     queue_cv_;
    std::thread                 worker_thread_;
    std::atomic<bool>           running_{false};
    /** workerLoop 返回前设为 true，供 stopJoinWithTimeout 在限时内 join（避免无限阻塞） */
    std::atomic<bool>           worker_thread_finished_{false};
    std::atomic<bool>           hba_running_{false};
    int                         trigger_count_ = 0;
    int                         frozen_count_  = 0;

    // GPS 数据（对齐后缓存）
    bool gps_aligned_ = false;
    GPSAlignResult gps_align_result_;
    Eigen::Vector3d lever_arm_ = Eigen::Vector3d::Zero();

    std::vector<HBADoneCallback> done_cbs_;

    /** 构造/init 时缓存，避免 worker 线程访问 ConfigManager 单例导致 shutdown 时 SIGSEGV */
    bool hba_enabled_{true};
    bool hba_gtsam_fallback_enabled_{false};
    int gps_min_accepted_quality_level_{3};
    double gps_keyframe_match_window_s_{0.5};
    int hba_total_layers_{3};
    int hba_thread_num_{8};
    int hba_trigger_submaps_{10};
    bool hba_on_loop_{false};

    void workerLoop();
    HBAResult runHBA(const PendingTask& task);
#ifdef USE_GTSAM_FALLBACK
    /** 无 HBA API 时使用 GTSAM 批量优化作为 fallback */
    HBAResult runGTSAMFallback(const PendingTask& task);
#endif
    std::vector<KeyFrame::Ptr> collectKeyFramesFromSubmaps(
        const std::vector<SubMap::Ptr>& submaps) const;
};

} // namespace automap_pro
