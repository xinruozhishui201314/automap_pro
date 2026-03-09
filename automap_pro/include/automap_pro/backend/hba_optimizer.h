#pragma once

#include "automap_pro/core/data_types.h"
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
    void stop();   // 等待优化完成并停止

    // ── 数据输入 ──────────────────────────────────────────────────────────

    /** 子图冻结时调用（自动触发条件检查） */
    void onSubmapFrozen(const SubMap::Ptr& submap);

    /** 强制异步触发 HBA（建图结束时调用） */
    void triggerAsync(const std::vector<SubMap::Ptr>& all_submaps,
                      bool wait = false);

    /** 等待队列清空且当前无 HBA 运行，最多等待 timeout_ms 毫秒（用于关闭时限时等待） */
    void waitUntilIdleFor(std::chrono::milliseconds timeout_ms);

    /** GPS 对齐完成后，批量为已有关键帧添加 GPS 因子 */
    void onGPSAligned(const GPSAlignResult& align_result,
                      const std::vector<SubMap::Ptr>& all_submaps);

    // ── 状态查询 ──────────────────────────────────────────────────────────
    bool isRunning() const { return hba_running_.load(); }
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
        bool enable_gps = false;
    };

    std::queue<PendingTask>     pending_queue_;
    mutable std::mutex          queue_mutex_;
    std::condition_variable     queue_cv_;
    std::thread                 worker_thread_;
    std::atomic<bool>           running_{false};
    std::atomic<bool>           hba_running_{false};
    int                         trigger_count_ = 0;
    int                         frozen_count_  = 0;

    // GPS 数据（对齐后缓存）
    bool gps_aligned_ = false;
    GPSAlignResult gps_align_result_;

    std::vector<HBADoneCallback> done_cbs_;

    void workerLoop();
    HBAResult runHBA(const PendingTask& task);
    std::vector<KeyFrame::Ptr> collectKeyFramesFromSubmaps(
        const std::vector<SubMap::Ptr>& submaps) const;
    /** 在 queue_mutex_ 下检查是否空闲 */
    bool isIdle() const;
};

} // namespace automap_pro
