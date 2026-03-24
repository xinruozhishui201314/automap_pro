#pragma once

#include "automap_pro/v3/module_base.h"
#include "automap_pro/v3/semantic_processor.h"
#include <array>
#include <atomic>
#include <chrono>
#include <deque>

namespace automap_pro::v3 {

/**
 * @brief 语义处理模块 (SemanticModule)
 * 🏛️ [架构演进] 专门负责深度学习推理与特征拟合，异步执行以避免阻塞前端。
 */
class SemanticModule : public ModuleBase {
public:
    using Ptr = std::shared_ptr<SemanticModule>;

    SemanticModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node);

    void start() override;
    void stop() override;

    bool isIdle() const override {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return task_queue_.empty();
    }

protected:
    void run() override;

private:
    void processTask(const SyncedFrameEvent& event);
    void enqueueTaskLocked(const SyncedFrameEvent& ev);
    void flushPendingKeyframeTasksLocked();
    void handleSyncedFrameEvent(const SyncedFrameEvent& ev);
    bool normalizeSemanticInputFrame(const SyncedFrameEvent& in, SyncedFrameEvent& out) const;
    void tryRecoverFromDegradedState(double now_s);
    void retryPendingFramesLocked();

    rclcpp::Node::SharedPtr node_;
    SemanticProcessor::Ptr semantic_processor_;
    std::atomic<bool> semantic_runtime_ready_{false};
    SemanticProcessor::Config semantic_cfg_;

    // 任务队列
    mutable std::mutex queue_mutex_;
    std::deque<SyncedFrameEvent> task_queue_;
    std::deque<SyncedFrameEvent> pending_keyframe_queue_;
    static constexpr size_t kMaxQueueSize = 5; // 限制队列长度，防止积压导致高延迟
    static constexpr size_t kCoalesceThreshold = 2; // 队列拥塞时合并为最新帧，避免语义线程长期处理陈旧数据
    static constexpr size_t kMaxPendingKeyframeQueueSize = 32; // 关键帧信息晚到时短暂缓存，避免误丢真正关键帧
    static constexpr double kPendingTaskMaxAgeSec = 0.5;       // 超过该时长仍无法确认关键帧则丢弃

    // 🏛️ [产品化加固] 自动降级策略：连续多次崩溃或异常后自动停用语义模块
    std::atomic<int> consecutive_errors_{0};
    static constexpr int kMaxConsecutiveErrors = 3;
    std::atomic<bool> semantic_degraded_{false};
    std::atomic<double> next_recovery_retry_s_{0.0};
    std::atomic<int> recovery_attempts_{0};
    std::atomic<bool> recovery_exhausted_reported_{false};
    static constexpr double kRecoveryCooldownSec = 10.0;
    static constexpr int kMaxRecoveryAttempts = 3;

    // 运行时统计：用于诊断“线程在跑但无有效输出/慢处理”场景
    std::atomic<uint64_t> processed_tasks_{0};
    std::atomic<uint64_t> no_landmark_tasks_{0};
    std::atomic<uint64_t> coalesced_tasks_{0};
    std::atomic<uint64_t> backpressure_drops_{0};
    std::atomic<uint64_t> skipped_non_keyframe_{0};
    std::atomic<uint64_t> skipped_duplicate_keyframe_{0};
    std::atomic<uint64_t> pending_requeued_{0};
    std::atomic<uint64_t> pending_dropped_{0};
    // dt = |frame_ts - kf_ts| 统计桶（秒）
    // [0,0.01], (0.01,0.03], (0.03,0.05], (0.05,0.10], (0.10,0.20], (0.20,0.50], >0.50, missing_kf_ts
    std::array<std::atomic<uint64_t>, 8> dt_hist_bins_{
        std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0},
        std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0}};
    std::array<uint64_t, 8> dt_hist_last_snapshot_{{0, 0, 0, 0, 0, 0, 0, 0}};
    // pending 生命周期样本（仅记录因超龄丢弃的 age 秒），用于 P50/P90/P99 估计
    std::deque<double> pending_drop_age_samples_sec_;
    static constexpr size_t kMaxPendingAgeSamples = 2048;
    std::chrono::steady_clock::time_point last_stats_log_tp_{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point last_dt_delta_log_tp_{std::chrono::steady_clock::now()};
    bool keyframes_only_{false};
    double keyframe_time_tolerance_s_{0.03};
    std::atomic<double> latest_kf_ts_seen_{-1.0};
    std::atomic<double> last_enqueued_kf_ts_{-1.0};
};

} // namespace automap_pro::v3
