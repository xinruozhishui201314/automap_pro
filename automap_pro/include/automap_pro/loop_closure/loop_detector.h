#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/loop_closure/overlap_transformer_infer.h"
#include "automap_pro/loop_closure/teaser_matcher.h"
#include <automap_pro/msg/loop_constraint_msg.hpp>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#ifdef USE_OVERLAP_TRANSFORMER_MSGS
#include <overlap_transformer_msgs/srv/compute_descriptor.hpp>
#endif

namespace automap_pro {

/**
 * 异步回环检测器（优先队列调度 + 并行描述子计算）
 *
 * 流水线（4阶段）：
 *
 *   [Stage 0] 描述子计算（并行 Worker Pool）
 *     - LibTorch 进程内推理（<5ms/frame，线程安全）
 *     - 或异步 Python Service（callback 推进）
 *
 *   [Stage 1] 候选检索（余弦相似度 + GPS过滤）
 *     - 优先级：overlap_score 高的子图先处理
 *
 *   [Stage 2] TEASER++ 粗配准（并行多候选）
 *     - 对每个候选帧同步执行 TEASER++
 *
 *   [Stage 3] ICP 精配准（可选）
 *     - 仅对通过 TEASER++ 验证的候选执行
 *
 *   [回调] 回环约束 → IncrementalOptimizer.addLoopFactor()
 *
 * 线程模型：
 *   - N 个 Worker 线程并行处理描述子计算（I/O 密集）
 *   - 1 个 Matcher 线程顺序处理 TEASER++（CPU 密集，避免竞争）
 *   - 优先队列：高 overlap 分的子图优先匹配
 */
class LoopDetector {
public:
    explicit LoopDetector();
    ~LoopDetector();

    void init(rclcpp::Node::SharedPtr node);
    void start();  // 启动 Worker 线程
    void stop();   // 优雅关闭

    // ── 输入接口 ──────────────────────────────────────────────────────────

    /** 子图冻结时提交到描述子计算队列 */
    void addSubmap(const SubMap::Ptr& submap);

    /** 从历史 session 加载描述子数据库（增量建图） */
    void addToDatabase(const SubMap::Ptr& submap);

    /** 清空当前 session 数据库（保留跨 session DB） */
    void clearCurrentSessionDB();

    /** 序列化描述子数据库（供 SessionManager 持久化） */
    std::vector<std::pair<int, Eigen::VectorXf>> exportDescriptorDB() const;

    // ── 状态查询 ──────────────────────────────────────────────────────────
    size_t dbSize()    const;
    size_t queueSize() const;
    /** 累计检测到的回环数量（用于低频数据流日志） */
    int loopDetectedCount() const { return loop_detected_count_.load(); }

    // ── 回调注册 ──────────────────────────────────────────────────────────
    void registerLoopCallback(LoopConstraintCallback cb) {
        loop_cbs_.push_back(std::move(cb));
    }

private:
    // ── 优先队列（overlap_score 高优先）────────────────────────────────
    struct PrioritizedSubmap {
        SubMap::Ptr submap;
        float       priority;
        bool operator<(const PrioritizedSubmap& o) const { return priority < o.priority; }
    };

    // 描述子计算队列（Stage 0）
    std::priority_queue<PrioritizedSubmap> desc_queue_;
    mutable std::mutex                   desc_mutex_;  // mutable for const queueSize()
    std::condition_variable                desc_cv_;

    // 几何验证队列（Stage 2，子图已有描述子后提交）
    // query_cloud：入队时深拷贝，避免 worker 读 SubMap::downsampled_cloud 与主线程并发导致 use-after-free
    struct MatchTask {
        SubMap::Ptr query;
        CloudXYZIPtr query_cloud;  // 入队时拷贝，worker 仅用此指针
        std::vector<OverlapTransformerInfer::Candidate> candidates;
    };
    std::queue<MatchTask> match_queue_;
    std::mutex            match_mutex_;
    std::condition_variable match_cv_;

    // 描述子数据库
    std::vector<SubMap::Ptr>  db_submaps_;
    mutable std::shared_mutex db_mutex_;

    // ── 算法组件 ─────────────────────────────────────────────────────────
    OverlapTransformerInfer overlap_infer_;
    TeaserMatcher           teaser_matcher_;

    // ── ROS2 资源 ─────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<automap_pro::msg::LoopConstraintMsg>::SharedPtr constraint_pub_;

#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    rclcpp::Client<overlap_transformer_msgs::srv::ComputeDescriptor>::SharedPtr desc_client_;
    bool use_external_desc_service_ = false;
#endif

    // ── 数据流统计（低频日志）─────────────────────────────────────────────
    std::atomic<int> loop_detected_count_{0};

    // ── 线程控制 ─────────────────────────────────────────────────────────
    std::atomic<bool> running_{false};
    std::vector<std::thread> desc_workers_;   // 描述子计算 Worker 池
    std::thread               match_worker_;   // TEASER++ 匹配 Worker

    // ── 参数 ─────────────────────────────────────────────────────────────
    double overlap_threshold_  = 0.3;
    int    top_k_              = 5;
    double min_temporal_gap_   = 30.0;
    int    min_submap_gap_     = 3;
    double gps_search_radius_  = 200.0;
    double min_inlier_ratio_   = 0.30;
    double max_rmse_           = 0.3;
    bool   use_icp_refine_     = true;
    int    worker_thread_num_  = 2;

    std::vector<LoopConstraintCallback> loop_cbs_;

    // ── 私有方法 ──────────────────────────────────────────────────────────
    void descWorkerLoop();
    void matchWorkerLoop();

    void computeDescriptorAsync(const SubMap::Ptr& submap);
    void onDescriptorReady(const SubMap::Ptr& submap);
    void processMatchTask(const MatchTask& task);

    bool computeDescExternal(const SubMap::Ptr& submap);
    void publishLoopConstraint(const LoopConstraint::Ptr& lc);
};

} // namespace automap_pro
