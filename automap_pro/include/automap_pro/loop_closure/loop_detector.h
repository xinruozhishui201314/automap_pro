#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/loop_closure/overlap_transformer_infer.h"
#include "automap_pro/loop_closure/teaser_matcher.h"
#include <automap_pro/msg/loop_constraint_msg.hpp>

// ScanContext
#include "Scancontext/Scancontext.h"

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

    // ── 子图内回环检测 ───────────────────────────────────────────────────────
    /**
     * @brief 子图内回环检测（复用 LoopDetector 的描述子匹配和几何验证）
     * @param submap 子图（需已填充 keyframe_clouds_ds）
     * @param query_keyframe_idx 当前关键帧索引
     * @return 检测到的回环约束（可能有多个），空表示无回环
     */
    std::vector<LoopConstraint::Ptr> detectIntraSubmapLoop(
        const SubMap::Ptr& submap,
        int query_keyframe_idx);

    /** 为子图内回环检测准备描述子数据库（首次调用时计算所有关键帧描述子） */
    void prepareIntraSubmapDescriptors(const SubMap::Ptr& submap);

    /**
     * 增量确保子图内描述子/下采样点云到指定索引（含），只计算缺失的尾部，避免每帧全量 prepare。
     * 用于每帧触发子图内回环时仅追加当前关键帧描述子。
     */
    void ensureIntraSubmapDescriptorsUpTo(const SubMap::Ptr& submap, int up_to_index_inclusive);

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
    /** 子图间关键帧级候选：(submap_id, keyframe_idx, score) */
    struct InterKfCandidate {
        int submap_id = -1;
        int keyframe_idx = -1;
        float score = 0.f;
    };
    struct MatchTask {
        SubMap::Ptr query;
        CloudXYZIPtr query_cloud;  // 子图级=downsampled_cloud；关键帧级=该关键帧下采样点云
        std::vector<OverlapTransformerInfer::Candidate> candidates;
        /** 关键帧级子图间：query 关键帧索引，-1 表示子图级匹配 */
        int query_kf_idx = -1;
        /** 关键帧级子图间：候选 (target_submap_id, target_keyframe_idx, score)，非空时走关键帧级 TEASER */
        std::vector<InterKfCandidate> candidates_kf;
    };
    std::queue<MatchTask> match_queue_;
    std::mutex            match_mutex_;
    std::condition_variable match_cv_;

    // 描述子数据库
    std::vector<SubMap::Ptr>  db_submaps_;
    mutable std::shared_mutex db_mutex_;

    /** 保护子图内描述子准备：keyframe_scancontexts_/keyframe_descriptors/keyframe_clouds_ds 的并发写。
     *  避免 prepareIntraSubmapDescriptors 与 addToDatabase/onDescriptorReady 多线程同时执行导致 double-free。 */
    std::mutex intra_prepare_mutex_;

    // ── 算法组件 ─────────────────────────────────────────────────────────
    OverlapTransformerInfer overlap_infer_;
    TeaserMatcher           teaser_matcher_;

    // ── ScanContext 回环检索 ─────────────────────────────────────────────
    SCManager               sc_manager_;
    mutable std::mutex     sc_mutex_;  // 保护 SCManager 的并发访问
    bool                    use_scancontext_ = false;
    double                  sc_dist_threshold_ = 0.13;
    int                     sc_num_candidates_ = 5;
    int                     sc_exclude_recent_ = 50;
    int                     sc_tree_making_period_ = 50;

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
    int    min_submap_gap_     = 0;   // 0=不按子图间隔过滤(允许相邻子图回环)
    double gps_search_radius_  = 200.0;
    /** 几何距离预筛：候选与 query 子图锚定位置距离 > 此值(米)则过滤。0=关闭 */
    double geo_prefilter_max_distance_m_ = 0.0;
    /** 高置信度绕过：描述子 score ≥ 此值时跳过几何预筛（参考 OverlapTransformer/SeqOT）。0=关闭 */
    double geo_prefilter_skip_above_score_ = 0.0;
    double min_inlier_ratio_   = 0.30;
    double max_rmse_           = 0.3;
    bool   use_icp_refine_     = true;
    int    worker_thread_num_  = 2;

    std::vector<LoopConstraintCallback> loop_cbs_;

    // ── 子图内回环检测参数 ──────────────────────────────────────────────────
    /** 子图内回环检测：是否启用 */
    bool intra_submap_enabled_ = false;
    /** 子图内回环检测：关键帧之间最小时间间隔（秒） */
    double intra_submap_min_temporal_gap_ = 5.0;
    /** 子图内回环检测：关键帧之间最小索引间隔（避免相邻帧假回环） */
    int intra_submap_min_keyframe_gap_ = 10;
    /** 子图内回环检测：关键帧之间最小距离间隔(米)，避免过密假回环 */
    double intra_submap_min_distance_gap_ = 3.0;
    /** 子图内回环检测：描述子相似度阈值 */
    double intra_submap_overlap_threshold_ = 0.3;
    /** 子图内回环：每帧最多对多少候选做 TEASER（≤0 不限制），避免单帧 10s+ */
    int intra_submap_max_teaser_candidates_ = 5;

    /** 子图间回环使用关键帧级（true=关键帧↔关键帧） */
    bool inter_keyframe_level_enabled_ = true;
    /** 子图间关键帧级：query 子图内采样步长（每 step 个关键帧取 1 个） */
    int inter_keyframe_sample_step_ = 5;
    /** 子图间关键帧级：每个候选子图内取 top-K 关键帧 */
    int inter_keyframe_top_k_per_submap_ = 3;

    // ── 私有方法 ──────────────────────────────────────────────────────────
    void descWorkerLoop();
    void matchWorkerLoop();

    void computeDescriptorAsync(const SubMap::Ptr& submap);
    void onDescriptorReady(const SubMap::Ptr& submap);
    void processMatchTask(const MatchTask& task);

    // ScanContext 候选检索
    std::vector<OverlapTransformerInfer::Candidate> retrieveUsingScanContext(
        const SubMap::Ptr& submap,
        const std::vector<SubMap::Ptr>& db_copy);

    bool computeDescExternal(const SubMap::Ptr& submap);
    void publishLoopConstraint(const LoopConstraint::Ptr& lc);
};

} // namespace automap_pro
