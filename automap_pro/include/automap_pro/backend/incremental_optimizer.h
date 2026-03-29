#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/isam2_factor_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <future>
#include <chrono>
#include <deque>

namespace automap_pro {

/**
 * iSAM2 实时增量优化器
 *
 * 两轨制设计：
 *   轨道1（实时）: iSAM2 ← 每帧里程计因子 + 每个回环因子 → 实时 <50ms 更新
 *   轨道2（低频）: HBA  ← 每 N 个子图或建图结束时触发（见 HBAOptimizer）
 *
 * 因子类型：
 *   - SubMap 级里程计因子（子图间）：用于粗定位
 *   - SubMap 级回环因子（Loop Closure）：iSAM2 触发重线性化
 *   - GPS 绝对位置因子（对齐后添加）：全局约束
 *
 * 坐标约定：
 *   iSAM2 使用 GTSAM Pose3，与 automap_pro Pose3d(Eigen::Isometry3d) 相互转换
 */
class IncrementalOptimizer {
public:
    explicit IncrementalOptimizer();
    ~IncrementalOptimizer();


    // ── 因子图操作（线程安全） ────────────────────────────────────────────

    /** 添加初始子图节点（第一个 submap 设 fixed=true） */
    void addSubMapNode(int sm_id, const Pose3d& init_pose, bool fixed = false);

    /** 强制更新现有节点的位姿（用于HBA完成后同步） */
    void updateSubMapNodePose(int sm_id, const Pose3d& pose);

    /** 批量更新现有节点的位姿（用于HBA完成后同步，效率更高） */
    void updateSubMapNodePosesBatch(const std::unordered_map<int, Pose3d>& poses);

    /** 添加子图间里程计因子，可能触发 commit */
    void addOdomFactor(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);

    /**
     * 添加回环约束，立即触发 iSAM2 update（代价：O(K) K=受影响节点数）
     * 返回优化后所有 submap 的新位姿
     */
    OptimizationResult addLoopFactor(int from, int to,
                                      const Pose3d& rel, const Mat66d& info_matrix);

    /** Overload for LoopConstraint::Ptr */
    OptimizationResult addLoopFactor(const LoopConstraint::Ptr& lc);

    /**
     * 在关键帧图（KF 节点）上添加回环约束，不新建节点，仅 Between(KF(i), KF(j))。
     * 要求 kf_id_i、kf_id_j 已通过 addKeyFrameNode 加入图中。
     */
    OptimizationResult addLoopFactorBetweenKeyframes(int kf_id_i, int kf_id_j,
                                                      const Pose3d& rel, const Mat66d& info_matrix);

    /**
     * 仅将回环约束加入 pending，不触发 commit（用于子图内多条回环时批量提交，避免每条一次 commit）
     * 调用方需在本帧内随后调用 forceUpdate() 一次完成提交
     */
    void addLoopFactorDeferred(int from, int to,
                               const Pose3d& rel, const Mat66d& info_matrix);

    /**
     * 添加 GPS 绝对位置因子（GPSFactor，仅 X/Y/Z 位置，不约束姿态）
     * @param sm_id   对应 submap 节点 ID
     * @param pos_map GPS 位置（已对齐到地图坐标系的 ENU，单位：米）
     * @param cov3x3  位置协方差矩阵（3×3，ENU精度）
     */
    void addGPSFactor(int sm_id, const Eigen::Vector3d& pos_map,
                      const Eigen::Matrix3d& cov3x3);

    // ── KeyFrame 级别因子（与 HBA 对齐，增强 GPS 约束） ────────────────────

    /** 添加 keyframe 节点（KF 级别，与 HBA 的 level1 对齐）。
     * @param is_first_kf_of_submap 若为 true 则强制添加 Prior，避免新子图首帧无约束导致 IndeterminantLinearSystemException
     * @param submap_id_for_sm_kf_anchor 该关键帧所属子图 ID（与 KeyFrame::submap_id 一致）。用于 Between(SM(sm),KF(首帧))；
     *        不得使用 kf_id/MAX_KF_PER_SUBMAP（全局 kf id 递增时恒为 0）。传 -1 表示不登记锚点（仅兼容旧路径）。 */
    void addKeyFrameNode(int kf_id, const Pose3d& init_pose, bool fixed = false,
                         bool is_first_kf_of_submap = false, int submap_id_for_sm_kf_anchor = -1);

    /** 添加 keyframe 间的里程计因子 */
    void addOdomFactorBetweenKeyframes(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);

    /** 添加 keyframe 级别的 GPS 因子（KF 级别） */
    void addGPSFactorForKeyFrame(int kf_id, const Eigen::Vector3d& pos_map,
                                 const Eigen::Matrix3d& cov3x3);

    /** 添加 keyframe 级别的圆柱地标因子 */
    void addCylinderFactorForKeyFrame(int kf_id, const CylinderFactorItemKF& factor);
    /** 添加 keyframe 级别的平面地标因子 */
    void addPlaneFactorForKeyFrame(int kf_id, const PlaneFactorItemKF& factor);

    /** 获取 keyframe 节点位姿 */
    Pose3d getKeyFramePose(int kf_id) const;

    /** 检查 keyframe 节点是否存在 */
    bool keyFrameExists(int kf_id) const;

    /** 批量添加 keyframe GPS 因子 */
    struct GPSFactorItemKF { int kf_id; Eigen::Vector3d pos; Eigen::Matrix3d cov; };
    void addGPSFactorsForKeyFramesBatch(const std::vector<GPSFactorItemKF>& factors);

    /** 批量添加 GPS 因子并只做一次 commitAndUpdate（由 opt 线程调用，用于避免多次 update 触发 GTSAM 内部竞态） */
    struct GPSFactorItem { int sm_id; Eigen::Vector3d pos; Eigen::Matrix3d cov; };
    void addGPSFactorsBatch(const std::vector<GPSFactorItem>& factors);

    // ── 查询当前最优位姿 ─────────────────────────────────────────────────

    /**
     * @brief 获取子图位姿（不推荐使用，返回 Identity 作为 fallback 会导致假漂移）
     * @deprecated 请使用 getPoseOptional() 替代
     */
    Pose3d getPose(int sm_id) const;

    /**
     * @brief 获取子图位姿的可选版本，明确区分三种状态
     * @param sm_id 子图 ID
     * @return std::nullopt_t 如果节点完全不存在于因子图中
     *         std::nullopt_t 如果节点存在但尚未被 forceUpdate 提交到 current_estimate_
     *         Pose3d 实际位姿（如果已优化）
     */
    std::optional<Pose3d> getPoseOptional(int sm_id) const;

    std::unordered_map<int, Pose3d> getAllPoses() const;
    std::unordered_map<uint64_t, Pose3d> getAllKeyFramePoses() const;

    /**
     * @brief 检查节点是否存在于因子图但不在 current_estimate_ 中
     * @param sm_id 子图 ID
     * @return true 表示节点存在但未进入 estimate（需要 forceUpdate）
     *         false 表示节点要么在 estimate 中，要么完全不存在
     */
    bool hasNodePendingEstimate(int sm_id) const;

    /** 强制触发完整 iSAM2 update（用于批量 GPS 因子添加后） */
    OptimizationResult forceUpdate();

    /** 是否存在未提交的因子或 value（用于 GPS 批前先 flush，避免 Prior+GPS 同批 update） */
    bool hasPendingFactorsOrValues() const;

    /** 重置（新 session 开始） */
    void reset();

    /** 🏛️ [架构加固] 修复 GPS 对齐发散：将现有历史数据转换到 MAP 坐标系并重建因子图
     * @param T_map_odom         坐标系转换：p_map = T_map_odom * p_odom
     * @param suppress_pose_notify 若为 true，REBUILD 后不立即调用 notifyPoseUpdate。
     *        用于 processGPSBatchKF 场景：REBUILD 之后紧跟 addGPSFactorsForKeyFramesBatch，
     *        后者有自己的 notifyPoseUpdate，避免发布 GPS 因子尚未收敛的中间态位姿。
     */
    void transformHistoryAndRebuild(const Pose3d& T_map_odom, bool suppress_pose_notify = false);

    /** 获取当前因子图坐标系语义 */
    PoseFrame getPoseFrame() const { return current_pose_frame_; }

    /** 获取所有子图数据（用于 GPS 对齐后重建） */
    std::vector<SubmapData> getAllSubmapData() const;

    /** 获取所有里程计因子（用于 GPS 对齐后重建） */
    std::vector<OdomFactorItem> getOdomFactors() const;

    /** 获取所有回环因子（用于 GPS 对齐后重建） */
    std::vector<LoopFactorItem> getLoopFactors() const;

    /** 获取所有关键帧数据（用于 GPS 对齐后重建） */
    std::vector<KeyFrameData> getKeyFrameData() const;

    /** 获取所有关键帧里程计因子（用于 GPS 对齐后重建） */
    std::vector<OdomFactorItemKF> getKFOdomFactors() const;

    /** 获取所有关键帧回环因子（用于 GPS 对齐后重建） */
    std::vector<LoopFactorItemKF> getKFLoopFactors() const;

    /** GPS 对齐后重建因子图 */
    void rebuildAfterGPSAlign(const std::vector<SubmapData>& submap_data,
                              const std::vector<OdomFactorItem>& odom_factors,
                              const std::vector<LoopFactorItem>& loop_factors,
                              const std::vector<KeyFrameData>& keyframe_data = {},
                              const std::vector<OdomFactorItemKF>& kf_odom_factors = {},
                              const std::vector<LoopFactorItemKF>& kf_loop_factors = {});

    /** 关闭前清空因子图与 iSAM2 状态并释放 prior_noise_，保证可控析构顺序。
     * 不调用 ConfigManager，可重复调用（析构时再次调用安全）；调用后 addSubMapNode 等将不再添加因子。
     */
    void clearForShutdown();

    int nodeCount() const;
    int factorCount() const;

    // ── 回调 ─────────────────────────────────────────────────────────────
    /** 位姿更新回调类型：返回优化后的子图与关键帧位姿 */
    using PoseUpdateCallback = std::function<void(const OptimizationResult&)>;
    void registerPoseUpdateCallback(PoseUpdateCallback cb) {
        pose_update_cbs_.push_back(std::move(cb));
    }
    
    // ── P0 异步优化接口 ───────────────────────────────────────────────
    
    /** 优化任务类型 */
    enum class OptimTaskType {
        LOOP_FACTOR,    // 回环因子
        GPS_FACTOR,     // GPS 因子
        BATCH_UPDATE,   // 批量更新（强制提交）
        REBUILD         // 重建任务（GPS对齐后重建）
    };

    /** 优化任务数据 */
    struct OptimTask {
        OptimTaskType type;
        int from_id;
        int to_id;
        Pose3d rel_pose;
        Mat66d info_matrix;
        Eigen::Vector3d gps_pos;
        Eigen::Matrix3d gps_cov;
        /** 为 true 时表示回环为关键帧级，from_id/to_id 为 KF 节点 id，用 Between(KF(i), KF(j)) */
        bool loop_is_keyframe_level = false;
        // 用于批量更新
        std::function<void()> action;
        // REBUILD 任务专用字段
        std::vector<SubmapData> submap_data;
        std::vector<OdomFactorItem> odom_factors;
        std::vector<LoopFactorItem> loop_factors;
    };

    /** 提交优化任务到队列（立即返回，异步执行） */
    void enqueueOptTask(const OptimTask& task);
    
    /** 批量提交优化任务（用于多个 GPS 因子） */
    void enqueueOptTasks(const std::vector<OptimTask>& tasks);
    
    /** 等待所有已入队任务完成 */
    void waitForPendingTasks();
    
    /** 获取当前队列深度 */
    size_t getQueueDepth() const;

    // ── 健康检查与恢复 ───────────────────────────────────────────────────

    /** 优化器健康状态 */
    struct HealthStatus {
        bool is_healthy = true;
        int consecutive_failures = 0;
        int total_optimizations = 0;
        int failed_optimizations = 0;
        double last_success_time_ms = 0.0;
        std::string last_error_message;
    };

    /** 获取当前健康状态 */
    HealthStatus getHealthStatus() const;

    /** 记录优化失败（用于健康监控） */
    void recordOptimizationFailure(const std::string& error_msg);

    /** 记录优化成功 */
    void recordOptimizationSuccess(double elapsed_ms);

    /** 重置优化器（用于崩溃恢复） */
    void resetForRecovery();

    // ===== 公共诊断方法 =====
    /** 手动将 pending values 标记为存在于 current_estimate_（GTSAM 单节点延迟问题的 workaround） */
    void markPendingValuesAsEstimated();

    /** 获取当前 pending values 数量（用于诊断） */
    size_t pendingValuesCount() const;

    /** 获取当前 pending factors 数量（用于诊断） */
    size_t pendingFactorsCount() const;

    // ── KeyFrame 级别 GPS 因子刷新 ───────────────────────────────────────
    /** 刷新 pending keyframe GPS 因子（在子图冻结或HBA完成后调用） */
    int flushPendingGPSFactorsForKeyFrames();
    
    /** 刷新 pending keyframe GPS 因子（内部版本，假设锁已由调用者持有） */
    int flushPendingGPSFactorsForKeyFramesInternal();
    /** 刷新 pending 语义地标因子（内部版本，假设锁已由调用者持有） */
    int flushPendingSemanticFactorsForKeyFramesInternal();

    /** 刷新 pending submap-level GPS 因子（在子图冻结成功后调用） */
    int flushPendingGPSFactors();

private:
    // 健康状态数据
    std::atomic<int> consecutive_failures_{0};
    std::atomic<int> total_optimizations_{0};
    std::atomic<int> failed_optimizations_{0};
    std::atomic<double> last_success_time_ms_{0.0};
    std::string last_error_message_;
    mutable std::mutex health_mutex_;
    gtsam::ISAM2         isam2_;
    gtsam::NonlinearFactorGraph pending_graph_;
    gtsam::Values               pending_values_;
    gtsam::Values               current_estimate_;

    /** 先验方差（6 维），用于每次 addSubMapNode 时新建 Prior noise，避免共享 noise 在 linearize 时 double free */
    gtsam::Vector prior_var6_;
    /** 仅用于 shutdown 检查与 has_prior_ 语义；Prior 因子改用 prior_var6_ 新建 noise */
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;

    /** 锁顺序约定（防死锁）：由 opt_worker 等调用方在不长时间持 submap_update_mutex_ 的前提下调用本类 add*；本类持 rw_mutex_。禁止在持 rw_mutex_ 时调用会获取 AutoMapSystem::submap_update_mutex_ 或 SubMapManager::mutex_ 的代码。见 BACKEND_POTENTIAL_ISSUES 1.3.3、BACKEND_MODULARITY_AND_STABILITY_ANALYSIS.md */
    mutable std::shared_mutex rw_mutex_;
    std::unordered_map<int, bool> node_exists_;
    // KeyFrame 级别节点跟踪（与 HBA level1 对齐）
    std::unordered_map<int, bool> keyframe_node_exists_;
    int keyframe_count_ = 0;
    /** 上一 keyframe 的 id 与位姿，用于添加 KF(i)->KF(i+1) 的 Between 因子，避免欠定 */
    int last_keyframe_id_ = -1;
    Pose3d last_keyframe_pose_;

    // ── GPS对齐后重建所需历史数据 ──
    mutable std::mutex history_mutex_;
    std::vector<SubmapData> history_submap_data_;
    std::vector<OdomFactorItem> history_odom_factors_;
    std::vector<LoopFactorItem> history_loop_factors_;
    std::vector<KeyFrameData> history_keyframe_data_;
    std::vector<OdomFactorItemKF> history_kf_odom_factors_;
    std::vector<LoopFactorItemKF> history_kf_loop_factors_;

    int    node_count_   = 0;
    int    factor_count_ = 0;
    bool   has_prior_    = false;
    PoseFrame current_pose_frame_ = PoseFrame::ODOM; // 🏛️ [架构加固] 跟踪当前因子图坐标系语义

    std::vector<PoseUpdateCallback> pose_update_cbs_;

    // ── 🏛️ [架构加固] 事务性图插入 (Transactional Graph Insertion) ──
    // 解决异步数据流导致的“孤立节点”引发 IndeterminantLinearSystemException 崩溃。
    // 新增节点与因子首先进入 Staging 区，只有当节点满足 grounding 条件（有 Prior 或链接到已入图节点）
    // 才原子化地移入 pending_values_ / pending_graph_。
    gtsam::Values               staged_values_;
    gtsam::NonlinearFactorGraph staged_factors_;
    /** 检查并尝试将满足条件的 Staging 数据移入 Pending */
    void tryMoveStagedToPendingInternal();
    /** 判断 Key 是否已经“着陆”（在 isam2 内部或已在 pending 中） */
    bool isGroundedInternal(gtsam::Key key) const;

    /** 在 SM 与首关键帧之间添加 Identity Between（每子图至多一条）；调用方须已持 rw_mutex_ */
    void tryLinkSubmapAnchorKeyFrame_(int sm_id, int anchor_kf_id);
    /** 子图 id -> 该子图首关键帧全局 id（用于 SM 节点晚于 KF 入图时补链） */
    std::unordered_map<int, int> submap_anchor_kf_id_;
    /** 已为 Between(SM,KF) 锚定过的子图 id */
    std::unordered_set<int> submap_sm_kf_anchor_linked_;

    // ── P0 异步优化队列与工作线程 ─────────────────────────────────────────
    // 🔧 V2 修复：删除内部 opt_thread_，GTSAM 写入由外部 opt_worker_thread_ 独占
    /*
    static constexpr size_t kMaxQueueSize = 64;
    std::queue<OptimTask>  opt_queue_;
    mutable std::mutex     opt_queue_mutex_;
    std::condition_variable opt_queue_cv_;
    std::atomic<bool>      opt_running_{true};
    std::thread            opt_thread_;
    void optLoop();
    */

    /** 是否有线程正在执行 commitAndUpdate（用于 ensureBackendCompletedAndFlushBeforeHBA 等待「后端真正空闲」再触发 HBA） */
    std::atomic<bool>      optimization_in_progress_{false};

    // ── 私有工具 ──────────────────────────────────────────────────────────
    gtsam::Pose3 toPose3(const Pose3d& T) const;
    Pose3d       fromPose3(const gtsam::Pose3& p) const;
    /** 信息矩阵转噪声（秩缺/病态时返回 Diagonal 以与双路 GTSAM 策略一致；当前仅 infoToNoiseDiagonal 用于因子图） */
    gtsam::noiseModel::Base::shared_ptr infoToNoise(const Mat66d& info) const;
    /** 从信息矩阵对角线构造 Diagonal 噪声，避免 Gaussian::Covariance 在 linearize 路径 double free */
    gtsam::noiseModel::Diagonal::shared_ptr infoToNoiseDiagonal(const Mat66d& info) const;

    // ── 🏛️ [架构加固] 内部无锁版本，用于 transformHistoryAndRebuild 等复合操作 ──
    void resetInternal();
    void addKeyFrameNodeInternal(int kf_id, const Pose3d& pose, bool fixed, bool is_first_kf_of_submap, int sm_anchor);
    void addSubMapNodeInternal(int id, const Pose3d& pose, bool fixed);
    void addOdomFactorBetweenKeyframesInternal(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);
    void addLoopFactorInternal(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);
    void addLoopFactorDeferredInternal(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);
    void addOdomFactorInternal(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);

    OptimizationResult commitAndUpdate();
    void notifyPoseUpdate(const OptimizationResult& res);
    double computeCylinderResidualUnsafe_(int kf_id, const CylinderFactorItemKF& factor) const;
    void logSemanticResidualDiagnosticsUnsafe_(double residual);

    /** 首节点 defer 时无法加 GPS 因子，缓存待 forceUpdate 成功后补加 */
    std::vector<GPSFactorItem> pending_gps_factors_;
    /** 子图里程计缺节点时延迟入图，避免新增 submap 节点孤立 */
    std::vector<OdomFactorItem> pending_odom_factors_submap_;
    /** KeyFrame 级别 pending GPS 因子 */
    std::vector<GPSFactorItemKF> pending_gps_factors_kf_;
    /** KeyFrame 级别 pending 语义圆柱因子 */
    std::vector<CylinderFactorItemKF> pending_cylinder_factors_kf_;
    /** KeyFrame 级别 pending 语义平面因子 */
    std::vector<PlaneFactorItemKF> pending_plane_factors_kf_;
    /** 内部版本：假设锁已由调用者持有（由 public flushPendingGPSFactors 调用） */
    int flushPendingGPSFactorsInternal();
    /** 失败路径：将本批 pending 中的 keyframe id 从 keyframe_node_exists_ 移除并回退 keyframe_count_/last_keyframe_id_（调用前由调用方从 pending_values_ 收集 kf_ids）。见 BACKEND_POTENTIAL_ISSUES 1.3.1 */
    void rollbackKeyframeStateForPendingKeys(const std::vector<int>& kf_ids_in_pending);

    // ===== 修复2: 单节点超时强制优化 =====
    /** 记录单节点延迟开始时间（用于超时强制优化） */
    std::chrono::steady_clock::time_point single_node_pending_start_;
    /** 单节点pending超时阈值（毫秒），超过后强制执行优化 */
    static constexpr int kSingleNodePendingTimeoutMs = 5000;

    /** 构造时缓存，避免 worker/commit 路径访问 ConfigManager 单例导致 shutdown 时 SIGSEGV */
    bool backend_verbose_trace_{false};
    double isam2_relin_thresh_{0.01};
    int isam2_relin_skip_{1};
    bool isam2_enable_relin_{true};
    int backend_max_pending_gps_kf_{1000};
    // addGPSFactor 用 GPS 协方差/异常值相关参数
    bool gps_enable_dynamic_cov_{true};
    int gps_min_satellites_{4};
    double gps_high_altitude_threshold_{100.0};
    double gps_high_altitude_scale_{2.0};
    bool gps_enable_outlier_detection_{true};
    double gps_outlier_cov_scale_{100.0};
    // [GPS高度约束修复] 默认禁用高度约束，防止不可靠 GPS 高度导致多重地面重影
    bool gps_disable_altitude_constraint_{true};
    double gps_altitude_variance_override_{1e6};
    double semantic_switchable_residual_scale_m_{0.25};
    mutable std::deque<double> semantic_residual_window_;
    static constexpr size_t kSemanticResidualWindowSize = 400;
    /** 回环 Between 相对量上限（与 ConfigManager loop_closure.constraint_max_*）；0=不启用 */
    double loop_constraint_max_translation_m_{0.0};
    double loop_constraint_max_rotation_deg_{0.0};
    /** true=拒绝入图（addLoop* / rebuild 重放）；打 VALIDATION 日志 */
    bool loopConstraintMaxNormViolated_(const Pose3d& rel, const char* tag, int from, int to) const;
};

} // namespace automap_pro
