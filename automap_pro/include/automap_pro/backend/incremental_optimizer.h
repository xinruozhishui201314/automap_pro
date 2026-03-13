#pragma once

#include "automap_pro/core/data_types.h"

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
#include <vector>
#include <functional>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <future>

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

    /** 添加子图间里程计因子，可能触发 commit */
    void addOdomFactor(int from, int to, const Pose3d& rel, const Mat66d& info_matrix);

    /**
     * 添加回环约束，立即触发 iSAM2 update（代价：O(K) K=受影响节点数）
     * 返回优化后所有 submap 的新位姿
     */
    OptimizationResult addLoopFactor(int from, int to,
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

    /** 批量添加 GPS 因子并只做一次 commitAndUpdate（由 opt 线程调用，用于避免多次 update 触发 GTSAM 内部竞态） */
    struct GPSFactorItem { int sm_id; Eigen::Vector3d pos; Eigen::Matrix3d cov; };
    void addGPSFactorsBatch(const std::vector<GPSFactorItem>& factors);

    // ── 查询当前最优位姿 ─────────────────────────────────────────────────

    Pose3d getPose(int sm_id) const;
    std::unordered_map<int, Pose3d> getAllPoses() const;

    /** 强制触发完整 iSAM2 update（用于批量 GPS 因子添加后） */
    OptimizationResult forceUpdate();

    /** 是否存在未提交的因子或 value（用于 GPS 批前先 flush，避免 Prior+GPS 同批 update） */
    bool hasPendingFactorsOrValues() const;

    /** 重置（新 session 开始） */
    void reset();

    /**
     * 关闭前清空因子图与 iSAM2 状态并释放 prior_noise_，保证可控析构顺序。
     * 不调用 ConfigManager，可重复调用（析构时再次调用安全）；调用后 addSubMapNode 等将不再添加因子。
     */
    void clearForShutdown();

    int nodeCount() const;
    int factorCount() const;

    // ── 回调 ─────────────────────────────────────────────────────────────
    using PoseUpdateCallback = std::function<void(const std::unordered_map<int, Pose3d>&)>;
    void registerPoseUpdateCallback(PoseUpdateCallback cb) {
        pose_update_cbs_.push_back(std::move(cb));
    }
    
    // ── P0 异步优化接口 ───────────────────────────────────────────────
    
    /** 优化任务类型 */
    enum class OptimTaskType {
        LOOP_FACTOR,    // 回环因子
        GPS_FACTOR,     // GPS 因子
        BATCH_UPDATE     // 批量更新（强制提交）
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
        
        // 用于批量更新
        std::function<void()> action;
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

    mutable std::shared_mutex rw_mutex_;
    std::unordered_map<int, bool> node_exists_;

    int    node_count_   = 0;
    int    factor_count_ = 0;
    bool   has_prior_    = false;

    std::vector<PoseUpdateCallback> pose_update_cbs_;

    // ── P0 异步优化队列与工作线程 ─────────────────────────────────────────
    static constexpr size_t kMaxQueueSize = 64;
    std::queue<OptimTask>  opt_queue_;
    mutable std::mutex     opt_queue_mutex_;
    std::condition_variable opt_queue_cv_;
    std::atomic<bool>      opt_running_{true};
    std::thread            opt_thread_;
    void optLoop();

    /** 是否有线程正在执行 commitAndUpdate（用于 ensureBackendCompletedAndFlushBeforeHBA 等待「后端真正空闲」再触发 HBA） */
    std::atomic<bool>      optimization_in_progress_{false};

    // ── 私有工具 ──────────────────────────────────────────────────────────
    gtsam::Pose3 toPose3(const Pose3d& T) const;
    Pose3d       fromPose3(const gtsam::Pose3& p) const;
    /** 信息矩阵转噪声（秩缺/病态时返回 Diagonal 以与双路 GTSAM 策略一致；当前仅 infoToNoiseDiagonal 用于因子图） */
    gtsam::noiseModel::Base::shared_ptr infoToNoise(const Mat66d& info) const;
    /** 从信息矩阵对角线构造 Diagonal 噪声，避免 Gaussian::Covariance 在 linearize 路径 double free */
    gtsam::noiseModel::Diagonal::shared_ptr infoToNoiseDiagonal(const Mat66d& info) const;
    OptimizationResult commitAndUpdate();
    void notifyPoseUpdate(const std::unordered_map<int, Pose3d>& poses);

    /** 首节点 defer 时无法加 GPS 因子，缓存待 forceUpdate 成功后补加 */
    std::vector<GPSFactorItem> pending_gps_factors_;
    /** 在 current_estimate_ 已有节点后，将 pending_gps_factors_ 中可加入的因子加入图；调用时需已持 rw_mutex_；返回本次加入的因子数 */
    int flushPendingGPSFactors();
};

} // namespace automap_pro
