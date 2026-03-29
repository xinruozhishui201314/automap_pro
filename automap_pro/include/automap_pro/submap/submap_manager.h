#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/core/error_code.h"
#include <automap_pro/msg/sub_map_event_msg.hpp>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <atomic>
#include <vector>
#include <functional>
#include <thread>
#include <queue>
#include <condition_variable>
#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

/**
 * SubMap 生命周期管理器（MS-Mapping 增量式建图核心）
 *
 * 子图状态机：
 *   ACTIVE → (isFull) → FROZEN → (iSAM2优化) → OPTIMIZED → (持久化) → ARCHIVED
 *
 * MS-Mapping 增量式建图特性：
 *   1. 同一 session 内支持跨子图回环（历史描述子数据库）
 *   2. 跨 session 回环：通过 SessionManager 加载历史描述子
 *   3. 子图位姿优化后，点云自动重新投影（reproject）
 *   4. 新 session 与历史 session 点云融合（map merging）
 */
class SubMapManager {
public:
    explicit SubMapManager();
    ~SubMapManager();

    void init(rclcpp::Node::SharedPtr node);
    void stop();

    // ── 主要接口 ──────────────────────────────────────────────────────────

    /**
     * 添加新关键帧到当前活跃子图
     * 内部自动处理子图切分和冻结
     */
    void addKeyFrame(const KeyFrame::Ptr& kf);

    /**
     * 子图锚定位姿更新（由 iSAM2 优化后回调）
     * 同时更新子图内所有关键帧的 T_map_b_optimized
     */
    void updateSubmapPose(int submap_id, const Pose3d& new_pose, PoseFrame pose_frame, uint64_t alignment_epoch = 0);

    /** 批量更新子图位姿（来自 iSAM2 优化结果），比逐个调用 updateSubmapPose 更高效，且支持版本号一致性 */
    void batchUpdateSubmapPoses(const std::unordered_map<int, Pose3d>& updates, uint64_t version, PoseFrame pose_frame, uint64_t alignment_epoch = 0);

    /** 🏛️ [架构加固] 批量更新关键帧绝对位姿（SSoT 语义网关应用） */
    void batchUpdateKeyFramePoses(const std::unordered_map<uint64_t, Pose3d>& updates, uint64_t version, PoseFrame pose_frame, uint64_t alignment_epoch = 0);

    /** 批量更新子图位姿（来自 HBA 结果） */
    void updateAllFromHBA(const HBAResult& result);

    /** HBA 完成后重建 merged_cloud（使用优化后的位姿） */
    void rebuildMergedCloudFromOptimizedPoses();

    /**
     * GPS 对齐成功后：将仍为 ODOM 语义的关键帧升为 MAP（T_map_b = T_map_odom * T_odom_b），
     * 重同步各子图 pose_map_anchor_optimized 与 T_submap_kf，清空全局图缓存并按 T_map_b_optimized 重建 merged_cloud。
     * 与 IncrementalOptimizer::transformHistoryAndRebuild 及 MappingModule 新建 KF 升级路径一致；
     * 应在 freezeSubmap 之前调用，避免冻结后仍携带对齐前世界系的 merged_cloud。
     */
    void applyGpsMapOriginToOdomKeyframes(const Eigen::Matrix3d& R_enu_to_map,
                                          const Eigen::Vector3d& t_enu_to_map,
                                          uint64_t map_version,
                                          uint64_t alignment_epoch = 0);

    /**
     * 启动新会话（增量建图）
     * @param session_id  新 session ID（由 SessionManager 分配）
     */
    void startNewSession(uint64_t session_id);

    // ── 查询接口 ──────────────────────────────────────────────────────────

    /** 🏛️ [架构加固] 允许外部持锁读取子图状态，确保 MappingModule 与 OptimizerModule 间的线程安全 */
    std::mutex& mutex() const { return mutex_; }

    SubMap::Ptr         getActiveSubmap() const;
    SubMap::Ptr         getSubmap(int id)  const;
    std::vector<SubMap::Ptr> getAllSubmaps() const;
    std::vector<SubMap::Ptr> getFrozenSubmaps() const;
    int                 submapCount()  const;
    int                 keyframeCount()const;

    // ── 持久化 ────────────────────────────────────────────────────────────

    /** 将已优化子图序列化到磁盘（增量式建图的核心：允许跨 session 加载） */
    bool archiveSubmap(const SubMap::Ptr& submap, const std::string& dir);

    /** 从磁盘恢复子图（跨 session 加载历史地图） */
    bool loadArchivedSubmap(const std::string& dir, int submap_id, SubMap::Ptr& out);

    // ── 回调注册 ──────────────────────────────────────────────────────────
    /** 注册子图冻结回调。回调在锁外执行；禁止在回调中调用本类任何会获取 mutex_ 的接口（如 getFrozenSubmaps、addKeyFrame 等），否则可能死锁。入参 sm 即为本次冻结的子图，足以完成逻辑。 */
    void registerSubmapFrozenCallback(SubMapFrozenCallback cb) {
        std::lock_guard<std::mutex> lk(frozen_cbs_mutex_);
        frozen_cbs_.push_back(std::move(cb));
    }

    // ── 全局点云构建 ──────────────────────────────────────────────────────

    /** 构建全局合并点云（使用优化后位姿） */
    CloudXYZIPtr buildGlobalMap(float voxel_size = 0.2f, uint64_t alignment_epoch_limit = 0) const;
    
    /** 异步构建全局点云（返回 future，调用方可异步等待） */
    std::future<CloudXYZIPtr> buildGlobalMapAsync(float voxel_size = 0.2f, uint64_t alignment_epoch_limit = 0) const;

    /** 使全局图体素缓存失效；终局 save 前或需强制全量重算时调用（HBA 写回内在持锁路径内直接清空缓存）。 */
    void invalidateGlobalMapCache() const;

    /** 更新子图的 GPS 重力中心 */
    void updateGPSGravityCenter(const KeyFrame::Ptr& kf);

    /** 检查子图管理器是否空闲（没有正在进行的合并或后处理任务） */
    bool isIdle() const {
        std::lock_guard<std::mutex> lk(merge_mutex_);
        std::lock_guard<std::mutex> lk2(freeze_post_mutex_);
        return merge_queue_.empty() && freeze_post_queue_.empty() && 
               active_merge_tasks_.load() == 0;
    }

    /** 发布错误事件到 ROS2 话题 */
    void publishErrorEvent(int submap_id, const automap_pro::ErrorDetail& error);

    /**
     * 建图结束前强制冻结当前活跃子图（若存在），使最后一子图进入 iSAM2 因子图，保证 submap 数量与图节点一致。
     * 同步执行：voxel + frozen 回调在本次调用内完成，返回后 ensureBackend/forceUpdate 可包含该子图。
     */
    void forceFreezeActiveSubmapForFinish();

    /** 检查当前活跃子图是否满足冻结条件 */
    bool needFreezeSubmap() const;

    /** 冻结当前活跃子图并返回它（用于异步处理） */
    SubMap::Ptr freezeCurrentSubmap();

    /**
     * 冻结指定的活跃子图（必须是 ACTIVE 状态）
     * @param sm 要冻结的子图指针
     */
    void freezeSubmap(const SubMap::Ptr& sm);

    /**
     * @brief 关联关键帧中的语义地标到子图
     * @param sm 子图指针
     * @param kf 关键帧指针
     */
    void associateLandmarks(SubMap::Ptr& sm, const KeyFrame::Ptr& kf);
    bool isSemanticProtectionModeActive() const;

private:
    std::vector<SubMap::Ptr>  submaps_;
    SubMap::Ptr               active_submap_;
    mutable std::mutex        mutex_;
    std::atomic<int>          submap_id_counter_{0};
    uint64_t                  current_session_id_ = 0;

    // 关键帧计数（全局）
    std::atomic<uint64_t>     kf_id_counter_{0};

    // ── 全局点云构建缓存（P0 架构优化：解决高频冗余构建导致的性能卡顿） ──
    mutable CloudXYZIPtr      cached_global_map_;
    mutable uint64_t          last_build_kf_count_ = 0;
    mutable uint64_t          last_build_map_version_ = 0;
    mutable float             last_build_voxel_size_ = 0.0f;
    uint64_t                  current_map_version_ = 0; // 跟踪 MapRegistry 传入的位姿版本

    // [RC5/RC6 修复] 标记 merged_cloud 已被逻辑上废弃，等待 rebuildMergedCloudFromOptimizedPoses 完成。
    // 在 applyGpsMapOriginToOdomKeyframes / batchUpdateSubmapPoses / batchUpdateKeyFramePoses
    // 释放锁、触发重建之前设为 true；rebuildMergedCloudFromOptimizedPoses 结束时重置为 false。
    // buildGlobalMap 的 fallback 路径在此标志为 true 时跳过 merged_cloud，避免旧坐标系点云重影。
    mutable std::atomic<bool> merged_cloud_dirty_{false};

    // ✅ 修复：VoxelGrid 对象池缓存（按分辨率索引，mutable 以便 const 方法可延迟填充）
    mutable std::unordered_map<float, std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZI>>> voxel_grid_cache_;
    mutable std::mutex vg_cache_mutex_;
    
    // 常用分辨率
    static constexpr float RES_MATCH = 0.4f;
    static constexpr float RES_MERGE = 0.2f;
    
    // 获取缓存的 VoxelGrid 对象（const 版本供 mergeCloudToSubmap 使用）
    pcl::VoxelGrid<pcl::PointXYZI>* getVoxelGrid(float leaf_size) const {
        std::lock_guard<std::mutex> lk(vg_cache_mutex_);
        
        auto it = voxel_grid_cache_.find(leaf_size);
        if (it != voxel_grid_cache_.end()) {
            return it->second.get();
        }
        
        // 创建并缓存新对象
        auto vg = std::make_unique<pcl::VoxelGrid<pcl::PointXYZI>>();
        vg->setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid_cache_[leaf_size] = std::move(vg);
        
        return voxel_grid_cache_[leaf_size].get();
    }

    // ✅ 修复：点云对象池（避免频繁内存分配，mutable 以便 const 方法可借用）
    static constexpr size_t CLOUD_POOL_SIZE = 10;
    mutable std::array<CloudXYZIPtr, CLOUD_POOL_SIZE> cloud_pool_;
    mutable std::atomic<size_t> pool_index_{0};
    mutable std::mutex pool_mutex_;
    
    // 从对象池获取点云（const 版本供 mergeCloudToSubmap 使用）
    // ✅ P0-2 修复：返回深拷贝，避免多线程竞争同一 pool 槽位导致的数据竞争与崩溃
    CloudXYZIPtr getCloudFromPool() const {
        std::lock_guard<std::mutex> lk(pool_mutex_);
        size_t idx = pool_index_.fetch_add(1) % CLOUD_POOL_SIZE;
        
        if (!cloud_pool_[idx] || cloud_pool_[idx]->empty()) {
            cloud_pool_[idx] = std::make_shared<CloudXYZI>();
        }
        
        // 深拷贝点云内容到新对象，避免调用方并发修改同一 pool 槽位
        // 有性能损失，但保证线程安全，运行中不出现崩溃
        auto cloud_copy = std::make_shared<CloudXYZI>();
        cloud_copy->points = cloud_pool_[idx]->points;
        return cloud_copy;
    }

    // 参数
    int    max_kf_       = 100;
    double max_spatial_  = 100.0;   // 米
    double max_temporal_ = 60.0;    // 秒
    double match_res_    = 0.4;     // 降采样分辨率（用于回环匹配）
    double merge_res_    = 0.2;     // 降采样分辨率（合并地图，至少 0.2 避免 PCL 溢出）

    /** 构造时缓存，避免 merge/freeze/build 线程访问 ConfigManager 单例导致 shutdown 时 SIGSEGV */
    bool retain_cloud_body_{true};
    bool map_statistical_filter_{true};
    int map_stat_filter_mean_k_{50};
    double map_stat_filter_std_mul_{1.0};
    double submap_rebuild_thresh_trans_{2.0};
    double submap_rebuild_thresh_rot_{5.0};
    bool parallel_voxel_downsample_{true};
    /** 子图 merge / 全量压缩 / freeze 下采样：语义 intensity 众数投票（构造时从配置读取） */
    bool submap_merge_semantic_intensity_vote_{true};
    bool backend_verbose_trace_{false};

    // Semantic association thresholds (configurable via semantic.geometric.association.*)
    double assoc_cyl_max_dist_xy_m_{0.4};
    double assoc_cyl_max_dist_z_m_{1.0};
    double assoc_cyl_max_angle_deg_{10.0};
    double assoc_cyl_max_radius_diff_m_{0.08};
    double assoc_cyl_alpha_min_{0.05};
    double assoc_cyl_alpha_max_{0.4};
    double assoc_cyl_mahalanobis_gate_{9.49};
    int assoc_cyl_min_confirmations_{2};
    double assoc_cyl_min_observability_{0.25};
    double assoc_cyl_duplicate_merge_dist_xy_m_{0.25};
    double assoc_cyl_duplicate_merge_max_angle_deg_{8.0};
    bool assoc_cyl_protection_mode_enabled_{true};
    double assoc_cyl_protect_trigger_tree_new_rate_pct_{35.0};
    double assoc_cyl_protect_recover_tree_new_rate_pct_{20.0};
    double assoc_cyl_protect_trigger_duplicate_density_{0.03};
    double assoc_cyl_protect_recover_duplicate_density_{0.01};
    bool assoc_cyl_protection_mode_active_{false};
    double assoc_plane_max_angle_deg_{10.0};
    double assoc_plane_max_distance_diff_m_{0.4};
    double assoc_plane_max_tangent_offset_m_{1.5};
    double assoc_plane_alpha_min_{0.05};
    double assoc_plane_alpha_max_{0.35};

    // Semantic association stats
    std::atomic<uint64_t> assoc_tree_matched_total_{0};
    std::atomic<uint64_t> assoc_tree_new_total_{0};
    std::atomic<uint64_t> assoc_plane_matched_total_{0};
    std::atomic<uint64_t> assoc_plane_new_total_{0};
    std::atomic<uint64_t> assoc_tree_duplicate_merge_total_{0};

    std::weak_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<automap_pro::msg::SubMapEventMsg>::SharedPtr event_pub_;

    std::vector<SubMapFrozenCallback> frozen_cbs_;
    mutable std::mutex frozen_cbs_mutex_;  // 保护 frozen_cbs_，回调前复制列表、锁外执行，避免重入死锁

    // 子图冻结后处理异步：voxel(merged_cloud)→downsampled_cloud 与 frozen_cbs_ 在专用线程执行，避免阻塞 addKeyFrame（有界队列+超时防死锁）
    static constexpr size_t   kMaxFreezePostQueueSize = 128;
    std::queue<SubMap::Ptr>   freeze_post_queue_;
    mutable std::mutex        freeze_post_mutex_;  // isIdle() 等 const 方法需检查队列
    std::condition_variable   freeze_post_cv_;
    std::thread               freeze_post_thread_;
    std::atomic<bool>         freeze_post_running_{true};
    void freezePostProcessLoop();
    
    // 🏛️ V3: 点云合并异步化 (Merge Worker)
    struct MergeTask {
        SubMap::Ptr sm;
        KeyFrame::Ptr kf;
    };
    static constexpr size_t   kMaxMergeQueueSize = 256;
    int                       merge_thread_count_ = 4;
    std::queue<MergeTask>     merge_queue_;
    mutable std::mutex        merge_mutex_;  // isIdle() 等 const 方法需检查队列
    std::condition_variable   merge_cv_;
    std::vector<std::thread>  merge_threads_; // 🏛️ [P0 性能优化] 支持多线程合并
    std::atomic<bool>         merge_running_{true};
    std::atomic<int>          active_merge_tasks_{0}; // 🏛️ [修复] 跟踪正在处理的任务数，用于 isIdle 判断
    // [RC-3 修复] 每个 merge 任务完成后通知等待者（freezeActiveSubmap 等 pending_merge_count==0）
    std::condition_variable   merge_done_cv_;
    void mergeWorkerLoop();

    // ── 私有方法 ──────────────────────────────────────────────────────────
    /**
     * 内部实现：不持锁的地图构建（供同步 buildGlobalMap 使用，持锁调用）。
     * @param poses_snapshot 若非空，则使用此快照中的位姿（与 submaps_copy 的 KF 迭代顺序一致），
     *                       避免与 onPoseUpdated/updateAllFromHBA 并发写导致的重影（见 GHOSTING_DEEP_ANALYSIS）。
     * @param build_id 非 0 时用于 GHOSTING_DIAG 日志串联（同一 build 的 snapshot/enter/exit 使用相同 build_id）。
     */
    CloudXYZIPtr buildGlobalMapInternal(
        const std::vector<SubMap::Ptr>& submaps_copy,
        float voxel_size,
        const std::vector<Pose3d>* poses_snapshot = nullptr,
        uint64_t build_id = 0,
        uint64_t alignment_epoch_limit = 0) const;

    /**
     * 从 (cloud, pose) 快照构建全局图，不访问 SubMap/KeyFrame，供异步 buildGlobalMapAsync 使用以消除与后端的并发访问导致的 SIGSEGV。
     */
    CloudXYZIPtr buildGlobalMapInternalFromSnapshot(
        const std::vector<std::pair<CloudXYZIPtr, Pose3d>>& cloud_pose_snapshot,
        float voxel_size,
        uint64_t build_id = 0) const;

    /** 安全获取节点指针 */
    rclcpp::Node::SharedPtr node() const { return node_.lock(); }

    /** 与 HBAOptimizer::collectKeyFramesFromSubmaps 相同顺序：过滤(有效 cloud_body + 有限 T_w_b)、按 timestamp 排序、按 timestamp 去重。用于 updateAllFromHBA 写回时与 result.optimized_poses 一一对应，避免顺序错位导致 PCD 重影。调用方须已持 mutex_。 */
    std::vector<KeyFrame::Ptr> collectKeyframesInHBAOrder() const;

    SubMap::Ptr createNewSubmap(const KeyFrame::Ptr& first_kf);
    /** 无参版本仅用于兼容；实际冻结请用传入 submap 的版本，且须在未持 mutex_ 时调用，避免回调内 getFrozenSubmaps 死锁 */
    void        freezeActiveSubmap();
    /** 对指定子图执行冻结并触发回调。调用方不得持有 mutex_。回调在锁外执行，且禁止在回调中调用本类会获取 mutex_ 的接口（如 getFrozenSubmaps），否则可能死锁。 */
    void        freezeActiveSubmap(const SubMap::Ptr& sm);
    bool        isFull(const SubMap::Ptr& sm) const;
    /** 与 merge / rebuild 一致：body 系 merge 分辨率下采样（可选对齐语义 + SOR） */
    CloudXYZIPtr downsampleKeyframeBodyForMerging_(const KeyFrame::Ptr& kf) const;
    void        mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const;
    /** merged 已在地图世界系更新后，用当前 pose_map_anchor_optimized 投到锚点系并以 match_res_ 体素化生成 downsampled_cloud，与 freeze_post / 子图 TEASER 契约一致。调用方须已持 mutex_。 */
    void        refreshDownsampledCloudFromMergedInAnchorFrame_(SubMap::Ptr& sm);
    void        mergeDuplicateTreeLandmarks_(SubMap::Ptr& sm);
    void        publishEvent(const SubMap::Ptr& sm, const std::string& event);
};

} // namespace automap_pro
