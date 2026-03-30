#pragma once
/**
 * @file v3/mapping_module.h
 * @brief V3 微内核：模块编排、事件总线、Registry、前端/语义/优化流水线。
 */


#include "automap_pro/v3/module_base.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/backend/hba_optimizer.h"
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace automap_pro::v3 {

/**
 * @brief 建图调度模块 (MappingModule)
 * 
 * 职责：
 * 1. 订阅 FilteredFrameEventRequiredDs，执行关键帧创建逻辑
 * 2. 管理 KeyFrameManager 和 SubMapManager
 * 3. 处理子图冻结事件 (onSubmapFrozen)，生成子图因子任务
 * 4. 管理 HBA 优化触发逻辑
 * 5. 维护当前的 GPS 对齐状态，并应用到新关键帧
 */
class MappingModule : public ModuleBase {
public:
    using Ptr = std::shared_ptr<MappingModule>;

    MappingModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node);

    void start() override;
    void stop() override;
    bool isIdle() const override;
    std::vector<std::pair<std::string, size_t>> queueDepths() const override;
    std::string idleDetail() const override;

    /**
     * @brief 建图收尾：将当前 ACTIVE 子图强制冻结并入图（触发与正常 freeze 相同的回调链）。
     *        供 finish_mapping 在最终 HBA/存档前调用，避免最后一段轨迹未进入 registry/因子图。
     */
    void forceFreezeActiveSubmapForFinish();

    /**
     * @brief 当 backend.hba.enabled 且 backend.hba.trigger_on_finish 为真时，对当前 registry 中
     *        全部 FROZEN/OPTIMIZED 子图 + 已缓存回环执行一次全图 HBA，并阻塞至 HBA 完成（同 triggerAsync wait=true）。
     * @return 是否实际触发了 HBA（配置关闭或无子图时返回 false）。
     */
    bool runFinalFullHbaBlockingIfConfigured();

protected:
    void run() override;

private:
    void processFrame(const FilteredFrameEventRequiredDs& event);
    void onSubmapFrozen(const SubMap::Ptr& submap);

    // 🏛️ 生产级命令处理 (Command Handlers)
    void handleSaveMap(const SaveMapRequestEvent& ev);
    /** 收尾保存：在 optimized/ 下写出全图 HBA 后的关键帧位姿与 GPS 约束点（map 系，GPS 仅位置时 rpy=0） */
    void savePoseExportsAfterGlobalMap(const std::string& optimized_dir);
    void handleGlobalMapBuild(const GlobalMapBuildRequestEvent& ev);
    /** 发布 GlobalMapBuildResultEvent 并紧跟 MapUpdateEvent(GLOBAL_MAP_REBUILT)，保证 RViz 点云与轨迹刷新顺序一致 */
    void publishGlobalMapResultAndTriggerVizSync(const CloudXYZIPtr& global);

    // GPS 对齐状态处理
    void updateGPSAlignment(const GPSAlignedEvent& ev);

    // 🏛️ V3: 位姿跳变修正 logic
    void onPoseOptimized(const OptimizationResultEvent& ev);
    void onPoseDelta(const OptimizationDeltaEvent& ev);
    
    // 🏛️ [架构演进] 处理异步语义地标
    void onSemanticLandmarks(const SemanticLandmarkEvent& ev);
    
    /**
     * @brief 🏛️ [架构加固] 统一位姿应用网关
     * 处理坐标系补偿 (ODOM -> MAP) 并分发到各个管理器
     */
    /**
     * @param skip_optimize_driven_global_map_request 为 true 时不触发「优化驱动」的全局图构建请求（由调用方在 HBA rebuild 后同步构图）。
     */
    bool applyOptimizedPoses(const std::unordered_map<int, Pose3d>& sm_poses,
                             const std::unordered_map<uint64_t, Pose3d>& kf_poses,
                             PoseFrame frame, uint64_t version,
                             uint64_t alignment_epoch = 0,
                             bool skip_optimize_driven_global_map_request = false);
    bool shouldAcceptOptimizationEvent(const OptimizationResultEvent& ev);
    bool shouldAcceptOptimizationDelta(const OptimizationDeltaEvent& ev) const;

    // 辅助函数
    Mat66d computeOdomInfoMatrix(const SubMap::Ptr& prev, const SubMap::Ptr& curr, const Pose3d& rel) const;
    void publishGraphTaskEvent(GraphTaskEvent& ev, double source_ts);

    // 🏛️ 生产级权限隔离：MappingModule 拥有创建权 (Ownership of Managers)
    KeyFrameManager kf_manager_;
    SubMapManager sm_manager_;

    // 成员变量 (SSoT: Managers moved to MapRegistry)
    HBAOptimizer hba_optimizer_;

    std::deque<FilteredFrameEventRequiredDs> frame_queue_;
    std::deque<OptimizationResultEvent> pose_opt_queue_;
    std::deque<OptimizationDeltaEvent> pose_delta_queue_;
    std::deque<GPSAlignedEvent> gps_event_queue_;
    std::deque<SemanticLandmarkEvent> semantic_landmark_queue_;
    
    // 🏛️ 生产级命令队列
    struct Command {
        enum class Type { SAVE_MAP, BUILD_GLOBAL_MAP } type;
        std::string output_dir;
        float voxel_size;
        bool async;
        std::shared_ptr<std::promise<void>> save_completion;
    };
    std::deque<Command> command_queue_;

    mutable std::mutex queue_mutex_;  // 保护所有队列
    
    std::atomic<bool> gps_aligned_{false};
    Eigen::Matrix3d gps_transform_R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d gps_transform_t_ = Eigen::Vector3d::Zero();
    mutable std::mutex gps_transform_mutex_;

    uint64_t current_session_id_ = 0;
    rclcpp::Node::SharedPtr node_;
    
    // 🏛️ 生产级版本控制：本地已应用的最新的地图版本
    std::atomic<uint64_t> processed_map_version_{0};
    std::atomic<uint64_t> processed_alignment_epoch_{0};
    std::atomic<uint64_t> last_gps_event_seq_{0};
    double last_barrier_wait_start_time_ = -1.0; // 🏛️ [P1 稳定性] 屏障超时计时器
    uint64_t last_applied_version_{0};
    uint64_t last_applied_batch_hash_{0};
    uint64_t last_applied_event_id_{0};

    // 统计
    std::atomic<int> frozen_submap_count_{0};
    std::atomic<int> processed_frame_count_{0}; // 🏛️ [P0 优化] 记录处理帧数以执行按帧频率的地图构建触发
    std::atomic<int> optimized_apply_count_{0}; // 优化结果应用计数，用于节流优化驱动的全局地图构建
    std::atomic<uint64_t> semantic_in_total_{0};
    std::atomic<uint64_t> semantic_defer_total_{0};
    std::atomic<uint64_t> semantic_dispatch_total_{0};
    std::atomic<uint64_t> semantic_assoc_tree_input_total_{0};
    std::atomic<uint64_t> semantic_assoc_tree_emitted_total_{0};
    std::atomic<uint64_t> semantic_assoc_tree_suppressed_total_{0};
    std::atomic<uint64_t> semantic_assoc_plane_input_total_{0};
    std::atomic<uint64_t> semantic_assoc_plane_emitted_total_{0};
    std::atomic<uint64_t> semantic_assoc_plane_suppressed_total_{0};
    std::atomic<uint64_t> graph_event_seq_{0};
    std::atomic<uint64_t> route_advice_recv_total_{0};
    std::atomic<bool> route_takeover_enabled_{false};
    std::atomic<uint64_t> delta_apply_total_{0};
    std::atomic<uint64_t> delta_reject_total_{0};
    std::atomic<uint64_t> legacy_opt_observe_total_{0};
    std::atomic<uint64_t> legacy_opt_apply_total_{0};
    std::atomic<uint64_t> legacy_opt_skip_takeover_total_{0};
    std::atomic<uint64_t> consistency_mismatch_total_{0};
    mutable std::mutex delta_consistency_mutex_;
    std::unordered_map<uint64_t, uint64_t> recent_delta_hash_by_version_;

    /// 上一次已冻结子图（仅用于子图间 ODOM 因子）。禁止在冻结回调里调用 getFrozenSubmaps()（会与 merge 持锁竞态）。
    SubMap::Ptr prev_frozen_for_odom_;
    mutable std::mutex prev_frozen_for_odom_mutex_;
    
    // 回环缓存（临时，用于 HBA）
    std::vector<LoopConstraint::Ptr> loop_constraints_;
    std::mutex loop_constraints_mutex_;

    // 🏛️ [P1 稳定性修复] 异步语义地标待处理队列（解决 KeyFrame 尚未分配 submap_id 的竞态）
    size_t max_frame_queue_size_ = 1024;
    size_t max_semantic_queue_size_ = 4096;
    size_t max_pending_semantic_events_ = 4096;
    double semantic_timestamp_match_tolerance_s_ = 0.15;
    std::deque<SemanticLandmarkEvent> pending_semantic_landmarks_;
    mutable std::mutex pending_semantic_mutex_;
};

} // namespace automap_pro::v3
