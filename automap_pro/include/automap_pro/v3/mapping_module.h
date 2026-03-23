#pragma once

#include "automap_pro/v3/module_base.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/backend/hba_optimizer.h"
#include <deque>
#include <mutex>

namespace automap_pro::v3 {

/**
 * @brief 建图调度模块 (MappingModule)
 * 
 * 职责：
 * 1. 订阅 SyncedFrameEvent，执行关键帧创建逻辑
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

protected:
    void run() override;

private:
    void processFrame(const SyncedFrameEvent& event);
    void onSubmapFrozen(const SubMap::Ptr& submap);

    // 🏛️ 生产级命令处理 (Command Handlers)
    void handleSaveMap(const SaveMapRequestEvent& ev);
    void handleGlobalMapBuild(const GlobalMapBuildRequestEvent& ev);

    // GPS 对齐状态处理
    void updateGPSAlignment(const GPSAlignedEvent& ev);

    // 🏛️ V3: 位姿跳变修正 logic
    void onPoseOptimized(const OptimizationResultEvent& ev);

    // 辅助函数
    Mat66d computeOdomInfoMatrix(const SubMap::Ptr& prev, const SubMap::Ptr& curr, const Pose3d& rel) const;

    // 🏛️ 生产级权限隔离：MappingModule 拥有创建权 (Ownership of Managers)
    KeyFrameManager kf_manager_;
    SubMapManager sm_manager_;

    // 成员变量 (SSoT: Managers moved to MapRegistry)
    HBAOptimizer hba_optimizer_;

    std::deque<SyncedFrameEvent> frame_queue_;
    std::deque<OptimizationResultEvent> pose_opt_queue_;
    std::deque<GPSAlignedEvent> gps_event_queue_;
    
    // 🏛️ 生产级命令队列
    struct Command {
        enum class Type { SAVE_MAP, BUILD_GLOBAL_MAP } type;
        std::string output_dir;
        float voxel_size;
        bool async;
    };
    std::deque<Command> command_queue_;

    std::mutex queue_mutex_;  // 保护所有队列
    
    std::atomic<bool> gps_aligned_{false};
    Eigen::Matrix3d gps_transform_R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d gps_transform_t_ = Eigen::Vector3d::Zero();
    mutable std::mutex gps_transform_mutex_;

    uint64_t current_session_id_ = 0;
    rclcpp::Node::SharedPtr node_;
    
    // 🏛️ 生产级版本控制：本地已应用的最新的地图版本
    std::atomic<uint64_t> processed_map_version_{0};

    // 统计
    std::atomic<int> frozen_submap_count_{0};
    std::atomic<int> processed_frame_count_{0}; // 🏛️ [P0 优化] 记录处理帧数以执行按帧频率的地图构建触发

    /// 上一次已冻结子图（仅用于子图间 ODOM 因子）。禁止在冻结回调里调用 getFrozenSubmaps()（会与 merge 持锁竞态）。
    SubMap::Ptr prev_frozen_for_odom_;
    
    // 回环缓存（临时，用于 HBA）
    std::vector<LoopConstraint::Ptr> loop_constraints_;
    std::mutex loop_constraints_mutex_;
};

} // namespace automap_pro::v3
