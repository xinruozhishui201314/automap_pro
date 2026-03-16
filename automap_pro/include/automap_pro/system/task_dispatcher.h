#pragma once

/**
 * 任务调度器模块
 *
 * 封装后端优化任务的分发和处理逻辑，提供统一的接口。
 * 职责：
 * - 任务提交接口
 * - 任务优先级管理
 * - 任务处理（委托给 IncrementalOptimizer）
 */

#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/core/opt_task_types.h"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <deque>
#include <condition_variable>

namespace automap_pro {

/**
 * 任务调度器
 *
 * 封装任务提交和处理逻辑，提供更清晰的接口。
 * 注意：当前版本保留原有的 opt_task_queue_ 队列，新调度器作为包装器。
 */
class TaskDispatcher {
public:
    /**
     * 构造函数
     * @param optimizer iSAM2 优化器引用
     * @param system_node ROS2 节点（用于日志）
     */
    TaskDispatcher(IncrementalOptimizer& optimizer, rclcpp::Node* node = nullptr);

    /**
     * 构造函数（使用外部队列）
     * @param opt_task_queue 外部任务队列指针
     * @param opt_task_mutex 外部互斥锁指针
     * @param opt_task_cv 外部条件变量指针
     * @param max_queue_size 队列最大大小
     */
    TaskDispatcher(std::deque<OptTaskItem>* opt_task_queue,
                   std::mutex* opt_task_mutex,
                   std::condition_variable* opt_task_cv,
                   size_t max_queue_size = 64);

    ~TaskDispatcher();
    
    // 禁止拷贝
    TaskDispatcher(const TaskDispatcher&) = delete;
    TaskDispatcher& operator=(const TaskDispatcher&) = delete;
    
    /**
     * 启动调度器
     */
    void start();
    
    /**
     * 停止调度器
     */
    void stop();
    
    /**
     * 提交回环因子任务
     */
    bool submitLoopFactor(const LoopConstraint::Ptr& lc);
    
    /**
     * 提交 GPS 因子任务
     */
    bool submitGPSFactor(int sm_id, const Eigen::Vector3d& pos, 
                        const Eigen::Matrix3d& cov);
    
    /**
     * 提交子图节点任务
     */
    bool submitSubmapNode(int sm_id, const Pose3d& pose, bool fixed = false);
    
    /**
     * 提交里程计因子任务
     */
    bool submitOdomFactor(int from, int to, const Pose3d& rel, 
                         const Mat66d& info);
    
    /**
     * 提交重建任务（GPS 对齐后）
     */
    bool submitRebuild(const std::vector<SubmapData>& submaps,
                      const std::vector<OdomFactorItem>& odom_factors,
                      const std::vector<LoopFactorItem>& loop_factors);
    
    /**
     * 提交 GPS 对齐完成任务
     */
    bool submitGPSAlignComplete(const std::vector<SubmapData>& submaps,
                              const std::vector<OdomFactorItem>& odom_factors,
                              const std::vector<LoopFactorItem>& loop_factors,
                              const Eigen::Matrix3d& R_enu_to_map,
                              const Eigen::Vector3d& t_enu_to_map);
    
    /**
     * 提交重置任务
     */
    bool submitReset();
    
    /**
     * 提交强制更新任务
     */
    bool submitForceUpdate();
    
    /**
     * 提交关键帧创建任务
     */
    bool submitKeyFrameCreate(const KeyFrame::Ptr& kf,
                             bool has_prev_kf,
                             int prev_kf_id,
                             bool gps_aligned,
                             const Eigen::Matrix3d& gps_transform_R,
                             const Eigen::Vector3d& gps_transform_t);
    
    /**
     * 等待所有任务完成
     */
    void waitForCompletion();
    
    /**
     * 获取当前队列大小
     */
    size_t queueSize() const;
    
    /**
     * 是否正在运行
     */
    bool isRunning() const;
    
    /**
     * 注册优化结果回调
     */
    using PoseUpdateCallback = std::function<void(const std::unordered_map<int, Pose3d>&)>;
    void registerPoseCallback(PoseUpdateCallback cb);
    
    /**
     * 设置外部回调处理函数（用于在外部线程执行任务）
     * @param handler 任务处理函数
     */
    void setExternalHandler(std::function<void(const OptTaskItem&)> handler);
    
    /**
     * 获取待处理的 OptTaskItem（供外部线程调用）
     */
    bool tryDequeue(OptTaskItem& task);
    
private:
    IncrementalOptimizer* optimizer_;
    rclcpp::Node* node_;
    
    // 任务队列（原系统）
    std::deque<OptTaskItem>* opt_task_queue_;
    std::mutex* opt_task_mutex_;
    std::condition_variable* opt_task_cv_;
    size_t max_queue_size_;
    
    // 状态
    std::atomic<bool> running_{false};
    
    // 回调
    std::vector<PoseUpdateCallback> pose_cbs_;
    mutable std::mutex cbs_mutex_;
    
    // 外部处理器
    std::function<void(const OptTaskItem&)> external_handler_;
    
    // 内部处理函数
    void processTask(const OptTaskItem& task);
    
    // 辅助函数
    void logWarn(const std::string& msg);
    void logInfo(const std::string& msg);
};

}  // namespace automap_pro
