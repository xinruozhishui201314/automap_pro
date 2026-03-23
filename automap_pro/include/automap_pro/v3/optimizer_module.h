#pragma once

#include "automap_pro/v3/module_base.h"
#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/core/opt_task_types.h"

#include <deque>
#include <mutex>

namespace automap_pro::v3 {

/**
 * @brief 增量优化模块 (Incremental Optimizer Micro-Kernel Module)
 * 
 * 职责：
 * 1. 负责全图因子图的维护和优化 (iSAM2)
 * 2. 封装 IncrementalOptimizer，实现完全的微服务化
 * 3. 监听所有因子添加事件并按版本顺序应用
 */
class OptimizerModule : public ModuleBase {
public:
    OptimizerModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node);

    /**
     * @brief 直接提交任务（过渡期使用）
     */
    void submitTask(const OptTaskItem& task) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_queue_.push_back(task);
        cv_.notify_one();
    }

    /**
     * @brief 获取内部优化器引用（过渡期使用）
     */
    IncrementalOptimizer& getOptimizer() { return optimizer_; }

protected:
    void run() override;

private:
    void processTask(const OptTaskItem& task);
    void processGPSBatchKF(const OptTaskItem& task);
    void processKeyframeCreate(const OptTaskItem& task);
    Mat66d computeOdomInfoMatrixForKeyframes(const KeyFrame::Ptr& prev_kf, const KeyFrame::Ptr& curr_kf, const Pose3d& rel) const;

    rclcpp::Node::SharedPtr node_;
    IncrementalOptimizer optimizer_;
    
    std::deque<OptTaskItem> task_queue_;
    std::mutex task_mutex_;
};

} // namespace automap_pro::v3
