#pragma once

#include "automap_pro/v3/module_base.h"
#include "automap_pro/v3/semantic_processor.h"
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

    rclcpp::Node::SharedPtr node_;
    SemanticProcessor::Ptr semantic_processor_;

    // 任务队列
    mutable std::mutex queue_mutex_;
    std::deque<SyncedFrameEvent> task_queue_;
    static constexpr size_t kMaxQueueSize = 5; // 限制队列长度，防止积压导致高延迟
};

} // namespace automap_pro::v3
