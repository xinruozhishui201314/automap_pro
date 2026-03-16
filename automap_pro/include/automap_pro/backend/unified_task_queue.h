#pragma once

/**
 * 统一任务队列模块
 *
 * 提供优先级任务队列，支持：
 * - 优先级调度（CRITICAL, HIGH, NORMAL, LOW）
 * - 有界队列，防止内存无限增长
 * - 线程安全操作
 */

#include "automap_pro/core/opt_task_types.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <functional>

namespace automap_pro {

// 前向声明 - OptTaskItem 定义在 automap_system.h 的 AutoMapSystem 类中
struct OptTaskItem;

/**
 * 任务优先级枚举
 */
enum class TaskPriority : uint8_t {
    CRITICAL = 0,   // RESET, GPS_ALIGN_COMPLETE - 必须立即处理
    HIGH = 1,        // LOOP_FACTOR, KEYFRAME_CREATE - 高优先级
    NORMAL = 2,      // GPS_FACTOR, ODOM_FACTOR, SUBMAP_NODE - 普通任务
    LOW = 3          // FORCE_UPDATE, REBUILD - 可延迟
};

/**
 * 统一任务项（包装 OptTaskItem 添加优先级）
 */
struct UnifiedTaskItem {
    TaskPriority priority = TaskPriority::NORMAL;
    int64_t enqueue_time_ms = 0;
    OptTaskItem task;
    
    // 用于优先级队列比较
    bool operator<(const UnifiedTaskItem& other) const {
        return priority > other.priority;  // 越小优先级越高
    }
};

/**
 * 统一任务队列管理器
 *
 * 提供：
 * - 优先级任务队列
 * - 工作线程封装
 * - 任务处理回调
 */
class UnifiedTaskQueue {
public:
    /**
     * 构造函数
     * @param max_size 队列最大容量
     */
    explicit UnifiedTaskQueue(size_t max_size = 128);
    
    ~UnifiedTaskQueue();
    
    // 禁止拷贝和移动
    UnifiedTaskQueue(const UnifiedTaskQueue&) = delete;
    UnifiedTaskQueue& operator=(const UnifiedTaskQueue&) = delete;
    UnifiedTaskQueue(UnifiedTaskQueue&&) = delete;
    UnifiedTaskQueue& operator=(UnifiedTaskQueue&&) = delete;
    
    /**
     * 启动工作线程
     * @param handler 任务处理回调函数
     */
    void start(std::function<void(const OptTaskItem&)> handler);
    
    /**
     * 停止工作线程（等待完成）
     */
    void stop();
    
    /**
     * 提交任务（带优先级）
     * @param task 任务数据
     * @param priority 任务优先级
     * @return 是否成功入队
     */
    bool enqueue(const OptTaskItem& task, TaskPriority priority = TaskPriority::NORMAL);
    
    /**
     * 提交关键帧创建任务（高优先级）
     */
    bool enqueueKeyFrameCreate(const OptTaskItem& task);
    
    /**
     * 提交回环因子任务（高优先级）
     */
    bool enqueueLoopFactor(const OptTaskItem& task);
    
    /**
     * 提交 RESET 任务（最高优先级）
     */
    bool enqueueReset(const OptTaskItem& task);
    
    /**
     * 提交 GPS 对齐完成任务（最高优先级）
     */
    bool enqueueGPSAlignComplete(const OptTaskItem& task);
    
    /**
     * 等待所有任务完成
     */
    void waitForCompletion();
    
    /**
     * 获取当前队列深度
     */
    size_t size() const;
    
    /**
     * 检查队列是否为空
     */
    bool empty() const;
    
    /**
     * 检查是否正在运行
     */
    bool isRunning() const;
    
private:
    /**
     * 工作线程主循环
     */
    void workerLoop();
    
    // 优先级队列
    std::priority_queue<UnifiedTaskItem> queue_;
    
    // 同步原语
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_{false};
    
    // 工作线程
    std::thread worker_thread_;
    
    // 任务处理回调
    std::function<void(const OptTaskItem&)> handler_;
    
    // 配置
    const size_t max_size_;
    
    // 状态统计
    std::atomic<uint64_t> tasks_enqueued_{0};
    std::atomic<uint64_t> tasks_processed_{0};
    std::atomic<uint64_t> tasks_dropped_{0};
};

/**
 * 任务优先级辅助函数（在 cpp 中实现）
 */
// TaskPriority getPriorityForTaskType(OptTaskItem::Type type);  // 已移除，避免重复定义问题

}  // namespace automap_pro
