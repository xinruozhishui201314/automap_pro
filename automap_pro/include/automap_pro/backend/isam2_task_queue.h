#pragma once

/**
 * iSAM2 异步优化任务队列模块
 *
 * 提供后台优化线程和任务队列管理，用于异步执行优化任务。
 */

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/isam2_factor_types.h"

#include <gtsam/geometry/Pose3.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace automap_pro {

/**
 * 异步优化任务队列管理器
 *
 * 提供后台工作线程和任务队列，用于异步执行优化任务。
 */
class ISAM2TaskQueue {
public:
    /** 最大队列大小 */
    static constexpr size_t kMaxQueueSize = 64;

    ISAM2TaskQueue();
    ~ISAM2TaskQueue();

    /**
     * 提交优化任务到队列（立即返回，异步执行）
     *
     * @param task 优化任务
     */
    void enqueue(const OptimTask& task);

    /**
     * 批量提交优化任务
     *
     * @param tasks 优化任务列表
     */
    void enqueueMany(const std::vector<OptimTask>& tasks);

    /**
     * 等待所有已入队任务完成
     */
    void waitForCompletion();

    /**
     * 获取当前队列深度
     *
     * @return 当前待处理的任务数量
     */
    size_t size() const;

    /**
     * 检查队列是否正在运行
     *
     * @return 是否正在运行
     */
    bool isRunning() const;

    /**
     * 停止队列处理
     */
    void stop();

    /**
     * 设置任务处理回调
     *
     * @param callback 处理任务的回调函数
     */
    void setProcessCallback(std::function<void(const OptimTask&)> callback);

private:
    /**
     * 工作线程主循环
     */
    void workerLoop();

    /** 构造时缓存，避免 enqueue 中访问 ConfigManager 单例（shutdown 时 worker 可能仍入队） */
    size_t max_queue_size_ = 64;

    std::queue<OptimTask> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_;
    std::thread worker_thread_;
    std::function<void(const OptimTask&)> process_callback_;
};

}  // namespace automap_pro
