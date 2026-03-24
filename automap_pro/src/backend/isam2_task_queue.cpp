#include "automap_pro/backend/isam2_task_queue.h"
#include "automap_pro/backend/isam2_factor_types.h"

#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"

#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

ISAM2TaskQueue::ISAM2TaskQueue() : running_(true), max_queue_size_(ConfigManager::instance().isLoaded() ? static_cast<size_t>(ConfigManager::instance().maxOptimizationQueueSize()) : 64u) {
    worker_thread_ = std::thread(&ISAM2TaskQueue::workerLoop, this);
}

ISAM2TaskQueue::~ISAM2TaskQueue() {
    stop();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void ISAM2TaskQueue::enqueue(const OptimTask& task) {
    std::lock_guard<std::mutex> lk(mutex_);

    if (queue_.size() >= max_queue_size_) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2TaskQueue] queue full (%zu), drop task", max_queue_size_);
        MetricsRegistry::instance().incrementCounter("isam2_task_dropped", 1.0);
        return;
    }

    queue_.push(task);
    MetricsRegistry::instance().setGauge("isam2_queue_depth", static_cast<double>(queue_.size()));
    cv_.notify_one();
}

void ISAM2TaskQueue::enqueueMany(const std::vector<OptimTask>& tasks) {
    std::lock_guard<std::mutex> lk(mutex_);

    size_t dropped = 0;
    for (const auto& t : tasks) {
        if (queue_.size() >= max_queue_size_) {
            dropped++;
            continue;
        }
        queue_.push(t);
    }

    if (dropped > 0) {
        MetricsRegistry::instance().incrementCounter("isam2_task_dropped", static_cast<double>(dropped));
    }

    MetricsRegistry::instance().setGauge("isam2_queue_depth", static_cast<double>(queue_.size()));
    if (!tasks.empty()) cv_.notify_one();
}

void ISAM2TaskQueue::waitForCompletion() {
    constexpr int kMaxWaitMs = 5000;
    constexpr int kChunkMs = 10;
    int waited_ms = 0;

    while (waited_ms < kMaxWaitMs) {
        // 检查队列是否为空且工作线程是否已结束
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (queue_.empty() && !running_.load(std::memory_order_acquire)) {
                return;  // 队列为空且工作线程已停止
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kChunkMs));
        waited_ms += kChunkMs;
    }

    // 超时后再次检查，如果仍有问题则记录警告
    std::lock_guard<std::mutex> lk(mutex_);
    if (!queue_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[ISAM2TaskQueue] waitForCompletion timeout: %zu tasks still in queue", queue_.size());
    }
}

size_t ISAM2TaskQueue::size() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return queue_.size();
}

bool ISAM2TaskQueue::isRunning() const {
    return running_.load(std::memory_order_acquire);
}

void ISAM2TaskQueue::stop() {
    running_.store(false, std::memory_order_release);
    cv_.notify_all();
}

void ISAM2TaskQueue::setProcessCallback(std::function<void(const OptimTask&)> callback) {
    process_callback_ = std::move(callback);
}

void ISAM2TaskQueue::workerLoop() {
    // 批量处理参数
    static constexpr size_t kBatchSizeThreshold = 5;
    static constexpr int kBatchTimeoutMs = 2000;

    while (true) {
        std::vector<OptimTask> batch_tasks;

        {
            std::unique_lock<std::mutex> lk(mutex_);

            if (!running_ && queue_.empty()) {
                break;
            }

            // 如果队列为空，等待新任务
            if (queue_.empty()) {
                cv_.wait_for(lk, std::chrono::milliseconds(kBatchTimeoutMs), [this] {
                    return !running_ || !queue_.empty();
                });
            }

            if (!running_ && queue_.empty()) {
                break;
            }

            // 收集任务
            while (!queue_.empty() && batch_tasks.size() < kBatchSizeThreshold) {
                batch_tasks.push_back(queue_.front());
                queue_.pop();
            }
        }

        if (batch_tasks.empty()) {
            continue;
        }

        // 处理任务
        for (const auto& task : batch_tasks) {
            if (process_callback_) {
                try {
                    process_callback_(task);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[ISAM2TaskQueue] task processing failed: %s", e.what());
                }
            }
        }
    }
}

}  // namespace automap_pro
