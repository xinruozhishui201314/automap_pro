#include "automap_pro/backend/unified_task_queue.h"

#include "automap_pro/core/logger.h"

#include <chrono>

namespace automap_pro {

UnifiedTaskQueue::UnifiedTaskQueue(size_t max_size)
    : max_size_(max_size) {
}

UnifiedTaskQueue::~UnifiedTaskQueue() {
    stop();
}

void UnifiedTaskQueue::start(std::function<void(const OptTaskItem&)> handler) {
    if (running_.load()) {
        return;
    }
    
    handler_ = std::move(handler);
    running_.store(true);
    
    worker_thread_ = std::thread([this]() {
#ifdef __linux__
        pthread_setname_np(pthread_self(), "unified_taskq");
#endif
        workerLoop();
    });
}

void UnifiedTaskQueue::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    cv_.notify_all();
    
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

bool UnifiedTaskQueue::enqueue(const OptTaskItem& task, TaskPriority priority) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (queue_.size() >= max_size_) {
        tasks_dropped_.fetch_add(1);
        return false;
    }
    
    UnifiedTaskItem item;
    item.priority = priority;
    item.task = task;
    item.enqueue_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    queue_.push(item);
    tasks_enqueued_.fetch_add(1);
    cv_.notify_one();
    
    return true;
}

bool UnifiedTaskQueue::enqueueKeyFrameCreate(const OptTaskItem& task) {
    return enqueue(task, TaskPriority::HIGH);
}

bool UnifiedTaskQueue::enqueueLoopFactor(const OptTaskItem& task) {
    return enqueue(task, TaskPriority::HIGH);
}

bool UnifiedTaskQueue::enqueueReset(const OptTaskItem& task) {
    return enqueue(task, TaskPriority::CRITICAL);
}

bool UnifiedTaskQueue::enqueueGPSAlignComplete(const OptTaskItem& task) {
    return enqueue(task, TaskPriority::CRITICAL);
}

void UnifiedTaskQueue::waitForCompletion() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] {
        return queue_.empty() || !running_.load();
    });
}

size_t UnifiedTaskQueue::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

bool UnifiedTaskQueue::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

bool UnifiedTaskQueue::isRunning() const {
    return running_.load();
}

void UnifiedTaskQueue::workerLoop() {
    ALOG_INFO("UnifiedTaskQueue", "Worker thread started");
    
    while (running_.load()) {
        UnifiedTaskItem item;
        bool got_task = false;
        
        {
            std::unique_lock<std::mutex> lock(mutex_);
            
            // 等待直到有任务或需要停止
            cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return !running_.load() || !queue_.empty();
            });
            
            if (!running_.load()) {
                break;
            }
            
            if (!queue_.empty()) {
                item = std::move(const_cast<UnifiedTaskItem&>(queue_.top()));
                queue_.pop();
                got_task = true;
            }
        }
        
        if (got_task && handler_) {
            try {
                handler_(item.task);
                tasks_processed_.fetch_add(1);
            } catch (const std::exception& e) {
                ALOG_ERROR("UnifiedTaskQueue", "Task processing error: %s", e.what());
            }
        }
    }
    
    ALOG_INFO("UnifiedTaskQueue", "Worker thread stopped, processed=%lu dropped=%lu",
              tasks_processed_.load(), tasks_dropped_.load());
}

TaskPriority getPriorityForTaskType(OptTaskItem::Type type) {
    switch (type) {
        case OptTaskItem::Type::RESET:
        case OptTaskItem::Type::GPS_ALIGN_COMPLETE:
            return TaskPriority::CRITICAL;
            
        case OptTaskItem::Type::LOOP_FACTOR:
        case OptTaskItem::Type::KEYFRAME_CREATE:
            return TaskPriority::HIGH;
            
        case OptTaskItem::Type::GPS_FACTOR:
        case OptTaskItem::Type::SUBMAP_NODE:
        case OptTaskItem::Type::ODOM_FACTOR:
            return TaskPriority::NORMAL;
            
        case OptTaskItem::Type::FORCE_UPDATE:
        case OptTaskItem::Type::REBUILD:
            return TaskPriority::LOW;
            
        default:
            return TaskPriority::NORMAL;
    }
}

}  // namespace automap_pro
