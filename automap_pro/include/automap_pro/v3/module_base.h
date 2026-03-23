#pragma once

#include "automap_pro/v3/event_bus.h"
#include "automap_pro/v3/map_registry.h"
#include "automap_pro/core/logger.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>

namespace automap_pro::v3 {

/**
 * @brief 微内核模块基类 (Micro-Kernel Module Base)
 * 
 * 职责：
 * 1. 自动注入 EventBus 和 MapRegistry
 * 2. 提供标准化的生命周期管理 (Start/Stop/Pause)
 * 3. 封装后台线程循环逻辑
 * 4. 强制实现核心任务处理函数
 */
class ModuleBase {
public:
    using Ptr = std::shared_ptr<ModuleBase>;

    ModuleBase(const std::string& name, EventBus::Ptr event_bus, MapRegistry::Ptr map_registry)
        : name_(name), event_bus_(event_bus), map_registry_(map_registry) {}

    virtual ~ModuleBase() { stop(); }

    /**
     * @brief 启动模块后台线程
     */
    virtual void start() {
        if (running_) return;
        ALOG_INFO("Pipeline", "[PIPELINE][V3] module START name={} tid={} lwp={}",
                  name_, automap_pro::logThreadId(), automap_pro::logLwp());
        running_ = true;
        updateHeartbeat();
        thread_ = std::thread(&ModuleBase::run, this);
    }

    /**
     * @brief 停止模块后台线程
     */
    virtual void stop() {
        if (!running_) return;
        ALOG_INFO("Pipeline", "[PIPELINE][V3] module STOP name={} (joining worker)", name_);
        running_ = false;
        cv_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
        ALOG_INFO("Pipeline", "[PIPELINE][V3] module STOPPED name={}", name_);
    }

    const std::string& getName() const { return name_; }

    /**
     * @brief 获取最后心跳时间 (L1 级健康检查)
     */
    double getLastHeartbeat() const { return last_heartbeat_s_.load(); }

    /**
     * @brief 更新心跳 (应在业务循环内调用)
     */
    void updateHeartbeat() {
        auto now = std::chrono::steady_clock::now();
        double ts = std::chrono::duration<double>(now.time_since_epoch()).count();
        last_heartbeat_s_.store(ts);
    }

    /**
     * @brief 检查模块是否空闲（队列为空）
     */
    virtual bool isIdle() const { return true; }

protected:
    /**
     * @brief 核心业务循环逻辑
     */
    virtual void run() = 0;

    /**
     * @brief 在循环内注册事件订阅的便捷方法
     */
    template <typename TEvent>
    void onEvent(std::function<void(const TEvent&)> handler) {
        event_bus_->subscribe<TEvent>(handler);
    }

    /**
     * @brief 异步注册事件订阅（不阻塞发布者，回调在独立线程执行）
     */
    template <typename TEvent>
    void onEventAsync(std::function<void(const TEvent&)> handler) {
        event_bus_->subscribeAsync<TEvent>(handler);
    }

    // --- 状态控制 ---
    std::string name_;
    EventBus::Ptr event_bus_;
    MapRegistry::Ptr map_registry_;
    
    std::atomic<bool> running_{false};
    std::atomic<double> last_heartbeat_s_{0.0};
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace automap_pro::v3
