#pragma once

#include "automap_pro/v3/event_bus.h"
#include "automap_pro/v3/map_registry.h"
#include "automap_pro/v3/module_base.h"
#include "automap_pro/core/logger.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>
#include <mutex>
#include <chrono>

namespace automap_pro::v3 {

struct ModuleIdleStatus {
    std::string name;
    bool        is_idle{true};
    bool        is_quiescing{false};
    double      heartbeat_age_s{0.0};
    std::vector<std::pair<std::string, size_t>> queue_depths;
    std::string idle_detail;
};

/**
 * @brief V3 架构上下文 (Architecture Context)
 *
 * 职责：
 * 1. 管理整个系统的核心组件 (EventBus, MapRegistry)
 * 2. 挂载和生命周期管理所有模块
 */
class V3Context {
public:
    using Ptr = std::shared_ptr<V3Context>;

    V3Context() {
        event_bus_ = std::make_shared<EventBus>();
    }

    /**
     * @param node ROS node for timers/logging; may be nullptr during parent ctor
     *        (shared_from_this() unsafe there). Call attachNode() once a node exists.
     */
    void init(rclcpp::Node::SharedPtr node) {
        node_ = node;
        map_registry_ = std::make_shared<MapRegistry>(event_bus_);
        startHealthMonitorIfNodeReady();
    }

    /** Bind node and start periodic health checks (call when shared_from_this() is valid). */
    void attachNode(rclcpp::Node::SharedPtr node) {
        node_ = std::move(node);
        startHealthMonitorIfNodeReady();
    }

    ~V3Context() {
        if (health_timer_) health_timer_->cancel();
        stopAll();
    }

    EventBus::Ptr eventBus() { return event_bus_; }
    MapRegistry::Ptr mapRegistry() { return map_registry_; }

    /**
     * @brief 注册并启动一个新模块
     */
    void registerModule(ModuleBase::Ptr module) {
        std::lock_guard<std::mutex> lock(module_mutex_);
        const size_t idx = modules_.size();
        ALOG_INFO("Pipeline", "[PIPELINE][V3] V3Context::registerModule idx={} name={}", idx, module->getName());
        modules_.push_back(module);
        module->start();
        ALOG_INFO("Pipeline", "[PIPELINE][V3] V3Context::registerModule START OK idx={} name={}", idx, module->getName());
    }

    /**
     * @brief 停止并销毁所有模块
     */
    void stopAll() {
        std::lock_guard<std::mutex> lock(module_mutex_);
        ALOG_INFO("Pipeline", "[PIPELINE][V3] V3Context::stopAll begin count={}", modules_.size());
        for (size_t i = 0; i < modules_.size(); ++i) {
            ALOG_INFO("Pipeline", "[PIPELINE][V3] V3Context::stopAll idx={} name={}", i, modules_[i]->getName());
            modules_[i]->stop();
        }
        modules_.clear();
        ALOG_INFO("Pipeline", "[PIPELINE][V3] V3Context::stopAll done");
    }

    /**
     * @brief 检查所有模块是否空闲
     */
    bool isAllIdle() const {
        std::lock_guard<std::mutex> lock(module_mutex_);
        for (const auto& mod : modules_) {
            if (!mod->isIdle()) return false;
        }
        return true;
    }

    /**
     * @brief 导出模块 idle/quiesce/heartbeat 快照（用于 finish_mapping 排空阶段诊断）
     */
    std::vector<ModuleIdleStatus> getIdleStatusSnapshot() const {
        std::vector<ModuleIdleStatus> out;
        std::lock_guard<std::mutex> lock(module_mutex_);
        out.reserve(modules_.size());
        auto now = std::chrono::steady_clock::now();
        const double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
        for (const auto& mod : modules_) {
            ModuleIdleStatus s;
            s.name = mod->getName();
            s.is_idle = mod->isIdle();
            s.is_quiescing = mod->isQuiescing();
            const double hb = mod->getLastHeartbeat();
            s.heartbeat_age_s = (hb > 0.0) ? (now_s - hb) : 1e9;
            s.queue_depths = mod->queueDepths();
            s.idle_detail = mod->idleDetail();
            out.push_back(std::move(s));
        }
        return out;
    }

    /**
     * @brief 系统健康巡检 (Periodic Health Check)
     */
    void checkHealth() {
        if (!node_) return;

        SystemStatusEvent ev;
        auto now = std::chrono::steady_clock::now();
        ev.timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
        ev.overall_ok = true;

        {
            std::lock_guard<std::mutex> lock(module_mutex_);
            for (const auto& mod : modules_) {
                ModuleStatus status;
                status.name = mod->getName();
                status.last_heartbeat = mod->getLastHeartbeat();
                status.age_s = ev.timestamp - status.last_heartbeat;

                // 🏛️ 健康阈值：如果模块超过 10s 没有心跳，标记为异常 (Zombie detect)
                status.ok = (status.age_s < 10.0);
                if (!status.ok) ev.overall_ok = false;
                ev.modules.push_back(status);

                if (!status.ok) {
                    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                        "[CRITICAL_V3] Module '%s' seems to be HUNG! (age=%.1fs) (grep V3 ZOMBIE)",
                        status.name.c_str(), status.age_s);
                }
            }
        }

        event_bus_->publish(ev);
    }

private:
    void startHealthMonitorIfNodeReady() {
        if (!node_) return;
        if (health_timer_) {
            health_timer_->cancel();
            health_timer_.reset();
        }
        health_timer_ = node_->create_wall_timer(
            std::chrono::seconds(2), [this]() { this->checkHealth(); });
        RCLCPP_INFO(node_->get_logger(), "[PIPELINE][V3] Health monitor initialized (2s interval)");
    }

    EventBus::Ptr event_bus_;
    MapRegistry::Ptr map_registry_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr health_timer_;

    mutable std::mutex module_mutex_;
    std::vector<ModuleBase::Ptr> modules_;
};

} // namespace automap_pro::v3
