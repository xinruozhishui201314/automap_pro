#pragma once

#include "automap_pro/v3/event_bus.h"
#include "automap_pro/v3/map_registry.h"
#include "automap_pro/v3/module_base.h"
#include "automap_pro/core/logger.h"

#include <memory>
#include <vector>
#include <mutex>

namespace automap_pro::v3 {

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

    void init(rclcpp::Node::SharedPtr /*node*/) {
        map_registry_ = std::make_shared<MapRegistry>(event_bus_);
    }

    ~V3Context() {
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

private:
    EventBus::Ptr event_bus_;
    MapRegistry::Ptr map_registry_;
    
    std::mutex module_mutex_;
    std::vector<ModuleBase::Ptr> modules_;
};

} // namespace automap_pro::v3
