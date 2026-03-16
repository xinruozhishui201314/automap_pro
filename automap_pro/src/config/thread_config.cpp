#include "automap_pro/config/thread_config.h"

#include "automap_pro/core/config_manager.h"

namespace automap_pro {

ThreadConfig* ThreadConfig::instance_ = nullptr;

const ThreadConfig& ThreadConfig::instance() {
    if (!instance_) {
        instance_ = new ThreadConfig();
    }
    return *instance_;
}

ThreadConfig& ThreadConfig::mutableInstance() {
    if (!instance_) {
        instance_ = new ThreadConfig();
    }
    return *instance_;
}

void ThreadConfig::loadFromConfigManager() {
    auto& cfg = mutableInstance();
    
    // 队列大小配置
    cfg.max_frame_queue_size = 500;  // 可从 ConfigManager 获取
    
    // 背压配置
    cfg.backpressure_max_waits = ConfigManager::instance().backpressureMaxWaits();
    cfg.backpressure_wait_sec = ConfigManager::instance().backpressureWaitSec();
    
    // 传感器空闲超时
    cfg.sensor_idle_timeout_sec = ConfigManager::instance().sensorIdleTimeoutSec();
}

}  // namespace automap_pro
