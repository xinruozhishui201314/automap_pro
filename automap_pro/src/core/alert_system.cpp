/**
 * @file core/alert_system.cpp
 * @brief 核心实现。
 */
#include "automap_pro/core/alert_system.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"

#define MOD "AlertSystem"

namespace automap_pro {

AlertSystem::AlertSystem() {
}

AlertSystem::~AlertSystem() {
    stop();
}

AlertSystem& AlertSystem::instance() {
    static AlertSystem inst;
    return inst;
}

void AlertSystem::init() {
    ALOG_INFO(MOD, "AlertSystem initialized");
}

void AlertSystem::start() {
    running_.store(true);
    ALOG_INFO(MOD, "AlertSystem started");
}

void AlertSystem::stop() {
    running_.store(false);
    if (check_thread_.joinable()) {
        check_thread_.join();
    }
    ALOG_INFO(MOD, "AlertSystem stopped");
}

void AlertSystem::triggerAlert(const std::string& rule_name, AlertLevel level,
                             const std::string& message) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (level == AlertLevel::CRITICAL) {
        ALOG_CRITICAL(MOD, "[ALERT] {}: {}", rule_name, message);
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
    } else if (level == AlertLevel::ERROR) {
        ALOG_ERROR(MOD, "[ALERT] {}: {}", rule_name, message);
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
    } else if (level == AlertLevel::WARNING) {
        ALOG_WARN(MOD, "[ALERT] {}: {}", rule_name, message);
        METRICS_INCREMENT(metrics::WARNINGS_TOTAL);
    } else {
        ALOG_INFO(MOD, "[ALERT] {}: {}", rule_name, message);
    }
    
    SLOG_EVENT(MOD, "alert_triggered",
               "rule={}, level={}, msg={}", rule_name,
               static_cast<int>(level), message);
}

void AlertSystem::checkMetrics() {
    if (!running_.load()) return;
    
    // Memory alert
    double memory_mb = METRICS_GAUGE_GET(metrics::MEMORY_USED_MB, 0.0);
    if (memory_mb >= 8192.0) {
        triggerAlert("memory_critical", AlertLevel::CRITICAL,
                     fmt::format("Memory critically high: {:.0f}MB", memory_mb));
    } else if (memory_mb >= 4096.0) {
        triggerAlert("memory_warning", AlertLevel::WARNING,
                     fmt::format("Memory high: {:.0f}MB", memory_mb));
    }
    
    // CPU alert
    double cpu_percent = METRICS_GAUGE_GET(metrics::CPU_PERCENT, 0.0);
    if (cpu_percent >= 95.0) {
        triggerAlert("cpu_critical", AlertLevel::CRITICAL,
                     fmt::format("CPU critically high: {:.1f}%", cpu_percent));
    } else if (cpu_percent >= 80.0) {
        triggerAlert("cpu_warning", AlertLevel::WARNING,
                     fmt::format("CPU high: {:.1f}%", cpu_percent));
    }
}

