#pragma once
/**
 * AutoMap-Pro Alert System
 */

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"

namespace automap_pro {

enum class AlertLevel { INFO = 0, WARNING = 1, ERROR = 2, CRITICAL = 3 };

struct AlertRule {
    std::string name;
    std::string metric_name;
    double warning_threshold;
    double critical_threshold;
    bool enabled;
};

class AlertSystem {
public:
    static AlertSystem& instance();
    
    void init();
    void start();
    void stop();
    
    void addRule(const AlertRule& rule);
    void triggerAlert(const std::string& rule_name, AlertLevel level, 
                     const std::string& message);
    void checkMetrics();
    
private:
    AlertSystem() = default;
    
    std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::map<std::string, AlertRule> rules_;
    std::thread check_thread_;
};

namespace alert_rules {

inline AlertRule memoryCritical() {
    AlertRule rule;
    rule.name = "memory_critical";
    rule.metric_name = "memory_used_mb";
    rule.warning_threshold = 4096.0;
    rule.critical_threshold = 8192.0;
    rule.enabled = true;
    return rule;
}

inline AlertRule cpuCritical() {
    AlertRule rule;
    rule.name = "cpu_critical";
    rule.metric_name = "cpu_percent";
    rule.warning_threshold = 80.0;
    rule.critical_threshold = 95.0;
    rule.enabled = true;
    return rule;
}

inline AlertRule optimizationTimeout() {
    AlertRule rule;
    rule.name = "optimization_timeout";
    rule.metric_name = "isam2_opt_time_ms";
    rule.warning_threshold = 5000.0;
    rule.critical_threshold = 10000.0;
    rule.enabled = true;
    return rule;
}

} // namespace alert_rules

} // namespace automap_pro
