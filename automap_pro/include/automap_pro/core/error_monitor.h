#pragma once
/**
 * @file error_monitor.h
 * @brief 错误监控和上报系统
 * 
 * 功能：
 *   - 错误频率统计
 *   - 错误聚合（滑动窗口）
 *   - 监控系统上报（Prometheus 格式）
 *   - 告警阈值检查
 *   - 错误趋势分析
 */

#include <automap_pro/core/error_code.h>
#include <chrono>
#include <map>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <thread>

namespace automap_pro {

/**
 * @brief 告警配置
 */
struct AlertThreshold {
    std::string metric_name;           // 指标名称
    double error_rate_threshold;         // 错误率阈值（错误/分钟）
    double critical_rate_threshold;     // 严重错误率阈值
    std::chrono::seconds window;       // 统计窗口
    int consecutive_trigger;             // 连续触发次数
    
    AlertThreshold(const std::string& name, double rate_thresh, double crit_thresh,
                   std::chrono::seconds win = std::chrono::seconds(60),
                   int cons_trigger = 3)
        : metric_name(name), error_rate_threshold(rate_thresh), 
          critical_rate_threshold(crit_thresh), window(win),
          consecutive_trigger(cons_trigger) {}
};

/**
 * @brief 告警事件
 */
struct AlertEvent {
    ErrorCodeEx error_code;
    std::string component;
    std::string message;
    double error_rate;
    double critical_rate;
    std::chrono::system_clock::time_point timestamp;
    bool is_critical;
    std::string alert_level;  // "WARNING", "CRITICAL", "FATAL"
    
    std::string toJson() const;
};

/**
 * @brief 错误统计（滑动窗口）
 */
struct ErrorStatistics {
    std::map<uint32_t, uint64_t> error_counts;  // 错误码 -> 次数
    std::map<ErrorComponent, uint64_t> component_counts;  // 组件 -> 次数
    std::map<ErrorSeverity, uint64_t> severity_counts;    // 严重程度 -> 次数
    std::chrono::system_clock::time_point window_start;
    std::chrono::system_clock::time_point window_end;
    
    uint64_t total_errors = 0;
    uint64_t critical_errors = 0;
    
    ErrorStatistics(std::chrono::seconds window = std::chrono::seconds(60));
    
    void record(const ErrorDetail& error);
    void slideWindow();
    double getErrorRate() const;  // 错误/秒
    double getCriticalRate() const;  // 严重错误/秒
    std::string toPrometheusMetrics() const;
};

/**
 * @brief 监控上报回调
 */
using AlertCallback = std::function<void(const AlertEvent&)>;
using MetricCallback = std::function<void(const std::string&)>;

/**
 * @brief 错误监控器（单例）
 */
class ErrorMonitor {
public:
    static ErrorMonitor& instance();
    
    // 记录错误
    void recordError(const ErrorDetail& error);
    void recordException(const std::exception& e, ErrorCodeEx code);
    
    // 配置
    void setStatisticsWindow(std::chrono::seconds window);
    void addAlertThreshold(const AlertThreshold& threshold);
    void clearAlertThresholds();
    
    // 回调注册
    void registerAlertCallback(AlertCallback callback);
    void registerMetricCallback(MetricCallback callback);
    
    // 查询
    ErrorStatistics getCurrentStatistics() const;
    std::vector<AlertEvent> getRecentAlerts(size_t limit = 100) const;
    std::string generateDiagnosticsReport() const;
    
    // Prometheus 导出
    std::string exportPrometheusMetrics() const;
    
    // 控制
    void start();
    void stop();
    bool isRunning() const { return running_; }

private:
    ErrorMonitor();
    ~ErrorMonitor();
    
    void checkAlerts();
    void triggerAlert(const ErrorDetail& error, const AlertThreshold& threshold);
    void publishMetrics();
    void monitoringThreadLoop();
    
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> monitoring_thread_;
    std::chrono::seconds stats_window_{std::chrono::seconds(60)};
    
    mutable std::mutex stats_mutex_;
    mutable std::mutex alerts_mutex_;
    mutable std::mutex callbacks_mutex_;
    
    ErrorStatistics statistics_;
    std::vector<AlertThreshold> alert_thresholds_;
    std::vector<AlertEvent> recent_alerts_;
    std::vector<AlertCallback> alert_callbacks_;
    std::vector<MetricCallback> metric_callbacks_;
};

} // namespace automap_pro
