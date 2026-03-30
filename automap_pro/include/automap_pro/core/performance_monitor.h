#pragma once
/**
 * @file core/performance_monitor.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */


#include <string>
#include <map>
#include <vector>
#include <chrono>
#include <functional>
#include <mutex>
#include <atomic>
#include "automap_pro/core/logger.h"

namespace automap_pro {

// 性能统计信息
struct PerformanceStats {
    std::string operation_name;
    size_t count{0};
    double total_time_ms{0.0};
    double min_time_ms{std::numeric_limits<double>::max()};
    double max_time_ms{0.0};
    std::vector<double> recent_samples; // 最近N个样本
    
    double getAverageTime() const {
        return count > 0 ? total_time_ms / count : 0.0;
    }
    
    double getPercentile(double p) const {
        if (recent_samples.empty()) return 0.0;
        
        std::vector<double> sorted = recent_samples;
        std::sort(sorted.begin(), sorted.end());
        
        size_t idx = static_cast<size_t>(p * sorted.size());
        idx = std::min(idx, sorted.size() - 1);
        return sorted[idx];
    }
    
    void reset() {
        count = 0;
        total_time_ms = 0.0;
        min_time_ms = std::numeric_limits<double>::max();
        max_time_ms = 0.0;
        recent_samples.clear();
    }
};

// 性能监控器
class PerformanceMonitor {
public:
    static PerformanceMonitor& instance();
    
    void init(double report_interval_sec = 10.0, double slow_threshold_ms = 100.0);
    void shutdown();
    
    // 记录操作耗时
    void recordOperation(const std::string& operation, double duration_ms);
    
    // 获取统计信息
    PerformanceStats getStats(const std::string& operation) const;
    std::map<std::string, PerformanceStats> getAllStats() const;
    
    // 重置统计
    void resetStats();
    void resetStats(const std::string& operation);
    
    // 设置慢操作阈值
    void setSlowThreshold(double threshold_ms);
    double getSlowThreshold() const;
    
    // 自动报告
    void enableAutoReport(bool enable);
    void setReportInterval(double interval_sec);
    
    // 手动报告
    void reportStats();
    
    // 性能计时器（RAII）
    class Timer {
    public:
        Timer(const std::string& operation, bool auto_record = true);
        ~Timer();
        
        void stop();
        double getElapsedTimeMs() const;
        
    private:
        std::string operation_;
        bool auto_record_;
        std::chrono::steady_clock::time_point start_time_;
        bool stopped_;
    };
    
    static std::unique_ptr<Timer> createTimer(const std::string& operation);
    
    // 资源监控
    struct ResourceUsage {
        double memory_mb{0.0};
        double cpu_percent{0.0};
        size_t thread_count{0};
    };
    
    ResourceUsage getResourceUsage() const;
    
private:
    PerformanceMonitor();
    ~PerformanceMonitor();
    PerformanceMonitor(const PerformanceMonitor&) = delete;
    PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;
    
    void backgroundReporter();
    void checkSlowOperation(const std::string& operation, double duration_ms);
    
    std::map<std::string, PerformanceStats> stats_;
    mutable std::mutex mutex_;
    
    double slow_threshold_ms_{100.0};
    double report_interval_sec_{10.0};
    bool auto_report_enabled_{false};
    bool running_{false};
    
    std::thread reporter_thread_;
    std::condition_variable cv_;
};

// 便捷宏
#define PERF_TIMER(op) auto CONCAT(perf_timer_, __LINE__) = PerformanceMonitor::createTimer(op)
#define PERF_TIMER_NORECORD(op) auto CONCAT(perf_timer_, __LINE__) = \
    std::make_unique<PerformanceMonitor::Timer>(op, false)

// 字符串拼接宏
#define CONCAT_IMPL(x, y) x##y
#define CONCAT(x, y) CONCAT_IMPL(x, y)

} // namespace automap_pro
