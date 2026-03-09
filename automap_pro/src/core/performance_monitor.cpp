#include "automap_pro/core/performance_monitor.h"
#include <limits>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <unistd.h>
#include <sys/sysinfo.h>

namespace automap_pro {

// Timer实现
PerformanceMonitor::Timer::Timer(const std::string& operation, bool auto_record)
    : operation_(operation), auto_record_(auto_record), stopped_(false) {
    start_time_ = std::chrono::steady_clock::now();
}

PerformanceMonitor::Timer::~Timer() {
    stop();
}

void PerformanceMonitor::Timer::stop() {
    if (!stopped_) {
        stopped_ = true;
        if (auto_record_) {
            auto duration_ms = getElapsedTimeMs();
            PerformanceMonitor::instance().recordOperation(operation_, duration_ms);
        }
    }
}

double PerformanceMonitor::Timer::getElapsedTimeMs() const {
    if (!stopped_) {
        auto end_time = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(end_time - start_time_).count();
    }
    auto end_time = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end_time - start_time_).count();
}

// PerformanceMonitor实现
PerformanceMonitor& PerformanceMonitor::instance() {
    static PerformanceMonitor instance;
    return instance;
}

PerformanceMonitor::PerformanceMonitor() 
    : slow_threshold_ms_(100.0)
    , report_interval_sec_(10.0)
    , auto_report_enabled_(false)
    , running_(false) {
}

PerformanceMonitor::~PerformanceMonitor() {
    shutdown();
}

void PerformanceMonitor::init(double report_interval_sec, double slow_threshold_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    report_interval_sec_ = report_interval_sec;
    slow_threshold_ms_ = slow_threshold_ms;
    
    std::ostringstream oss;
    oss << "PerformanceMonitor initialized: report_interval=" << report_interval_sec
        << "s, slow_threshold=" << slow_threshold_ms << "ms";
    LOG_INFO(oss.str());
}

void PerformanceMonitor::shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        running_ = false;
    }
    
    cv_.notify_all();
    
    if (reporter_thread_.joinable()) {
        reporter_thread_.join();
    }
    
    LOG_INFO("PerformanceMonitor shutdown");
}

void PerformanceMonitor::recordOperation(const std::string& operation, double duration_ms) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto& stats = stats_[operation];
        stats.operation_name = operation;
        stats.count++;
        stats.total_time_ms += duration_ms;
        stats.min_time_ms = std::min(stats.min_time_ms, duration_ms);
        stats.max_time_ms = std::max(stats.max_time_ms, duration_ms);
        
        // 保留最近1000个样本用于计算百分位数
        stats.recent_samples.push_back(duration_ms);
        if (stats.recent_samples.size() > 1000) {
            stats.recent_samples.erase(stats.recent_samples.begin());
        }
    }
    
    // 检查慢操作
    checkSlowOperation(operation, duration_ms);
    
    // 记录到日志（使用统一 ALOG 宏，避免依赖不存在的 logWithContext）
    ALOG_DEBUG("PerfMon", "Performance: {} (duration_ms={})", operation, duration_ms);
}

PerformanceStats PerformanceMonitor::getStats(const std::string& operation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = stats_.find(operation);
    if (it != stats_.end()) {
        return it->second;
    }
    
    return PerformanceStats{operation, 0, 0.0, std::numeric_limits<double>::max(), 0.0, {}};
}

std::map<std::string, PerformanceStats> PerformanceMonitor::getAllStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void PerformanceMonitor::resetStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (auto& [op, stats] : stats_) {
        stats.reset();
    }
    
    LOG_INFO("PerformanceMonitor: All stats reset");
}

void PerformanceMonitor::resetStats(const std::string& operation) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = stats_.find(operation);
    if (it != stats_.end()) {
        it->second.reset();
        LOG_INFO(std::string("PerformanceMonitor: Stats reset for operation: ") + operation);
    }
}

void PerformanceMonitor::setSlowThreshold(double threshold_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    slow_threshold_ms_ = threshold_ms;
}

double PerformanceMonitor::getSlowThreshold() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return slow_threshold_ms_;
}

void PerformanceMonitor::enableAutoReport(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (auto_report_enabled_ == enable) {
        return;
    }
    
    auto_report_enabled_ = enable;
    
    if (enable && !running_) {
        running_ = true;
        reporter_thread_ = std::thread(&PerformanceMonitor::backgroundReporter, this);
        LOG_INFO("PerformanceMonitor: Auto-report enabled");
    } else if (!enable && running_) {
        running_ = false;
        cv_.notify_all();
        if (reporter_thread_.joinable()) {
            reporter_thread_.join();
        }
        LOG_INFO("PerformanceMonitor: Auto-report disabled");
    }
}

void PerformanceMonitor::setReportInterval(double interval_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    report_interval_sec_ = interval_sec;
    cv_.notify_all();
}

void PerformanceMonitor::reportStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (stats_.empty()) {
        LOG_DEBUG("PerformanceMonitor: No stats to report");
        return;
    }
    
    LOG_INFO("PerformanceMonitor: Statistics Report");
    LOG_INFO("========================================");
    
    for (const auto& [operation, stats] : stats_) {
        if (stats.count == 0) continue;
        
        std::ostringstream oss;
        oss << "  " << operation << ":\n";
        oss << "    count:   " << stats.count << "\n";
        oss << "    total:   " << std::fixed << std::setprecision(2) 
            << stats.total_time_ms << " ms\n";
        oss << "    average: " << stats.getAverageTime() << " ms\n";
        oss << "    min:     " << stats.min_time_ms << " ms\n";
        oss << "    max:     " << stats.max_time_ms << " ms\n";
        oss << "    p50:     " << stats.getPercentile(0.5) << " ms\n";
        oss << "    p95:     " << stats.getPercentile(0.95) << " ms\n";
        oss << "    p99:     " << stats.getPercentile(0.99) << " ms";
        
        LOG_INFO(oss.str());
    }
    
    LOG_INFO("========================================");
}

std::unique_ptr<PerformanceMonitor::Timer> PerformanceMonitor::createTimer(
    const std::string& operation) {
    return std::make_unique<Timer>(operation, true);
}

PerformanceMonitor::ResourceUsage PerformanceMonitor::getResourceUsage() const {
    ResourceUsage usage;
    
    // 获取内存使用（从/proc/self/status）
    std::ifstream proc_file("/proc/self/status");
    if (proc_file.is_open()) {
        std::string line;
        while (std::getline(proc_file, line)) {
            if (line.find("VmRSS:") == 0) {
                // VmRSS: 12345 kB
                std::istringstream iss(line);
                std::string key;
                long value;
                std::string unit;
                iss >> key >> value >> unit;
                usage.memory_mb = value / 1024.0; // kB to MB
                break;
            }
        }
    }
    
    // 获取CPU使用（通过/proc/stat，简化版本）
    // 实际实现需要更复杂的计算
    
    // 获取线程数
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        usage.thread_count = info.procs;
    }
    
    return usage;
}

void PerformanceMonitor::backgroundReporter() {
    while (running_) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            // 等待report_interval或shutdown
            if (cv_.wait_for(lock, std::chrono::duration<double>(report_interval_sec_),
                            [this] { return !running_; })) {
                break;  // 被shutdown唤醒
            }
        }
        // 在锁外调用 reportStats()，避免死锁（reportStats 内部会再次获取 mutex_）
        reportStats();
    }
}

void PerformanceMonitor::checkSlowOperation(const std::string& operation, 
                                            double duration_ms) {
    if (duration_ms > slow_threshold_ms_) {
        ALOG_WARN("PerfMon", "Slow operation detected: {} took {} ms (threshold: {} ms)",
                  operation, duration_ms, slow_threshold_ms_);
    }
}

} // namespace automap_pro
