#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include <algorithm>
#include <thread>

#define MOD "HealthMonitor"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────
// ResourceHealthChecker 实现
// ─────────────────────────────────────────────────────────────────────
ResourceHealthChecker::ResourceHealthChecker(const HealthMonitorConfig& config)
    : config_(config) {}

void ResourceHealthChecker::updateMetrics(double memory_mb, double cpu_percent,
                                        int submap_queue_size, int loop_queue_size,
                                        int pointcloud_size) {
    std::lock_guard<std::mutex> lk(mutex_);
    current_memory_mb_ = memory_mb;
    current_cpu_percent_ = cpu_percent;
    current_submap_queue_size_ = submap_queue_size;
    current_loop_queue_size_ = loop_queue_size;
    current_pointcloud_size_ = pointcloud_size;
}

HealthCheckResult ResourceHealthChecker::check() {
    std::lock_guard<std::mutex> lk(mutex_);
    HealthCheckResult result;
    result.name = "Resource";
    result.state = HealthState::HEALTHY;
    result.message = "All resources within normal range";
    
    // Memory check
    result.items["memory"].name = "memory";
    result.items["memory"].state = HealthState::HEALTHY;
    result.items["memory"].current_value = current_memory_mb_;
    result.items["memory"].unit = "MB";
    result.items["memory"].threshold_warning = config_.memory_warning_threshold_mb;
    result.items["memory"].threshold_critical = config_.memory_critical_threshold_mb;
    result.items["memory"].fail_count = 0;
    result.items["memory"].is_ok = current_memory_mb_ < config_.memory_warning_threshold_mb;

    if (current_memory_mb_ >= config_.memory_critical_threshold_mb) {
        result.items["memory"].state = HealthState::CRITICAL;
        result.items["memory"].message = fmt::format("Memory critically high: {:.0f} MB", current_memory_mb_);
        result.items["memory"].fail_count++;
        result.state = HealthState::CRITICAL;
    } else if (current_memory_mb_ >= config_.memory_warning_threshold_mb) {
        result.items["memory"].state = HealthState::DEGRADED;
        result.items["memory"].message = fmt::format("Memory usage high: {:.0f} MB", current_memory_mb_);
    }

    // CPU check
    result.items["cpu"].name = "cpu";
    result.items["cpu"].state = HealthState::HEALTHY;
    result.items["cpu"].current_value = current_cpu_percent_;
    result.items["cpu"].unit = "%";
    result.items["cpu"].threshold_warning = config_.cpu_warning_threshold;
    result.items["cpu"].threshold_critical = config_.cpu_critical_threshold;
    result.items["cpu"].fail_count = 0;
    result.items["cpu"].is_ok = current_cpu_percent_ < config_.cpu_warning_threshold;

    if (current_cpu_percent_ >= config_.cpu_critical_threshold) {
        result.items["cpu"].state = HealthState::CRITICAL;
        result.items["cpu"].message = fmt::format("CPU critically high: {:.1f}%", current_cpu_percent_);
        result.items["cpu"].fail_count++;
        if (result.state < HealthState::CRITICAL) result.state = HealthState::CRITICAL;
    } else if (current_cpu_percent_ >= config_.cpu_warning_threshold) {
        result.items["cpu"].state = HealthState::DEGRADED;
        result.items["cpu"].message = fmt::format("CPU usage high: {:.1f}%", current_cpu_percent_);
        if (result.state < HealthState::DEGRADED) result.state = HealthState::DEGRADED;
    }

    // SubMap queue check
    result.items["submap_queue"].name = "submap_queue";
    result.items["submap_queue"].state = HealthState::HEALTHY;
    result.items["submap_queue"].current_value = static_cast<double>(current_submap_queue_size_);
    result.items["submap_queue"].unit = "count";
    result.items["submap_queue"].threshold_warning = static_cast<double>(config_.queue_size_warning_threshold);
    result.items["submap_queue"].threshold_critical = static_cast<double>(config_.queue_size_critical_threshold);
    result.items["submap_queue"].fail_count = 0;
    result.items["submap_queue"].is_ok = current_submap_queue_size_ < config_.queue_size_warning_threshold;

    if (current_submap_queue_size_ >= static_cast<int>(config_.queue_size_critical_threshold)) {
        result.items["submap_queue"].state = HealthState::CRITICAL;
        result.items["submap_queue"].message = fmt::format("SubMap queue critically high: {}", current_submap_queue_size_);
        result.items["submap_queue"].fail_count++;
        if (result.state < HealthState::CRITICAL) result.state = HealthState::CRITICAL;
    } else if (current_submap_queue_size_ >= static_cast<int>(config_.queue_size_warning_threshold)) {
        result.items["submap_queue"].state = HealthState::DEGRADED;
        result.items["submap_queue"].message = fmt::format("SubMap queue high: {}", current_submap_queue_size_);
        if (result.state < HealthState::DEGRADED) result.state = HealthState::DEGRADED;
    }

    // Loop queue check
    result.items["loop_queue"].name = "loop_queue";
    result.items["loop_queue"].current_value = static_cast<double>(current_loop_queue_size_);
    result.items["loop_queue"].unit = "count";
    result.items["loop_queue"].threshold_warning = static_cast<double>(config_.queue_size_warning_threshold);
    result.items["loop_queue"].threshold_critical = static_cast<double>(config_.queue_size_critical_threshold);
    result.items["loop_queue"].fail_count = 0;
    result.items["loop_queue"].is_ok = current_loop_queue_size_ < config_.queue_size_warning_threshold;

    if (current_loop_queue_size_ >= static_cast<int>(config_.queue_size_critical_threshold)) {
        result.items["loop_queue"].state = HealthState::CRITICAL;
        result.items["loop_queue"].message = fmt::format("Loop queue critically high: {}", current_loop_queue_size_);
        result.items["loop_queue"].fail_count++;
        if (result.state < HealthState::CRITICAL) result.state = HealthState::CRITICAL;
    } else if (current_loop_queue_size_ >= static_cast<int>(config_.queue_size_warning_threshold)) {
        result.items["loop_queue"].state = HealthState::DEGRADED;
        result.items["loop_queue"].message = fmt::format("Loop queue high: {}", current_loop_queue_size_);
        if (result.state < HealthState::DEGRADED) result.state = HealthState::DEGRADED;
    }

    // Point cloud size check
    result.items["pointcloud_size"].name = "pointcloud_size";
    result.items["pointcloud_size"].state = HealthState::HEALTHY;
    result.items["pointcloud_size"].current_value = static_cast<double>(current_pointcloud_size_);
    result.items["pointcloud_size"].unit = "points";
    result.items["pointcloud_size"].threshold_warning = 0.0;
    result.items["pointcloud_size"].threshold_critical = 0.0;
    result.items["pointcloud_size"].fail_count = 0;
    result.items["pointcloud_size"].is_ok = true;

    // Update check time
    auto now = std::chrono::system_clock::now();
    for (auto& [name, item] : result.items) {
        item.last_check_time = now;
    }

    // Update overall message
    if (result.state == HealthState::HEALTHY) {
        result.message = "All resources within normal range";
    } else if (result.state == HealthState::CRITICAL) {
        result.message = "Resource critically exhausted";
    } else if (result.state == HealthState::UNHEALTHY) {
        result.message = "Resource in unhealthy state";
    } else if (result.state == HealthState::DEGRADED) {
        result.message = "Resource usage high but acceptable";
    }

    return result;
}

// ─────────────────────────────────────────────────────────────────────
// SensorHealthChecker 实现
// ─────────────────────────────────────────────────────────────────────
SensorHealthChecker::SensorHealthChecker() {}

void SensorHealthChecker::updateLiDARStatus(double last_time_ms, bool is_timeout) {
    std::lock_guard<std::mutex> lk(mutex_);
    lidar_last_time_ = std::chrono::system_clock::now();
    lidar_timeout_ = is_timeout;
}

void SensorHealthChecker::updateIMUStatus(bool is_receiving, int invalid_count) {
    std::lock_guard<std::mutex> lk(mutex_);
    imu_receiving_ = is_receiving;
    imu_invalid_count_ = invalid_count;
}

void SensorHealthChecker::updateGPSStatus(int hdop, bool has_fix) {
    std::lock_guard<std::mutex> lk(mutex_);
    current_hdop_ = hdop;
    gps_has_fix_ = has_fix;
}

void SensorHealthChecker::updateCameraStatus(double last_time_ms, bool is_receiving) {
    std::lock_guard<std::mutex> lk(mutex_);
    camera_last_time_ = std::chrono::system_clock::now();
    camera_receiving_ = is_receiving;
}

HealthCheckResult SensorHealthChecker::check() {
    std::lock_guard<std::mutex> lk(mutex_);
    HealthCheckResult result;
    result.name = "Sensor";
    result.state = HealthState::HEALTHY;
    result.message = "All sensors operational";

    // LiDAR check
    result.items["lidar"].name = "lidar";
    result.items["lidar"].unit = "s";
    result.items["lidar"].threshold_warning = 5.0;
    result.items["lidar"].threshold_critical = 10.0;
    
    if (lidar_timeout_) {
        result.items["lidar"].state = HealthState::CRITICAL;
        result.items["lidar"].message = "LiDAR timeout - no data received";
        result.items["lidar"].fail_count++;
        result.state = HealthState::CRITICAL;
    } else {
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - lidar_last_time_).count();
        result.items["lidar"].current_value = elapsed / 1000.0;
        result.items["lidar"].is_ok = elapsed < 10000.0;  // 10秒阈值
        result.items["lidar"].last_check_time = lidar_last_time_;
        
        if (elapsed >= 10000.0) {
            result.items["lidar"].state = HealthState::UNHEALTHY;
            result.items["lidar"].message = fmt::format("LiDAR stale: {:.1f}s", elapsed / 1000.0);
            if (result.state < HealthState::UNHEALTHY) result.state = HealthState::UNHEALTHY;
        }
    }

    // IMU check
    result.items["imu"].name = "imu";
    result.items["imu"].unit = "count";
    result.items["imu"].threshold_warning = 10.0;
    result.items["imu"].threshold_critical = 50.0;
    result.items["imu"].current_value = static_cast<double>(imu_invalid_count_);
    result.items["imu"].is_ok = imu_invalid_count_ < 10;
    result.items["imu"].fail_count = 0;
    
    if (!imu_receiving_) {
        result.items["imu"].state = HealthState::CRITICAL;
        result.items["imu"].message = "IMU not receiving data";
        result.items["imu"].fail_count++;
        if (result.state < HealthState::CRITICAL) result.state = HealthState::CRITICAL;
    } else if (imu_invalid_count_ >= 50) {
        result.items["imu"].state = HealthState::CRITICAL;
        result.items["imu"].message = fmt::format("IMU high invalid count: {}", imu_invalid_count_);
        result.items["imu"].fail_count++;
    } else if (imu_invalid_count_ >= 10) {
        result.items["imu"].state = HealthState::UNHEALTHY;
        result.items["imu"].message = fmt::format("IMU invalid count: {}", imu_invalid_count_);
        if (result.state < HealthState::UNHEALTHY) result.state = HealthState::UNHEALTHY;
    }

    // GPS check
    result.items["gps"].name = "gps";
    result.items["gps"].unit = "hdop";
    result.items["gps"].threshold_warning = 5.0;
    result.items["gps"].threshold_critical = 10.0;
    result.items["gps"].current_value = static_cast<double>(current_hdop_);
    result.items["gps"].is_ok = current_hdop_ < 5.0 && gps_has_fix_;
    result.items["gps"].fail_count = 0;
    
    if (!gps_has_fix_) {
        result.items["gps"].state = HealthState::UNHEALTHY;
        result.items["gps"].message = "GPS has no fix";
        result.items["gps"].fail_count++;
        if (result.state < HealthState::UNHEALTHY) result.state = HealthState::UNHEALTHY;
    } else if (current_hdop_ >= 10.0) {
        result.items["gps"].state = HealthState::CRITICAL;
        result.items["gps"].message = fmt::format("GPS HDOP critically high: {:.1f}", current_hdop_);
        result.items["gps"].fail_count++;
        if (result.state < HealthState::CRITICAL) result.state = HealthState::CRITICAL;
    } else if (current_hdop_ >= 5.0) {
        result.items["gps"].state = HealthState::DEGRADED;
        result.items["gps"].message = fmt::format("GPS HDOP high: {:.1f}", current_hdop_);
        if (result.state < HealthState::DEGRADED) result.state = HealthState::DEGRADED;
    }

    // Camera check
    result.items["camera"].name = "camera";
    result.items["camera"].unit = "s";
    result.items["camera"].threshold_warning = 5.0;
    result.items["camera"].threshold_critical = 10.0;
    
    if (!camera_receiving_) {
        result.items["camera"].state = HealthState::UNHEALTHY;
        result.items["camera"].message = "Camera not receiving data";
        result.items["camera"].fail_count++;
        if (result.state < HealthState::UNHEALTHY) result.state = HealthState::UNHEALTHY;
    } else {
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - camera_last_time_).count();
        result.items["camera"].current_value = elapsed / 1000.0;
        result.items["camera"].is_ok = elapsed < 10000.0;
        result.items["camera"].last_check_time = camera_last_time_;
        
        if (elapsed >= 10000.0) {
            result.items["camera"].state = HealthState::UNHEALTHY;
            result.items["camera"].message = fmt::format("Camera stale: {:.1f}s", elapsed / 1000.0);
            if (result.state < HealthState::UNHEALTHY) result.state = HealthState::UNHEALTHY;
        }
    }

    // Determine overall state
    for (const auto& [name, item] : result.items) {
        if (item.state == HealthState::CRITICAL && result.state < HealthState::CRITICAL) {
            result.state = HealthState::CRITICAL;
        } else if (item.state == HealthState::UNHEALTHY && result.state < HealthState::UNHEALTHY) {
            result.state = HealthState::UNHEALTHY;
        } else if (item.state == HealthState::DEGRADED && result.state < HealthState::DEGRADED) {
            result.state = HealthState::DEGRADED;
        }
    }

    if (result.state == HealthState::HEALTHY) {
        result.message = "All sensors operational";
    } else if (result.state == HealthState::CRITICAL) {
        result.message = "One or more sensors in critical state";
    } else if (result.state == HealthState::UNHEALTHY) {
        result.message = "One or more sensors in unhealthy state";
    } else if (result.state == HealthState::DEGRADED) {
        result.message = "One or more sensors degraded";
    }

    // Update check time
    auto now = std::chrono::system_clock::now();
    for (auto& [name, item] : result.items) {
        item.last_check_time = now;
    }

    return result;
}

// ─────────────────────────────────────────────────────────────────────
// PerformanceHealthChecker 实现
// ─────────────────────────────────────────────────────────────────────
PerformanceHealthChecker::PerformanceHealthChecker(const HealthMonitorConfig& config)
    : config_(config) {}

void PerformanceHealthChecker::recordOptimizationTime(const std::string& optimizer_name, double time_ms) {
    std::lock_guard<std::mutex> lk(mutex_);
    optimization_times_[optimizer_name].push_back(time_ms);
    
    // 只保留最近 100 个样本
    if (optimization_times_[optimizer_name].size() > 100) {
        optimization_times_[optimizer_name].erase(
            optimization_times_[optimizer_name].begin(),
            optimization_times_[optimizer_name].begin() + 
            (optimization_times_[optimizer_name].size() - 100)
        );
    }
}

void PerformanceHealthChecker::recordProcessingTime(const std::string& stage_name, double time_ms) {
    std::lock_guard<std::mutex> lk(mutex_);
    processing_times_[stage_name].push_back(time_ms);
    
    // 只保留最近 100 个样本
    if (processing_times_[stage_name].size() > 100) {
        processing_times_[stage_name].erase(
            processing_times_[stage_name].begin(),
            processing_times_[stage_name].begin() + 
            (processing_times_[stage_name].size() - 100)
        );
    }
}

HealthCheckResult PerformanceHealthChecker::check() {
    std::lock_guard<std::mutex> lk(mutex_);
    HealthCheckResult result;
    result.name = "Performance";
    result.state = HealthState::HEALTHY;
    result.message = "Performance within acceptable range";

    // Check optimization times
    for (const auto& [name, times] : optimization_times_) {
        if (times.empty()) continue;
        
        auto last_time = times.back();
        
        result.items[name + "_opt_time"].name = name + "_opt_time";
        result.items[name + "_opt_time"].unit = "ms";
        result.items[name + "_opt_time"].current_value = last_time;
        result.items[name + "_opt_time"].threshold_warning = config_.optimization_timeout_threshold_ms * 0.7;
        result.items[name + "_opt_time"].threshold_critical = config_.optimization_timeout_threshold_ms;
        result.items[name + "_opt_time"].is_ok = last_time < config_.optimization_timeout_threshold_ms;
        result.items[name + "_opt_time"].fail_count = 0;
        
        if (last_time >= config_.optimization_timeout_threshold_ms) {
            result.items[name + "_opt_time"].state = HealthState::CRITICAL;
            result.items[name + "_opt_time"].message = fmt::format("{} optimization timeout: {:.0f}ms", name, last_time);
            result.items[name + "_opt_time"].fail_count++;
            result.state = HealthState::CRITICAL;
        } else if (last_time >= config_.optimization_timeout_threshold_ms * 0.9) {
            result.items[name + "_opt_time"].state = HealthState::UNHEALTHY;
            result.items[name + "_opt_time"].message = fmt::format("{} optimization slow: {:.0f}ms", name, last_time);
            if (result.state < HealthState::UNHEALTHY) result.state = HealthState::UNHEALTHY;
        } else if (last_time >= config_.optimization_timeout_threshold_ms * 0.7) {
            result.items[name + "_opt_time"].state = HealthState::DEGRADED;
            result.items[name + "_opt_time"].message = fmt::format("{} optimization degraded: {:.0f}ms", name, last_time);
            if (result.state < HealthState::DEGRADED) result.state = HealthState::DEGRADED;
        }
    }

    // Check processing times
    for (const auto& [name, times] : processing_times_) {
        if (times.empty()) continue;
        
        auto last_time = times.back();
        
        // 根据不同阶段设置不同阈值
        double warning_threshold = 100.0;
        double critical_threshold = 500.0;
        if (name.find("icp") != std::string::npos) {
            warning_threshold = 50.0;
            critical_threshold = 200.0;
        } else if (name.find("descriptor") != std::string::npos) {
            warning_threshold = 200.0;
            critical_threshold = 1000.0;
        } else if (name.find("teaser") != std::string::npos) {
            warning_threshold = 100.0;
            critical_threshold = 500.0;
        }
        
        result.items[name + "_proc_time"].name = name + "_proc_time";
        result.items[name + "_proc_time"].unit = "ms";
        result.items[name + "_proc_time"].current_value = last_time;
        result.items[name + "_proc_time"].threshold_warning = warning_threshold;
        result.items[name + "_proc_time"].threshold_critical = critical_threshold;
        result.items[name + "_proc_time"].is_ok = last_time < critical_threshold;
        result.items[name + "_proc_time"].fail_count = 0;
        
        if (last_time >= critical_threshold) {
            result.items[name + "_proc_time"].state = HealthState::CRITICAL;
            result.items[name + "_proc_time"].message = fmt::format("{} processing critically slow: {:.0f}ms", name, last_time);
            result.items[name + "_proc_time"].fail_count++;
            if (result.state < HealthState::CRITICAL) result.state = HealthState::CRITICAL;
        } else if (last_time >= warning_threshold) {
            result.items[name + "_proc_time"].state = HealthState::DEGRADED;
            result.items[name + "_proc_time"].message = fmt::format("{} processing slow: {:.0f}ms", name, last_time);
            if (result.state < HealthState::DEGRADED) result.state = HealthState::DEGRADED;
        }
    }

    // Update last check time
    auto now = std::chrono::system_clock::now();
    for (auto& [name, item] : result.items) {
        item.last_check_time = now;
    }

    // Update overall message
    if (result.state == HealthState::HEALTHY) {
        result.message = "Performance within acceptable range";
    } else if (result.state == HealthState::CRITICAL) {
        result.message = "Performance critically degraded";
    } else if (result.state == HealthState::UNHEALTHY) {
        result.message = "Performance degraded";
    } else if (result.state == HealthState::DEGRADED) {
        result.message = "Performance acceptable but slow";
    }

    return result;
}

// ─────────────────────────────────────────────────────────────────────
// HealthMonitor 实现
// ─────────────────────────────────────────────────────────────────────
HealthMonitor::~HealthMonitor() {
    stop();
}

HealthMonitor& HealthMonitor::instance() {
    static HealthMonitor inst;
    return inst;
}

void HealthMonitor::init(rclcpp::Node::SharedPtr node, const HealthMonitorConfig& config) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!node_.expired()) return;  // Already initialized

    node_ = node;
    config_ = config;

    // Create health status publisher (use param node; node_ is weak_ptr)
    health_pub_ = node->create_publisher<automap_pro::msg::MappingStatusMsg>(
        "/automap/health/status", 10);
    
    ALOG_INFO(MOD, "HealthMonitor initialized (check_interval={:.1f}s, heartbeat_interval={:.1f}s)",
               config_.check_interval_sec, config_.heartbeat_interval_sec);
}

void HealthMonitor::start() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (running_.load()) return;
    
    running_.store(true);
    
    // Start check thread
    check_thread_ = std::thread(&HealthMonitor::checkLoop, this);
    
    // Start heartbeat thread
    heartbeat_thread_ = std::thread(&HealthMonitor::heartbeatLoop, this);
    
    ALOG_INFO(MOD, "HealthMonitor started");
}

void HealthMonitor::stop() {
    std::lock_guard<std::mutex> lk(mutex_);
    running_.store(false);
    
    if (check_thread_.joinable()) {
        check_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    
    ALOG_INFO(MOD, "HealthMonitor stopped");
}

bool HealthMonitor::isRunning() const {
    return running_.load();
}

void HealthMonitor::setConfig(const HealthMonitorConfig& config) {
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;
}

void HealthMonitor::registerChecker(std::shared_ptr<HealthChecker> checker) {
    std::lock_guard<std::mutex> lk(mutex_);
    checkers_.push_back(checker);
}

void HealthMonitor::registerHealthStateCallback(HealthStateCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    health_state_callbacks_.push_back(cb);
}

void HealthMonitor::registerDegradationCallback(DegradationCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    degradation_callbacks_.push_back(cb);
}

void HealthMonitor::registerRecoveryCallback(RecoveryCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    recovery_callbacks_.push_back(cb);
}

void HealthMonitor::updateResourceMetrics(double memory_mb, double cpu_percent,
                                       int submap_queue_size, int loop_queue_size,
                                       int pointcloud_size) {
    if (checkers_.empty()) return;
    
    auto resource_checker = std::dynamic_pointer_cast<ResourceHealthChecker>(checkers_[0]);
    if (resource_checker) {
        resource_checker->updateMetrics(memory_mb, cpu_percent, submap_queue_size, 
                                      loop_queue_size, pointcloud_size);
        METRICS_GAUGE_SET(metrics::MEMORY_USED_MB, memory_mb);
        METRICS_GAUGE_SET(metrics::CPU_PERCENT, cpu_percent);
        METRICS_GAUGE_SET(metrics::SUBMAP_QUEUE_SIZE, submap_queue_size);
        METRICS_GAUGE_SET(metrics::LOOP_QUEUE_SIZE, loop_queue_size);
        METRICS_GAUGE_SET(metrics::POINTCLOUD_SIZE, pointcloud_size);
    }
}

void HealthMonitor::updateSensorLiDAR(double last_time_ms, bool is_timeout) {
    if (checkers_.size() < 2) return;
    auto sensor_checker = std::dynamic_pointer_cast<SensorHealthChecker>(checkers_[1]);
    if (sensor_checker) {
        sensor_checker->updateLiDARStatus(last_time_ms, is_timeout);
    }
}

void HealthMonitor::updateSensorIMU(bool is_receiving, int invalid_count) {
    if (checkers_.size() < 2) return;
    auto sensor_checker = std::dynamic_pointer_cast<SensorHealthChecker>(checkers_[1]);
    if (sensor_checker) {
        sensor_checker->updateIMUStatus(is_receiving, invalid_count);
    }
}

void HealthMonitor::updateSensorGPS(int hdop, bool has_fix) {
    if (checkers_.size() < 2) return;
    auto sensor_checker = std::dynamic_pointer_cast<SensorHealthChecker>(checkers_[1]);
    if (sensor_checker) {
        sensor_checker->updateGPSStatus(hdop, has_fix);
    }
}

void HealthMonitor::updateSensorCamera(double last_time_ms, bool is_receiving) {
    if (checkers_.size() < 2) return;
    auto sensor_checker = std::dynamic_pointer_cast<SensorHealthChecker>(checkers_[1]);
    if (sensor_checker) {
        sensor_checker->updateCameraStatus(last_time_ms, is_receiving);
    }
}

void HealthMonitor::recordOptimizationTime(const std::string& optimizer_name, double time_ms) {
    if (checkers_.size() < 3) return;
    auto perf_checker = std::dynamic_pointer_cast<PerformanceHealthChecker>(checkers_[2]);
    if (perf_checker) {
        perf_checker->recordOptimizationTime(optimizer_name, time_ms);
        METRICS_HISTOGRAM_OBSERVE(metrics::ISAM2_OPTIMIZE_TIME_MS, time_ms);
        METRICS_HISTOGRAM_OBSERVE(metrics::HBA_OPTIMIZE_TIME_MS, time_ms);
    }
}

void HealthMonitor::recordProcessingTime(const std::string& stage_name, double time_ms) {
    if (checkers_.size() < 3) return;
    auto perf_checker = std::dynamic_pointer_cast<PerformanceHealthChecker>(checkers_[2]);
    if (perf_checker) {
        perf_checker->recordProcessingTime(stage_name, time_ms);
        METRICS_HISTOGRAM_OBSERVE(metrics::DESCRIPTOR_COMPUTE_TIME_MS, time_ms);
        METRICS_HISTOGRAM_OBSERVE(metrics::TEASER_MATCH_TIME_MS, time_ms);
        METRICS_HISTOGRAM_OBSERVE(metrics::ICP_REFINE_TIME_MS, time_ms);
    }
}

HealthReport HealthMonitor::performHealthCheck() {
    HealthReport report;
    report.timestamp = std::chrono::system_clock::now();
    
    // Execute all checkers
    for (auto& checker : checkers_) {
        auto result = checker->check();
        HealthCheckItem item;
        item.name = result.name;
        item.state = result.state;
        item.message = result.message;
        item.is_ok = (result.state == HealthState::HEALTHY);
        item.last_check_time = std::chrono::system_clock::now();
        report.items[result.name] = item;
    }
    
    // Determine overall state
    report.overall_state = HealthState::HEALTHY;
    for (const auto& [name, result] : report.items) {
        if (result.state == HealthState::CRITICAL) {
            report.overall_state = HealthState::CRITICAL;
            break;
        } else if (result.state == HealthState::UNHEALTHY) {
            report.overall_state = HealthState::UNHEALTHY;
        }
    }
    
    // Generate summary
    if (report.overall_state == HealthState::HEALTHY) {
        report.summary = "System healthy - all checks passed";
    } else if (report.overall_state == HealthState::DEGRADED) {
        report.summary = "System degraded - some checks warnings";
    } else if (report.overall_state == HealthState::UNHEALTHY) {
        report.summary = "System unhealthy - some checks failed";
    } else if (report.overall_state == HealthState::CRITICAL) {
        report.summary = "System critical - multiple critical issues";
    }
    
    // Collect warnings and errors
    for (const auto& [name, result] : report.items) {
        if (result.state == HealthState::CRITICAL) {
            report.errors.push_back(result.name + ": " + result.message);
        } else if (result.state == HealthState::UNHEALTHY) {
            report.errors.push_back(result.name + ": " + result.message);
        } else if (result.state == HealthState::DEGRADED) {
            report.warnings.push_back(result.name + ": " + result.message);
        }
    }
    
    latest_report_ = report;
    return report;
}

void HealthMonitor::publishHealthReport(const HealthReport& report) {
    if (!health_pub_) return;

    auto n = node_.lock();
    if (!n) return;

    auto msg = std::make_shared<automap_pro::msg::MappingStatusMsg>();
    msg->header.stamp = n->now();
    msg->state = healthStateToString(report.overall_state);
    
    // Add summary information
    msg->gps_aligned = false;
    msg->gps_alignment_score = static_cast<float>(static_cast<int>(report.overall_state)) / 3.0 * 100.0;
    msg->keyframe_count = static_cast<uint32_t>(report.items.size());
    msg->submap_count = static_cast<uint32_t>(report.items.size());
    
    health_pub_->publish(*msg);
    
    // Structured logging
    std::string err_str, warn_str;
    for (const auto& e : report.errors) err_str += e + "; ";
    for (const auto& w : report.warnings) warn_str += w + "; ";
    SLOG_EVENT(MOD, "health_report", "State={}, Summary={}, Errors={}, Warnings={}",
               healthStateToString(report.overall_state), report.summary, err_str, warn_str);
}

void HealthMonitor::checkLoop() {
    while (running_.load()) {
        // Perform health check
        auto report = performHealthCheck();
        
        // Publish report
        publishHealthReport(report);
        
        // Trigger health state callbacks
        if (report.overall_state != current_state_) {
            for (auto& cb : health_state_callbacks_) {
                cb(report.overall_state, report);
            }
            current_state_ = report.overall_state;
            
            SLOG_EVENT(MOD, "health_state_change", "State changed to {}", healthStateToString(report.overall_state));
        }
        
        // Check if we need to trigger degradation
        if (shouldTriggerDegradation(report.overall_state)) {
            for (auto& cb : degradation_callbacks_) {
                cb(report);
            }
            SLOG_WARN(MOD, "Triggering degradation due to state={}", healthStateToString(report.overall_state));
        }
        
        // Check if we need to trigger recovery
        if (shouldTriggerRecovery(report.overall_state)) {
            for (auto& cb : recovery_callbacks_) {
                cb(report);
            }
            SLOG_INFO(MOD, "Triggering recovery due to state={}", healthStateToString(report.overall_state));
        }
        
        // Sleep until next check
        std::this_thread::sleep_for(std::chrono::duration<double>(
            config_.check_interval_sec * 1000.0));
    }
}

void HealthMonitor::heartbeatLoop() {
    while (running_.load()) {
        auto n = node_.lock();
        if (!n) break;
        // Publish heartbeat
        auto msg = std::make_shared<automap_pro::msg::MappingStatusMsg>();
        msg->header.stamp = n->now();
        msg->state = "HEALTHY";
        msg->gps_aligned = false;
        
        health_pub_->publish(*msg);
        
        // Structured logging
        SLOG_DEBUG(MOD, "heartbeat", "System alive");
        
        // Sleep until next heartbeat
        std::this_thread::sleep_for(std::chrono::duration<double>(
            config_.heartbeat_interval_sec * 1000.0));
    }
}

HealthState HealthMonitor::getOverallState() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return current_state_;
}

HealthReport HealthMonitor::getLatestReport() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return latest_report_;
}

void HealthMonitor::updateOverallState() {
    // Already updated in performHealthCheck
}

bool HealthMonitor::shouldTriggerDegradation(HealthState state) const {
    return state >= HealthState::DEGRADED && state > current_state_;
}

bool HealthMonitor::shouldTriggerRecovery(HealthState state) const {
    return state == HealthState::HEALTHY && current_state_ > HealthState::HEALTHY;
}

} // namespace automap_pro
