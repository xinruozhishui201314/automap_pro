#pragma once
/**
 * AutoMap-Pro 健康检查系统
 *
 * 设计目标：
 *   - 周期性健康检查（心跳、资源使用、关键指标）
 *   - 健康状态分级（HEALTHY/DEGRADED/UNHEALTHY/CRITICAL）
 *   - 健康检查结果发布（ROS2 话题 + Prometheus 指标）
 *   - 自动降级/恢复机制
 *   - 支持自定义健康检查项
 */

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <mutex>
#include <chrono>
#include <functional>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <automap_pro/msg/mapping_status_msg.hpp>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 健康状态
// ─────────────────────────────────────────────────────────────────────────────
enum class HealthState : uint8_t {
    HEALTHY    = 0,  // 正常运行
    DEGRADED   = 1,  // 性能降级（如内存高）
    UNHEALTHY = 2,  // 某些功能异常
    CRITICAL   = 3   // 严重故障
};

std::string healthStateToString(HealthState state) {
    switch (state) {
        case HealthState::HEALTHY:    return "HEALTHY";
        case HealthState::DEGRADED:   return "DEGRADED";
        case HealthState::UNHEALTHY: return "UNHEALTHY";
        case HealthState::CRITICAL:   return "CRITICAL";
        default:                      return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 健康检查项
// ─────────────────────────────────────────────────────────────────────────────
struct HealthCheckItem {
    std::string name;                    // 检查项名称
    HealthState state;                   // 当前状态
    std::string message;                  // 详细消息
    double threshold_warning;              // 警告阈值
    double threshold_critical;            // 严重阈值
    double current_value;                // 当前值
    std::string unit;                    // 单位（如 MB、%、ms）
    bool is_ok;                         // 是否正常
    std::chrono::system_clock::time_point last_check_time;  // 上次检查时间
    int fail_count;                       // 失败次数（用于判断是否需要报警）
};

// ─────────────────────────────────────────────────────────────────────────────
// 健康检查报告
// ─────────────────────────────────────────────────────────────────────────────
struct HealthReport {
    std::chrono::system_clock::time_point timestamp;
    HealthState overall_state;              // 整体状态（取最差的）
    std::map<std::string, HealthCheckItem> items;
    std::vector<std::string> warnings;       // 警告列表
    std::vector<std::string> errors;         // 错误列表
    std::string summary;                   // 摘要信息

    HealthReport() : overall_state(HealthState::HEALTHY),
                       timestamp(std::chrono::system_clock::now()) {}
};

// ─────────────────────────────────────────────────────────────────────────────
// 健康检查配置
// ─────────────────────────────────────────────────────────────────────────────
struct HealthMonitorConfig {
    double check_interval_sec = 10.0;       // 检查间隔（秒）
    double heartbeat_interval_sec = 5.0;    // 心跳发布间隔（秒）
    bool enable_alerts = true;             // 启用告警
    int consecutive_fail_threshold = 3;       // 连续失败次数阈值（触发告警）
    double memory_warning_threshold_mb = 4096.0;   // 内存警告阈值（MB）
    double memory_critical_threshold_mb = 8192.0;  // 内存严重阈值（MB）
    double cpu_warning_threshold = 80.0;       // CPU 警告阈值（%）
    double cpu_critical_threshold = 95.0;       // CPU 严重阈值（%）
    double queue_size_warning_threshold = 100;  // 队列大小警告阈值
    double queue_size_critical_threshold = 500; // 队列大小严重阈值
    int optimization_timeout_threshold_ms = 10000;  // 优化超时阈值（ms）
};

// ─────────────────────────────────────────────────────────────────────────────
// 健康检查器基类
// ─────────────────────────────────────────────────────────────────────────────
class HealthChecker {
public:
    virtual ~HealthChecker() = default;
    virtual std::string name() const = 0;
    virtual HealthCheckResult check() = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// 资源健康检查器
// ─────────────────────────────────────────────────────────────────────────────
class ResourceHealthChecker : public HealthChecker {
public:
    ResourceHealthChecker(const HealthMonitorConfig& config);
    void updateMetrics(double memory_mb, double cpu_percent, int submap_queue_size,
                      int loop_queue_size, int pointcloud_size);

    std::string name() const override { return "Resource"; }
    HealthCheckResult check() override;

private:
    HealthMonitorConfig config_;
    std::mutex mutex_;
    double current_memory_mb_ = 0.0;
    double current_cpu_percent_ = 0.0;
    int current_submap_queue_size_ = 0;
    int current_loop_queue_size_ = 0;
    int current_pointcloud_size_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// 传感器健康检查器
// ─────────────────────────────────────────────────────────────────────────────
class SensorHealthChecker : public HealthChecker {
public:
    SensorHealthChecker();

    void updateLiDARStatus(double last_time_ms, bool is_timeout);
    void updateIMUStatus(bool is_receiving, int invalid_count);
    void updateGPSStatus(int hdop, bool has_fix);
    void updateCameraStatus(double last_time_ms, bool is_receiving);

    std::string name() const override { return "Sensor"; }
    HealthCheckResult check() override;

private:
    std::mutex mutex_;
    // LiDAR
    std::chrono::system_clock::time_point lidar_last_time_;
    bool lidar_timeout_ = false;
    // IMU
    int imu_invalid_count_ = 0;
    bool imu_receiving_ = false;
    // GPS
    int current_hdop_ = 99;
    bool gps_has_fix_ = false;
    // Camera
    std::chrono::system_clock::time_point camera_last_time_;
    bool camera_receiving_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// 性能健康检查器
// ─────────────────────────────────────────────────────────────────────────────
class PerformanceHealthChecker : public HealthChecker {
public:
    PerformanceHealthChecker(const HealthMonitorConfig& config);

    void recordOptimizationTime(const std::string& optimizer_name, double time_ms);
    void recordProcessingTime(const std::string& stage_name, double time_ms);

    std::string name() const override { return "Performance"; }
    HealthCheckResult check() override;

private:
    HealthMonitorConfig config_;
    std::mutex mutex_;
    std::map<std::string, std::vector<double>> optimization_times_;
    std::map<std::string, std::vector<double>> processing_times_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 健康监控器（主类）
// ─────────────────────────────────────────────────────────────────────────────
class HealthMonitor {
public:
    static HealthMonitor& instance();

    // 初始化
    void init(rclcpp::Node::SharedPtr node, const HealthMonitorConfig& config = HealthMonitorConfig());
    void start();
    void stop();
    void setConfig(const HealthMonitorConfig& config);

    // 注册检查器
    void registerChecker(std::shared_ptr<HealthChecker> checker);

    // 更新接口（由各个模块调用）
    void updateResourceMetrics(double memory_mb, double cpu_percent,
                            int submap_queue_size, int loop_queue_size, int pointcloud_size);
    void updateSensorLiDAR(double last_time_ms, bool is_timeout);
    void updateSensorIMU(bool is_receiving, int invalid_count);
    void updateSensorGPS(int hdop, bool has_fix);
    void updateSensorCamera(double last_time_ms, bool is_receiving);
    void recordOptimizationTime(const std::string& optimizer_name, double time_ms);
    void recordProcessingTime(const std::string& stage_name, double time_ms);

    // 检查和报告
    HealthReport performHealthCheck();
    void publishHealthReport(const HealthReport& report);

    // 获取当前状态
    HealthState getOverallState() const;
    HealthReport getLatestReport() const;

    // 降级策略
    bool shouldTriggerDegradation(HealthState state) const;
    bool shouldTriggerRecovery(HealthState state) const;

    // 回调类型
    using HealthStateCallback = std::function<void(HealthState, const HealthReport&)>;
    using DegradationCallback = std::function<void(const HealthReport&)>;
    using RecoveryCallback = std::function<void(const HealthReport&)>;

    void registerHealthStateCallback(HealthStateCallback cb);
    void registerDegradationCallback(DegradationCallback cb);
    void registerRecoveryCallback(RecoveryCallback cb);

private:
    HealthMonitor() = default;
    ~HealthMonitor();

    // 检查线程
    void checkLoop();
    void heartbeatLoop();

    // 状态更新
    void updateOverallState();

    // 成员变量
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Publisher<automap_pro::msg::MappingStatusMsg>> health_pub_;
    std::vector<std::shared_ptr<HealthChecker>> checkers_;

    std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::thread check_thread_;
    std::thread heartbeat_thread_;

    HealthMonitorConfig config_;

    HealthState current_state_ = HealthState::HEALTHY;
    HealthReport latest_report_;
    int consecutive_unhealthy_count_ = 0;

    std::vector<HealthStateCallback> health_state_callbacks_;
    std::vector<DegradationCallback> degradation_callbacks_;
    std::vector<RecoveryCallback> recovery_callbacks_;

    std::chrono::steady_clock::time_point last_check_time_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 便捷宏
// ─────────────────────────────────────────────────────────────────────────────
#define HEALTH_UPDATE_MEMORY(mb) \
    HealthMonitor::instance().updateResourceMetrics(mb, 0.0, 0, 0, 0)
#define HEALTH_UPDATE_CPU(percent) \
    HealthMonitor::instance().updateResourceMetrics(0.0, percent, 0, 0, 0)
#define HEALTH_UPDATE_QUEUE(type, size) \
    HealthMonitor::instance().updateResourceMetrics(0.0, 0.0, (type == "submap" ? size : 0), \
                                                (type == "loop" ? size : 0), 0)

#define HEALTH_UPDATE_LIDAR(timeout) \
    HealthMonitor::instance().updateSensorLiDAR(0.0, timeout)
#define HEALTH_UPDATE_IMU(receiving, invalid) \
    HealthMonitor::instance().updateSensorIMU(receiving, invalid)
#define HEALTH_UPDATE_GPS(hdop, fix) \
    HealthMonitor::instance().updateSensorGPS(hdop, fix)
#define HEALTH_UPDATE_CAMERA(timeout, receiving) \
    HealthMonitor::instance().updateSensorCamera(0.0, receiving)

#define HEALTH_RECORD_OPT_TIME(name, ms) \
    HealthMonitor::instance().recordOptimizationTime(name, ms)
#define HEALTH_RECORD_PROC_TIME(name, ms) \
    HealthMonitor::instance().recordProcessingTime(name, ms)

} // namespace automap_pro
