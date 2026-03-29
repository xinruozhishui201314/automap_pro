#pragma once
/**
 * AutoMap-Pro 指标收集系统
 *
 * 设计目标：
 *   - 支持多种指标类型（Counter/Gauge/Timer/Histogram）
 *   - 支持 Prometheus 格式导出
 *   - 支持自定义指标
 *   - 獬态高效采集
 *   - 线程安全
 */

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 指标类型
// ─────────────────────────────────────────────────────────────────────────────
enum class MetricType {
    COUNTER,   // 计数类型: 只增不减
    GAUGE,     // 仪表类型: 可增可减
    TIMER,     // 计时器类型: 时间测量
    HISTOGRAM  // 直方图类型: 分布展示
};

// ─────────────────────────────────────────────────────────────────────────────
// 讌标统计信息
// ─────────────────────────────────────────────────────────────────────────────
struct MetricStats {
    double value = 0.0;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    double sum = 0.0;
    size_t count = 0;
    std::vector<double> recent_values;  // 最近 N 个值（用于滑动窗口统计）

    void update(double v) {
        value = v;
        min = std::min(min, v);
        max = std::max(max, v);
        sum += v;
        count++;
        recent_values.push_back(v);
        if (recent_values.size() > 100) {
            recent_values.erase(recent_values.begin());
        }
    }

    void reset() {
        value = 0.0;
        min = std::numeric_limits<double>::max();
        max = std::numeric_limits<double>::lowest();
        sum = 0.0;
        count = 0;
        recent_values.clear();
    }

    double avg() const {
        return count > 0 ? sum / count : 0.0;
    }

    double percentile(double p) const {
        if (recent_values.empty()) return 0.0;
        std::vector<double> sorted = recent_values;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(sorted.size() * p / 100.0);
        return sorted[std::min(idx, sorted.size() - 1)];
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// 讌标计数器
// ─────────────────────────────────────────────────────────────────────────────
class MetricCounter {
public:
    explicit MetricCounter(const std::string& name, const std::string& help = "")
        : name_(name), help_(help) {}

    void increment(double delta = 1.0) {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.update(stats_.value + delta);
        total_ += delta;
    }

    double get() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return stats_.value;
    }

    double getTotal() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return total_;
    }

    void reset() {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.reset();
    }

    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    const MetricStats& getStats() const { return stats_; }

private:
    std::string name_;
    std::string help_;
    MetricStats stats_;
    double total_ = 0.0;
    mutable std::mutex mutex_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 指标仪表
// ─────────────────────────────────────────────────────────────────────────────
class MetricGauge {
public:
    explicit MetricGauge(const std::string& name, const std::string& help = "")
        : name_(name), help_(help) {}

    void set(double value) {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.update(value);
    }

    void add(double delta) {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.update(stats_.value + delta);
    }

    void sub(double delta) {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.update(stats_.value - delta);
    }

    double get() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return stats_.value;
    }

    void reset() {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.reset();
    }

    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    const MetricStats& getStats() const { return stats_; }

private:
    std::string name_;
    std::string help_;
    MetricStats stats_;
    mutable std::mutex mutex_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 指标计时器
// ─────────────────────────────────────────────────────────────────────────────
class MetricTimer {
public:
    explicit MetricTimer(const std::string& name, const std::string& help = "")
        : name_(name), help_(help) {}

    void start() {
        std::lock_guard<std::mutex> lk(mutex_);
        start_time_ = std::chrono::steady_clock::now();
        running_ = true;
    }

    double stop() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!running_) return 0.0;
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time_).count();
        stats_.update(duration);
        running_ = false;
        return duration;
    }

    double elapsed() const {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!running_) return 0.0;
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start_time_).count();
    }

    void reset() {
        std::lock_guard<std::mutex> lk(mutex_);
        stats_.reset();
        running_ = false;
    }

    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    const MetricStats& getStats() const { return stats_; }

private:
    std::string name_;
    std::string help_;
    MetricStats stats_;
    std::chrono::steady_clock::time_point start_time_;
    bool running_ = false;
    mutable std::mutex mutex_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 指标直方图
// ─────────────────────────────────────────────────────────────────────────────
class MetricHistogram {
public:
    explicit MetricHistogram(const std::string& name, const std::string& help = "",
                            const std::vector<double>& bucket_boundaries = {})
        : name_(name), help_(help) {
        if (bucket_boundaries.empty()) {
            bucket_boundaries_ = {0.0, 10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0};
        } else {
            bucket_boundaries_ = bucket_boundaries;
        }
        counts_.resize(bucket_boundaries_.size() + 1, 0);
    }

    void observe(double value) {
        std::lock_guard<std::mutex> lk(mutex_);
        size_t bucket_idx = 0;
        for (size_t i = 0; i < bucket_boundaries_.size(); ++i) {
            if (value <= bucket_boundaries_[i]) {
                bucket_idx = i;
                break;
            }
            bucket_idx = i + 1;  // 大于所有桶边界，放入最后一个桶
        }
        counts_[bucket_idx]++;
        sum_ += value;
        count_++;
    }

    void reset() {
        std::lock_guard<std::mutex> lk(mutex_);
        std::fill(counts_.begin(), counts_.end(), 0);
        sum_ = 0.0;
        count_ = 0;
    }

    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    const std::vector<double>& getBucketBoundaries() const { return bucket_boundaries_; }
    const std::vector<uint64_t>& getCounts() const { return counts_; }
    double getSum() const { return sum_; }
    uint64_t getCount() const { return count_; }

private:
    std::string name_;
    std::string help_;
    std::vector<double> bucket_boundaries_;
    std::vector<uint64_t> counts_;
    double sum_ = 0.0;
    uint64_t count_ = 0;
    mutable std::mutex mutex_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 预定义的指标名称
// ─────────────────────────────────────────────────────────────────────────────
namespace metrics {

// 稡块指标 (计数器)
inline constexpr const char* LOOP_CLOSURES_DETECTED = "loop_closures_detected";
inline constexpr const char* SUBMAPS_CREATED = "submaps_created";
inline constexpr const char* SUBMAPS_FROZEN = "submaps_frozen";
inline constexpr const char* KEYFRAMES_CREATED = "keyframes_created";
inline constexpr const char* GPS_MEASUREMENTS_VALID = "gps_measurements_valid";
// P1 修复：回退路径检测
inline constexpr const char* FALLBACK_TO_MERGED_CLOUD = "fallback_to_merged_cloud";

inline constexpr const char* OPTIMIZATIONS_RUN = "optimizations_run";

// iSAM2 优化队列与健康（V1）
inline constexpr const char* ISAM2_QUEUE_DEPTH = "isam2_queue_depth";
inline constexpr const char* ISAM2_TASK_DROPPED = "isam2_task_dropped";
// 可观测性：上次优化是否成功 1=成功 0=失败（V2）
inline constexpr const char* ISAM2_LAST_SUCCESS = "isam2_last_success";
/** Keyframe 级 pending GPS 因子队列长度（观测无界增长）。见 BACKEND_POTENTIAL_ISSUES 1.4.2 */
inline constexpr const char* ISAM2_PENDING_GPS_KF = "isam2_pending_gps_kf";
/** 体素下采样因 overflow 最终未滤波即返回的次数。见 BACKEND_POTENTIAL_ISSUES 1.2.3 / 1.4.2 */
inline constexpr const char* VOXEL_OVERFLOW_DROPPED = "voxel_overflow_dropped";
/** 60s 卡死触发的强制 ISAM2 reset 次数。见 BACKEND_POTENTIAL_ISSUES 1.3.4 */
inline constexpr const char* ISAM2_FORCED_RESET = "isam2_forced_reset";

inline constexpr const char* MAP_EXPORTS = "map_exports";
inline constexpr const char* ERRORS_TOTAL = "errors_total";
inline constexpr const char* WARNINGS_TOTAL = "warnings_total";

// 性能指标 (计时器/直方图)
inline constexpr const char* ISAM2_OPTIMIZE_TIME_MS = "isam2_optimize_time_ms";
inline constexpr const char* HBA_OPTIMIZE_TIME_MS = "hba_optimize_time_ms";
inline constexpr const char* MAP_BUILD_TIME_MS = "map_build_time_ms";
inline constexpr const char* MAP_EXPORT_TIME_MS = "map_export_time_ms";
inline constexpr const char* DISK_IO_TIME_MS = "disk_io_time_ms";
inline constexpr const char* DESCRIPTOR_COMPUTE_TIME_MS = "descriptor_compute_time_ms";
inline constexpr const char* TEASER_MATCH_TIME_MS = "teaser_match_time_ms";
inline constexpr const char* ICP_REFINE_TIME_MS = "icp_refine_time_ms";
inline constexpr const char* GPS_SEARCH_TIME_MS = "gps_search_time_ms";
inline constexpr const char* POINTCLOUD_PROCESS_TIME_MS = "pointcloud_process_time_ms";

// 资源指标 (仪表)
inline constexpr const char* MEMORY_USED_MB = "memory_used_mb";
inline constexpr const char* CPU_PERCENT = "cpu_percent";
inline constexpr const char* SUBMAP_QUEUE_SIZE = "submap_queue_size";
inline constexpr const char* LOOP_QUEUE_SIZE = "loop_queue_size";
inline constexpr const char* POINTCLOUD_SIZE = "pointcloud_size";

// 质量指标 (仪表)
inline constexpr const char* LOOP_RMSE_METERS = "loop_rmse_meters";
inline constexpr const char* LOOP_INLIER_RATIO = "loop_inlier_ratio";
inline constexpr const char* GPS_ALIGN_SCORE = "gps_align_score";
inline constexpr const char* ODOMETRY_QUALITY = "odometry_quality";
inline constexpr const char* TEASER_INLIER_RATIO = "teaser_inlier_ratio";
// P2 修复：HBA↔iSAM2 同步诊断
inline constexpr const char* HBA_ISAM2_SEPARATION_M = "hba_isam2_separation_m";
inline constexpr const char* FRAME_MISMATCH_TOTAL = "frame_mismatch_total";
inline constexpr const char* DUPLICATE_OPTIMIZATION_EVENT_TOTAL = "duplicate_optimization_event_total";
inline constexpr const char* UNKNOWN_FRAME_RESULT_TOTAL = "unknown_frame_result_total";
inline constexpr const char* STALE_VERSION_DROP_TOTAL = "stale_version_drop_total";
/** V3：SyncedFrame 可视化等待 Registry 屏障超时次数（与 waitRegistryBarrierForSyncedVisualization 一致） */
inline constexpr const char* V3_VIZ_REGISTRY_BARRIER_TIMEOUT_TOTAL = "v3_viz_registry_barrier_timeout_total";
/** V3：当前云链式与 GPS 直接对齐平移差超过 POSE_DRIFT 阈值次数 */
inline constexpr const char* V3_VIZ_POSE_DRIFT_WARN_TOTAL = "v3_viz_pose_drift_warn_total";
/** V3：publishEverything 聚合轨迹时 Registry 内存在多个 session_id（全局 T_map_odom 仅对单会话严格正确） */
inline constexpr const char* V3_VIZ_MULTI_SESSION_AGGREGATE_TOTAL = "v3_viz_multi_session_aggregate_total";
/** V3：最近一次「当前云 / 语义 world / 树干 world」链式/GPS 混合权重 blend_w，范围 [0,1]（多路径共享） */
inline constexpr const char* V3_VIZ_LAST_CURRENT_CLOUD_BLEND_W = "v3_viz_last_current_cloud_blend_w";
/** V3：上述路径链式 vs 直接 T_map_odom 平移差（米）分布 */
inline constexpr const char* V3_VIZ_CURRENT_CLOUD_CHAIN_DRIFT_M = "v3_viz_current_cloud_chain_drift_m";

} // namespace metrics

// ─────────────────────────────────────────────────────────────────────────────
// 指标注册表 (单例模式)
// ─────────────────────────────────────────────────────────────────────────────
class MetricsRegistry {
public:
    static MetricsRegistry& instance() {
        static MetricsRegistry inst;
        return inst;
    }

    // 初始化
    void init(rclcpp::Node::SharedPtr node = nullptr) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (initialized_) return;
        node_ = node;
        initialized_ = true;

        // 注册默认指标
        registerDefaultMetrics();
    }

    // 注册计数器
    void registerCounter(const std::string& name, const std::string& help = "") {
        std::lock_guard<std::mutex> lk(mutex_);
        if (counters_.find(name) == counters_.end()) {
            counters_[name] = std::make_shared<MetricCounter>(name, help);
        }
    }

    // 注册仪表
    void registerGauge(const std::string& name, const std::string& help = "") {
        std::lock_guard<std::mutex> lk(mutex_);
        if (gauges_.find(name) == gauges_.end()) {
            gauges_[name] = std::make_shared<MetricGauge>(name, help);
        }
    }

    // 注册计时器
    void registerTimer(const std::string& name, const std::string& help = "") {
        std::lock_guard<std::mutex> lk(mutex_);
        if (timers_.find(name) == timers_.end()) {
            timers_[name] = std::make_shared<MetricTimer>(name, help);
        }
    }

    // 注册直方图
    void registerHistogram(const std::string& name, const std::string& help = "",
                        const std::vector<double>& bucket_boundaries = {}) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (histograms_.find(name) == histograms_.end()) {
            histograms_[name] = std::make_shared<MetricHistogram>(name, help, bucket_boundaries);
        }
    }

    // 获取指标
    std::shared_ptr<MetricCounter> getCounter(const std::string& name) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = counters_.find(name);
        return it != counters_.end() ? it->second : nullptr;
    }

    std::shared_ptr<MetricGauge> getGauge(const std::string& name) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = gauges_.find(name);
        return it != gauges_.end() ? it->second : nullptr;
    }

    std::shared_ptr<MetricTimer> getTimer(const std::string& name) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = timers_.find(name);
        return it != timers_.end() ? it->second : nullptr;
    }

    std::shared_ptr<MetricHistogram> getHistogram(const std::string& name) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = histograms_.find(name);
        return it != histograms_.end() ? it->second : nullptr;
    }

    // 便捷方法: 增加计数器
    void incrementCounter(const std::string& name, double delta = 1.0) {
        auto counter = getCounter(name);
        if (counter) {
            counter->increment(delta);
        }
    }

    // 便捷方法: 设置仪表
    void setGauge(const std::string& name, double value) {
        auto gauge = getGauge(name);
        if (gauge) {
            gauge->set(value);
        }
    }

    // 便捷方法: 观察直方图
    void observeHistogram(const std::string& name, double value) {
        auto hist = getHistogram(name);
        if (hist) {
            hist->observe(value);
        }
    }

    // 便捷方法: 开始计时
    void startTimer(const std::string& name) {
        auto timer = getTimer(name);
        if (timer) {
            timer->start();
        }
    }

    // 便捷方法: 停止计时
    double stopTimer(const std::string& name) {
        auto timer = getTimer(name);
        if (timer) {
            return timer->stop();
        }
        return 0.0;
    }

    // 导出 Prometheus 格式
    std::string exportPrometheus() const {
        std::lock_guard<std::mutex> lk(mutex_);
        std::ostringstream oss;

        // 计数器
        oss << "# TYPE automap_counter counter\n";
        for (const auto& [name, counter] : counters_) {
            oss << "# HELP automap_" << name << " " << counter->help() << "\n";
            oss << "automap_" << name << "_total " << counter->getTotal() << "\n";
        }

        // 仪表
        oss << "# TYPE automap_gauge gauge\n";
        for (const auto& [name, gauge] : gauges_) {
            oss << "# HELP automap_" << name << " " << gauge->help() << "\n";
            oss << "automap_" << name << " " << gauge->get() << "\n";
        }

        // 计时器（摘要）
        oss << "# TYPE automap_timer summary\n";
        for (const auto& [name, timer] : timers_) {
            const auto& stats = timer->getStats();
            oss << "# HELP automap_" << name << " " << timer->help() << "\n";
            oss << "automap_" << name << "_count " << stats.count << "\n";
            oss << "automap_" << name << "_sum " << stats.sum << "\n";
            oss << "automap_" << name << "_avg " << stats.avg() << "\n";
            oss << "automap_" << name << "_min " << stats.min << "\n";
            oss << "automap_" << name << "_max " << stats.max << "\n";
        }

        // 直方图
        oss << "# TYPE automap_histogram histogram\n";
        for (const auto& [name, hist] : histograms_) {
            oss << "# HELP automap_" << name << " " << hist->help() << "\n";
            const auto& boundaries = hist->getBucketBoundaries();
            const auto& counts = hist->getCounts();
            uint64_t cumulative = 0;
            for (size_t i = 0; i < counts.size(); ++i) {
                cumulative += counts[i];
                double le = (i < boundaries.size()) ? boundaries[i] : std::numeric_limits<double>::infinity();
                oss << "automap_" << name << "_bucket{le=\"" << le << "} " << cumulative << "\n";
            }
            oss << "automap_" << name << "_sum " << hist->getSum() << "\n";
            oss << "automap_" << name << "_count " << hist->getCount() << "\n";
        }

        return oss.str();
    }

    // 重置所有指标
    void resetAll() {
        std::lock_guard<std::mutex> lk(mutex_);
        for (auto& [name, counter] : counters_) {
            counter->reset();
        }
        for (auto& [name, gauge] : gauges_) {
            gauge->reset();
        }
        for (auto& [name, timer] : timers_) {
            timer->reset();
        }
        for (auto& [name, hist] : histograms_) {
            hist->reset();
        }
    }

    // 获取所有指标摘要
    std::map<std::string, double> getAllCounters() const {
        std::lock_guard<std::mutex> lk(mutex_);
        std::map<std::string, double> result;
        for (const auto& [name, counter] : counters_) {
            result[name] = counter->get();
        }
        return result;
    }

    std::map<std::string, double> getAllGauges() const {
        std::lock_guard<std::mutex> lk(mutex_);
        std::map<std::string, double> result;
        for (const auto& [name, gauge] : gauges_) {
            result[name] = gauge->get();
        }
        return result;
    }

private:
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<MetricCounter>> counters_;
    std::map<std::string, std::shared_ptr<MetricGauge>> gauges_;
    std::map<std::string, std::shared_ptr<MetricTimer>> timers_;
    std::map<std::string, std::shared_ptr<MetricHistogram>> histograms_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;

    std::string last_error_;

    void registerDefaultMetrics() {
        // 系统指标
        registerCounter(metrics::LOOP_CLOSURES_DETECTED, "Loop closures detected");
        registerCounter(metrics::SUBMAPS_CREATED, "SubMaps created");
        registerCounter(metrics::SUBMAPS_FROZEN, "SubMaps frozen");
        registerCounter(metrics::KEYFRAMES_CREATED, "KeyFrames created");
        registerCounter(metrics::GPS_MEASUREMENTS_VALID, "Valid GPS measurements");
        registerCounter(metrics::OPTIMIZATIONS_RUN, "Optimizations run");
        registerCounter(metrics::ISAM2_TASK_DROPPED, "iSAM2 optimization tasks dropped (queue full)");
        registerGauge(metrics::ISAM2_QUEUE_DEPTH, "iSAM2 optimization queue depth");
        registerGauge(metrics::ISAM2_LAST_SUCCESS, "iSAM2 last update success (1=ok 0=fail)");
        registerGauge(metrics::ISAM2_PENDING_GPS_KF, "iSAM2 pending GPS keyframe factors count");
        registerCounter(metrics::VOXEL_OVERFLOW_DROPPED, "Voxel downsample overflow (returned without filter)");
        registerCounter(metrics::ISAM2_FORCED_RESET, "iSAM2 forced reset (stuck >60s)");
        registerCounter(metrics::MAP_EXPORTS, "Map exports");
        registerCounter(metrics::ERRORS_TOTAL, "Total errors");
        registerCounter(metrics::WARNINGS_TOTAL, "Total warnings");

        // 性能指标
        registerHistogram(metrics::ISAM2_OPTIMIZE_TIME_MS, "iSAM2 optimization time (ms)", {5, 10, 20, 50, 100, 200, 500});
        registerHistogram(metrics::HBA_OPTIMIZE_TIME_MS, "HBA optimization time (ms)", {100, 500, 1000, 2000, 5000, 10000});
        registerHistogram(metrics::MAP_BUILD_TIME_MS, "Map build time (ms)", {50, 100, 200, 500, 1000});
        registerHistogram(metrics::MAP_EXPORT_TIME_MS, "Map export time (ms)", {100, 500, 1000, 5000});
        registerHistogram(metrics::DESCRIPTOR_COMPUTE_TIME_MS, "Descriptor compute time (ms)", {10, 50, 100, 200, 500});
        registerHistogram(metrics::TEASER_MATCH_TIME_MS, "TEASER++ match time (ms)", {10, 50, 100, 200, 500});
        registerHistogram(metrics::ICP_REFINE_TIME_MS, "ICP refine time (ms)", {5, 10, 20, 50, 100});
        registerHistogram(metrics::GPS_SEARCH_TIME_MS, "GPS search time (ms)", {1, 5, 10, 20, 50});
        registerHistogram(metrics::POINTCLOUD_PROCESS_TIME_MS, "Point cloud processing (ms)", {10, 50, 100, 200});

        // 资源指标
        registerGauge(metrics::MEMORY_USED_MB, "Memory usage (MB)");
        registerGauge(metrics::CPU_PERCENT, "CPU usage (percent)");
        registerGauge(metrics::SUBMAP_QUEUE_SIZE, "SubMap processing queue size");
        registerGauge(metrics::LOOP_QUEUE_SIZE, "Loop detection queue size");
        registerGauge(metrics::POINTCLOUD_SIZE, "Point cloud size (points)");

        // 质量指标
        registerGauge(metrics::LOOP_RMSE_METERS, "Loop closure RMSE (meters)");
        registerGauge(metrics::LOOP_INLIER_RATIO, "Loop closure inlier ratio");
        registerGauge(metrics::GPS_ALIGN_SCORE, "GPS alignment score");
        registerGauge(metrics::ODOMETRY_QUALITY, "Odometry quality");
        registerGauge(metrics::TEASER_INLIER_RATIO, "TEASER++ inlier ratio");
        registerGauge(metrics::HBA_ISAM2_SEPARATION_M, "HBA vs iSAM2 pose separation (m)");
        registerCounter(metrics::FRAME_MISMATCH_TOTAL, "Frame mismatch contract violations");
        registerCounter(metrics::DUPLICATE_OPTIMIZATION_EVENT_TOTAL, "Duplicate optimization events dropped");
        registerCounter(metrics::UNKNOWN_FRAME_RESULT_TOTAL, "UNKNOWN frame optimization results dropped");
        registerCounter(metrics::STALE_VERSION_DROP_TOTAL, "Stale optimization events dropped");
        registerCounter(metrics::V3_VIZ_REGISTRY_BARRIER_TIMEOUT_TOTAL,
                        "V3 visualization registry barrier wait timeouts");
        registerCounter(metrics::V3_VIZ_POSE_DRIFT_WARN_TOTAL,
                        "V3 visualization POSE_DRIFT threshold crossings");
        registerCounter(metrics::V3_VIZ_MULTI_SESSION_AGGREGATE_TOTAL,
                        "V3 full-map publish with multiple keyframe sessions");
        registerGauge(metrics::V3_VIZ_LAST_CURRENT_CLOUD_BLEND_W,
                      "V3 last viz chain/GPS blend weight [0,1] (SyncedFrame + semantic/trunk world paths)");
        registerHistogram(metrics::V3_VIZ_CURRENT_CLOUD_CHAIN_DRIFT_M,
                          "V3 viz chain vs direct T_map_odom translation drift (m, multi-path)",
                          {0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0});
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// RAII 计时辅助类
// ─────────────────────────────────────────────────────────────────────────────
class ScopedMetricTimer {
public:
    ScopedMetricTimer(const std::string& name)
        : name_(name) {
        MetricsRegistry::instance().startTimer(name);
    }

    ~ScopedMetricTimer() {
        double elapsed = MetricsRegistry::instance().stopTimer(name_);
        MetricsRegistry::instance().observeHistogram(name_, elapsed);
    }

    const std::string& name() const { return name_; }

private:
    std::string name_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 便捷宏
// ─────────────────────────────────────────────────────────────────────────────
#define METRICS_INIT(node) \
    MetricsRegistry::instance().init(node)

#define METRICS_REGISTER_COUNTER(name, help) \
    MetricsRegistry::instance().registerCounter(name, help)

#define METRICS_REGISTER_GAUGE(name, help) \
    MetricsRegistry::instance().registerGauge(name, help)

#define METRICS_REGISTER_TIMER(name, help) \
    MetricsRegistry::instance().registerTimer(name, help)

#define METRICS_REGISTER_HISTOGRAM(name, help, buckets) \
    MetricsRegistry::instance().registerHistogram(name, help, buckets)

#define METRICS_INCREMENT_ONE(name) \
    MetricsRegistry::instance().incrementCounter(name, 1.0)
#define METRICS_INCREMENT_TWO(name, delta) \
    MetricsRegistry::instance().incrementCounter(name, delta)
#define METRICS_INCREMENT(...) \
    GET_METRICS_INCREMENT_OPT(__VA_ARGS__, 2, 1)(__VA_ARGS__)
#define GET_METRICS_INCREMENT_OPT(_1, _2, N, ...) METRICS_INCREMENT_IMPL##N
#define METRICS_INCREMENT_IMPL1(_1) METRICS_INCREMENT_ONE(_1)
#define METRICS_INCREMENT_IMPL2(_1, _2) METRICS_INCREMENT_TWO(_1, _2)

#define METRICS_GAUGE_SET(name, value) \
    MetricsRegistry::instance().setGauge(name, value)

#define METRICS_TIMER_START(name) \
    MetricsRegistry::instance().startTimer(name)

#define METRICS_TIMER_STOP(name) \
    MetricsRegistry::instance().stopTimer(name)

#define METRICS_HISTOGRAM_OBSERVE(name, value) \
    MetricsRegistry::instance().observeHistogram(name, value)

#define METRICS_EXPORT_PROMETHEUS() \
    MetricsRegistry::instance().exportPrometheus()

#define METRICS_RESET_ALL() \
    MetricsRegistry::instance().resetAll()

// RAII 计时宏
#define METRIC_TIMED_SCOPE(name) \
    automap_pro::ScopedMetricTimer _metric_timer_##__LINE__(name)

#define METRIC_TIMED(name) \
    automap_pro::ScopedMetricTimer _metric_timer_##__LINE__(name)

} // namespace automap_pro
