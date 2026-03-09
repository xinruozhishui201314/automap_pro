#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/error_code.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include <nlohmann/json.hpp>
#include <fmt/core.h>
#include <sstream>
#include <iomanip>

namespace automap_pro {

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// ErrorStatistics
// ─────────────────────────────────────────────────────────────────────────────

ErrorStatistics::ErrorStatistics(std::chrono::seconds window) {
    window_end = std::chrono::system_clock::now();
    window_start = window_end - window;
}

void ErrorStatistics::record(const ErrorDetail& error) {
    total_errors++;
    error_counts[static_cast<uint32_t>(error.code())]++;
    component_counts[error.component()]++;
    severity_counts[error.severity()]++;
    if (error.severity() == ErrorSeverity::CRITICAL || error.severity() == ErrorSeverity::FATAL) {
        critical_errors++;
    }
    window_end = std::chrono::system_clock::now();
}

void ErrorStatistics::slideWindow() {
    auto now = std::chrono::system_clock::now();
    window_start = now;
    window_end = now;
    error_counts.clear();
    component_counts.clear();
    severity_counts.clear();
    total_errors = 0;
    critical_errors = 0;
}

double ErrorStatistics::getErrorRate() const {
    auto sec = std::chrono::duration<double>(window_end - window_start).count();
    return sec > 0 ? static_cast<double>(total_errors) / sec : 0.0;
}

double ErrorStatistics::getCriticalRate() const {
    auto sec = std::chrono::duration<double>(window_end - window_start).count();
    return sec > 0 ? static_cast<double>(critical_errors) / sec : 0.0;
}

std::string ErrorStatistics::toPrometheusMetrics() const {
    std::ostringstream oss;
    oss << "# HELP automap_errors_total Total errors in window\n";
    oss << "# TYPE automap_errors_total counter\n";
    oss << "automap_errors_total " << total_errors << "\n";
    oss << "# HELP automap_errors_critical_total Critical errors in window\n";
    oss << "# TYPE automap_errors_critical_total counter\n";
    oss << "automap_errors_critical_total " << critical_errors << "\n";
    oss << "# HELP automap_errors_by_code Errors by error code\n";
    oss << "# TYPE automap_errors_by_code counter\n";
    for (const auto& [code, count] : error_counts) {
        oss << "automap_errors_by_code{code=\"0x" << std::hex << std::setw(8) << std::setfill('0') << code << "\"} " << count << "\n";
    }
    oss << std::dec;
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// AlertEvent
// ─────────────────────────────────────────────────────────────────────────────

std::string AlertEvent::toJson() const {
    json j;
    j["error_code"] = fmt::format("0x{:08X}", static_cast<uint32_t>(error_code));
    j["component"] = component;
    j["message"] = message;
    j["error_rate"] = error_rate;
    j["critical_rate"] = critical_rate;
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()).count();
    j["is_critical"] = is_critical;
    j["alert_level"] = alert_level;
    return j.dump(2);
}

// ─────────────────────────────────────────────────────────────────────────────
// ErrorMonitor
// ─────────────────────────────────────────────────────────────────────────────

ErrorMonitor& ErrorMonitor::instance() {
    static ErrorMonitor inst;
    return inst;
}

ErrorMonitor::ErrorMonitor() {
    statistics_ = ErrorStatistics(stats_window_);
    // 默认告警阈值
    alert_thresholds_.emplace_back("errors_general", 10.0, 30.0, std::chrono::seconds(60), 2);
    alert_thresholds_.emplace_back("errors_critical", 2.0, 10.0, std::chrono::seconds(60), 1);
}

ErrorMonitor::~ErrorMonitor() {
    stop();
}

void ErrorMonitor::recordError(const ErrorDetail& error) {
    {
        std::lock_guard<std::mutex> lk(stats_mutex_);
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - statistics_.window_start).count();
        if (elapsed >= stats_window_.count()) {
            statistics_.slideWindow();
        }
        statistics_.record(error);
    }
    METRICS_INCREMENT(metrics::ERRORS_TOTAL);
    if (error.severity() == ErrorSeverity::WARNING) {
        MetricsRegistry::instance().incrementCounter(metrics::WARNINGS_TOTAL);
    }
    checkAlerts();
}

void ErrorMonitor::recordException(const std::exception& e, ErrorCodeEx code) {
    auto detail = ErrorDetail::fromException(e, code);
    recordError(detail);
}

void ErrorMonitor::setStatisticsWindow(std::chrono::seconds window) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    stats_window_ = window;
}

void ErrorMonitor::addAlertThreshold(const AlertThreshold& threshold) {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    alert_thresholds_.push_back(threshold);
}

void ErrorMonitor::clearAlertThresholds() {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    alert_thresholds_.clear();
}

void ErrorMonitor::registerAlertCallback(AlertCallback callback) {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    alert_callbacks_.push_back(std::move(callback));
}

void ErrorMonitor::registerMetricCallback(MetricCallback callback) {
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    metric_callbacks_.push_back(std::move(callback));
}

ErrorStatistics ErrorMonitor::getCurrentStatistics() const {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    return statistics_;
}

std::vector<AlertEvent> ErrorMonitor::getRecentAlerts(size_t limit) const {
    std::lock_guard<std::mutex> lk(alerts_mutex_);
    if (recent_alerts_.size() <= limit) return recent_alerts_;
    return std::vector<AlertEvent>(recent_alerts_.end() - static_cast<std::ptrdiff_t>(limit), recent_alerts_.end());
}

std::string ErrorMonitor::generateDiagnosticsReport() const {
    auto stats = getCurrentStatistics();
    json j;
    j["total_errors"] = stats.total_errors;
    j["critical_errors"] = stats.critical_errors;
    j["error_rate_per_sec"] = stats.getErrorRate();
    j["critical_rate_per_sec"] = stats.getCriticalRate();
    j["window_sec"] = stats_window_.count();
    j["by_component"] = json::object();
    for (const auto& [comp, count] : stats.component_counts) {
        j["by_component"][error_utils::componentToString(comp)] = count;
    }
    j["by_severity"] = json::object();
    for (const auto& [sev, count] : stats.severity_counts) {
        j["by_severity"][error_utils::severityToString(sev)] = count;
    }
    return j.dump(2);
}

std::string ErrorMonitor::exportPrometheusMetrics() const {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    return statistics_.toPrometheusMetrics();
}

void ErrorMonitor::start() {
    if (running_.exchange(true)) return;
    monitoring_thread_ = std::make_unique<std::thread>(&ErrorMonitor::monitoringThreadLoop, this);
    ALOG_INFO("ErrorMonitor", "ErrorMonitor started (window={}s)", stats_window_.count());
}

void ErrorMonitor::stop() {
    if (!running_.exchange(false)) return;
    if (monitoring_thread_ && monitoring_thread_->joinable()) {
        monitoring_thread_->join();
        monitoring_thread_.reset();
    }
    ALOG_INFO("ErrorMonitor", "ErrorMonitor stopped");
}

void ErrorMonitor::checkAlerts() {
    auto stats = getCurrentStatistics();
    double rate_per_min = stats.getErrorRate() * 60.0;
    double crit_rate_per_min = stats.getCriticalRate() * 60.0;
    std::lock_guard<std::mutex> lk(callbacks_mutex_);
    for (const auto& thresh : alert_thresholds_) {
        bool trigger = false;
        bool critical_trigger = false;
        if (thresh.metric_name == "errors_general" && rate_per_min >= thresh.error_rate_threshold) trigger = true;
        if (thresh.metric_name == "errors_critical" && crit_rate_per_min >= thresh.critical_rate_threshold) critical_trigger = true;
        if (thresh.metric_name == "errors_general" && rate_per_min >= thresh.critical_rate_threshold) critical_trigger = true;
        if (!trigger && !critical_trigger) continue;
        ErrorCodeEx code = (thresh.metric_name == "errors_critical") ? errors::ISAM2_DIVERGED : errors::UNKNOWN_ERROR;
        ErrorDetail dummy(code, "Alert: " + thresh.metric_name + " threshold exceeded");
        triggerAlert(dummy, thresh);
    }
}

void ErrorMonitor::triggerAlert(const ErrorDetail& error, const AlertThreshold& threshold) {
    auto stats = getCurrentStatistics();
    AlertEvent evt;
    evt.error_code = error.code();
    evt.component = error_utils::componentToString(error.component());
    evt.message = error.message();
    evt.error_rate = stats.getErrorRate() * 60.0;
    evt.critical_rate = stats.getCriticalRate() * 60.0;
    evt.timestamp = std::chrono::system_clock::now();
    evt.is_critical = (evt.critical_rate >= threshold.critical_rate_threshold);
    evt.alert_level = evt.is_critical ? "CRITICAL" : "WARNING";
    {
        std::lock_guard<std::mutex> lk(alerts_mutex_);
        recent_alerts_.push_back(evt);
        if (recent_alerts_.size() > 500) recent_alerts_.erase(recent_alerts_.begin());
    }
    ALOG_WARN("ErrorMonitor", "Alert [{}] {}: rate={:.1f}/min critical_rate={:.1f}/min",
              evt.alert_level, threshold.metric_name, evt.error_rate, evt.critical_rate);
    std::vector<AlertCallback> copy;
    {
        std::lock_guard<std::mutex> lk(callbacks_mutex_);
        copy = alert_callbacks_;
    }
    for (auto& cb : copy) {
        try { cb(evt); } catch (const std::exception& e) {
            ALOG_WARN("ErrorMonitor", "alert callback exception: {}", e.what());
        } catch (...) {
            ALOG_WARN("ErrorMonitor", "alert callback unknown exception");
        }
    }
}

void ErrorMonitor::publishMetrics() {
    std::string prom = exportPrometheusMetrics();
    std::vector<MetricCallback> copy;
    {
        std::lock_guard<std::mutex> lk(callbacks_mutex_);
        copy = metric_callbacks_;
    }
    for (auto& cb : copy) {
        try { cb(prom); } catch (const std::exception& e) {
            ALOG_WARN("ErrorMonitor", "metric callback exception: {}", e.what());
        } catch (...) {
            ALOG_WARN("ErrorMonitor", "metric callback unknown exception");
        }
    }
}

void ErrorMonitor::monitoringThreadLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        if (!running_) break;
        publishMetrics();
    }
}

} // namespace automap_pro
