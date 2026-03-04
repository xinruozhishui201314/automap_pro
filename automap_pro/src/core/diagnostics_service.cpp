#include "automap_pro/core/diagnostics_service.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"

#define MOD "Diagnostics"

namespace automap_pro {

DiagnosticsService& DiagnosticsService::instance() {
    static DiagnosticsService inst;
    return inst;
}

void DiagnosticsService::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    
    // Create diagnostics publisher
    diagnostics_pub_ = node_->create_publisher<automap_pro::msg::DiagnosticMsg>(
        "/automap/diagnostics", 10);
    
    // Create status service
    status_service_ = node_->create_service<automap_pro::srv::GetStatus>(
        "/automap/get_status",
        std::bind(&DiagnosticsService::handleGetStatus, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // Create diagnostics service
    diagnostics_service_ = node_->create_service<automap_pro::srv::GetDiagnostics>(
        "/automap/get_diagnostics",
        std::bind(&DiagnosticsService::handleGetDiagnostics, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    ALOG_INFO(MOD, "DiagnosticsService initialized");
}

void DiagnosticsService::start() {
    // Publish initial status
    publishSystemStatus();
    
    ALOG_INFO(MOD, "DiagnosticsService started");
}

void DiagnosticsService::stop() {
    ALOG_INFO(MOD, "DiagnosticsService stopped");
}

void DiagnosticsService::registerModule(const std::string& name, 
                                      std::function<ModuleStatus()> status_cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    module_callbacks_[name] = status_cb;
    
    // Initialize with current status
    ModuleStatus status = status_cb();
    module_status_[name] = status;
    
    SLOG_INFO(MOD, "Module registered: {}", name);
}

void DiagnosticsService::unregisterModule(const std::string& name) {
    std::lock_guard<std::mutex> lk(mutex_);
    module_callbacks_.erase(name);
    module_status_.erase(name);
    
    SLOG_INFO(MOD, "Module unregistered: {}", name);
}

void DiagnosticsService::recordPerformance(const std::string& operation_name, double time_ms) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto& profile = performance_profiles_[operation_name];
    profile.record(time_ms);
    
    SLOG_DEBUG(MOD, "Performance recorded: {} took {:.1f}ms", operation_name, time_ms);
}

void DiagnosticsService::recordError(const std::string& module, const std::string& error_message) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto& status = module_status_[module];
    status.total_errors++;
    status.last_error_message = error_message;
    status.last_error_time = std::chrono::system_clock::now();
    status.healthy = false;
    
    // Update metric
    METRICS_INCREMENT(metrics::ERRORS_TOTAL);
    
    SLOG_ERROR(MOD, "Error recorded in {}: {}", module, error_message);
}

void DiagnosticsService::recordWarning(const std::string& module, const std::string& warning_message) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto& status = module_status_[module];
    status.total_warnings++;
    
    // Update metric
    METRICS_INCREMENT(metrics::WARNINGS_TOTAL);
    
    SLOG_WARN(MOD, "Warning recorded in {}: {}", module, warning_message);
}

ModuleStatus DiagnosticsService::getModuleStatus(const std::string& name) const {
    std::shared_lock<std::shared_mutex> lk(mutex_);
    auto it = module_status_.find(name);
    if (it != module_status_.end()) {
        return it->second;
    }
    return ModuleStatus{name: name};
}

SystemDiagnostics DiagnosticsService::getDiagnostics() const {
    std::shared_lock<std::shared_mutex> lk(mutex_);
    
    SystemDiagnostics diag;
    diag.timestamp = std::chrono::system_clock::now();
    diag.system_status = module_status_.empty() ? "UNKNOWN" : "HEALTHY";
    diag.config_snapshot = getCurrentConfig();
    
    // Collect module status
    for (const auto& [name, cb] : module_callbacks_) {
        auto status = cb();
        diag.module_status[name] = status;
        
        // Determine system health
        if (!status.healthy) {
            diag.system_status = "UNHEALTHY";
        }
        
        // Add to active errors/warnings
        if (status.total_errors > 0 && 
            (std::chrono::system_clock::now() - status.last_error_time).count() < 3600) {  // Last 1 hour
            active_errors_.push_back(fmt::format("{}: {}", name, status.last_error_message));
        }
        if (status.total_warnings > 0) {
            active_warnings_.push_back(fmt::format("{}: {}", name, fmt::format("{} warnings", status.total_warnings)));
        }
    }
    
    diag.active_errors = active_errors_;
    diag.active_warnings = active_warnings_;
    
    // Generate summary
    if (diag.active_errors.empty() && diag.active_warnings.empty()) {
        diag.overall_health_summary = "All systems operational";
    } else {
        diag.overall_health_summary = fmt::format("{} errors, {} warnings",
                                                     diag.active_errors.size(),
                                                     diag.active_warnings.size());
    }
    
    return diag;
}

std::map<std::string, PerformanceProfile> DiagnosticsService::getPerformanceProfiles() const {
    std::shared_lock<std::shared_mutex> lk(mutex_);
    return performance_profiles_;
}

std::string DiagnosticsService::exportToJson() const {
    auto diag = getDiagnostics();
    nlohmann::json j;
    
    j["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
    j["system_status"] = diag.system_status;
    j["overall_health"] = diag.overall_health_summary;
    j["config"] = diag.config_snapshot;
    j["active_errors"] = diag.active_errors;
    j["active_warnings"] = diag.active_warnings;
    
    j["modules"] = nlohmann::json::object();
    for (const auto& [name, status] : diag.module_status) {
        j["modules"][name] = status.toJson();
    }
    
    j["performance"] = nlohmann::json::object();
    for (const auto& [name, profile] : performance_profiles_) {
        j["performance"][name] = profile.toJson();
    }
    
    return j.dump(2);
}

std::string DiagnosticsService::exportToHtml() const {
    auto diag = getDiagnostics();
    
    std::stringstream html;
    html << "<!DOCTYPE html>\n";
    html << "<html>\n<head>\n";
    html << "<title>AutoMap-Pro Diagnostics</title>\n";
    html << "<style>\n";
    html << "body { font-family: Arial, sans-serif; margin: 20px; }\n";
    html << "table { border-collapse: collapse; width: 100%; margin-bottom: 20px; }\n";
    html << "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }\n";
    html << "th { background-color: #f4f4f; }\n";
    html << ".status-healthy { color: green; }\n";
    html << ".status-unhealthy { color: red; }\n";
    html << "</style>\n";
    html << "</head>\n<body>\n";
    html << "<h1>AutoMap-Pro System Diagnostics</h1>\n";
    html << "<p><strong>System Status:</strong> " << diag.system_status << "</p>\n";
    html << "<p><strong>Health Summary:</strong> " << diag.overall_health_summary << "</p>\n";
    
    html << "<h2>Module Status</h2>\n";
    html << "<table>\n";
    html << "<tr><th>Module</th><th>Healthy</th><th>CPU %</th><th>Memory MB</th><th>Errors</th><th>Warnings</th></tr>\n";
    
    for (const auto& [name, status] : diag.module_status) {
        std::string status_class = status.healthy ? "status-healthy" : "status-unhealthy";
        html << "<tr class=\"" << status_class << "\">\n";
        html << "<td>" << name << "</td>\n";
        html << "<td>" << (status.healthy ? "Yes" : "No") << "</td>\n";
        html << "<td>" << std::fixed << std::setprecision(2) << status.cpu_usage << "</td>\n";
        html << "<td>" << std::fixed << std::setprecision(2) << status.memory_mb << "</td>\n";
        html << "<td>" << status.total_errors << "</td>\n";
        html << "<td>" << status.total_warnings << "</td>\n";
        html << "</tr>\n";
    }
    
    html << "</table>\n";
    
    html << "<h2>Performance Profiles</h2>\n";
    html << "<table>\n";
    html << "<tr><th>Operation</th><th>Count</th><th>Avg (ms)</th><th>P50 (ms)</th><th>P95 (ms)</th><th>P99 (ms)</th></tr>\n";
    
    for (const auto& [name, profile] : performance_profiles_) {
        html << "<tr>\n";
        html << "<td>" << name << "</td>\n";
        html << "<td>" << profile.call_count << "</td>\n";
        html << "<td>" << std::fixed << std::setprecision(2) << profile.avg_time_ms << "</td>\n";
        html << "<td>" << std::fixed << std::setprecision(2) << profile.p50_time_ms << "</td>\n";
        html << "<td>" << std::fixed << std::setprecision(2) << profile.p95_time_ms << "</td>\n";
        html << "<td>" << std::fixed << std::setprecision(2) << profile.p99_time_ms << "</td>\n";
        html << "</tr>\n";
    }
    
    html << "</table>\n";
    html << "</body>\n</html>\n";
    
    return html.str();
}

void DiagnosticsService::saveSnapshot(const std::string& path) {
    std::string json_str = exportToJson();
    
    std::ofstream out(path);
    out << json_str;
    out.close();
    
    SLOG_INFO(MOD, "Diagnostics snapshot saved to {}", path);
}

// ─────────────────────────────────────────────────────
// Private Methods
// ─────────────────────────────────────────────────────

void DiagnosticsService::publishSystemStatus() {
    if (!diagnostics_pub_) return;
    
    auto diag = getDiagnostics();
    
    auto msg = std::make_shared<automap_pro::msg::DiagnosticMsg>();
    msg->header.stamp = node_->now();
    msg->system_status = diag.system_status;
    msg->overall_health = diag.overall_health_summary;
    
    diagnostics_pub_->publish(*msg);
}

void DiagnosticsService::handleGetStatus(
    const std::shared_ptr<automap_pro::srv::GetStatus::Request>,
    std::shared_ptr<automap_pro::srv::GetStatus::Response> response) {
    
    SLOG_INFO(MOD, "GetStatus requested");
    
    response->system_status = module_status_.empty() ? "UNKNOWN" : "HEALTHY";
    response->overall_health = module_status_.empty() ? "All systems operational" : "Some issues detected";
}

void DiagnosticsService::handleGetDiagnostics(
    const std::shared_ptr<automap_pro::srv::GetDiagnostics::Request>,
    std::shared_ptr<automap_pro::srv::GetDiagnostics::Response> response) {
    
    SLOG_INFO(MOD, "GetDiagnostics requested");
    
    response->diagnostics_json = exportToJson();
    response->diagnostics_html = exportToHtml();
}

} // namespace automap_pro
