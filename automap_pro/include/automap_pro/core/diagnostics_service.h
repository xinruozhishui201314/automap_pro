#pragma once
/**
 * @file core/diagnostics_service.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */

/**
 * AutoMap-Pro Runtime Diagnostics Service
 */

#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <automap_pro/msg/DiagnosticMsg.hpp>

namespace automap_pro {

struct ModuleStatus {
    std::string name;
    bool healthy = true;
    double cpu_usage = 0.0;
    double memory_mb = 0.0;
    uint64_t error_count = 0;
    
    nlohmann::json toJson() const {
        nlohmann::json j;
        j["name"] = name;
        j["healthy"] = healthy;
        j["cpu_usage"] = cpu_usage;
        j["memory_mb"] = memory_mb;
        j["error_count"] = error_count;
        return j;
    }
};

class DiagnosticsService {
public:
    static DiagnosticsService& instance();
    
    void init(rclcpp::Node::SharedPtr node);
    void start();
    void stop();
    
    ModuleStatus getModuleStatus(const std::string& name) const;
    void updateModuleStatus(const std::string& name, const ModuleStatus& status);
    nlohmann::json getSystemStatus() const;
    
private:
    DiagnosticsService() = default;
    ~DiagnosticsService();
    
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Publisher<automap_pro::msg::DiagnosticMsg>> status_pub_;
    std::map<std::string, ModuleStatus> module_status_;
    std::mutex mutex_;
};

} // namespace automap_pro
