#include "automap_pro/core/error_code.h"
#include <nlohmann/json.hpp>
#include <sstream>
#include <iomanip>
#include <thread>

namespace automap_pro {

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// ErrorContext 实现
// ─────────────────────────────────────────────────────────────────────────────

std::string ErrorContext::toString() const {
    std::ostringstream oss;
    oss << "[trace=" << trace_id;
    if (!span_id.empty()) oss << " span=" << span_id;
    if (!operation.empty()) oss << " op=" << operation;
    if (!file.empty()) oss << " " << file << ":" << line;
    if (!function.empty()) oss << " in " << function;
    oss << "]";
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// ErrorDetail 实现
// ─────────────────────────────────────────────────────────────────────────────

ErrorDetail::ErrorDetail(ErrorCodeEx code) : code_(code) {
    message_ = error_utils::getErrorDescription(code);
    suggestions_ = error_utils::getDefaultSuggestions(code);
}

ErrorDetail::ErrorDetail(ErrorCodeEx code, const std::string& message)
    : code_(code), message_(message) {
    suggestions_ = error_utils::getDefaultSuggestions(code);
}

ErrorDetail ErrorDetail::fromException(const std::exception& e, ErrorCodeEx code) {
    ErrorDetail detail(code, e.what());
    detail.context_.thread_id = std::to_string(
        std::hash<std::thread::id>{}(std::this_thread::get_id()));
    return detail;
}

std::string ErrorDetail::toShortString() const {
    std::ostringstream oss;
    oss << "[0x" << std::hex << std::setw(8) << std::setfill('0')
        << static_cast<uint32_t>(code_) << "] "
        << error_utils::severityToString(code_.severity()) << ": "
        << message_;
    return oss.str();
}

std::string ErrorDetail::toString() const {
    std::ostringstream oss;

    // 错误码和严重程度
    oss << "Error[0x" << std::hex << std::setw(8) << std::setfill('0')
        << static_cast<uint32_t>(code_) << std::dec << "] "
        << "(" << error_utils::severityToString(code_.severity()) << ")\n";

    // 组件
    oss << "  Component: " << error_utils::componentToString(code_.component()) << "\n";

    // 消息
    oss << "  Message: " << message_ << "\n";

    // 上下文
    if (!context_.trace_id.empty()) {
        oss << "  Context: " << context_.toString() << "\n";
    }

    // 原因链
    if (!causes_.empty()) {
        oss << "  Caused by:\n";
        for (size_t i = 0; i < causes_.size(); ++i) {
            oss << "    [" << i << "] " << causes_[i].toShortString() << "\n";
        }
    }

    // 恢复建议
    if (!suggestions_.empty()) {
        oss << "  Recovery suggestions:\n";
        for (const auto& sug : suggestions_) {
            oss << "    - [" << sug.priority << "] " << sug.action;
            if (sug.auto_recoverable) oss << " (auto)";
            oss << "\n";
        }
    }

    // 重试信息
    if (retryable_) {
        oss << "  Retryable: yes (max=" << max_retry_attempts_
            << ", delay=" << retry_delay_ms_ << "ms)\n";
    }

    return oss.str();
}

std::string ErrorDetail::toJson() const {
    json j;

    j["code"] = static_cast<uint32_t>(code_);
    j["code_hex"] = fmt::format("0x{:08X}", static_cast<uint32_t>(code_));
    j["severity"] = error_utils::severityToString(code_.severity());
    j["component"] = error_utils::componentToString(code_.component());
    j["message"] = message_;

    // 上下文
    json ctx = json::object();
    if (!context_.trace_id.empty()) ctx["trace_id"] = context_.trace_id;
    if (!context_.span_id.empty()) ctx["span_id"] = context_.span_id;
    if (!context_.operation.empty()) ctx["operation"] = context_.operation;
    if (!context_.file.empty()) {
        ctx["file"] = context_.file;
        ctx["line"] = context_.line;
    }
    if (!context_.function.empty()) ctx["function"] = context_.function;
    if (!context_.metadata.empty()) ctx["metadata"] = context_.metadata;

    auto t_secs = std::chrono::duration_cast<std::chrono::seconds>(
        context_.timestamp.time_since_epoch()).count();
    ctx["timestamp"] = t_secs;

    if (!ctx.empty()) j["context"] = ctx;

    // 原因链
    if (!causes_.empty()) {
        json causes = json::array();
        for (const auto& c : causes_) {
            causes.push_back(json::parse(c.toJson()));
        }
        j["causes"] = causes;
    }

    // 恢复建议
    if (!suggestions_.empty()) {
        json sugs = json::array();
        for (const auto& s : suggestions_) {
            sugs.push_back({
                {"action", s.action},
                {"expected_result", s.expected_result},
                {"priority", s.priority},
                {"auto_recoverable", s.auto_recoverable}
            });
        }
        j["suggestions"] = sugs;
    }

    // 重试信息
    j["retryable"] = retryable_;
    if (retryable_) {
        j["max_retry_attempts"] = max_retry_attempts_;
        j["retry_delay_ms"] = retry_delay_ms_;
    }

    return j.dump(2);
}

// ─────────────────────────────────────────────────────────────────────────────
// error_utils 实现
// ─────────────────────────────────────────────────────────────────────────────

namespace error_utils {

std::string severityToString(ErrorSeverity sev) {
    switch (sev) {
        case ErrorSeverity::INFO:     return "INFO";
        case ErrorSeverity::WARNING:  return "WARNING";
        case ErrorSeverity::ERROR:    return "ERROR";
        case ErrorSeverity::CRITICAL: return "CRITICAL";
        case ErrorSeverity::FATAL:    return "FATAL";
        default:                      return "UNKNOWN";
    }
}

ErrorSeverity stringToSeverity(const std::string& str) {
    if (str == "INFO")     return ErrorSeverity::INFO;
    if (str == "WARNING")  return ErrorSeverity::WARNING;
    if (str == "ERROR")    return ErrorSeverity::ERROR;
    if (str == "CRITICAL") return ErrorSeverity::CRITICAL;
    if (str == "FATAL")    return ErrorSeverity::FATAL;
    return ErrorSeverity::ERROR;
}

std::string componentToString(ErrorComponent comp) {
    switch (comp) {
        case ErrorComponent::CORE_SYSTEM:    return "Core/System";
        case ErrorComponent::CORE_CONFIG:    return "Core/Config";
        case ErrorComponent::CORE_LOGGER:    return "Core/Logger";
        case ErrorComponent::CORE_VALIDATOR: return "Core/Validator";

        case ErrorComponent::SENSOR_LIDAR:   return "Sensor/LiDAR";
        case ErrorComponent::SENSOR_IMU:     return "Sensor/IMU";
        case ErrorComponent::SENSOR_GPS:     return "Sensor/GPS";
        case ErrorComponent::SENSOR_CAMERA:  return "Sensor/Camera";
        case ErrorComponent::SENSOR_SYNC:    return "Sensor/TimeSync";
        case ErrorComponent::SENSOR_MANAGER: return "Sensor/Manager";

        case ErrorComponent::FRONTEND_LIVO:     return "Frontend/LIVO";
        case ErrorComponent::FRONTEND_KEYFRAME: return "Frontend/KeyFrame";
        case ErrorComponent::FRONTEND_GPS:      return "Frontend/GPS";
        case ErrorComponent::FRONTEND_BRIDGE:   return "Frontend/Bridge";

        case ErrorComponent::SUBMAP_MANAGER: return "SubMap/Manager";
        case ErrorComponent::SUBMAP_SESSION: return "SubMap/Session";
        case ErrorComponent::SUBMAP_ARCHIVE: return "SubMap/Archive";

        case ErrorComponent::LOOP_DETECTOR:   return "Loop/Detector";
        case ErrorComponent::LOOP_DESCRIPTOR: return "Loop/Descriptor";
        case ErrorComponent::LOOP_MATCHER:    return "Loop/Matcher";
        case ErrorComponent::LOOP_ICP:        return "Loop/ICP";

        case ErrorComponent::BACKEND_ISAM2:     return "Backend/iSAM2";
        case ErrorComponent::BACKEND_HBA:       return "Backend/HBA";
        case ErrorComponent::BACKEND_POSEGRAPH: return "Backend/PoseGraph";
        case ErrorComponent::BACKEND_GPS:       return "Backend/GPS";

        case ErrorComponent::MAP_GLOBAL:   return "Map/Global";
        case ErrorComponent::MAP_BUILDER:  return "Map/Builder";
        case ErrorComponent::MAP_EXPORTER: return "Map/Exporter";
        case ErrorComponent::MAP_FILTER:   return "Map/Filter";

        case ErrorComponent::IO_FILE:     return "IO/File";
        case ErrorComponent::IO_PCD:      return "IO/PCD";
        case ErrorComponent::IO_PLY:      return "IO/PLY";
        case ErrorComponent::IO_DATABASE: return "IO/Database";

        case ErrorComponent::SYSTEM_MEMORY:   return "System/Memory";
        case ErrorComponent::SYSTEM_THREAD:   return "System/Thread";
        case ErrorComponent::SYSTEM_TIMEOUT:  return "System/Timeout";
        case ErrorComponent::SYSTEM_RESOURCE: return "System/Resource";

        case ErrorComponent::COMM_ROS:     return "Comm/ROS";
        case ErrorComponent::COMM_SERVICE: return "Comm/Service";
        case ErrorComponent::COMM_ACTION:  return "Comm/Action";
        case ErrorComponent::COMM_TF:      return "Comm/TF";

        case ErrorComponent::EXT_GTSAM:  return "External/GTSAM";
        case ErrorComponent::EXT_PCL:    return "External/PCL";
        case ErrorComponent::EXT_OPENCV: return "External/OpenCV";
        case ErrorComponent::EXT_TORCH:  return "External/Torch";
        case ErrorComponent::EXT_TEASER: return "External/TEASER";
        case ErrorComponent::EXT_HBA:    return "External/HBA";

        default: return "Unknown";
    }
}

ErrorComponent stringToComponent(const std::string& str) {
    if (str == "Core/System")    return ErrorComponent::CORE_SYSTEM;
    if (str == "Core/Config")    return ErrorComponent::CORE_CONFIG;
    if (str == "Sensor/LiDAR")   return ErrorComponent::SENSOR_LIDAR;
    if (str == "Sensor/IMU")     return ErrorComponent::SENSOR_IMU;
    if (str == "Sensor/GPS")     return ErrorComponent::SENSOR_GPS;
    // ... 其他映射
    return ErrorComponent::UNKNOWN;
}

std::string getErrorDescription(ErrorCodeEx code) {
    // 按组件分组
    switch (code.code) {
        // 传感器
        case static_cast<uint32_t>(errors::LIDAR_TIMEOUT):
            return "LiDAR data timeout - no point cloud received within expected interval";
        case static_cast<uint32_t>(errors::LIDAR_POINT_COUNT_LOW):
            return "LiDAR point count below minimum threshold";
        case static_cast<uint32_t>(errors::IMU_DATA_INVALID):
            return "IMU data contains invalid values (NaN or out of range)";
        case static_cast<uint32_t>(errors::IMU_BIAS_HIGH):
            return "IMU bias exceeds acceptable threshold";
        case static_cast<uint32_t>(errors::GPS_HDOP_HIGH):
            return "GPS HDOP value indicates poor satellite geometry";
        case static_cast<uint32_t>(errors::GPS_SATELLITES_LOW):
            return "Insufficient GPS satellites for reliable positioning";
        case static_cast<uint32_t>(errors::GPS_NO_FIX):
            return "GPS receiver has no position fix";
        case static_cast<uint32_t>(errors::SENSOR_SYNC_FAILED):
            return "Failed to synchronize timestamps across sensors";

        // 前端
        case static_cast<uint32_t>(errors::ODOMETRY_DIVERGED):
            return "Odometry estimation diverged - position uncertainty exceeded threshold";
        case static_cast<uint32_t>(errors::KEYFRAME_CREATE_FAILED):
            return "Failed to create keyframe - insufficient or invalid data";
        case static_cast<uint32_t>(errors::KEYFRAME_LOW_QUALITY):
            return "Keyframe quality below threshold - high covariance detected";
        case static_cast<uint32_t>(errors::LIVO_BRIDGE_FAILED):
            return "FAST-LIVO2 bridge communication failed";
        case static_cast<uint32_t>(errors::GPS_ALIGN_FAILED):
            return "GPS-LiDAR trajectory alignment failed - insufficient correspondences";

        // 子图
        case static_cast<uint32_t>(errors::SUBMAP_OVERFLOW):
            return "SubMap exceeded maximum size limit";
        case static_cast<uint32_t>(errors::SUBMAP_EMPTY):
            return "Attempted to operate on empty SubMap";
        case static_cast<uint32_t>(errors::SUBMAP_MERGE_FAILED):
            return "Failed to merge point clouds in SubMap";
        case static_cast<uint32_t>(errors::SUBMAP_STATE_INVALID):
            return "Invalid SubMap state transition attempted";
        case static_cast<uint32_t>(errors::SESSION_LOAD_FAILED):
            return "Failed to load archived session data";
        case static_cast<uint32_t>(errors::SESSION_SAVE_FAILED):
            return "Failed to save session data to disk";
        case static_cast<uint32_t>(errors::ARCHIVE_CORRUPTED):
            return "Archive data is corrupted or incomplete";

        // 回环
        case static_cast<uint32_t>(errors::LOOP_CLOSURE_REJECTED):
            return "Loop closure candidate rejected - geometric verification failed";
        case static_cast<uint32_t>(errors::LOOP_NO_CANDIDATES):
            return "No loop closure candidates found in database";
        case static_cast<uint32_t>(errors::DESCRIPTOR_COMPUTE_FAIL):
            return "Failed to compute overlap descriptor";
        case static_cast<uint32_t>(errors::TEASER_MATCH_FAIL):
            return "TEASER++ matching failed - insufficient inliers";
        case static_cast<uint32_t>(errors::TEASER_RMSE_HIGH):
            return "TEASER++ match RMSE exceeds acceptable threshold";

        // 后端
        case static_cast<uint32_t>(errors::ISAM2_UPDATE_FAILED):
            return "iSAM2 optimization update failed - numerical issues detected";
        case static_cast<uint32_t>(errors::ISAM2_COVARIANCE_FAIL):
            return "Failed to compute covariance from iSAM2 result";
        case static_cast<uint32_t>(errors::ISAM2_DIVERGED):
            return "iSAM2 optimization diverged - cost increased beyond threshold";
        case static_cast<uint32_t>(errors::HBA_OPTIMIZATION_FAIL):
            return "HBA optimization failed to converge";
        case static_cast<uint32_t>(errors::HBA_TIMEOUT):
            return "HBA optimization exceeded time limit";
        case static_cast<uint32_t>(errors::POSEGRAPH_ADD_FAILED):
            return "Failed to add factor to pose graph";

        // 地图
        case static_cast<uint32_t>(errors::MAP_BUILD_FAILED):
            return "Failed to build global map";
        case static_cast<uint32_t>(errors::MAP_MEMORY_LIMIT):
            return "Global map memory limit exceeded";
        case static_cast<uint32_t>(errors::MAP_EXPORT_FAILED):
            return "Failed to export map to file";

        // IO
        case static_cast<uint32_t>(errors::FILE_NOT_FOUND):
            return "Required file not found";
        case static_cast<uint32_t>(errors::FILE_READ_ERROR):
            return "Error reading file";
        case static_cast<uint32_t>(errors::FILE_WRITE_ERROR):
            return "Error writing file";
        case static_cast<uint32_t>(errors::PCD_LOAD_FAILED):
            return "Failed to load PCD point cloud file";
        case static_cast<uint32_t>(errors::PCD_SAVE_FAILED):
            return "Failed to save PCD point cloud file";

        // 系统
        case static_cast<uint32_t>(errors::OUT_OF_MEMORY):
            return "System out of memory - allocation failed";
        case static_cast<uint32_t>(errors::MEMORY_ALLOCATION_FAIL):
            return "Memory allocation failed";
        case static_cast<uint32_t>(errors::CONFIG_LOAD_FAILED):
            return "Failed to load configuration file";
        case static_cast<uint32_t>(errors::CONFIG_PARSE_ERROR):
            return "Error parsing configuration file";
        case static_cast<uint32_t>(errors::CONFIG_KEY_MISSING):
            return "Required configuration key is missing";

        // 外部库
        case static_cast<uint32_t>(errors::TORCH_MODEL_LOAD_FAIL):
            return "Failed to load PyTorch/LibTorch model";
        case static_cast<uint32_t>(errors::TORCH_INFERENCE_FAIL):
            return "PyTorch/LibTorch inference failed";
        case static_cast<uint32_t>(errors::TEASER_EXCEPTION):
            return "TEASER++ library threw exception";

        case static_cast<uint32_t>(errors::SUCCESS):
            return "Success";
        default:
            return "Unknown error";
    }
}

std::vector<RecoverySuggestion> getDefaultSuggestions(ErrorCodeEx code) {
    std::vector<RecoverySuggestion> suggestions;

    switch (code.code) {
        // 传感器超时
        case static_cast<uint32_t>(errors::LIDAR_TIMEOUT):
            suggestions.push_back({"Check LiDAR connection and power",
                                  "LiDAR should start streaming", 1, false});
            suggestions.push_back({"Check topic name matches configuration",
                                  "Messages should appear on topic", 1, false});
            suggestions.push_back({"Verify sensor driver is running",
                                  "Driver should publish data", 2, false});
            break;

        // GPS 信号差
        case static_cast<uint32_t>(errors::GPS_HDOP_HIGH):
            suggestions.push_back({"Wait for better GPS conditions",
                                  "HDOP should decrease", 3, true});
            suggestions.push_back({"Check antenna placement",
                                  "Clear sky view recommended", 2, false});
            suggestions.push_back({"Consider RTK correction if available",
                                  "Improved accuracy expected", 2, false});
            break;

        // 回环被拒
        case static_cast<uint32_t>(errors::LOOP_CLOSURE_REJECTED):
            suggestions.push_back({"Lower overlap threshold in config",
                                  "More candidates accepted", 3, true});
            suggestions.push_back({"Adjust TEASER++ noise bound",
                                  "More inliers found", 3, true});
            suggestions.push_back({"Check for scene changes",
                                  "Loop may be invalid due to changes", 2, false});
            break;

        // iSAM2 优化失败
        case static_cast<uint32_t>(errors::ISAM2_DIVERGED):
            suggestions.push_back({"Reset iSAM2 and rebuild graph",
                                  "Optimization should recover", 1, true});
            suggestions.push_back({"Check for outlier loop closures",
                                  "Remove bad constraints", 2, false});
            suggestions.push_back({"Increase relinearization threshold",
                                  "More stable optimization", 3, true});
            break;

        // 配置错误
        case static_cast<uint32_t>(errors::CONFIG_LOAD_FAILED):
            suggestions.push_back({"Check file path and permissions",
                                  "File should be readable", 1, false});
            suggestions.push_back({"Verify YAML syntax",
                                  "Parse should succeed", 1, false});
            suggestions.push_back({"Use default configuration",
                                  "System can start with defaults", 3, true});
            break;

        // 内存不足
        case static_cast<uint32_t>(errors::OUT_OF_MEMORY):
            suggestions.push_back({"Reduce submap size limit",
                                  "Lower memory usage", 1, true});
            suggestions.push_back({"Archive old submaps to disk",
                                  "Free memory", 1, true});
            suggestions.push_back({"Increase system memory",
                                  "More headroom for large maps", 2, false});
            break;

        // HBA 超时
        case static_cast<uint32_t>(errors::HBA_TIMEOUT):
            suggestions.push_back({"Increase HBA timeout setting",
                                  "Optimization completes", 3, true});
            suggestions.push_back({"Reduce optimization iterations",
                                  "Faster but less accurate", 3, true});
            suggestions.push_back({"Run HBA on subset of keyframes",
                                  "Faster optimization", 2, false});
            break;

        default:
            // 通用建议
            suggestions.push_back({"Check system logs for details",
                                  "More information available", 2, false});
            suggestions.push_back({"Restart the affected component",
                                  "May clear transient errors", 3, true});
            break;
    }

    return suggestions;
}

bool isIgnorable(ErrorCodeEx code) {
    return code.severity() == ErrorSeverity::INFO ||
           code.severity() == ErrorSeverity::WARNING;
}

bool requiresImmediateAction(ErrorCodeEx code) {
    return code.severity() == ErrorSeverity::CRITICAL ||
           code.severity() == ErrorSeverity::FATAL;
}

bool requiresSystemRestart(ErrorCodeEx code) {
    switch (code.code) {
        case static_cast<uint32_t>(errors::OUT_OF_MEMORY):
        case static_cast<uint32_t>(errors::CONFIG_PARSE_ERROR):
        case static_cast<uint32_t>(errors::TORCH_MODEL_LOAD_FAIL):
            return true;
        default:
            return false;
    }
}

} // namespace error_utils

} // namespace automap_pro
