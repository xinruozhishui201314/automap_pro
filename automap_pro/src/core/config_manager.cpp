#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_code.h"
#include "automap_pro/core/error_monitor.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace automap_pro {

void ConfigManager::load(const std::string& yaml_path) {
    try {
        if (yaml_path.empty()) {
            throw std::invalid_argument("ConfigManager::load: yaml_path is empty");
        }
        
        // 检查文件是否存在
        std::ifstream test_file(yaml_path);
        if (!test_file.good()) {
            throw std::runtime_error("Config file not found or not readable: " + yaml_path);
        }
        test_file.close();
        
        cfg_ = YAML::LoadFile(yaml_path);
        
        if (cfg_.IsNull()) {
            throw std::runtime_error("Config file loaded as null: " + yaml_path);
        }
        
        ALOG_INFO("ConfigManager", "Successfully loaded config from: {}", yaml_path);
        // 诊断：直接读取 sensor.gps.topic，便于排查 LivoBridge 收到默认 /gps/fix 的问题
        try {
            YAML::Node s = cfg_["sensor"];
            if (s.IsMap() && s["gps"]) {
                YAML::Node g = s["gps"];
                if (g.IsMap() && g["topic"]) {
                    std::string raw_topic = g["topic"].as<std::string>();
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[ConfigManager][GPS_DIAG] sensor.gps.topic read from YAML = '%s' (file=%s)",
                        raw_topic.c_str(), yaml_path.c_str());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[ConfigManager][GPS_DIAG] sensor.gps.topic key missing in YAML (sensor.gps exists but no 'topic'); gpsTopic() will return default /gps/fix. Check indent/key in %s",
                        yaml_path.c_str());
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[ConfigManager][GPS_DIAG] sensor or sensor.gps missing in YAML; gpsTopic() will return default /gps/fix. File: %s",
                    yaml_path.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ConfigManager][GPS_DIAG] exception reading sensor.gps.topic from YAML: %s", e.what());
        }

        // ========== 快速修复：离线模式超时配置（EARLY_SHUTDOWN_QUICK_FIX）==========
        std::string mode_type = "offline";
        double online_timeout = 10.0;
        double offline_timeout = 7200.0;
        try {
            if (cfg_["mode"] && cfg_["mode"].IsMap()) {
                if (cfg_["mode"]["type"]) {
                    mode_type = cfg_["mode"]["type"].as<std::string>();
                }
                if (cfg_["mode"]["online"] && cfg_["mode"]["online"]["sensor_idle_timeout_sec"]) {
                    online_timeout = cfg_["mode"]["online"]["sensor_idle_timeout_sec"].as<double>();
                }
                if (cfg_["mode"]["offline"] && cfg_["mode"]["offline"]["sensor_idle_timeout_sec"]) {
                    offline_timeout = cfg_["mode"]["offline"]["sensor_idle_timeout_sec"].as<double>();
                }
            }
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ConfigManager] mode.* timeout read failed, using defaults (online=10, offline=7200)");
        }
        if (cfg_["mode"] && cfg_["mode"].IsMap() && cfg_["mode"]["type"]) {
            sensor_idle_timeout_sec_ = (mode_type == "offline") ? offline_timeout : online_timeout;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CONFIG] %s mode, sensor_idle_timeout = %.1fs", mode_type.c_str(), sensor_idle_timeout_sec_);
        } else {
            sensor_idle_timeout_sec_ = get<double>("system.sensor_idle_timeout_sec", 10.0);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CONFIG] no mode section, sensor_idle_timeout = %.1fs (from system.sensor_idle_timeout_sec)", sensor_idle_timeout_sec_);
        }
        // ========== 快速修复结束 ==========

    } catch (const YAML::BadFile& e) {
        std::string err = fmt::format("YAML bad file '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_LOAD_FAILED, err));
        throw std::runtime_error(err);
    } catch (const YAML::ParserException& e) {
        std::string err = fmt::format("YAML parse error in '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_PARSE_ERROR, err));
        throw std::runtime_error(err);
    } catch (const YAML::Exception& e) {
        std::string err = fmt::format("YAML exception loading '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_LOAD_FAILED, err));
        throw std::runtime_error(err);
    } catch (const std::exception& e) {
        std::string err = fmt::format("Failed to load config '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordException(e, errors::CONFIG_LOAD_FAILED);
        throw;
    }
}

namespace {
Eigen::Vector3d readVector3d(const YAML::Node& cfg, const std::string& key, double dx, double dy, double dz) {
    try {
        YAML::Node node = cfg;
        std::istringstream ss(key);
        std::string token;
        while (std::getline(ss, token, '.')) {
            if (!node.IsMap() || !node[token]) return Eigen::Vector3d(dx, dy, dz);
            node = node[token];
        }
        if (!node.IsSequence() || node.size() < 3) return Eigen::Vector3d(dx, dy, dz);
        return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
    } catch (...) {
        return Eigen::Vector3d(dx, dy, dz);
    }
}
}  // namespace

std::string ConfigManager::getSensorGpsTopicRaw() const {
    try {
        YAML::Node s = cfg_["sensor"];
        if (!s.IsMap() || !s["gps"]) return "";
        YAML::Node g = s["gps"];
        if (!g.IsMap() || !g["topic"]) return "";
        return g["topic"].as<std::string>();
    } catch (...) {
        return "";
    }
}

std::string ConfigManager::gpsTopic() const {
    const std::string default_val("/gps/fix");
    std::string from_get = get<std::string>("sensor.gps.topic", default_val);
    if (from_get != default_val) return from_get;
    std::string raw = getSensorGpsTopicRaw();
    if (!raw.empty()) return raw;
    return default_val;
}

Eigen::Vector3d ConfigManager::gpsCovExcellent() const {
    return readVector3d(cfg_, "gps.cov_excellent", 0.5, 0.5, 1.0);
}
Eigen::Vector3d ConfigManager::gpsCovHigh() const {
    return readVector3d(cfg_, "gps.cov_high", 1.0, 1.0, 2.0);
}
Eigen::Vector3d ConfigManager::gpsCovMedium() const {
    return readVector3d(cfg_, "gps.cov_medium", 2.0, 2.0, 4.0);
}
Eigen::Vector3d ConfigManager::gpsCovLow() const {
    return readVector3d(cfg_, "gps.cov_low", 4.0, 4.0, 8.0);
}

std::vector<std::string> ConfigManager::previousSessionDirs() const {
    std::vector<std::string> dirs;
    try {
        auto node = cfg_["session"]["previous_session_dirs"];
        if (node && node.IsSequence()) {
            for (const auto& item : node) {
                if (item.IsScalar()) {
                    std::string dir = item.as<std::string>();
                    if (!dir.empty()) {
                        dirs.push_back(dir);
                    }
                }
            }
        }
    } catch (const YAML::Exception& e) {
        ALOG_WARN("ConfigManager", "Failed to read previous_session_dirs: {}", e.what());
        RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] previous_session_dirs: %s", e.what());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_PARSE_ERROR, std::string("previous_session_dirs: ") + e.what()));
    } catch (const std::exception& e) {
        ALOG_WARN("ConfigManager", "Exception reading previous_session_dirs: {}", e.what());
        RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] previous_session_dirs: %s", e.what());
        ErrorMonitor::instance().recordException(e, errors::CONFIG_KEY_NOT_FOUND);
    }
    return dirs;
}

} // namespace automap_pro
