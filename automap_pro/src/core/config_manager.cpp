#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_code.h"
#include "automap_pro/core/error_monitor.h"
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
        
    } catch (const YAML::BadFile& e) {
        std::string err = fmt::format("YAML bad file '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_LOAD_FAILED, err));
        throw std::runtime_error(err);
    } catch (const YAML::ParserException& e) {
        std::string err = fmt::format("YAML parse error in '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_PARSE_ERROR, err));
        throw std::runtime_error(err);
    } catch (const YAML::Exception& e) {
        std::string err = fmt::format("YAML exception loading '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_LOAD_FAILED, err));
        throw std::runtime_error(err);
    } catch (const std::exception& e) {
        std::string err = fmt::format("Failed to load config '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
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
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_PARSE_ERROR, std::string("previous_session_dirs: ") + e.what()));
    } catch (const std::exception& e) {
        ALOG_WARN("ConfigManager", "Exception reading previous_session_dirs: {}", e.what());
        ErrorMonitor::instance().recordException(e, errors::CONFIG_KEY_NOT_FOUND);
    }
    return dirs;
}

} // namespace automap_pro
