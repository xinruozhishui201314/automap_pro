#include "automap_pro/core/config_manager.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace automap_pro {

void ConfigManager::load(const std::string& yaml_path) {
    try {
        cfg_ = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load config: " + yaml_path + " (" + e.what() + ")");
    }
}

std::vector<std::string> ConfigManager::previousSessionDirs() const {
    std::vector<std::string> dirs;
    try {
        auto node = cfg_["session"]["previous_session_dirs"];
        if (node && node.IsSequence()) {
            for (const auto& item : node) {
                dirs.push_back(item.as<std::string>());
            }
        }
    } catch (...) {}
    return dirs;
}

} // namespace automap_pro
