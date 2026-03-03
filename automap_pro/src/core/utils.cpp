#include "automap_pro/core/utils.h"
#include <filesystem>

namespace automap_pro {

namespace utils {

bool fileExists(const std::string& path) {
    return std::filesystem::exists(path);
}

void createDirectories(const std::string& path) {
    std::filesystem::create_directories(path);
}

}  // namespace utils

}  // namespace automap_pro
