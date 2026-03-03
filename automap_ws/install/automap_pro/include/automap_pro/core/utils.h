#pragma once

#include <string>

namespace automap_pro {

namespace utils {

/** @return true if path exists (file or directory). */
bool fileExists(const std::string& path);

/** Create directory and parents; no-op if already exists. */
void createDirectories(const std::string& path);

}  // namespace utils

}  // namespace automap_pro
