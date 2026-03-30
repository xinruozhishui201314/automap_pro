/**
 * @file core/map_frame_config.cpp
 * @brief 核心实现。
 */
#include "automap_pro/core/map_frame_config.h"
#include "automap_pro/core/logger.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <cmath>

#define MOD "MapFrameConfig"

namespace automap_pro {

namespace MapFrameConfig {

namespace {

std::string trim(const std::string& s) {
    auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end == std::string::npos ? std::string::npos : end - start + 1);
}

bool parseKeyValue(const std::string& line, std::string& key, std::string& value) {
    size_t eq = line.find('=');
    if (eq == std::string::npos) return false;
    key   = trim(line.substr(0, eq));
    value = trim(line.substr(eq + 1));
    return !key.empty();
}

}  // namespace

std::optional<ENUOriginResult> read(const std::string& cfg_path) {
    if (cfg_path.empty()) {
        ALOG_DEBUG(MOD, "read: cfg_path empty");
        return std::nullopt;
    }
    std::ifstream ifs(cfg_path);
    if (!ifs.is_open()) {
        ALOG_DEBUG(MOD, "read: cannot open {}", cfg_path);
        return std::nullopt;
    }
    ENUOriginResult out;
    std::string line;
    while (std::getline(ifs, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::string key, value;
        if (!parseKeyValue(line, key, value)) continue;
        if (key == "latitude") {
            try { out.latitude_deg = std::stod(value); } catch (...) { return std::nullopt; }
        } else if (key == "longitude") {
            try { out.longitude_deg = std::stod(value); } catch (...) { return std::nullopt; }
        } else if (key == "altitude") {
            try { out.altitude_m = std::stod(value); } catch (...) { return std::nullopt; }
        }
    }
    out.valid = std::isfinite(out.latitude_deg) && std::isfinite(out.longitude_deg) && std::isfinite(out.altitude_m)
                && std::abs(out.latitude_deg) <= 90.0 && std::abs(out.longitude_deg) <= 180.0;
    if (!out.valid) {
        ALOG_WARN(MOD, "read: invalid coords lat={} lon={} alt={}", out.latitude_deg, out.longitude_deg, out.altitude_m);
        return std::nullopt;
    }
    ALOG_DEBUG(MOD, "read: {} lat={:.6f} lon={:.6f} alt={:.2f}", cfg_path, out.latitude_deg, out.longitude_deg, out.altitude_m);
    return out;
}

bool write(const std::string& cfg_path,
           double latitude_deg, double longitude_deg, double altitude_m) {
    if (cfg_path.empty()) {
        ALOG_WARN(MOD, "write: cfg_path empty, skip");
        return false;
    }
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) || !std::isfinite(altitude_m)) {
        ALOG_ERROR(MOD, "write: invalid coords lat={} lon={} alt={}", latitude_deg, longitude_deg, altitude_m);
        return false;
    }
    if (std::abs(latitude_deg) > 90.0 || std::abs(longitude_deg) > 180.0) {
        ALOG_ERROR(MOD, "write: lat/lon out of range");
        return false;
    }

    std::filesystem::path p(cfg_path);
    std::error_code ec;
    if (p.has_parent_path() && !std::filesystem::exists(p.parent_path(), ec)) {
        std::filesystem::create_directories(p.parent_path(), ec);
        if (ec) {
            ALOG_ERROR(MOD, "write: failed to create directory {}: {}", p.parent_path().string(), ec.message());
            return false;
        }
    }

    std::ofstream ofs(cfg_path);
    if (!ofs.is_open()) {
        ALOG_ERROR(MOD, "write: failed to open for write: {}", cfg_path);
        return false;
    }

    ofs << "# Map frame (ENU) - WGS84 origin for backend\n";
    ofs << "# East-North-Up; backend uses this single coordinate system\n";
    ofs << "coordinate_system = ENU\n";
    ofs << "latitude = " << latitude_deg << "\n";
    ofs << "longitude = " << longitude_deg << "\n";
    ofs << "altitude = " << altitude_m << "\n";
    ofs.flush();
    if (!ofs.good()) {
        ALOG_ERROR(MOD, "write: write failed: {}", cfg_path);
        return false;
    }
    ofs.close();

    ALOG_INFO(MOD, "map_frame.cfg written: {} (lat={:.6f} lon={:.6f} alt={:.2f})",
              cfg_path, latitude_deg, longitude_deg, altitude_m);
    return true;
}

}  // namespace MapFrameConfig

}  // namespace automap_pro
