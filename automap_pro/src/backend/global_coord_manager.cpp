#include "automap_pro/backend/global_coord_manager.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <fstream>
#include <sstream>
#include <filesystem>

#include "automap_pro/core/logger.h"

#define MOD "GlobalCoordManager"

namespace automap_pro {

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
    key = trim(line.substr(0, eq));
    value = trim(line.substr(eq + 1));
    return !key.empty();
}

}  // namespace

GlobalCoordManager::GlobalCoordManager(const std::string& config_path)
    : config_path_(config_path) {
    ALOG_INFO(MOD, "GlobalCoordManager created with config_path: %s", config_path.c_str());
}

bool GlobalCoordManager::loadFromConfig() {
    if (config_path_.empty()) {
        ALOG_DEBUG(MOD, "config_path empty, cannot load from config");
        return false;
    }
    
    std::ifstream ifs(config_path_);
    if (!ifs.is_open()) {
        ALOG_DEBUG(MOD, "Cannot open config file: %s", config_path_.c_str());
        return false;
    }
    
    ENUOrigin out;
    std::string line;
    while (std::getline(ifs, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::string key, value;
        if (!parseKeyValue(line, key, value)) continue;
        if (key == "latitude") {
            try { out.latitude = std::stod(value); } catch (...) { return false; }
        } else if (key == "longitude") {
            try { out.longitude = std::stod(value); } catch (...) { return false; }
        } else if (key == "altitude") {
            try { out.altitude = std::stod(value); } catch (...) { return false; }
        }
    }
    
    if (!out.isValid()) {
        ALOG_WARN(MOD, "ENU origin from config is invalid: lat=%.6f lon=%.6f alt=%.2f",
                  out.latitude, out.longitude, out.altitude);
        return false;
    }
    
    // 设置原点
    {
        std::lock_guard<std::mutex> lk(mutex_);
        origin_ = out;
    }
    origin_set_.store(true, std::memory_order_release);
    origin_source_.store(OriginSource::CONFIG_FILE);
    
    ALOG_INFO(MOD, "ENU origin loaded from config: lat=%.6f lon=%.6f alt=%.2f",
              out.latitude, out.longitude, out.altitude);
    return true;
}

bool GlobalCoordManager::setOrigin(double lat, double lon, double alt, OriginSource source) {
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)) {
        ALOG_ERROR(MOD, "Invalid origin attempt: lat=%.6f lon=%.6f alt=%.2f", lat, lon, alt);
        return false;
    }
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        ALOG_ERROR(MOD, "Origin coordinates out of range: lat=%.6f lon=%.6f", lat, lon);
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lk(mutex_);
        origin_.latitude = lat;
        origin_.longitude = lon;
        origin_.altitude = alt;
    }
    origin_set_.store(true, std::memory_order_release);
    origin_source_.store(source);
    
    ALOG_INFO(MOD, "ENU origin set: lat=%.6f lon=%.6f alt=%.2f source=%d",
              lat, lon, alt, static_cast<int>(source));
    
    // 重建LocalCartesian
    rebuildLocalCartesian();
    
    // 自动保存到配置文件
    saveToConfig();
    
    return true;
}

bool GlobalCoordManager::saveToConfig() const {
    if (config_path_.empty()) {
        ALOG_WARN(MOD, "config_path empty, cannot save to config");
        return false;
    }
    
    ENUOrigin current_origin;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        current_origin = origin_;
    }
    
    if (!current_origin.isValid()) {
        ALOG_WARN(MOD, "Cannot save invalid origin to config");
        return false;
    }
    
    std::filesystem::path p(config_path_);
    std::error_code ec;
    if (p.has_parent_path() && !std::filesystem::exists(p.parent_path(), ec)) {
        std::filesystem::create_directories(p.parent_path(), ec);
        if (ec) {
            ALOG_ERROR(MOD, "Failed to create directory %s: %s", 
                      p.parent_path().string().c_str(), ec.message().c_str());
            return false;
        }
    }
    
    std::ofstream ofs(config_path_);
    if (!ofs.is_open()) {
        ALOG_ERROR(MOD, "Failed to open config for write: %s", config_path_.c_str());
        return false;
    }
    
    ofs << "# Global ENU Coordinate System Origin\n";
    ofs << "# This file defines the origin of the global ENU coordinate system\n";
    ofs << "# WGS84 coordinates (latitude, longitude, altitude)\n";
    ofs << "coordinate_system = ENU\n";
    ofs << "latitude = " << current_origin.latitude << "\n";
    ofs << "longitude = " << current_origin.longitude << "\n";
    ofs << "altitude = " << current_origin.altitude << "\n";
    ofs.flush();
    
    if (!ofs.good()) {
        ALOG_ERROR(MOD, "Write failed to config: %s", config_path_.c_str());
        return false;
    }
    
    ofs.close();
    ALOG_INFO(MOD, "ENU origin saved to config: %s (lat=%.6f lon=%.6f alt=%.2f)",
              config_path_.c_str(), current_origin.latitude, current_origin.longitude, current_origin.altitude);
    return true;
}

void GlobalCoordManager::rebuildLocalCartesian() const {
    ENUOrigin current_origin;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        current_origin = origin_;
    }
    
    std::lock_guard<std::mutex> lk_cart(cartesian_mutex_);
    cartesian_holder_.lat = current_origin.latitude;
    cartesian_holder_.lon = current_origin.longitude;
    cartesian_holder_.alt = current_origin.altitude;
    cartesian_holder_.proj = std::make_unique<GeographicLib::LocalCartesian>(
        current_origin.latitude, current_origin.longitude, current_origin.altitude);
    
    ALOG_DEBUG(MOD, "LocalCartesian rebuilt: lat=%.6f lon=%.6f alt=%.2f",
               current_origin.latitude, current_origin.longitude, current_origin.altitude);
}

Eigen::Vector3d GlobalCoordManager::wgs84ToENU(double lat, double lon, double alt) const {
    if (!isOriginSet()) {
        ALOG_WARN(MOD, "ENU origin not set, returning zero vector");
        return Eigen::Vector3d::Zero();
    }
    
    // 延迟初始化LocalCartesian
    {
        std::lock_guard<std::mutex> lk_cart(cartesian_mutex_);
        if (!cartesian_holder_.proj) {
            const_cast<GlobalCoordManager*>(this)->rebuildLocalCartesian();
        }
    }
    
    std::lock_guard<std::mutex> lk_cart(cartesian_mutex_);
    double e, n, u;
    try {
        cartesian_holder_.proj->Forward(lat, lon, alt, e, n, u);
    } catch (const GeographicLib::GeographicErr& e) {
        ALOG_ERROR(MOD, "GeographicLib conversion failed: %s", e.what());
        return Eigen::Vector3d::Zero();
    }
    
    return Eigen::Vector3d(e, n, u);  // ENU: East, North, Up
}

Eigen::Vector3d GlobalCoordManager::enuToWGS84(const Eigen::Vector3d& enu) const {
    if (!isOriginSet()) {
        ALOG_WARN(MOD, "ENU origin not set, returning zero vector");
        return Eigen::Vector3d::Zero();
    }
    
    std::lock_guard<std::mutex> lk_cart(cartesian_mutex_);
    if (!cartesian_holder_.proj) {
        const_cast<GlobalCoordManager*>(this)->rebuildLocalCartesian();
    }
    
    double lat, lon, alt;
    try {
        cartesian_holder_.proj->Reverse(enu.x(), enu.y(), enu.z(), lat, lon, alt);
    } catch (const GeographicLib::GeographicErr& e) {
        ALOG_ERROR(MOD, "GeographicLib reverse failed: %s", e.what());
        return Eigen::Vector3d::Zero();
    }
    
    return Eigen::Vector3d(lat, lon, alt);
}

}  // namespace automap_pro
