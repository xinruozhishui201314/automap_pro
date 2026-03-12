#pragma once

#include <string>
#include <optional>
#include <atomic>
#include <mutex>
#include <memory>

#include <Eigen/Dense>

namespace automap_pro {

/**
 * Global Coordinate Manager - 管理全局ENU坐标系原点
 * 
 * 职责:
 *   1. 优先从配置文件读取ENU原点(lat, lon, alt)
 *   2. 若配置文件不存在,使用第一帧有效GPS作为ENU原点
 *   3. 将ENU原点持久化到配置文件
 *   4. 提供坐标转换接口 (WGS84 <-> ENU)
 */
class GlobalCoordManager {
public:
    struct ENUOrigin {
        double latitude = 0.0;   // 度
        double longitude = 0.0;  // 度
        double altitude = 0.0;   // 米
        
        bool isValid() const {
            return std::isfinite(latitude) && std::isfinite(longitude) && std::isfinite(altitude) &&
                   latitude >= -90.0 && latitude <= 90.0 &&
                   longitude >= -180.0 && longitude <= 180.0;
        }
    };
    
    enum class OriginSource {
        CONFIG_FILE,   // 从配置文件读取
        FIRST_GPS,    // 从第一帧有效GPS获取
        NOT_SET       // 尚未设置
    };
    
    using Ptr = std::shared_ptr<GlobalCoordManager>;
    
    explicit GlobalCoordManager(const std::string& config_path);
    
    /**
     * 初始化: 尝试从配置文件加载ENU原点
     * @return true 如果配置文件存在且有效
     */
    bool loadFromConfig();
    
    /**
     * 设置ENU原点(从GPS)
     * @param lat 纬度(度)
     * @param lon 经度(度)
     * @param alt 高度(米)
     * @param source 来源标记
     * @return true 设置成功
     */
    bool setOrigin(double lat, double lon, double alt, OriginSource source = OriginSource::FIRST_GPS);
    
    /**
     * 保存ENU原点到配置文件
     * @return true 保存成功
     */
    bool saveToConfig() const;
    
    /**
     * 检查ENU原点是否已设置
     */
    bool isOriginSet() const { 
        return origin_set_.load(std::memory_order_acquire); 
    }
    
    /**
     * 获取ENU原点(线程安全)
     */
    ENUOrigin getOrigin() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return origin_;
    }
    
    /**
     * 获取ENU原点来源
     */
    OriginSource getOriginSource() const {
        return origin_source_.load();
    }
    
    /**
     * WGS84转ENU
     * @param lat 纬度(度)
     * @param lon 经度(度)
     * @param alt 高度(米)
     * @return ENU坐标 (East, North, Up) 单位: 米
     */
    Eigen::Vector3d wgs84ToENU(double lat, double lon, double alt) const;
    
    /**
     * ENU转WGS84
     * @param enu ENU坐标
     * @return WGS84 (lat, lon, alt)
     */
    Eigen::Vector3d enuToWGS84(const Eigen::Vector3d& enu) const;
    
    /**
     * 获取配置文件路径
     */
    const std::string& getConfigPath() const { return config_path_; }
    
private:
    std::string config_path_;
    ENUOrigin origin_;
    std::atomic<bool> origin_set_{false};
    std::atomic<OriginSource> origin_source_{OriginSource::NOT_SET};
    mutable std::mutex mutex_;
    
    // GeographicLib LocalCartesian 对象(延迟初始化)
    struct LocalCartesianHolder {
        double lat = 0, lon = 0, alt = 0;
        std::unique_ptr<class LocalCartesian> proj;  // forward declare
        bool needRebuild(double lat, double lon, double alt) const;
    };
    mutable LocalCartesianHolder cartesian_holder_;
    mutable std::mutex cartesian_mutex_;
    
    void rebuildLocalCartesian() const;
};

} // namespace automap_pro
