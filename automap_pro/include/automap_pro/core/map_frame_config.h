#pragma once
/**
 * @file core/map_frame_config.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */


#include <string>
#include <optional>

namespace automap_pro {

/**
 * 地图坐标系配置（东北天 ENU 统一坐标系）
 *
 * 后端统一使用同一地图坐标系（ENU），其原点经纬度写入 .cfg 配置文件：
 * 先创建文件，再写入 latitude / longitude / altitude。
 * 其他模块通过 read() 读取该文件以保持坐标系一致。
 */
namespace MapFrameConfig {

/** 从 .cfg 读取的 ENU 原点结果，供其他模块使用 */
struct ENUOriginResult {
    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;
    double altitude_m    = 0.0;
    /** 三者均有效且在合理范围内时为 true */
    bool valid = false;
};

/** 写入地图坐标系原点到 .cfg 文件：若路径所在目录不存在则先创建，再写入数据 */
bool write(const std::string& cfg_path,
           double latitude_deg, double longitude_deg, double altitude_m);

/**
 * 从 .cfg 读取 ENU 原点，供其他模块与地图坐标系对齐使用。
 * @param cfg_path 配置文件路径（通常来自 ConfigManager::mapFrameConfigPath()）
 * @return 解析成功且经纬高有效时返回 ENUOriginResult（valid=true），否则 nullopt
 */
std::optional<ENUOriginResult> read(const std::string& cfg_path);

}  // namespace MapFrameConfig

}  // namespace automap_pro
