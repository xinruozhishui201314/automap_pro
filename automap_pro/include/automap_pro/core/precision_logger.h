/**
 * @file precision_logger.h
 * @brief 建图精度日志统一格式与宏，便于 grep/脚本解析与后续精度分析
 *
 * 所有精度日志均带 [PRECISION][模块] 前缀，字段建议 key=value 或 CSV 风格以便解析。
 * 使用方式：在对应模块 #include 后调用 PRECISION_LOG_* 或直接 RCLCPP_INFO(..., "[PRECISION][TAG] ...");
 */
#pragma once

namespace automap_pro {

/// 精度日志标签（与日志前缀一致，便于检索）
namespace precision_tags {
constexpr const char* LIO       = "LIO";
constexpr const char* KF        = "KF";
constexpr const char* SUBMAP    = "SUBMAP";
constexpr const char* LOOP      = "LOOP";
constexpr const char* LOOP_CAND = "LOOP_CAND";
constexpr const char* GPS       = "GPS";
constexpr const char* HBA       = "HBA";
constexpr const char* ODOM      = "ODOM";
constexpr const char* OPT       = "OPT";
}  // namespace precision_tags

}  // namespace automap_pro
