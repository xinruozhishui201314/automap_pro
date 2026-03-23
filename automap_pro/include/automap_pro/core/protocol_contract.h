#pragma once

#include <string>
#include <map>

namespace automap_pro::protocol {

/**
 * 🏛️ [架构契约] 系统全局 API 版本号
 * 
 * 规则：
 * 1. MAJOR: 不兼容的修改（修改后必须全部重新编译并同步部署）
 * 2. MINOR: 向后兼容的功能增加
 * 3. PATCH: 内部修复
 */
static constexpr int API_MAJOR = 3;
static constexpr int API_MINOR = 1;
static constexpr int API_PATCH = 0;

// 统一话题名管理 (Single Source of Truth)
namespace topics {
    static constexpr const char* SYNCED_FRAME    = "/automap/synced_frame";
    static constexpr const char* SUBMAP_EVENT    = "/automap/submap_event";
    static constexpr const char* GLOBAL_MAP      = "/automap/global_map";
    static constexpr const char* OPT_RESULT      = "/automap/optimization_result";
}

// 统一服务名管理
namespace services {
    static constexpr const char* FINISH_MAPPING  = "/automap/finish_mapping";
    static constexpr const char* SAVE_MAP        = "/automap/save_map";
    static constexpr const char* RESET_SYSTEM    = "/automap/reset";
}

/**
 * @brief 校验 API 版本是否兼容
 * @return true 兼容, false 不兼容
 */
inline bool isCompatible(int major, int minor) {
    // 强制要求大版本必须一致，小版本允许向上兼容
    return (major == API_MAJOR) && (minor <= API_MINOR);
}

inline std::string getVersionString() {
    return std::to_string(API_MAJOR) + "." + std::to_string(API_MINOR) + "." + std::to_string(API_PATCH);
}

} // namespace automap_pro::protocol
