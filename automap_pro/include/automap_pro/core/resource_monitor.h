#pragma once

#include <string>
#include <fstream>
#include <unistd.h>
#include <iostream>

namespace automap_pro::core {

/**
 * @brief 系统资源监控器 (System Resource Monitor)
 * 🏛️ [架构契约] 监控 CPU/内存/显存，支撑产品化配额协议。
 */
class ResourceMonitor {
public:
    struct MemoryStats {
        size_t total_kb;
        size_t free_kb;
        size_t available_kb;
        size_t used_kb;
        float usage_ratio;
    };

    /**
     * @brief 获取系统内存状态
     */
    static MemoryStats getMemoryStats() {
        MemoryStats stats = {0, 0, 0, 0, 0.0f};
        std::ifstream meminfo("/proc/meminfo");
        std::string line;
        while (std::getline(meminfo, line)) {
            if (line.find("MemTotal:") == 0) {
                stats.total_kb = std::stoull(line.substr(10));
            } else if (line.find("MemFree:") == 0) {
                stats.free_kb = std::stoull(line.substr(10));
            } else if (line.find("MemAvailable:") == 0) {
                stats.available_kb = std::stoull(line.substr(10));
            }
        }
        if (stats.total_kb > 0) {
            stats.used_kb = stats.total_kb - stats.available_kb;
            stats.usage_ratio = static_cast<float>(stats.used_kb) / stats.total_kb;
        }
        return stats;
    }

    /**
     * @brief 获取当前进程内存占用 (RSS)
     */
    static size_t getProcessRssKb() {
        std::ifstream stat_file("/proc/self/status");
        std::string line;
        while (std::getline(stat_file, line)) {
            if (line.find("VmRSS:") == 0) {
                return std::stoull(line.substr(6));
            }
        }
        return 0;
    }

    /**
     * @brief 检查是否处于内存压力状态
     * @param threshold_ratio 触发阈值 (默认 0.9 = 90%)
     */
    static bool isMemoryCritical(float threshold_ratio = 0.9f) {
        auto stats = getMemoryStats();
        return stats.usage_ratio > threshold_ratio;
    }
};

} // namespace automap_pro::core
