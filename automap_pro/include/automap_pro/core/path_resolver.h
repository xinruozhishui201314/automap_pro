#pragma once

#include <string>
#include <filesystem>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace automap_pro {

/**
 * @brief 🏛️ [架构演进] 资源路径解析器
 * 解决编译期硬编码 (CMAKE_CURRENT_SOURCE_DIR) 导致的环境迁移问题。
 * 支持动态搜索环境变量、ROS 包路径和当前工作目录。
 */
class PathResolver {
public:
    static std::string resolve(const std::string& raw_path, const std::string& package_name = "automap_pro") {
        if (raw_path.empty()) return "";
        
        std::string path = raw_path;
        
        // 1. 处理 ${CMAKE_CURRENT_SOURCE_DIR} 占位符 (兼容旧配置)
        const std::string cmake_var = "${CMAKE_CURRENT_SOURCE_DIR}";
        size_t pos = path.find(cmake_var);
        if (pos != std::string::npos) {
            std::string root = getProjectRoot(package_name);
            path.replace(pos, cmake_var.size(), root);
        }

        // 2. 处理环境变量 AUTOMAP_MODELS_DIR
        const char* env_models = std::getenv("AUTOMAP_MODELS_DIR");
        if (env_models && !std::filesystem::path(path).is_absolute()) {
            std::filesystem::path p = std::filesystem::path(env_models) / path;
            if (std::filesystem::exists(p)) return p.string();
        }

        // 3. 规范化为绝对路径
        try {
            std::filesystem::path p(path);
            if (p.is_relative()) {
                // 优先检查相对于项目根目录的路径
                std::filesystem::path root_p = std::filesystem::path(getProjectRoot(package_name)) / path;
                if (std::filesystem::exists(root_p)) {
                    p = root_p;
                } else {
                    p = std::filesystem::absolute(p);
                }
            }
            return p.lexically_normal().string();
        } catch (...) {
            return path;
        }
    }

    static std::string getProjectRoot(const std::string& package_name = "automap_pro") {
        // 1. 优先使用环境变量 (容器部署推荐)
        const char* env_root = std::getenv("AUTOMAP_PRO_ROOT");
        if (env_root) return env_root;

        // 2. 尝试从 ROS2 ament 索引获取 share 目录
        try {
            std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
            // 通常源码在 share_dir 的上两级 (针对 colcon symlink-install)
            std::filesystem::path p(share_dir);
            if (std::filesystem::exists(p / "models")) return p.string(); // 如果是 install 后的目录
            
            // 启发式搜索：向上寻找直到发现 models 目录
            for (int i = 0; i < 5; ++i) {
                if (std::filesystem::exists(p / "models")) return p.string();
                p = p.parent_path();
                if (p.empty() || p == "/") break;
            }
        } catch (...) {}

        // 3. 兜底使用当前工作目录
        return std::filesystem::current_path().string();
    }
};

} // namespace automap_pro
