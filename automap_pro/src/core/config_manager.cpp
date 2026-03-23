#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_code.h"
#include "automap_pro/core/error_monitor.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <filesystem>

namespace automap_pro {

namespace {
// 将 YAML 段转为多行字符串，单段最多 max_lines 行，便于 CONFIG_FILE_RAW 控制体积
void dumpYamlSection(const std::string& section_name, const YAML::Node& node,
    std::vector<std::string>& out_lines, int max_lines = 80) {
    if (!node.IsDefined() || node.IsNull()) return;
    try {
        std::string raw = YAML::Dump(node);
        std::istringstream ss(raw);
        std::string line;
        int n = 0;
        while (n < max_lines && std::getline(ss, line)) {
            out_lines.push_back("  " + line);
            n++;
        }
        if (n >= max_lines && std::getline(ss, line))
            out_lines.push_back("  ... (truncated)");
    } catch (...) {}
}

// 递归导出 YAML 叶子标量为 key=value，Map 继续递归，Sequence 整段 dump 为一行（避免爆炸）
void flattenYamlParams(const YAML::Node& node, const std::string& prefix, std::vector<std::pair<std::string, std::string>>& out) {
    if (!node.IsDefined() || node.IsNull()) return;
    try {
        if (node.IsScalar()) {
            std::string val = node.as<std::string>();
            if (!prefix.empty()) out.push_back({prefix, val});
            return;
        }
        if (node.IsMap()) {
            for (auto it = node.begin(); it != node.end(); ++it) {
                std::string key = it->first.as<std::string>();
                if (key.empty()) continue;
                std::string p = prefix.empty() ? key : (prefix + "." + key);
                flattenYamlParams(it->second, p, out);
            }
            return;
        }
        if (node.IsSequence()) {
            std::string raw = YAML::Dump(node);
            for (char& c : raw) if (c == '\n') c = ' ';
            if (raw.size() > 200) raw = raw.substr(0, 197) + "...";
            out.push_back({prefix, raw});
        }
    } catch (...) {}
}
}  // namespace

void ConfigManager::load(const std::string& yaml_path) {
    try {
        if (yaml_path.empty()) {
            throw std::invalid_argument("ConfigManager::load: yaml_path is empty");
        }

        // 全工程只读一次配置：已加载过则仅允许同路径再次调用（幂等），否则报错
        if (!config_file_path_.empty()) {
            if (config_file_path_ == yaml_path) {
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[ConfigManager] config already loaded from same path, skipping: %s", yaml_path.c_str());
                return;
            }
            std::string err = "ConfigManager::load: config already loaded from '" + config_file_path_
                + "'; cannot load again from '" + yaml_path + "'. Whole process must use single config source.";
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager] %s", err.c_str());
            throw std::runtime_error(err);
        }

        // 检查文件是否存在
        std::ifstream test_file(yaml_path);
        if (!test_file.good()) {
            throw std::runtime_error("Config file not found or not readable: " + yaml_path);
        }
        test_file.close();

        cfg_ = YAML::LoadFile(yaml_path);
        
        if (cfg_.IsNull()) {
            throw std::runtime_error("Config file loaded as null: " + yaml_path);
        }
        
        ALOG_INFO("ConfigManager", "Successfully loaded config from: {}", yaml_path);

        contract_strict_mode_ = get<bool>("contract.strict_mode", false);
        contract_frame_policy_ = get<std::string>("contract.frame_policy", "compat");
        if (contract_frame_policy_ != "compat" && contract_frame_policy_ != "strict_map_only") {
            std::string err = "Invalid contract.frame_policy='" + contract_frame_policy_ +
                "'. Allowed: compat | strict_map_only";
            RCLCPP_FATAL(rclcpp::get_logger("automap_system"), "[PROTOCOL_FATAL] %s", err.c_str());
            throw std::runtime_error(err);
        }

        // 🏛️ [架构契约] Fail-Fast 协议一致性校验
        // 产品化要求：如果配置中显式声明了 API 版本，则必须与二进制文件内核版本兼容
        if (cfg_["system"] && cfg_["system"]["api_version"]) {
            std::string cfg_version = cfg_["system"]["api_version"].as<std::string>();
            int major = 0, minor = 0;
            if (sscanf(cfg_version.c_str(), "%d.%d", &major, &minor) >= 2) {
                if (!protocol::isCompatible(major, minor)) {
                    std::string err = "[PROTOCOL_FATAL] Config API version (" + cfg_version + 
                                     ") is INCOMPATIBLE with System Kernal (v" + protocol::getVersionString() + 
                                     "). Refusing to start to avoid interface mismatch!";
                    RCLCPP_FATAL(rclcpp::get_logger("automap_system"), "%s", err.c_str());
                    throw std::runtime_error(err);
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[PROTOCOL] API Contract Verified: Config v%s <-> System v%s", 
                    cfg_version.c_str(), protocol::getVersionString().c_str());
            } else {
                std::string err = "[PROTOCOL_FATAL] Invalid system.api_version format: '" + cfg_version + "'. Expected 'MAJOR.MINOR'.";
                if (contract_strict_mode_) {
                    RCLCPP_FATAL(rclcpp::get_logger("automap_system"), "%s", err.c_str());
                    throw std::runtime_error(err);
                }
                RCLCPP_WARN(rclcpp::get_logger("automap_system"), "%s Running in COMPAT mode.", err.c_str());
            }
        } else {
            if (contract_strict_mode_) {
                std::string err = "[PROTOCOL_FATAL] strict_mode=true but missing required key: system.api_version";
                RCLCPP_FATAL(rclcpp::get_logger("automap_system"), "%s", err.c_str());
                throw std::runtime_error(err);
            }
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[PROTOCOL] Missing 'system.api_version' in config. Running in COMPAT mode.");
        }

        // ========== 最先读取 sensor.gps 并缓存，避免后续 get()/路径解析导致 enabled 误为 false ==========
        {
            bool s_ok = cfg_["sensor"].IsMap();
            bool g_ok = s_ok && cfg_["sensor"]["gps"].IsDefined() && !cfg_["sensor"]["gps"].IsNull();
            bool e_ok = false;
            bool t_ok = false;
            if (g_ok && cfg_["sensor"]["gps"].IsMap()) {
                e_ok = cfg_["sensor"]["gps"]["enabled"].IsDefined() && !cfg_["sensor"]["gps"]["enabled"].IsNull();
                t_ok = cfg_["sensor"]["gps"]["topic"].IsDefined() && !cfg_["sensor"]["gps"]["topic"].IsNull();
                if (e_ok) {
                    try {
                        sensor_gps_enabled_value_ = cfg_["sensor"]["gps"]["enabled"].as<bool>();
                        sensor_gps_enabled_cached_ = true;
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                            "[ConfigManager][CONFIG_PARAM_FROM_FILE] sensor.gps.enabled as<bool> failed: %s", e.what());
                    }
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ConfigManager][CONFIG_PARAM_FROM_FILE] sensor.IsMap=%d sensor.gps.defined=%d sensor.gps.enabled.defined=%d sensor.gps.topic.defined=%d -> cached_enabled=%s (file=%s)",
                s_ok ? 1 : 0, g_ok ? 1 : 0, e_ok ? 1 : 0, t_ok ? 1 : 0,
                sensor_gps_enabled_cached_ ? (sensor_gps_enabled_value_ ? "true" : "false") : "unset", yaml_path.c_str());
        }

        // ========== [ConfigManager][CONFIG_FILE_RAW] 配置文件原始内容（INFO 即输出，便于在日志中同时看到“文件内容”与“读取后的值”）==========
        try {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ConfigManager][CONFIG_FILE_RAW] ====== config file content (raw, file=%s) ======", yaml_path.c_str());
            const char* sections[] = {"system", "mode", "sensor", "frontend", "keyframe", "gps", "submap", "loop_closure", "backend", "map", "session"};
            const int max_lines_per_section = 35;  // 每段最多行数，控制日志体积
            for (const char* sec : sections) {
                YAML::Node n = cfg_[sec];
                if (!n.IsDefined() || n.IsNull()) continue;
                std::vector<std::string> lines;
                dumpYamlSection(sec, n, lines, max_lines_per_section);
                if (lines.empty()) continue;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[ConfigManager][CONFIG_FILE_RAW] -------- %s --------", sec);
                std::string block;
                for (const auto& l : lines) {
                    block += l + "\n";
                    if (block.size() > 4000) {
                        RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_RAW] %s", block.c_str());
                        block.clear();
                    }
                }
                if (!block.empty())
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_RAW] %s", block.c_str());
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_RAW] -------- end (file=%s) --------", yaml_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_RAW] dump failed: %s", e.what());
        }

        // ========== [ConfigManager][CONFIG_FILE_PARAMS] 文件中所有参数 key=value + flat 缓存（get() 路径失败时回退，见 LOG_ANALYSIS 配置修复）==========
        try {
            std::vector<std::pair<std::string, std::string>> flat;
            flattenYamlParams(cfg_, "", flat);
            flat_params_cache_.clear();
            for (const auto& p : flat) flat_params_cache_[p.first] = p.second;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_PARAMS] ====== all params from file (key=value) total=%zu ======", flat.size());
            std::string line;
            const size_t max_per_line = 20;
            for (size_t i = 0; i < flat.size(); ++i) {
                std::string kv = flat[i].first + "=" + flat[i].second;
                if (line.empty()) line = kv;
                else line += " | " + kv;
                if ((i + 1) % max_per_line == 0 || i == flat.size() - 1) {
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_PARAMS] %s", line.c_str());
                    line.clear();
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_PARAMS] -------- end --------");
            // 从 flat 填充 sensor 话题缓存，保证 lidarTopic()/imuTopic() 与文件一致（即使 cfg_["sensor"]["lidar"] 路径失败）
            auto it_li = flat_params_cache_.find("sensor.lidar.topic");
            auto it_im = flat_params_cache_.find("sensor.imu.topic");
            if (it_li != flat_params_cache_.end() && !it_li->second.empty()) {
                sensor_lidar_topic_value_ = it_li->second;
                sensor_lidar_topic_cached_ = true;
            }
            if (it_im != flat_params_cache_.end() && !it_im->second.empty()) {
                sensor_imu_topic_value_ = it_im->second;
                sensor_imu_topic_cached_ = true;
            }
            // overlap_transformer.model_path：展开 ${CMAKE_CURRENT_SOURCE_DIR} 为配置所在包根目录，并规范为绝对路径
            auto it_mp = flat_params_cache_.find("loop_closure.overlap_transformer.model_path");
            if (it_mp != flat_params_cache_.end() && !it_mp->second.empty()) {
                std::string path = it_mp->second;
                const std::string var = "${CMAKE_CURRENT_SOURCE_DIR}";
                size_t pos = path.find(var);
                if (pos != std::string::npos) {
                    std::string cfg_dir = yaml_path;
                    for (int i = 0; i < 2 && !cfg_dir.empty(); ++i) {
                        size_t s = cfg_dir.find_last_of("/\\");
                        if (s == std::string::npos) break;
                        cfg_dir.resize(s);
                    }
                    path.replace(pos, var.size(), cfg_dir.empty() ? "." : cfg_dir);
                }
                // 若为相对路径，转为基于当前工作目录的绝对路径，避免 CWD 变化导致找不到 .pt
                try {
                    std::filesystem::path p(path);
                    if (p.is_relative()) {
                        p = std::filesystem::absolute(p);
                        path = p.lexically_normal().string();
                    }
                } catch (const std::exception&) { /* 保持原 path */ }
                overlap_model_path_expanded_ = path;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_FILE_PARAMS] flatten failed: %s", e.what());
        }

        // 诊断：直接读取 sensor.gps.topic，便于排查 LivoBridge 收到默认 /gps/fix 的问题
        try {
            YAML::Node s = cfg_["sensor"];
            if (s.IsMap() && s["gps"]) {
                YAML::Node g = s["gps"];
                if (g.IsMap() && g["topic"]) {
                    std::string raw_topic = g["topic"].as<std::string>();
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[ConfigManager][GPS_DIAG] sensor.gps.topic read from YAML = '%s' (file=%s)",
                        raw_topic.c_str(), yaml_path.c_str());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[ConfigManager][GPS_DIAG] sensor.gps.topic key missing in YAML (sensor.gps exists but no 'topic'); gpsTopic() will return default /gps/fix. Check indent/key in %s",
                        yaml_path.c_str());
                }
                if (g.IsMap() && g["enabled"]) {
                    bool raw_enabled = g["enabled"].as<bool>();
                    sensor_gps_enabled_value_ = raw_enabled;
                    sensor_gps_enabled_cached_ = true;
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[ConfigManager][GPS_DIAG] sensor.gps.enabled read from YAML = %s (file=%s); gpsEnabled() uses cached value.",
                        raw_enabled ? "true" : "false", yaml_path.c_str());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[ConfigManager][GPS_DIAG] sensor.gps.enabled key missing in YAML; gpsEnabled() will fall back to get() (default false). Check indent in %s",
                        yaml_path.c_str());
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[ConfigManager][GPS_DIAG] sensor or sensor.gps missing in YAML; gpsTopic() will return default /gps/fix. File: %s",
                    yaml_path.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ConfigManager][GPS_DIAG] exception reading sensor.gps.topic from YAML: %s", e.what());
        }

        // ========== GPS 配置：一次性读入缓存并统一日志（避免 getter 与 DUMP 不一致）==========
        // 优先从已校验的 gps_node (Map) 直接读取，避免 get() 路径解析在重复键/解析顺序下取不到正确节点（见 LOG_ANALYSIS）
        try {
            YAML::Node gps_node = cfg_["gps"];
            if (gps_node && gps_node.IsMap()) {
                auto read_int = [&gps_node, this](const char* key, int default_val) {
                    YAML::Node n = gps_node[key];
                    if (n.IsDefined() && !n.IsNull()) try { return n.as<int>(); } catch (...) {}
                    return get<int>(std::string("gps.") + key, default_val);
                };
                auto read_double = [&gps_node, this](const char* key, double default_val) {
                    YAML::Node n = gps_node[key];
                    if (n.IsDefined() && !n.IsNull()) try { return n.as<double>(); } catch (...) {}
                    return get<double>(std::string("gps.") + key, default_val);
                };
                auto read_bool = [&gps_node, this](const char* key, bool default_val) {
                    YAML::Node n = gps_node[key];
                    if (n.IsDefined() && !n.IsNull()) try { return n.as<bool>(); } catch (...) {}
                    return get<bool>(std::string("gps.") + key, default_val);
                };
                gps_align_min_points_         = read_int("align_min_points", 50);
                gps_align_min_distance_m_     = read_double("align_min_distance_m", 30.0);
                gps_quality_threshold_hdop_   = read_double("quality_threshold_hdop", 2.0);
                gps_align_rmse_threshold_m_   = read_double("align_rmse_threshold_m", 1.5);
                gps_good_samples_needed_      = read_int("good_samples_needed", 30);
                gps_add_constraints_on_align_ = read_bool("add_constraints_on_align", true);
                gps_factor_interval_m_        = read_double("factor_interval_m", 5.0);
                gps_keyframe_match_window_s_  = std::max(0.1, read_double("keyframe_match_window_s", 0.5));
                gps_keyframe_max_hdop_        = read_double("keyframe_max_hdop", 12.0);
                gps_factor_weight_            = read_double("factor_weight", 1.0);
                gps_factor_quality_scale_excellent_ = read_double("factor_quality_scale_excellent", 2.0);
                gps_factor_quality_scale_high_     = read_double("factor_quality_scale_high", 1.0);
                gps_factor_quality_scale_medium_   = read_double("factor_quality_scale_medium", 0.5);
                gps_factor_quality_scale_low_      = read_double("factor_quality_scale_low", 0.25);
                gps_cached_ = true;

                #define _CFG_LOG(fmt, ...) RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[ConfigManager][GPS_CONFIG_DUMP] " fmt, ##__VA_ARGS__)
                _CFG_LOG("====== GPS config (cached once at load) ======");
                _CFG_LOG("align_min_points = %d", gps_align_min_points_);
                _CFG_LOG("align_min_distance_m = %.2f", gps_align_min_distance_m_);
                _CFG_LOG("quality_threshold_hdop = %.2f", gps_quality_threshold_hdop_);
                _CFG_LOG("keyframe_match_window_s = %.2f", gps_keyframe_match_window_s_);
                _CFG_LOG("align_rmse_threshold_m = %.2f", gps_align_rmse_threshold_m_);
                _CFG_LOG("good_samples_needed = %d", gps_good_samples_needed_);
                _CFG_LOG("keyframe_max_hdop = %.2f", gps_keyframe_max_hdop_);
                _CFG_LOG("add_constraints_on_align = %s", gps_add_constraints_on_align_ ? "true" : "false");
                _CFG_LOG("factor_interval_m = %.2f", gps_factor_interval_m_);
                _CFG_LOG("factor_weight = %.2f", gps_factor_weight_);
                _CFG_LOG("factor_quality_scale_excellent = %.2f", gps_factor_quality_scale_excellent_);
                _CFG_LOG("factor_quality_scale_high = %.2f", gps_factor_quality_scale_high_);
                _CFG_LOG("factor_quality_scale_medium = %.2f", gps_factor_quality_scale_medium_);
                _CFG_LOG("factor_quality_scale_low = %.2f", gps_factor_quality_scale_low_);
                _CFG_LOG("===========================================");
                #undef _CFG_LOG

                // 校验：用路径 get() 读回与缓存一致，避免对 gps_node[key] 调用导致 yaml-cpp operator[] on scalar（如重复键/解析顺序）
                auto check = [this](const char* key, double cached, const char* yaml_key) {
                    try {
                        std::string path = std::string("gps.") + yaml_key;
                        double raw = get<double>(path, cached);
                        if (std::abs(raw - cached) > 1e-6) {
                            std::string err = fmt::format(
                                "ConfigManager GPS config mismatch: {} cached={} but get({})={}. Fix config or cache logic.",
                                key, cached, path, raw);
                            RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][GPS_CONFIG_DUMP] %s", err.c_str());
                            throw std::runtime_error(err);
                        }
                    } catch (const std::runtime_error&) { throw; } catch (...) {}
                };
                check("quality_threshold_hdop", gps_quality_threshold_hdop_, "quality_threshold_hdop");
                check("align_min_distance_m", gps_align_min_distance_m_, "align_min_distance_m");
                check("keyframe_match_window_s", gps_keyframe_match_window_s_, "keyframe_match_window_s");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[ConfigManager][GPS_CONFIG_DUMP] No 'gps' section in config; using defaults (gps_cached_=false).");
            }
        } catch (const std::exception& e) {
            gps_cached_ = false;
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[ConfigManager][GPS_CONFIG_DUMP] Failed to load GPS config: %s", e.what());
            throw;
        }
        // ========== GPS 配置缓存结束 ==========

        // ========== system 节关键参数：每键先 YAML 直接读，未定义则 flat 回退，保证 CONFIG_READ_BACK 与文件一致 ==========
        try {
            YAML::Node sys = cfg_["system"];
            if (sys.IsDefined() && !sys.IsNull() && sys.IsMap()) {
                if (sys["frame_queue_max_size"].IsDefined() && !sys["frame_queue_max_size"].IsNull())
                    system_frame_queue_max_size_ = sys["frame_queue_max_size"].as<int>();
                if (sys["ingress_queue_max_size"].IsDefined() && !sys["ingress_queue_max_size"].IsNull())
                    system_ingress_queue_max_size_ = sys["ingress_queue_max_size"].as<int>();
                if (sys["offline_finish_after_bag"].IsDefined() && !sys["offline_finish_after_bag"].IsNull())
                    system_offline_finish_after_bag_ = sys["offline_finish_after_bag"].as<bool>();
            }
            if (!flat_params_cache_.empty()) {
                auto f1 = flat_params_cache_.find("system.frame_queue_max_size");
                auto f2 = flat_params_cache_.find("system.ingress_queue_max_size");
                auto f3 = flat_params_cache_.find("system.offline_finish_after_bag");
                if (f1 != flat_params_cache_.end()) try { system_frame_queue_max_size_ = std::stoi(f1->second); } catch (...) {}
                if (f2 != flat_params_cache_.end()) try { system_ingress_queue_max_size_ = std::stoi(f2->second); } catch (...) {}
                if (f3 != flat_params_cache_.end()) system_offline_finish_after_bag_ = (f3->second == "true" || f3->second == "1");
            }
            system_queue_cached_ = true;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[ConfigManager][CONFIG_SYSTEM_CACHE] frame_queue_max_size=%d ingress_queue_max_size=%d offline_finish_after_bag=%s (与 CONFIG_READ_BACK 一致)",
                system_frame_queue_max_size_, system_ingress_queue_max_size_, system_offline_finish_after_bag_ ? "true" : "false");
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ConfigManager][CONFIG_SYSTEM_CACHE] read failed: %s (getters will use get())", e.what());
        }

        // ========== 性能参数：一次性读入缓存，彻底解决多线程读取 YAML 导致的 SIGSEGV ==========
        try {
            perf_async_global_map_build_   = get<bool>("performance.async_global_map_build", true);
            perf_async_isam2_update_       = get<bool>("performance.async_isam2_update", false);
            perf_parallel_voxel_downsample_ = get<bool>("performance.parallel_voxel_downsample", true);
            perf_parallel_teaser_match_     = get<bool>("performance.parallel_teaser_match", true);
            perf_parallel_teaser_max_inflight_ = get<int>("performance.parallel_teaser_max_inflight", 4);
            perf_max_optimization_queue_size_ = get<int>("performance.max_optimization_queue_size", 64);
            
            RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                "[ConfigManager][PERF_CACHE] async_build=%d async_isam=%d parallel_voxel=%d parallel_teaser=%d parallel_teaser_max_inflight=%d",
                perf_async_global_map_build_, perf_async_isam2_update_, 
                perf_parallel_voxel_downsample_, perf_parallel_teaser_match_, perf_parallel_teaser_max_inflight_);
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][PERF_CACHE] preload failed, using defaults");
        }

        // ========== [ConfigManager][CONFIG_GET_DIAG] get() 与 raw/cache 对比，便于精准定位 key 解析问题 ==========
        try {
            auto L = rclcpp::get_logger("automap_system");
            std::string lidar_get = get<std::string>("sensor.lidar.topic", "/os1_cloud_node1/points");
            std::string lidar_raw = getSensorLidarTopicRaw();
            std::string imu_get  = get<std::string>("sensor.imu.topic", "/imu/imu");
            std::string imu_raw   = getSensorImuTopicRaw();
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_GET_DIAG] sensor.lidar.topic get=%s raw=%s match=%s",
                lidar_get.c_str(), lidar_raw.c_str(), (lidar_raw.empty() ? (lidar_get != "/os1_cloud_node1/points" ? "n/a" : "default") : (lidar_get == lidar_raw ? "yes" : "MISMATCH")));
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_GET_DIAG] sensor.imu.topic get=%s raw=%s match=%s",
                imu_get.c_str(), imu_raw.c_str(), (imu_raw.empty() ? (imu_get != "/imu/imu" ? "n/a" : "default") : (imu_get == imu_raw ? "yes" : "MISMATCH")));
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_GET_DIAG] failed: %s", e.what());
        }

        // ========== 快速修复：离线模式超时配置（EARLY_SHUTDOWN_QUICK_FIX）；显式 IsDefined/!IsNull + flat 回退 ==========
        std::string mode_type = "offline";
        double online_timeout = 10.0;
        double offline_timeout = 7200.0;
        bool mode_ok = false;
        try {
            YAML::Node mode_node = cfg_["mode"];
            if (mode_node.IsDefined() && !mode_node.IsNull() && mode_node.IsMap()) {
                YAML::Node type_node = mode_node["type"];
                if (type_node.IsDefined() && !type_node.IsNull()) {
                    mode_type = type_node.as<std::string>();
                    mode_ok = true;
                }
                YAML::Node on_node = mode_node["online"];
                if (on_node.IsDefined() && on_node.IsMap()) {
                    YAML::Node to = on_node["sensor_idle_timeout_sec"];
                    if (to.IsDefined() && !to.IsNull()) online_timeout = to.as<double>();
                }
                YAML::Node off_node = mode_node["offline"];
                if (off_node.IsDefined() && off_node.IsMap()) {
                    YAML::Node to = off_node["sensor_idle_timeout_sec"];
                    if (to.IsDefined() && !to.IsNull()) offline_timeout = to.as<double>();
                }
            }
            if (!mode_ok && !flat_params_cache_.empty()) {
                auto ft = flat_params_cache_.find("mode.type");
                auto fo = flat_params_cache_.find("mode.offline.sensor_idle_timeout_sec");
                auto fon = flat_params_cache_.find("mode.online.sensor_idle_timeout_sec");
                if (ft != flat_params_cache_.end() && !ft->second.empty()) { mode_type = ft->second; mode_ok = true; }
                if (fo != flat_params_cache_.end()) try { offline_timeout = std::stod(fo->second); } catch (...) {}
                if (fon != flat_params_cache_.end()) try { online_timeout = std::stod(fon->second); } catch (...) {}
            }
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[ConfigManager] mode.* timeout read failed, using defaults (online=10, offline=7200)");
        }
        sensor_idle_timeout_sec_ = (mode_type == "offline") ? offline_timeout : online_timeout;
        if (mode_ok) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CONFIG] %s mode, sensor_idle_timeout = %.1fs", mode_type.c_str(), sensor_idle_timeout_sec_);
        } else {
            sensor_idle_timeout_sec_ = get<double>("system.sensor_idle_timeout_sec", 10.0);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CONFIG] no mode section, sensor_idle_timeout = %.1fs (from system.sensor_idle_timeout_sec)", sensor_idle_timeout_sec_);
        }
        // ========== 快速修复结束 ==========

        // ========== [ConfigManager][CONFIG_READ_BACK] 所有 getter 读回值（key=value，便于与 CONFIG_FILE_PARAMS 精确对比）==========
        try {
            auto L = rclcpp::get_logger("automap_system");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] ====== read-back (values used by code, every getter) ======");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] system.name=%s system.output_dir=%s system.num_threads=%d system.log_level(from get)=%s",
                systemName().c_str(), outputDir().c_str(), numThreads(), get<std::string>("system.log_level", "INFO").c_str());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] system.sensor_idle_timeout_sec=%.1f system.frame_queue_max_size=%zu system.ingress_queue_max_size=%zu system.offline_finish_after_bag=%s",
                sensorIdleTimeoutSec(), frameQueueMaxSize(), ingressQueueMaxSize(), offlineFinishAfterBag() ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] sensor.lidar.topic=%s sensor.imu.topic=%s sensor.gps.enabled=%s sensor.gps.topic=%s sensor.camera.enabled=%s",
                lidarTopic().c_str(), imuTopic().c_str(), gpsEnabled() ? "true" : "false", gpsTopic().c_str(), cameraEnabled() ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] frontend.odom_topic=%s frontend.cloud_topic=%s frontend.cloud_frame=%s",
                fastLivoOdomTopic().c_str(), fastLivoCloudTopic().c_str(), frontendCloudFrame().c_str());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] dynamic_filter.enabled=%s dynamic_filter.shadow_mode=%s dynamic_filter.queue_max_size=%d dynamic_filter.voxel_size=%.2f dynamic_filter.min_static_observations=%d",
                dynamicFilterEnabled() ? "true" : "false",
                dynamicFilterShadowMode() ? "true" : "false",
                dynamicFilterQueueMaxSize(),
                dynamicFilterVoxelSize(),
                dynamicFilterMinStaticObservations());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] dynamic_filter.fault_injection.enabled=%s mode=%s every_n_frames=%d max_count=%d",
                dynamicFilterFaultInjectionEnabled() ? "true" : "false",
                dynamicFilterFaultInjectionMode().c_str(),
                dynamicFilterFaultInjectionEveryNFrames(),
                dynamicFilterFaultInjectionMaxCount());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] keyframe.min_translation=%.2f keyframe.min_rotation_deg=%.1f keyframe.max_interval=%.2f keyframe.retain_cloud_body=%s",
                kfMinTranslation(), kfMinRotationDeg(), kfMaxInterval(), retainCloudBody() ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] submap.max_keyframes=%d submap.match_resolution=%.2f",
                submapMaxKF(), submapMatchRes());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.overlap_threshold=%.2f loop_closure.top_k=%d loop_closure.min_submap_gap=%d loop_closure.min_temporal_gap_s=%.1f",
                overlapThreshold(), loopTopK(), loopMinSubmapGap(), loopMinTemporalGap());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.geo_prefilter_max_distance_m=%.1f loop_closure.geo_prefilter_skip_above_score=%.2f (skip_above: 0=off)",
                loopGeoPrefilterMaxDistanceM(), loopGeoPrefilterSkipAboveScore());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.ot_preferred_flow=%s loop_closure.allow_sc_fallback=%s loop_closure.allow_descriptor_fallback=%s loop_closure.allow_svd_geom_fallback=%s loop_closure.log_effective_flow=%s",
                loopOtPreferredFlow() ? "true" : "false",
                loopAllowScFallback() ? "true" : "false",
                loopAllowDescriptorFallback() ? "true" : "false",
                loopAllowSvdGeomFallback() ? "true" : "false",
                loopLogEffectiveFlow() ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.flow_mode=%s",
                loopFlowMode().c_str());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.auto_enable_scancontext_on_ot_failure=%s loop_closure.zero_accept_warn_consecutive_queries=%d",
                loopAutoEnableScancontextOnOtFailure() ? "true" : "false", loopZeroAcceptWarnConsecutiveQueries());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.teaser: noise_bound=%.2f voxel_size=%.2f max_points=%d min_inlier_ratio=%.2f min_safe_inliers=%d max_rmse_m=%.2f icp_refine=%s",
                teaserNoiseBound(), teaserVoxelSize(), teaserMaxPoints(), teaserMinInlierRatio(), teaserMinSafeInliers(), teaserMaxRMSE(), teaserICPRefine() ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] loop_closure.pose_consistency: max_trans_diff_m=%.1f max_rot_diff_deg=%.1f (0=off)",
                loopPoseConsistencyMaxTransDiffM(), loopPoseConsistencyMaxRotDiffDeg());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] backend.verbose_trace=%s backend.process_every_n=%d backend.publish_global_map_every_n=%d backend.hba.enabled=%s backend.hba.on_finish=%s backend.hba.frontend_idle_trigger_sec=%.1f backend.hba.frontend_idle_min_submaps=%d backend.hba.enable_gtsam_fallback=%s",
                backendVerboseTrace() ? "true" : "false", backendProcessEveryNFrames(), backendPublishGlobalMapEveryNProcessed(), hbaEnabled() ? "true" : "false", hbaOnFinish() ? "true" : "false", hbaFrontendIdleTriggerSec(), hbaFrontendIdleMinSubmaps(), hbaGtsamFallbackEnabled() ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] backend.isam2.relin_thresh=%.4f backend.isam2.relinearize_skip=%d backend.isam2.prior_variance=%.0e",
                isam2RelinThresh(), isam2RelinSkip(), isam2PriorVariance());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] map.voxel_size=%.2f map.frame_config_path=%s",
                mapVoxelSize(), mapFrameConfigPath().c_str());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] gps.align_min_points=%d gps.keyframe_match_window_s=%.2f gps.keyframe_max_hdop=%.2f gps_cached=%s",
                gpsAlignMinPoints(), gpsKeyframeMatchWindowS(), gpsKeyframeMaxHdop(), gps_cached_ ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] gps.min_accepted_quality_level=%d (0=INVALID..4=EXCELLENT)",
                gpsMinAcceptedQualityLevel());
            {
                Eigen::Vector3d la = gpsLeverArmImu();
                RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] gps.lever_arm_imu=[%.4f, %.4f, %.4f] (HBA GPS antenna vs IMU, m)",
                    la.x(), la.y(), la.z());
            }
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] performance.async_global_map_build=%s performance.async_isam2_update=%s",
                get<bool>("performance.async_global_map_build", true) ? "true" : "false", get<bool>("performance.async_isam2_update", false) ? "true" : "false");
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] performance.parallel_teaser_match=%s performance.parallel_teaser_max_inflight=%d",
                parallelTeaserMatch() ? "true" : "false", parallelTeaserMaxInflight());
            RCLCPP_INFO(L, "[ConfigManager][CONFIG_READ_BACK] =========================================== (file=%s)", yaml_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][CONFIG_READ_BACK] dump failed: %s", e.what());
        }

        config_file_path_ = yaml_path;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[ConfigManager] Single config source set: %s (frontend/backend use ConfigManager::instance() only)", yaml_path.c_str());

    } catch (const YAML::BadFile& e) {
        std::string err = fmt::format("YAML bad file '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_LOAD_FAILED, err));
        throw std::runtime_error(err);
    } catch (const YAML::ParserException& e) {
        std::string err = fmt::format("YAML parse error in '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_PARSE_ERROR, err));
        throw std::runtime_error(err);
    } catch (const YAML::Exception& e) {
        std::string err = fmt::format("YAML exception loading '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_LOAD_FAILED, err));
        throw std::runtime_error(err);
    } catch (const std::exception& e) {
        std::string err = fmt::format("Failed to load config '{}': {}", yaml_path, e.what());
        ALOG_ERROR("ConfigManager", "{}", err);
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] %s", err.c_str());
        ErrorMonitor::instance().recordException(e, errors::CONFIG_LOAD_FAILED);
        throw;
    }
}

namespace {
Eigen::Vector3d readVector3d(const YAML::Node& cfg, const std::string& key, double dx, double dy, double dz) {
    try {
        YAML::Node node = cfg;
        std::istringstream ss(key);
        std::string token;
        bool found = true;
        while (std::getline(ss, token, '.')) {
            if (!node.IsMap() || !node[token].IsDefined() || node[token].IsNull()) {
                found = false;
                break;
            }
            node = node[token];
        }
        if (found && node.IsSequence() && node.size() >= 3) {
            return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
        }
    } catch (...) {}
    return Eigen::Vector3d(dx, dy, dz);
}
}  // namespace

std::string ConfigManager::getSensorGpsTopicRaw() const {
    try {
        YAML::Node s = cfg_["sensor"];
        if (!s.IsMap() || !s["gps"]) return "";
        YAML::Node g = s["gps"];
        if (!g.IsMap() || !g["topic"]) return "";
        return g["topic"].as<std::string>();
    } catch (...) {
        return "";
    }
}

std::string ConfigManager::getSensorLidarTopicRaw() const {
    try {
        YAML::Node s = cfg_["sensor"];
        if (!s.IsMap() || !s["lidar"]) return "";
        YAML::Node n = s["lidar"];
        if (!n.IsMap() || !n["topic"]) return "";
        return n["topic"].as<std::string>();
    } catch (...) {
        return "";
    }
}

std::string ConfigManager::getSensorImuTopicRaw() const {
    try {
        YAML::Node s = cfg_["sensor"];
        if (!s.IsMap() || !s["imu"]) return "";
        YAML::Node n = s["imu"];
        if (!n.IsMap() || !n["topic"]) return "";
        return n["topic"].as<std::string>();
    } catch (...) {
        return "";
    }
}

std::string ConfigManager::lidarTopic() const {
    if (sensor_lidar_topic_cached_ && !sensor_lidar_topic_value_.empty())
        return sensor_lidar_topic_value_;
    const std::string default_val("/os1_cloud_node1/points");
    std::string from_get = get<std::string>("sensor.lidar.topic", default_val);
    if (from_get != default_val) return from_get;
    std::string raw = getSensorLidarTopicRaw();
    if (!raw.empty()) return raw;
    return default_val;
}

std::string ConfigManager::imuTopic() const {
    if (sensor_imu_topic_cached_ && !sensor_imu_topic_value_.empty())
        return sensor_imu_topic_value_;
    const std::string default_val("/imu/imu");
    std::string from_get = get<std::string>("sensor.imu.topic", default_val);
    if (from_get != default_val) return from_get;
    std::string raw = getSensorImuTopicRaw();
    if (!raw.empty()) return raw;
    return default_val;
}

bool ConfigManager::gpsEnabled() const {
    if (sensor_gps_enabled_cached_)
        return sensor_gps_enabled_value_;
    try {
        YAML::Node s = cfg_["sensor"];
        if (s.IsMap() && s["gps"]) {
            YAML::Node g = s["gps"];
            if (g.IsMap() && g["enabled"]) {
                return g["enabled"].as<bool>();
            }
        }
    } catch (...) {}
    return get<bool>("sensor.gps.enabled", false);
}

std::string ConfigManager::gpsTopic() const {
    const std::string default_val("/gps/fix");
    std::string from_get = get<std::string>("sensor.gps.topic", default_val);
    if (from_get != default_val) return from_get;
    std::string raw = getSensorGpsTopicRaw();
    if (!raw.empty()) return raw;
    return default_val;
}

std::string ConfigManager::overlapModelPath() const {
    if (!overlap_model_path_expanded_.empty()) return overlap_model_path_expanded_;
    // 与 sensor 话题一致：get() 可能因路径解析返回默认值，优先从 flat 缓存取原始值再展开
    std::string path;
    auto it = flat_params_cache_.find("loop_closure.overlap_transformer.model_path");
    if (it != flat_params_cache_.end() && !it->second.empty()) {
        path = it->second;
    } else {
        path = get<std::string>("loop_closure.overlap_transformer.model_path",
                                "${CMAKE_CURRENT_SOURCE_DIR}/models/overlapTransformer.pt");
    }
    if (path.empty()) return path;
    const std::string var = "${CMAKE_CURRENT_SOURCE_DIR}";
    size_t pos = path.find(var);
    if (pos != std::string::npos) {
        std::string cfg_dir = config_file_path_.empty() ? "." : config_file_path_;
        if (!config_file_path_.empty()) {
            for (int i = 0; i < 2 && !cfg_dir.empty(); ++i) {
                size_t s = cfg_dir.find_last_of("/\\");
                if (s == std::string::npos) break;
                cfg_dir.resize(s);
            }
        }
        path.replace(pos, var.size(), cfg_dir.empty() ? "." : cfg_dir);
    }
    return path;
}

Eigen::Vector3d ConfigManager::gpsCovExcellent() const {
    return readVector3d(cfg_, "gps.cov_excellent", 0.5, 0.5, 1.0);
}
Eigen::Vector3d ConfigManager::gpsCovHigh() const {
    return readVector3d(cfg_, "gps.cov_high", 1.0, 1.0, 2.0);
}
Eigen::Vector3d ConfigManager::gpsCovMedium() const {
    return readVector3d(cfg_, "gps.cov_medium", 2.0, 2.0, 4.0);
}
Eigen::Vector3d ConfigManager::gpsCovLow() const {
    return readVector3d(cfg_, "gps.cov_low", 4.0, 4.0, 8.0);
}

bool ConfigManager::gpsEnuOriginConfigured() const {
    try {
        YAML::Node g = cfg_["gps"];
        if (!g.IsMap()) return false;
        YAML::Node o = g["enu_origin"];
        return o.IsSequence() && o.size() >= 3;
    } catch (...) {
        return false;
    }
}

Eigen::Vector3d ConfigManager::gpsEnuOrigin() const {
    if (!gpsEnuOriginConfigured()) {
        return Eigen::Vector3d::Zero();
    }
    return readVector3d(cfg_, "gps.enu_origin", 0.0, 0.0, 0.0);
}

Eigen::Vector3d ConfigManager::gpsLeverArmImu() const {
    return readVector3d(cfg_, "gps.lever_arm_imu", 0.0, 0.0, 0.0);
}

std::vector<std::string> ConfigManager::previousSessionDirs() const {
    std::vector<std::string> dirs;
    try {
        auto node = cfg_["session"]["previous_session_dirs"];
        if (node && node.IsSequence()) {
            for (const auto& item : node) {
                if (item.IsScalar()) {
                    std::string dir = item.as<std::string>();
                    if (!dir.empty()) {
                        dirs.push_back(dir);
                    }
                }
            }
        }
    } catch (const YAML::Exception& e) {
        ALOG_WARN("ConfigManager", "Failed to read previous_session_dirs: {}", e.what());
        RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] previous_session_dirs: %s", e.what());
        ErrorMonitor::instance().recordError(ErrorDetail(errors::CONFIG_PARSE_ERROR, std::string("previous_session_dirs: ") + e.what()));
    } catch (const std::exception& e) {
        ALOG_WARN("ConfigManager", "Exception reading previous_session_dirs: {}", e.what());
        RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[ConfigManager][EXCEPTION] previous_session_dirs: %s", e.what());
        ErrorMonitor::instance().recordException(e, errors::CONFIG_KEY_NOT_FOUND);
    }
    return dirs;
}

} // namespace automap_pro
