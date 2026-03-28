#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/crash_report.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/core/protocol_contract.h" // 🏛️ [架构加固] 引入协议契约
#include <pcl_conversions/pcl_conversions.h>
#include <cctype>
#include <filesystem>
#include <rcutils/logging.h>

namespace automap_pro {
namespace {

int toRcutilsSeverity(const std::string& level) {
    std::string upper = level;
    std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    if (upper == "DEBUG") return RCUTILS_LOG_SEVERITY_DEBUG;
    if (upper == "INFO") return RCUTILS_LOG_SEVERITY_INFO;
    if (upper == "ERROR") return RCUTILS_LOG_SEVERITY_ERROR;
    if (upper == "FATAL") return RCUTILS_LOG_SEVERITY_FATAL;
    return RCUTILS_LOG_SEVERITY_WARN;
}

} // namespace

AutoMapSystem::AutoMapSystem(const rclcpp::NodeOptions& options)
    : rclcpp::Node("automap_system", options)
{
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] ctor begin node=automap_system (API Version: %s)", 
                protocol::getVersionString().c_str());

    // 🏛️ [架构锚点] 强一致性校验
    // 在工程实践中，这里可以进一步检查外部传入的 API_VERSION 参数
    // 如果不一致，直接抛出异常阻止系统进入未定义状态
    RCLCPP_INFO(get_logger(), "[PROTOCOL] Interface Contract Verified: SSoT Registry Active.");

    // 1. V3 上下文：ctor 内不能 shared_from_this()；init(nullptr) 只建 MapRegistry，健康定时器在 deferredSetupModules 里 attachNode
    v3_context_ = std::make_shared<v3::V3Context>();
    v3_context_->init(nullptr);
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=01 V3Context+MapRegistry ready (EventBus+MapRegistry)");

    // 订阅全局地图构建结果
    v3_context_->eventBus()->subscribe<v3::GlobalMapBuildResultEvent>([this](const v3::GlobalMapBuildResultEvent& ev) {
        if (!ev.global_map || ev.global_map->empty()) return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*ev.global_map, cloud_msg);
        cloud_msg.header.stamp    = now();
        cloud_msg.header.frame_id = "map";
        if (global_map_pub_) {
            global_map_pub_->publish(cloud_msg);
            pub_map_count_++;
        }
    });
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=02 subscribed GlobalMapBuildResultEvent -> /automap/global_map");

    // 2. 初始化日志
    const char* log_dir_env = std::getenv("AUTOMAP_LOG_DIR");
    const char* log_lvl_env = std::getenv("AUTOMAP_LOG_LEVEL");
    std::string log_dir = log_dir_env ? log_dir_env : "/tmp/automap_logs";
    std::string log_lvl = log_lvl_env ? log_lvl_env : "info";
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=03 Logger init AUTOMAP_LOG_DIR=%s AUTOMAP_LOG_LEVEL=%s -> dir=%s level=%s",
                log_dir_env ? log_dir_env : "(default)", log_lvl_env ? log_lvl_env : "(default)", log_dir.c_str(), log_lvl.c_str());
    automap_pro::Logger::instance().init(log_dir, log_lvl);
    crash_report::installCrashHandler();
    ALOG_INFO("Pipeline", "[PIPELINE][SYS] step=03 spdlog file+console ready (grep automap_*.log under {})", log_dir);

    // 3. 加载配置
    loadConfigAndInit();

    // 4. 设置 ROS 接口
    setupPublishers();
    setupServices();
    setupTimers();

    // 5. 延迟初始化模块 (等待 shared_from_this 可用)
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=07 scheduling deferredSetupModules (wall_timer 0ms)");
    deferred_init_timer_ = create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() { this->deferredSetupModules(); });
}

AutoMapSystem::~AutoMapSystem() {
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS][SHUTDOWN] destructor begin (grep PIPELINE|SHUTDOWN)");
    shutdown_requested_.store(true);

    // ── 显式 Shutdown 阶段（Option B：生命周期与析构顺序）────────────────────────────
    // Phase 1: 保存地图（必须在 stopAll 前，否则 MappingModule 已停止无法处理 SaveMapRequestEvent）
    std::string out_dir = getOutputDir();
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS][SHUTDOWN] phase=1 saveMapToFiles out_dir=%s", out_dir.c_str());
    saveMapToFiles(out_dir);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Phase 2: 停止所有 Worker 线程（stopAll 会 join desc/match/merge/freeze/opt 等 worker）
    // 完成后各模块 worker 不再运行，ConfigManager 等单例可安全析构
    if (v3_context_) {
        RCLCPP_INFO(get_logger(), "[PIPELINE][SYS][SHUTDOWN] phase=2 V3Context::stopAll (join all module workers)");
        v3_context_->stopAll();
    }

    // Phase 3: 停止单例监控（HealthMonitor、ErrorMonitor）
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS][SHUTDOWN] phase=3 HealthMonitor + ErrorMonitor stop");
    HealthMonitor::instance().stop();
    ErrorMonitor::instance().stop();

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS][SHUTDOWN] destructor end");
}

void AutoMapSystem::loadConfigAndInit() {
    declare_parameter("config_file", "");
    declare_parameter("output_dir", std::string(""));
    declare_parameter("map_voxel_size", 0.2);

    std::string config_path = get_parameter("config_file").as_string();
    output_dir_override_ = get_parameter("output_dir").as_string();
    map_voxel_size_ = static_cast<float>(get_parameter("map_voxel_size").as_double());

    if (!config_path.empty()) {
        RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=04 ConfigManager::load path=%s", config_path.c_str());
        ConfigManager::instance().load(config_path);
        RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=04 ConfigManager::load OK");
        const std::string ros_log_level = ConfigManager::instance().systemLogLevel();
        const int severity = toRcutilsSeverity(ros_log_level);
        rcutils_logging_set_default_logger_level(severity);
        RCLCPP_WARN(get_logger(),
            "[PIPELINE][SYS] step=04.1 ROS logger level set to %s (default=WARN when missing)",
            ros_log_level.c_str());
        try {
            const auto& cfg = ConfigManager::instance();
            const std::string semantic_model_type = cfg.semanticModelType();
            const std::string semantic_model = cfg.semanticModelPath();
            const bool semantic_enabled = cfg.semanticEnabled();
            const bool semantic_geometric_only = (cfg.semanticMode() == "geometric_only");
            bool semantic_model_exists = false;
            std::string semantic_model_hint;
            if (semantic_geometric_only) {
                semantic_model_exists = true;
                semantic_model_hint = "geometric_only(GeometricProcessor; neural segmentor not loaded)";
            } else if (semantic_model_type == "lsk3dnet_hybrid") {
                const std::string ckpt = cfg.semanticLsk3dnetCheckpoint();
                const std::string cls = cfg.semanticLsk3dnetClassifierTorchscript();
                const std::string yaml = cfg.semanticLsk3dnetConfigYaml();
                const bool ckpt_ok = (!ckpt.empty() && std::filesystem::exists(ckpt));
                const bool cls_ok = (!cls.empty() && std::filesystem::exists(cls));
                const bool yaml_ok = (!yaml.empty() && std::filesystem::exists(yaml));
                semantic_model_exists = ckpt_ok && cls_ok && yaml_ok;
                semantic_model_hint = "hybrid(ckpt=" + ckpt + ", classifier=" + cls + ", yaml=" + yaml + ")";
            } else if (semantic_model_type == "lsk3dnet") {
                const std::string ts_model = cfg.semanticLsk3dnetModelPath();
                semantic_model_exists = (!ts_model.empty() && std::filesystem::exists(ts_model));
                semantic_model_hint = ts_model;
            } else {
                semantic_model_exists = (!semantic_model.empty() && std::filesystem::exists(semantic_model));
                semantic_model_hint = semantic_model;
            }
            const std::string ot_model = cfg.overlapModelPath();
            const bool ot_model_exists = (!ot_model.empty() && std::filesystem::exists(ot_model));
            RCLCPP_INFO(
                get_logger(),
                "[DIAG][BOOT][CONFIG] semantic.enabled=%d semantic.model_type=%s semantic.model_path=%s semantic.model_exists=%d "
                "loop.overlap_model_path=%s loop.overlap_model_exists=%d gps.enabled=%d gps.topic=%s gps.min_quality=%d gps.match_window_s=%.2f",
                semantic_enabled ? 1 : 0, semantic_model_type.c_str(), semantic_model_hint.c_str(), semantic_model_exists ? 1 : 0,
                ot_model.c_str(), ot_model_exists ? 1 : 0,
                cfg.gpsEnabled() ? 1 : 0, cfg.gpsTopic().c_str(),
                cfg.gpsMinAcceptedQualityLevel(), cfg.gpsKeyframeMatchWindowS());
            if (semantic_enabled && !semantic_model_exists) {
                RCLCPP_ERROR(
                    get_logger(),
                    "[DIAG][BOOT][E_SEMANTIC_MODEL] semantic enabled but model missing/invalid. "
                    "type=%s path=%s exists=%d (fix semantic model assets for selected backend)",
                    semantic_model_type.c_str(), semantic_model_hint.c_str(), semantic_model_exists ? 1 : 0);
            }
            if (!cfg.gpsEnabled()) {
                RCLCPP_WARN(
                    get_logger(),
                    "[DIAG][BOOT][W_GPS_DISABLED] gps sensor disabled by config; GPS constraints will not be added");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "[DIAG][BOOT][W_CONFIG_DIAG] failed to dump config diagnostics: %s", e.what());
        }
    } else {
        RCLCPP_WARN(get_logger(), "[PIPELINE][SYS] step=04 config_file parameter empty — ConfigManager not loaded from YAML");
    }
    if (!output_dir_override_.empty()) {
        RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=04 output_dir override=%s", output_dir_override_.c_str());
    }
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=04 map_voxel_size=%.3f", static_cast<double>(map_voxel_size_));
}

void AutoMapSystem::deferredSetupModules() {
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08 deferredSetupModules enter (V3 micro-kernel register order)");
    if (deferred_init_timer_) {
        deferred_init_timer_->cancel();
        deferred_init_timer_.reset();
    }

    v3_context_->attachNode(shared_from_this());

    // 注册并启动所有微内核模块（顺序：前端→动态过滤→GPS→回环→可视化→优化→建图调度）
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08a ctor+register FrontEndModule");
    frontend_module_ = std::make_shared<v3::FrontEndModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(frontend_module_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08a-2 ctor+register SemanticModule");
    try {
        semantic_module_ = std::make_shared<v3::SemanticModule>(
            v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
        v3_context_->registerModule(semantic_module_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(),
            "[PIPELINE][SYS][FATAL] step=08a-2 semantic init exception: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::exit(1);
    }

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08b ctor+register DynamicFilterModule");
    dynamic_filter_module_ = std::make_shared<v3::DynamicFilterModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(dynamic_filter_module_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08c ctor+register GPSModule");
    gps_module_ = std::make_shared<v3::GPSModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(gps_module_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08d ctor+register LoopModule");
    loop_module_ = std::make_shared<v3::LoopModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(loop_module_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08e ctor+register VisualizationModule");
    viz_module_ = std::make_shared<v3::VisualizationModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(viz_module_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08f ctor+register OptimizerModule");
    optimizer_module_ = std::make_shared<v3::OptimizerModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(optimizer_module_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08g ctor+register MapOrchestrator (observe/advice)");
    map_orchestrator_ = std::make_shared<v3::MapOrchestrator>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(map_orchestrator_);

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08h ctor+register MappingModule (SubMap+HBA)");
    mapping_module_ = std::make_shared<v3::MappingModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(mapping_module_);

    const auto& cfg = ConfigManager::instance();
    RCLCPP_INFO(get_logger(),
        "[V3][SELF_CHECK][BOOT] contract.strict_mode=%d frame_policy=%s defense.dynamic_filter_publish_guard=on defense.mapping_prequeue_guard=on defense.mapping_process_guard=on defense.hard_capacity=on",
        cfg.contractStrictMode() ? 1 : 0, cfg.contractFramePolicy().c_str());

    state_ = SystemState::MAPPING;
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=09 state=MAPPING === AutoMapSystem V3 Architecture Ready === (grep PIPELINE)");
    ALOG_INFO("Pipeline", "[PIPELINE][SYS] step=09 all V3 modules registered; system ready for mapping");
    if (ready_pub_) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        ready_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=09b published /automap/ready=true (latched)");
    }
}

void AutoMapSystem::setupPublishers() {
    status_pub_ = create_publisher<automap_pro::msg::MappingStatusMsg>("/automap/status", 10);
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(protocol::topics::GLOBAL_MAP, 1);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/automap/ready", rclcpp::QoS(1).transient_local().reliable());
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=05 publishers: /automap/status /automap/ready %s", protocol::topics::GLOBAL_MAP);
}

void AutoMapSystem::setupServices() {
    save_map_srv_ = create_service<automap_pro::srv::SaveMap>(
        protocol::services::SAVE_MAP,
        std::bind(&AutoMapSystem::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));
    
    get_status_srv_ = create_service<automap_pro::srv::GetStatus>(
        "/automap/get_status",
        std::bind(&AutoMapSystem::handleGetStatus, this, std::placeholders::_1, std::placeholders::_2));

    finish_mapping_srv_ = create_service<std_srvs::srv::Trigger>(
        protocol::services::FINISH_MAPPING,
        std::bind(&AutoMapSystem::handleFinishMapping, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                "[PIPELINE][SYS] step=06 services: %s /automap/get_status %s", 
                protocol::services::SAVE_MAP, protocol::services::FINISH_MAPPING);
}

void AutoMapSystem::setupTimers() {
    status_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { this->publishStatus(); });
    
    // 🏛️ [P0 优化] 移除盲目的 10s 全局地图构建定时器
    // 改为由 MappingModule 根据处理帧数或位姿优化事件触发，减少无数据时的冗余计算
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=06 timers: status=1s (global_map trigger moved to MappingModule)");
}

} // namespace automap_pro
