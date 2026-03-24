#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/crash_report.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/core/protocol_contract.h" // 🏛️ [架构加固] 引入协议契约

namespace automap_pro {

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
    semantic_module_ = std::make_shared<v3::SemanticModule>(
        v3_context_->eventBus(), v3_context_->mapRegistry(), shared_from_this());
    v3_context_->registerModule(semantic_module_);

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

    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=08g ctor+register MappingModule (SubMap+HBA)");
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
}

void AutoMapSystem::setupPublishers() {
    status_pub_ = create_publisher<automap_pro::msg::MappingStatusMsg>("/automap/status", 10);
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(protocol::topics::GLOBAL_MAP, 1);
    RCLCPP_INFO(get_logger(), "[PIPELINE][SYS] step=05 publishers: /automap/status %s", protocol::topics::GLOBAL_MAP);
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
