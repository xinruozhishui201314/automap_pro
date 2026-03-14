#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/crash_report.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/backend/pose_graph.h"
#include <unistd.h>
#include <sys/syscall.h>
#ifdef __linux__
#include <pthread.h>
#endif
#define MOD "AutoMapSystem"

// 辅助函数：获取当前墙钟时间戳（毫秒）
static inline int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <filesystem>
#include <chrono>
#include <memory>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;
namespace automap_pro {

// P3: 关键帧ID编码常量
// 用于区分子图级节点和关键帧级节点：
//   - 子图级节点: id < MAX_KF_PER_SUBMAP (0, 1, 2, ...)
//   - 关键帧级节点: id >= MAX_KF_PER_SUBMAP (submap_id * MAX_KF_PER_SUBMAP + keyframe_index)
// 使用 100000 作为阈值，支持每个子图最多10万个关键帧
constexpr int MAX_KF_PER_SUBMAP = 100000;

// ─────────────────────────────────────────────────────────────────────────────
// 构造与初始化
// ─────────────────────────────────────────────────────────────────────────────
AutoMapSystem::AutoMapSystem(const rclcpp::NodeOptions& options)
    : rclcpp::Node("automap_system", options)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 0: Constructor entered");

    // 初始化日志系统（最先执行）
    const char* log_dir_env = std::getenv("AUTOMAP_LOG_DIR");
    const char* log_lvl_env = std::getenv("AUTOMAP_LOG_LEVEL");
    std::string log_dir = log_dir_env ? log_dir_env : "/tmp/automap_logs";
    std::string log_lvl = log_lvl_env ? log_lvl_env : "info";
    automap_pro::Logger::instance().init(log_dir, log_lvl);

    ALOG_INFO(MOD, "=== AutoMapSystem v2.0 starting ===");
    ALOG_INFO(MOD, "Build type={} log_dir={} log_level={}", CMAKE_BUILD_TYPE_STR, log_dir, log_lvl);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 1: Logger inited, log_dir=%s log_level=%s", log_dir.c_str(), log_lvl.c_str());
    crash_report::installCrashHandler();

    loadConfigAndInit();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 2: loadConfigAndInit() done");

    setupPublishers();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 3: setupPublishers() done");

    setupServices();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 4: setupServices() done");

    setupTimers();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 5: setupTimers() done");

    // 将依赖 shared_from_this() 的 setupModules 延后到首次 spin 后执行，避免 Composable Node 构造时 bad_weak_ptr
    deferred_init_timer_ = create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() { this->deferredSetupModules(); });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 6: deferred setupModules scheduled (will run after first spin)");

    // 提前启动 feeder、backend worker、地图发布、回环 iSAM2、可视化、状态发布线程（均异步，有界队列防阻塞/死锁）
    feeder_thread_ = std::thread(&AutoMapSystem::feederLoop, this);
    backend_worker_ = std::thread(&AutoMapSystem::backendWorkerLoop, this);
    map_publish_thread_ = std::thread(&AutoMapSystem::mapPublishLoop, this);
    loop_opt_thread_ = std::thread(&AutoMapSystem::loopOptThreadLoop, this);
    viz_thread_ = std::thread(&AutoMapSystem::vizThreadLoop, this);
    status_publisher_thread_ = std::thread(&AutoMapSystem::statusPublisherLoop, this);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 7: feeder + backend + map_publish + loop_opt + viz + status_pub threads started");
}

AutoMapSystem::~AutoMapSystem() {
    // 固化顺序：先请求退出 → 唤醒所有 cv → join 6 个工作线程 → loop_detector.stop → saveMap → hba.stop → clearForShutdown；clearForShutdown 之后禁止再触碰 isam2_
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=1] destructor entered, requesting backend exit");
    shutdown_requested_.store(true, std::memory_order_release);
    frame_queue_cv_.notify_all();
    frame_queue_not_full_cv_.notify_all();
    ingress_not_empty_cv_.notify_all();   // 唤醒 feeder（可能正等 ingress 有数据）
    ingress_not_full_cv_.notify_all();    // 唤醒可能正在等 ingress 有空间的 onCloud
    if (feeder_thread_.joinable()) {
        feeder_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2a] feeder thread joined");
    }
    if (backend_worker_.joinable()) {
        backend_worker_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2] backend worker joined");
    }
    map_publish_pending_.store(false);
    map_publish_cv_.notify_all();
    if (map_publish_thread_.joinable()) {
        map_publish_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2b] map_publish thread joined");
    }
    loop_opt_cv_.notify_all();
    if (loop_opt_thread_.joinable()) {
        loop_opt_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2c] loop_opt thread joined");
    }
    viz_cv_.notify_all();
    if (viz_thread_.joinable()) {
        viz_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2d] viz thread joined");
    }
    status_pub_cv_.notify_all();
    if (status_publisher_thread_.joinable()) {
        status_publisher_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2e] status_publisher thread joined");
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=3] calling loop_detector_.stop()");
    loop_detector_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=4] loop_detector_.stop() done");

    // 优先保存点云地图，确保 Ctrl+C 后一定能输出（此前先 stop HBA 再 triggerAsync(wait=true) 导致
    // 工作线程已退出、队列无人消费而永久阻塞，进程被 SIGKILL，地图从未保存）
    if (submap_manager_.submapCount() > 0) {
        try {
            std::string out_dir = getOutputDir();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=5a] saveMapToFiles enter output_dir=%s submaps=%d", out_dir.c_str(), submap_manager_.submapCount());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save output_dir=%s submaps=%d", out_dir.c_str(), submap_manager_.submapCount());
            saveMapToFiles(out_dir);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Auto-saved backend map to %s on shutdown", out_dir.c_str());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=5b] saveMapToFiles done");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save_done success=1");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Auto-save map on shutdown failed: %s", e.what());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=5b] saveMapToFiles exception");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save_done success=0 error=%s", e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Auto-save map on shutdown failed: unknown exception");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=5b] saveMapToFiles unknown exception");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save_done success=0 error=unknown");
        }
    } else {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=5] skip save (submap_count=0)");
    }

    // 关闭时不再触发最终 HBA，避免 stop() 等待工作线程跑完长时间优化导致进程被 SIGKILL、地图已在上方保存
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=6] hba_optimizer_.stop() enter");
    if (ConfigManager::instance().hbaOnFinish()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_skip_final_hba (map already saved, exit quickly)");
    }
    hba_optimizer_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=7] hba_optimizer_.stop() done");

    // 显式清空 iSAM2 因子图与状态并在 clearForShutdown 内释放 prior_noise_，避免析构阶段 double-free (SIGSEGV)
    isam2_optimizer_.clearForShutdown();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=8] isam2_optimizer_.clearForShutdown() done (prior_noise_ already released)");

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=9] Destructor body done. Member destructors next: RvizPublisher -> HBAOptimizer -> IncrementalOptimizer -> LoopDetector -> SubMapManager -> ...");
}

void AutoMapSystem::loadConfigAndInit() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][CONFIG] Declaring parameter config_file");
    declare_parameter("config_file", "");
    declare_parameter("output_dir", std::string(""));
    declare_parameter("trajectory_log_enable", true);
    declare_parameter("trajectory_log_after_mapping_only", true);
    declare_parameter("trajectory_log_dir", std::string(""));
    std::string config_path = get_parameter("config_file").as_string();
    output_dir_override_ = get_parameter("output_dir").as_string();
    while (!output_dir_override_.empty() && output_dir_override_.back() == '/') output_dir_override_.pop_back();
    if (!output_dir_override_.empty()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] output_dir from launch: %s (前后端地图将保存到此目录)", output_dir_override_.c_str());
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] config_file param=%s", config_path.empty() ? "(empty)" : config_path.c_str());
    if (!config_path.empty()) {
        ConfigManager::instance().load(config_path);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] Config loaded from %s", config_path.c_str());
        gps_enabled_from_config_ = ConfigManager::instance().gpsEnabled();
        gps_topic_from_config_  = ConfigManager::instance().gpsTopic();
        // M2DGR 数据集 bag 发布 /ublox/fix (NavSatFix)，若配置未正确读到 topic 则兜底
        const bool path_looks_m2dgr = (config_path.find("M2DGR") != std::string::npos);
        if (path_looks_m2dgr && gps_topic_from_config_ == "/gps/fix") {
            gps_topic_from_config_ = "/ublox/fix";
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem][CONFIG][GPS_DIAG] M2DGR config detected but sensor.gps.topic was default; using /ublox/fix (match bag topic). Check YAML sensor.gps.topic if this is unexpected.");
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] sensor.gps.enabled=%s sensor.gps.topic=%s (if trajectory CSV has no GPS, ensure enabled=true and bag publishes this topic)",
                    gps_enabled_from_config_ ? "true" : "false", gps_topic_from_config_.c_str());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_DIAG] config_path=%s gps_topic=%s (M2DGR bag: /ublox/fix; grep LivoBridge\\[GPS\\] for first message)",
                    config_path.c_str(), gps_topic_from_config_.c_str());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] backend.hba.enable_gtsam_fallback=%s (HBA GTSAM fallback; false=skip HBA when no hba_api, grep HBA CONFIG to verify)",
                    ConfigManager::instance().hbaGtsamFallbackEnabled() ? "true" : "false");
    } else {
        gps_enabled_from_config_ = false;
        gps_topic_from_config_   = "/gps/fix";
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][CONFIG] No config file specified, using defaults (sensor.gps.enabled=false). Pass config:=path/to/system_config_M2DGR.yaml for GPS.");
    }
    trajectory_log_enabled_ = get_parameter("trajectory_log_enable").as_bool();
    trajectory_log_after_mapping_only_ = get_parameter("trajectory_log_after_mapping_only").as_bool();
    trajectory_log_dir_    = get_parameter("trajectory_log_dir").as_string();
    if (trajectory_log_dir_.empty()) {
        const char* env_dir = std::getenv("AUTOMAP_LOG_DIR");
        trajectory_log_dir_ = env_dir ? env_dir : "logs";
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] trajectory_log_enable=%d trajectory_log_after_mapping_only=%d trajectory_log_dir=%s",
                trajectory_log_enabled_ ? 1 : 0, trajectory_log_after_mapping_only_ ? 1 : 0, trajectory_log_dir_.c_str());
    if (trajectory_log_enabled_ && trajectory_log_after_mapping_only_) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][CONFIG] trajectory_odom will be written only at save (keyframe+GPS, map frame); use the CSV in save output_dir for trajectory-GPS comparison.");
    }

    // 缓冲：帧队列长度与空闲超时，计算跟不上时可增大队列、拉长超时，允许“算慢一点”
    max_frame_queue_size_ = ConfigManager::instance().frameQueueMaxSize();
    max_ingress_queue_size_ = ConfigManager::instance().ingressQueueMaxSize();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] frame_queue_max_size=%zu ingress_queue_max_size=%zu sensor_idle_timeout_sec=%.1f (back-pressure in feeder thread, no drop)",
                max_frame_queue_size_, max_ingress_queue_size_, ConfigManager::instance().sensorIdleTimeoutSec());
}

void AutoMapSystem::setupModules() {
    // 保留空实现，实际逻辑在 deferredSetupModules() 中
}

void AutoMapSystem::deferredSetupModules() {
    if (deferred_init_timer_) {
        deferred_init_timer_->cancel();
        deferred_init_timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 1: deferredSetupModules entered (shared_from_this now valid)");

    // 启动错误监控（滑动窗口统计 + 告警阈值）
    if (!ErrorMonitor::instance().isRunning()) {
        ErrorMonitor::instance().start();
    }

    // 健康监控与降级：资源/队列超阈值时触发降级回调，临时增大 process_every_n 减负
    if (!HealthMonitor::instance().isRunning()) {
        HealthMonitorConfig hm_cfg;
        hm_cfg.check_interval_sec = 10.0;
        hm_cfg.heartbeat_interval_sec = 5.0;
        hm_cfg.memory_critical_threshold_mb = 8192.0;
        hm_cfg.queue_size_critical_threshold = 500;
        HealthMonitor::instance().init(shared_from_this(), hm_cfg);
        HealthMonitor::instance().registerChecker(std::make_shared<ResourceHealthChecker>(hm_cfg));
        HealthMonitor::instance().registerDegradationCallback(
            [this](const HealthReport& report) {
                process_every_n_override_.store(10, std::memory_order_release);
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][DEGRADATION] Health state critical/degraded (summary=%s), setting process_every_n_override=10 to reduce load",
                            report.summary.c_str());
            });
        HealthMonitor::instance().registerRecoveryCallback(
            [this](const HealthReport&) {
                process_every_n_override_.store(0, std::memory_order_release);
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][RECOVERY] Health recovered, clearing process_every_n_override");
            });
        HealthMonitor::instance().start();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] HealthMonitor started (degradation callback registered)");
    }

    const auto& cfg = ConfigManager::instance();
    map_voxel_size_ = static_cast<float>(cfg.mapVoxelSize());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] map_voxel_size=%.3f", map_voxel_size_);

    // 分配新 session ID
    current_session_id_ = static_cast<uint64_t>(
        std::chrono::system_clock::now().time_since_epoch().count());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 2: session_id=%lu", current_session_id_);

    // 子图管理器
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 3a: init SubMapManager");
    submap_manager_.init(shared_from_this());
    submap_manager_.startNewSession(current_session_id_);
    submap_manager_.registerSubmapFrozenCallback(
        [this](const SubMap::Ptr& sm) { onSubmapFrozen(sm); });

    // 综合可视化（前端/后端/回环/GPS/HBA）
    rviz_publisher_.init(shared_from_this());
    rviz_publisher_.setFrameId("map");
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 3b: SubMapManager inited and session started");

    // 回环检测器
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 4a: init LoopDetector");
    loop_detector_.init(shared_from_this());
    loop_detector_.registerLoopCallback(
        [this](const LoopConstraint::Ptr& lc) { onLoopDetected(lc); });
    loop_detector_.start();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 4b: LoopDetector inited and started");

    // iSAM2 优化器（位姿更新后触发延迟 GPS 补偿）
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 5: register iSAM2 pose callback");
    isam2_optimizer_.registerPoseUpdateCallback(
        [this](const std::unordered_map<int, Pose3d>& poses) {
            onPoseUpdated(poses);
            if (gps_compensator_ && gps_compensator_->isEnabled()) {
                gps_compensator_->onPoseOptimized(poses);
            }
        });

    // HBA 优化器
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 6a: init HBAOptimizer");
    hba_optimizer_.init();
    hba_optimizer_.registerDoneCallback(
        [this, weak_this = weak_from_this()](const HBAResult& result) {
            auto shared_this = weak_this.lock();
            if (!shared_this || shutdown_requested_.load(std::memory_order_acquire)) {
                return;
            }
            onHBADone(result);
        });
    hba_optimizer_.start();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 6c: HBAOptimizer started");

    // GPS 管理器：config 在 Node 构造时已加载，但 GPSManager 在 config 加载前就已构造，需在此重新应用 YAML 参数
    gps_manager_.applyConfig();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7: register GPSManager callbacks");
    gps_manager_.registerAlignCallback(
        [this](const GPSAlignResult& r) {
            onGPSAligned(r);
            if (gps_compensator_ && gps_compensator_->isEnabled()) {
                gps_compensator_->onGPSAligned(r);
                gps_compensator_->collectHistoricalSubmaps(submap_manager_.getAllSubmaps());
            }
        });
    gps_manager_.registerMeasurementLogCallback(
        [this](double ts, const Eigen::Vector3d& pos_enu) { onGPSMeasurementForLog(ts, pos_enu); });
    // GPS 对齐后，每次收到合格 GPS 都在后端优化中添加 GPS 约束（绑定到时间最近的子图）
    // 异步 GPS 对齐：达到条件时不在此线程执行 SVD，由 backend worker 在 runScheduledAlignment() 中执行，避免阻塞 ROS 回调
    gps_manager_.setAlignScheduler([]() { /* 仅标记 pending，backend 每轮循环会调用 runScheduledAlignment() */ });

    gps_manager_.registerGpsFactorCallback(
        [this](double ts, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov) {
            if (!gps_aligned_) return;
            auto submaps = submap_manager_.getFrozenSubmaps();
            if (submaps.empty()) return;

            // 找到时间最近且空间距离也在阈值内的子图
            int best_id = -1;
            double best_dt = 1e9;
            double best_dist = 1e9;
            const double max_bind_dt = 30.0;
            const double max_bind_dist = 100.0;  // 空间距离阈值（米）

            for (const auto& sm : submaps) {
                // 计算时间差
                double dt_end = std::abs(sm->t_end - ts);
                double dt_start = std::abs(sm->t_start - ts);
                double dt = std::min(dt_end, dt_start);

                // 计算空间距离（GPS位置到子图锚点位置）
                Eigen::Vector3d sm_center = sm->pose_w_anchor.translation();
                double spatial_dist = (pos - sm_center).norm();

                // 同时考虑时间和空间距离
                if (dt < best_dt && spatial_dist < max_bind_dist) {
                    best_dt = dt;
                    best_dist = spatial_dist;
                    best_id = sm->id;
                } else if (dt < best_dt * 0.5 && spatial_dist < best_dist) {
                    // 时间差特别小（小于当前最佳的一半），优先考虑
                    best_dt = dt;
                    best_dist = spatial_dist;
                    best_id = sm->id;
                }
            }

            // 更严格的绑定条件：时间<30s 且 空间<100m
            if (best_id >= 0 && best_dt <= max_bind_dt) {
                RCLCPP_DEBUG(get_logger(),
                    "[AutoMapSystem][GPS_BIND] sm_id=%d dt=%.2fs dist=%.2fm (thresh: dt=%.0fs dist=%.0fm)",
                    best_id, best_dt, best_dist, max_bind_dt, max_bind_dist);
                if (ConfigManager::instance().asyncIsam2Update()) {
                    IncrementalOptimizer::OptimTask t;
                    t.type = IncrementalOptimizer::OptimTaskType::GPS_FACTOR;
                    t.from_id = best_id;
                    t.gps_pos = pos;
                    t.gps_cov = cov;
                    isam2_optimizer_.enqueueOptTask(t);
                } else {
                    isam2_optimizer_.addGPSFactor(best_id, pos, cov);
                }
            } else {
                RCLCPP_DEBUG(get_logger(),
                    "[AutoMapSystem][GPS_BIND] No suitable submap: best_dt=%.2fs best_dist=%.2fm (thresh: dt=%.0fs dist=%.0fm)",
                    best_dt, best_dist, max_bind_dt, max_bind_dist);
            }
        });

    // 延迟 GPS 补偿器（对齐后为历史帧与回环帧补充 GPS 约束）
    gps_compensator_ = std::make_unique<DelayedGPSCompensator>();
    gps_compensator_->setSubmapGPSQueryCallback(
        [this](int sm_id) -> std::optional<GPSMeasurement> {
            auto sm = submap_manager_.getSubmap(sm_id);
            if (!sm || sm->keyframes.empty()) return std::nullopt;
            double ts = sm->t_start;
            return gps_manager_.queryByTimestampEnhanced(ts);
        });
    gps_compensator_->registerGpsFactorCallback(
        [this](int sm_id, const Eigen::Vector3d& pos_map, const Eigen::Matrix3d& cov, bool /*is_compensated*/) {
            isam2_optimizer_.addGPSFactor(sm_id, pos_map, cov);
            return true;
        });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7a: DelayedGPSCompensator inited");

    if (gps_enabled_from_config_ && cfg.gpsAttitudeEstimationEnable()) {
        AttitudeEstimator::Config att_cfg;
        att_cfg.imu_lowpass_cutoff_hz = cfg.gpsAttitudeImuLowpassHz();
        att_cfg.gravity_norm = cfg.imuGravity();
        att_cfg.min_velocity_for_yaw = cfg.gpsAttitudeMinVelocityForYaw();
        att_cfg.yaw_smoothing_window = cfg.gpsAttitudeYawSmoothingWindow();
        att_cfg.yaw_max_change_rad = cfg.gpsAttitudeYawMaxChangeRad();
        att_cfg.pitch_roll_base_var = cfg.gpsAttitudePitchRollBaseVar();
        att_cfg.yaw_var_velocity_scale = cfg.gpsAttitudeYawVarVelocityScale();
        att_cfg.use_gps_attitude_when_available = cfg.gpsHasAttitude();
        attitude_estimator_ = std::make_shared<AttitudeEstimator>(att_cfg);
        gps_manager_.setAttitudeEstimator(attitude_estimator_);
        std::string imu_topic = cfg.imuTopic();
        imu_sub_for_attitude_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 500,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                if (!msg || !attitude_estimator_) return;
                double ts = rclcpp::Time(msg->header.stamp).seconds();
                Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                Eigen::Vector3d gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                if (acc.allFinite() && gyr.allFinite())
                    attitude_estimator_->addIMU(ts, acc, gyr);
            });
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7b: AttitudeEstimator created, IMU topic=%s, gps.has_attitude=%s (use receiver attitude when injected)",
                   imu_topic.c_str(), cfg.gpsHasAttitude() ? "true" : "false");
    }

    // LivoBridge（最后初始化，开始接收数据；传入 loadConfigAndInit 已读的 GPS 配置，避免与 ConfigManager 时序不一致）
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 8a: init LivoBridge");
    livo_bridge_.init(shared_from_this(), gps_enabled_from_config_, gps_topic_from_config_);
    if (gps_enabled_from_config_) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][GPS] GPS enabled (optional): good quality used when available; mapping continues without GPS when absent or poor.");
    }
    livo_bridge_.registerOdomCallback(
        [this](double ts, const Pose3d& pose, const Mat66d& cov) {
            onOdometry(ts, pose, cov);
        });
    livo_bridge_.registerCloudCallback(
        [this](double ts, const CloudXYZIPtr& cloud) { onCloud(ts, cloud); });
    livo_bridge_.registerKFInfoCallback(
        [this](const LivoKeyFrameInfo& info) { onKFInfo(info); });
    livo_bridge_.registerGPSCallback(
        [this](double ts, double lat, double lon, double alt, double hdop, int sats) {
            onGPS(ts, lat, lon, alt, hdop, sats);
        });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 8b: LivoBridge inited and callbacks registered");

    state_ = SystemState::MAPPING;
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 9: state=MAPPING, all modules ready");
    if (!backend_worker_.joinable()) {
        backend_worker_ = std::thread(&AutoMapSystem::backendWorkerLoop, this);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 10: backend worker thread started here (queue_max=%zu)", max_frame_queue_size_);
    } else {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 10: backend worker already running (started in ctor), LivoBridge will feed queue");
    }
    RCLCPP_INFO(get_logger(), "=== AutoMapSystem READY (session_id=%lu) ===", current_session_id_);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] state=READY session_id=%lu (grep PIPELINE 可实时查看建图各环节)", current_session_id_);
}

void AutoMapSystem::setupPublishers() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PUB] Creating publishers");
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    // 全局地图用 Reliable，确保 RViz 完整接收大点云
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    odom_path_pub_  = create_publisher<nav_msgs::msg::Path>("/automap/odom_path", path_qos);
    opt_path_pub_   = create_publisher<nav_msgs::msg::Path>("/automap/optimized_path", path_qos);
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/automap/global_map", map_qos);
    status_pub_     = create_publisher<automap_pro::msg::MappingStatusMsg>("/automap/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][TOPIC] publish: /automap/odom_path, /automap/optimized_path, /automap/global_map, /automap/status");
}

void AutoMapSystem::setupServices() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][SRV] Creating services");
    save_map_srv_ = create_service<automap_pro::srv::SaveMap>(
        "/automap/save_map",
        std::bind(&AutoMapSystem::handleSaveMap, this,
                  std::placeholders::_1, std::placeholders::_2));
    get_status_srv_ = create_service<automap_pro::srv::GetStatus>(
        "/automap/get_status",
        std::bind(&AutoMapSystem::handleGetStatus, this,
                  std::placeholders::_1, std::placeholders::_2));
    trigger_hba_srv_ = create_service<automap_pro::srv::TriggerHBA>(
        "/automap/trigger_hba",
        std::bind(&AutoMapSystem::handleTriggerHBA, this,
                  std::placeholders::_1, std::placeholders::_2));
    trigger_opt_srv_ = create_service<automap_pro::srv::TriggerOptimize>(
        "/automap/trigger_optimize",
        std::bind(&AutoMapSystem::handleTriggerOptimize, this,
                  std::placeholders::_1, std::placeholders::_2));
    trigger_gps_srv_ = create_service<automap_pro::srv::TriggerGpsAlign>(
        "/automap/trigger_gps_align",
        std::bind(&AutoMapSystem::handleTriggerGpsAlign, this,
                  std::placeholders::_1, std::placeholders::_2));
    load_session_srv_ = create_service<automap_pro::srv::LoadSession>(
        "/automap/load_session",
        std::bind(&AutoMapSystem::handleLoadSession, this,
                  std::placeholders::_1, std::placeholders::_2));
    finish_mapping_srv_ = create_service<std_srvs::srv::Trigger>(
        "/automap/finish_mapping",
        std::bind(&AutoMapSystem::handleFinishMapping, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SRV] All 8 services created (save_map, get_status, trigger_hba, trigger_optimize, trigger_gps_align, load_session, finish_mapping)");
}

void AutoMapSystem::setupTimers() {
    // 不再使用周期定时器，避免阻塞 executor 线程。status/map/data_flow 由 backendWorkerLoop 数据触发：
    // 每处理一帧后按帧数触发 publishStatus(每约10帧)、publishDataFlowSummary(每约50帧)、publishGlobalMap(每约100帧)
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][TIMER] No wall timers for status/map/data_flow (data-triggered in backend worker)");
    
    // V2: 心跳监控定时器（每5秒检查一次线程健康状态）
    heartbeat_monitor_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        [this]() { this->checkThreadHeartbeats(); });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][TIMER] Heartbeat monitor timer started (5s interval)");
}

// ─────────────────────────────────────────────────────────────────────────────
// 数据流：里程计 → 关键帧 → 子图（后端接收入口统一打 [BACKEND][RECV] 便于 grep 追踪）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    static std::atomic<int> backend_odom_recv_count{0};
    const int seq = backend_odom_recv_count.fetch_add(1) + 1;

    // [TRACE] 精准追踪：里程计入口
    ALOG_TRACE_STEP("AutoMapSystem", "onOdometry_enter");

    if (seq <= 5 || seq % 500 == 0) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][BACKEND][RECV] odom #%d ts=%.3f pos=[%.2f,%.2f,%.2f] (backend entry)",
            seq, ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
        // 修复: 添加NaN检查，防止协方差矩阵元素为NaN时导致sqrt产生NaN
        double pos_std_x = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
        double pos_std_y = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
        double pos_std_z = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
        RCLCPP_INFO(get_logger(),
            "[PRECISION][ODOM] seq=%d ts=%.4f pos_std_xyz=[%.4f,%.4f,%.4f] pos=[%.3f,%.3f,%.3f]",
            seq, ts, pos_std_x, pos_std_y, pos_std_z,
            pose.translation().x(), pose.translation().y(), pose.translation().z());
    }
    if (!first_odom_logged_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA] First odometry received ts=%.3f pos=[%.2f,%.2f,%.2f]", ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=first_odom ts=%.3f", ts);
    }
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_odom_pose_ = pose;
        last_odom_ts_   = ts;
        last_cov_       = cov;
    }
    odomCacheAdd(ts, pose, cov);

    // 更新 GPS 关键帧轨迹（用于对齐）
    gps_manager_.addKeyFramePose(ts, pose);

    // 里程计路径（仅追加+发布，不做 O(n) 裁剪，避免阻塞回调；裁剪在 data_flow 定时器内做）
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = now();
    ps.header.frame_id = "map";
    auto& pp           = ps.pose.position;
    pp.x = pose.translation().x();
    pp.y = pose.translation().y();
    pp.z = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    ps.pose.orientation.w = q.w(); ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z();
    odom_path_.poses.push_back(ps);
    odom_path_.header = ps.header;
    // 节流发布，避免高频 publish 占满 executor（每 3 帧发一次，前 20 帧每帧发）
    if (seq <= 20 || seq % 3 == 0) {
        odom_path_pub_->publish(odom_path_);
        pub_odom_path_count_++;
    }

    // 轨迹对比记录：仅当「边建图边写」时每帧写入；默认建图完成后在 saveMapToFiles 中写关键帧+最终GPS，保证与 GPS 重合
    if (trajectory_log_enabled_ && !trajectory_log_after_mapping_only_) writeTrajectoryOdom(ts, pose, cov);

    if (attitude_estimator_) {
        const Eigen::Matrix3d& R = pose.linear();
        double yaw = std::atan2(R(1, 0), R(0, 0));
        attitude_estimator_->addOdometryYaw(ts, yaw);
    }

    // 关键帧由 onCloud 触发，保证 pose 与 cloud 同帧（fast_livo 先发 odom 再发 cloud，收到 cloud 时 last_odom_pose_ 已是本帧）
}

void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud) {
    // [TRACE] 精准追踪：点云入口
    ALOG_TRACE_STEP("AutoMapSystem", "onCloud_enter");

    const size_t pts = cloud ? cloud->size() : 0u;
    (void)pts;
    if (!first_cloud_logged_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA] First cloud received ts=%.3f pts=%zu", ts, pts);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=first_cloud ts=%.3f pts=%zu", ts, pts);
    }
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_cloud_    = cloud;
        last_cloud_ts_ = ts;
    }
    last_sensor_data_wall_time_ = std::chrono::steady_clock::now();
    FrameToProcess f;
    f.ts    = ts;
    f.cloud = cloud;
    // 只写入 ingress，快速返回；背压在 feeder 线程，避免长时间阻塞 Executor
    {
        static constexpr int kMaxConsecutiveTimeouts = 3;
        static constexpr int64_t kMaxIngressWaitMs = 2000;  // 最大等待2秒
        static thread_local int consecutive_ingress_timeouts = 0;
        static thread_local bool ingress_wait_logged_this_wait = false;
        static thread_local int64_t ingress_wait_start_ms = 0;
        std::unique_lock<std::mutex> lock(ingress_mutex_);
        const auto wait_start = std::chrono::steady_clock::now();
        while (ingress_queue_.size() >= max_ingress_queue_size_ && !shutdown_requested_.load(std::memory_order_acquire)) {
            // 检查最大等待时间硬限制
            const auto elapsed = std::chrono::steady_clock::now() - wait_start;
            const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
            if (elapsed_ms >= kMaxIngressWaitMs) {
                // 超过2秒硬限制，强制丢帧
                if (!ingress_queue_.empty()) {
                    ingress_queue_.pop();
                    RCLCPP_ERROR(get_logger(),
                        "[AutoMapSystem][INGRESS] HARD_TIMEOUT: waited %ldms > %dms, forced frame drop to avoid deadlock (queue_after=%zu)",
                        elapsed_ms, kMaxIngressWaitMs, ingress_queue_.size());
                }
                break;
            }
            if (!ingress_wait_logged_this_wait) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][INGRESS] wait_start queue=%zu max=%zu (Executor 阻塞等待 feeder 消费，grep INGRESS 可定位)",
                            ingress_queue_.size(), max_ingress_queue_size_);
                ingress_wait_logged_this_wait = true;
                ingress_wait_start_ms = elapsed_ms;
            }
            const bool woken = ingress_not_full_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return ingress_queue_.size() < max_ingress_queue_size_ || shutdown_requested_.load(std::memory_order_acquire);
            });
            if (!woken && ingress_queue_.size() >= max_ingress_queue_size_) {
                consecutive_ingress_timeouts++;
                if (consecutive_ingress_timeouts >= kMaxConsecutiveTimeouts) {
                    ingress_queue_.pop();
                    RCLCPP_WARN(get_logger(),
                        "[AutoMapSystem][INGRESS] full (max=%zu), dropped oldest frame after %d timeouts to unblock callback queue_after=%zu",
                        max_ingress_queue_size_, consecutive_ingress_timeouts, ingress_queue_.size());
                    consecutive_ingress_timeouts = 0;
                    ingress_wait_logged_this_wait = false;
                    break;
                }
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][INGRESS] wait_timeout consecutive_timeouts=%d/%d queue=%zu elapsed=%ldms (若持续→feeder 或 backend 卡滞)",
                            consecutive_ingress_timeouts, kMaxConsecutiveTimeouts, ingress_queue_.size(), elapsed_ms);
                rclcpp::Clock::SharedPtr clk = get_clock();
                if (clk) RCLCPP_WARN_THROTTLE(get_logger(), *clk, 3000,
                    "[AutoMapSystem][INGRESS] full (max=%zu), callback waiting for feeder (short block)...", max_ingress_queue_size_);
            } else {
                consecutive_ingress_timeouts = 0;
                ingress_wait_logged_this_wait = false;
            }
        }
        if (shutdown_requested_.load(std::memory_order_acquire)) return;
        ingress_queue_.push(std::move(f));
        ingress_not_empty_cv_.notify_one();

        // [TRACE] 精准追踪：点云已入队
        ALOG_TRACE_STEP("AutoMapSystem", "onCloud_queued");

    }
    size_t qs = 0;
    { std::lock_guard<std::mutex> lk(frame_queue_mutex_); qs = frame_queue_.size(); }
    (void)qs;
}

void AutoMapSystem::onKFInfo(const LivoKeyFrameInfo& info) {
    static std::atomic<int> backend_kfinfo_recv_count{0};
    const int seq = backend_kfinfo_recv_count.fetch_add(1) + 1;
    if (seq <= 5 || seq % 20 == 0) {
        // RCLCPP_INFO(get_logger(),
        //     "[AutoMapSystem][BACKEND][RECV] kfinfo #%d ts=%.3f cov_norm=%.4f degen=%d (backend entry)",
        //     seq, info.timestamp, info.esikf_cov_norm, info.is_degenerate ? 1 : 0);
    }
    { std::lock_guard<std::mutex> lk(data_mutex_); last_livo_info_ = info; }
    kfinfoCacheAdd(info.timestamp, info);
}

void AutoMapSystem::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats) {
    gps_manager_.addGPSMeasurement(ts, lat, lon, alt, hdop, sats);
}

void AutoMapSystem::odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    const bool was_empty = odom_cache_.empty();
    odom_cache_.push_back({ts, pose, cov});
    while (odom_cache_.size() > kMaxOdomCacheSize) odom_cache_.pop_front();
    if (was_empty) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] first odom cached ts=%.3f (worker can now match cloud by ts)", ts);
    }
}

bool AutoMapSystem::odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    if (odom_cache_.empty()) return false;
    // 找 odom_ts <= cloud_ts 的最近一条（从后往前），避免用“未来”里程计对齐点云
    const OdomCacheEntry* best = nullptr;
    for (auto it = odom_cache_.rbegin(); it != odom_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) return false;  // 全部缓存条目的 ts > cloud_ts 时不用，防止错误对齐
    out_pose = best->pose;
    out_cov  = best->cov;
    return true;
}

void AutoMapSystem::kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    kfinfo_cache_.push_back({ts, info});
    while (kfinfo_cache_.size() > kMaxKFinfoCacheSize) kfinfo_cache_.pop_front();
}

bool AutoMapSystem::kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    if (kfinfo_cache_.empty()) return false;
    // 找 kfinfo_ts <= ts 的最近一条，无则不用“未来”的 kfinfo
    const KFinfoCacheEntry* best = nullptr;
    for (auto it = kfinfo_cache_.rbegin(); it != kfinfo_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) return false;
    out_info = best->info;
    return true;
}

void AutoMapSystem::feederLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_feeder");
#endif
    // 从 ingress 取帧压入 frame_queue_；frame_queue_ 满时在此线程等待（背压），不阻塞订阅回调
    static std::atomic<int> feeder_frame_count{0};
    const int feeder_tid = static_cast<int>(syscall(SYS_gettid));
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] thread started tid=%d", feeder_tid);
    feeder_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 初始心跳
    
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        // [TRACE] 精准追踪：feeder 循环开始
        ALOG_TRACE_STEP("AutoMapSystem", "feeder_loop_iter");

        feeder_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 更新心跳
        FrameToProcess f;
        {
            std::unique_lock<std::mutex> lock(ingress_mutex_);
            const bool has_data = ingress_not_empty_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !ingress_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!has_data || ingress_queue_.empty()) continue;
            f = std::move(ingress_queue_.front());
            ingress_queue_.pop();
            ingress_not_full_cv_.notify_one();
        }
        
        const int feeder_seq = feeder_frame_count.fetch_add(1) + 1;
        
        // 可选：预计算体素降采样供 backend worker 使用（V1：带超时保护，避免无限阻塞）
        if (f.cloud && !f.cloud->empty()) {
            float ds_res = static_cast<float>(ConfigManager::instance().submapMatchRes());
            const int voxel_timeout_ms = 5000;  // 5 秒超时
            bool timed_out = false;
            auto t_voxel_start = std::chrono::steady_clock::now();
            try {
                f.cloud_ds = utils::voxelDownsampleWithTimeout(f.cloud, ds_res, voxel_timeout_ms, &timed_out);
                auto t_voxel_end = std::chrono::steady_clock::now();
                double voxel_ms = std::chrono::duration<double, std::milli>(t_voxel_end - t_voxel_start).count();
                if (feeder_seq <= 5 || feeder_seq % 100 == 0 || voxel_ms > 100.0 || timed_out) {
                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] voxel_downsample seq=%d pts=%zu ds_pts=%zu ms=%.1f timeout=%d",
                                feeder_seq, f.cloud->size(), f.cloud_ds ? f.cloud_ds->size() : 0, voxel_ms, timed_out ? 1 : 0);
                }
                if (timed_out) {
                    RCLCPP_WARN(get_logger(), "[AutoMapSystem][FEEDER] voxelDownsampleWithTimeout timed out seq=%d pts=%zu, using sanitized copy", feeder_seq, f.cloud->size());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][FEEDER] voxelDownsample exception seq=%d: %s", feeder_seq, e.what());
                f.cloud_ds = nullptr;  // 继续处理，只是没有预降采样
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][FEEDER] voxelDownsample unknown exception seq=%d", feeder_seq);
                f.cloud_ds = nullptr;
            }
        }
        
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            int wait_count = 0;
            const int max_waits = ConfigManager::instance().backpressureMaxWaits();
            const int wait_sec = ConfigManager::instance().backpressureWaitSec();
            while (frame_queue_.size() >= max_frame_queue_size_ && !shutdown_requested_.load(std::memory_order_acquire)) {
                if (wait_count == 0) {
                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] backpressure frame_queue_full queue=%zu max=%zu (feeder 等 backend 消费，grep FEEDER backpressure 可定位)",
                                frame_queue_.size(), max_frame_queue_size_);
                }
                if (wait_count >= max_waits) {
                    backpressure_force_drop_count_++;
                    RCLCPP_ERROR(get_logger(), "[AutoMapSystem][FEEDER][BACKPRESSURE] queue stuck for >%ds wait_count=%d, forcing frame drop to avoid deadlock (total_force_drops=%d)",
                                wait_sec * max_waits, wait_count, backpressure_force_drop_count_.load(std::memory_order_relaxed));
                    frame_queue_.pop();  // 强制丢弃最旧帧
                    break;
                }
                frame_queue_not_full_cv_.wait_for(lock, std::chrono::seconds(wait_sec), [this] {
                    return frame_queue_.size() < max_frame_queue_size_ || shutdown_requested_.load(std::memory_order_acquire);
                });
                wait_count++;
                if (wait_count > 0 && wait_count < max_waits && frame_queue_.size() >= max_frame_queue_size_) {
                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] backpressure wait_timeout wait_count=%d queue=%zu", wait_count, frame_queue_.size());
                }
            }
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            const size_t queue_size_before = frame_queue_.size();
            const double frame_ts = f.ts;  // 保存时间戳，move 后 f.ts 不可访问
            frame_queue_.push(std::move(f));
            frame_queue_cv_.notify_one();
            const size_t qs = frame_queue_.size();

            // 关键诊断日志：前5帧、每10帧、每50帧打印；每100帧打 HEARTBEAT 便于在合并日志中确认入队存活
            if (feeder_seq <= 5 || feeder_seq % 10 == 0 || feeder_seq % 50 == 0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] pushed frame seq=%d ts=%.3f queue=%zu->%zu (grep FEEDER 可追踪入队)",
                            feeder_seq, frame_ts, queue_size_before, qs);
            }
            if (feeder_seq % 100 == 0) {
                size_t ingress_sz = 0;
                { std::lock_guard<std::mutex> lk(ingress_mutex_); ingress_sz = ingress_queue_.size(); }
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER][HEARTBEAT] total_pushed=%d queue=%zu ingress=%zu (grep FEEDER HEARTBEAT 可确认入队存活; ingress 堆积或 backend 无 POP 可对照 BACKEND HEARTBEAT)",
                            feeder_seq, qs, ingress_sz);
            }
        }
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] thread exiting, total_frames=%d", feeder_frame_count.load());
}

void AutoMapSystem::backendWorkerLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_backend");
#endif

    // [TRACE] 精准追踪：后端 worker 启动
    ALOG_TRACE_STEP("AutoMapSystem", "backendWorkerLoop_start");

    const std::string cloud_frame = ConfigManager::instance().frontendCloudFrame();
    const int process_every_n_config = ConfigManager::instance().backendProcessEveryNFrames();
    const int override_n = process_every_n_override_.load(std::memory_order_acquire);
    const int process_every_n = (override_n > 0) ? override_n : process_every_n_config;
    if (process_every_n > 1) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][CONFIG] process_every_n_frames=%d (skip %d frames per processed frame)%s",
                    process_every_n, process_every_n - 1, override_n > 0 ? " [degradation override]" : "");
    }
    static thread_local int no_odom_skip_count = 0;
    const auto wait_chunk = std::chrono::milliseconds(2000);
    const double idle_timeout_sec = ConfigManager::instance().sensorIdleTimeoutSec();
    const bool auto_finish = ConfigManager::instance().autoFinishOnSensorIdle();
    backend_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 初始心跳
    static auto s_last_backend_heartbeat_wall = std::chrono::steady_clock::now();
    constexpr int BACKEND_HEARTBEAT_INTERVAL_SEC = 30;

    // 🔧 DEBUG: 记录后端启动时间，用于诊断卡死
    const auto backend_start_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] backendWorkerLoop started at t=0, waiting for frames...");

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        // 🔧 DEBUG: 每次循环开始时更新心跳，便于定位卡在循环的哪个阶段
        backend_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
        // 🔧 DEBUG: 记录循环迭代次数和运行时间
        static int loop_iter = 0;
        static auto loop_start_time = std::chrono::steady_clock::now();
        loop_iter++;
        auto now_wall = std::chrono::steady_clock::now();
        double loop_elapsed_sec = std::chrono::duration<double>(now_wall - loop_start_time).count();
        if (loop_iter % 10 == 1) {  // 每10次迭代输出一次
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] loop_iter=%d elapsed_sec=%.1f",
                        loop_iter, loop_elapsed_sec);
        }

        // 异步 GPS 对齐：若 addGPSMeasurement 已触发调度，在此线程执行 try_align()，不阻塞 ROS 回调
        gps_manager_.runScheduledAlignment();
        FrameToProcess f;
        size_t qs_after_wait = 0;
        bool woke_by_data = false;
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            // 🔧 DEBUG: 在等待数据前打印日志
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] waiting for frame... queue_size=%zu", frame_queue_.size());
            woke_by_data = frame_queue_cv_.wait_for(lock, wait_chunk, [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !frame_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;

            qs_after_wait = frame_queue_.size();
            // 🔧 DEBUG: 记录队列状态
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] wakeup: woke_by_data=%d queue_size=%zu",
                        woke_by_data ? 1 : 0, qs_after_wait);
            if (!woke_by_data) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] wait_done reason=timeout queue=%zu (backend 每 2s 唤醒一次，无数据则继续等；若长期无数据见 BACKEND HEARTBEAT)",
                              qs_after_wait);
            }

            if (!woke_by_data && frame_queue_.empty()) {
                // 超时且队列为空：检查是否传感器空闲超时，触发最终处理并结束建图（离线“播完再结束”时由 finish_mapping 服务触发，此处跳过）
                const bool offline_finish_after_bag = ConfigManager::instance().offlineFinishAfterBag();
                if (offline_finish_after_bag) {
                    // 不依赖传感器空闲结束，等待 /automap/finish_mapping 被调用（launch 在 bag 播完后调用）
                } else {
                auto now = std::chrono::steady_clock::now();
                double idle_sec = std::chrono::duration<double>(now - last_sensor_data_wall_time_).count();
                if (auto_finish && !sensor_idle_finish_triggered_.load(std::memory_order_acquire) &&
                    first_cloud_logged_.load(std::memory_order_acquire) && idle_sec >= idle_timeout_sec) {
                    sensor_idle_finish_triggered_.store(true, std::memory_order_release);
                    lock.unlock();
                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sensor_idle_timeout idle_sec=%.1f timeout=%.1f submaps=%d (→ final HBA + save + rclcpp::shutdown)",
                                idle_sec, idle_timeout_sec, submap_manager_.submapCount());
                    try {
                        if (submap_manager_.submapCount() > 0) {
                            if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnFinish()) {
                                auto all = submap_manager_.getAllSubmaps();
                                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sensor_idle_final_hba_enter submaps=%zu", all.size());
                                ensureBackendCompletedAndFlushBeforeHBA();
                                hba_optimizer_.triggerAsync(all, true);
                                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sensor_idle_final_hba_done");
                            }
                            std::string out_dir = getOutputDir();
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sensor_idle_save_enter output_dir=%s", out_dir.c_str());
                            saveMapToFiles(out_dir);
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sensor_idle_save_done output_dir=%s", out_dir.c_str());
                        }
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Sensor idle: requesting context shutdown (end mapping)");
                        rclcpp::shutdown();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Sensor idle finish failed: %s", e.what());
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=auto_finish_failed error=%s", e.what());
                    } catch (...) {
                        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Sensor idle finish failed: unknown exception");
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=auto_finish_failed error=unknown");
                    }
                    break;
                }
                }
                // 若配置了 frontend_idle_trigger_sec：10s 无前端数据且后端队列/ingress 已空时触发一次 HBA（不结束建图，仅做一次优化）
                const double idle_trigger_sec = ConfigManager::instance().hbaFrontendIdleTriggerSec();
                if (idle_trigger_sec > 0 && ConfigManager::instance().hbaEnabled() &&
                    first_cloud_logged_.load(std::memory_order_acquire) && submap_manager_.submapCount() > 0) {
                    auto now_idle = std::chrono::steady_clock::now();
                    double frontend_idle_sec = std::chrono::duration<double>(now_idle - last_sensor_data_wall_time_).count();
                    if (frontend_idle_sec >= idle_trigger_sec && !hba_triggered_by_frontend_idle_.exchange(true)) {
                        size_t ingress_sz = 0;
                        { std::lock_guard<std::mutex> lk(ingress_mutex_); ingress_sz = ingress_queue_.size(); }
                        if (ingress_sz == 0) {
                            lock.unlock();
                            RCLCPP_INFO(get_logger(),
                                "[AutoMapSystem][PIPELINE] event=hba_triggered_by_frontend_idle frontend_idle_sec=%.1f submaps=%d (10s 无前端数据且后端已处理完)",
                                frontend_idle_sec, submap_manager_.submapCount());
                            ensureBackendCompletedAndFlushBeforeHBA();
                            auto all = submap_manager_.getAllSubmaps();
                            hba_optimizer_.triggerAsync(all, false);
                            continue;
                        }
                    }
                }
                // 队列空时也打心跳，便于在合并日志中确认后端在“等数据”而非卡死；带 livo/ingress 诊断便于定位断点
                {
                    auto now_wall = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now_wall - s_last_backend_heartbeat_wall).count() >= BACKEND_HEARTBEAT_INTERVAL_SEC) {
                        s_last_backend_heartbeat_wall = now_wall;
                        const int popped = backend_cloud_frames_processed_.load(std::memory_order_acquire);
                        const int processed = backend_frames_actually_processed_.load(std::memory_order_acquire);
                        const int livo_cloud = livo_bridge_.cloudCount();
                        const int livo_odom  = livo_bridge_.odomCount();
                        size_t ingress_sz = 0;
                        { std::lock_guard<std::mutex> lk(ingress_mutex_); ingress_sz = ingress_queue_.size(); }
                        RCLCPP_INFO(get_logger(),
                            "[AutoMapSystem][BACKEND][HEARTBEAT] waiting_for_data frames_popped=%d processed_no=%d queue=0 livo_cloud=%d livo_odom=%d ingress=%zu (若 livo 不再增长: 回调未交付; ingress>0: feeder 可能卡住)",
                            popped, processed, livo_cloud, livo_odom, ingress_sz);

                        // 停滞诊断：超过 60s 且 LivoBridge 计数未增加则打 WARN，便于定位「后端等数据但前端已停」
                        static int s_last_heartbeat_livo_cloud = -1;
                        static int s_last_heartbeat_livo_odom  = -1;
                        static auto s_last_stall_check_wall     = now_wall;
                        const auto elapsed_stall_sec = std::chrono::duration_cast<std::chrono::seconds>(now_wall - s_last_stall_check_wall).count();
                        if (elapsed_stall_sec >= 60 && s_last_heartbeat_livo_cloud >= 0 &&
                            livo_cloud == s_last_heartbeat_livo_cloud && livo_odom == s_last_heartbeat_livo_odom) {
                            RCLCPP_WARN(get_logger(),
                                "[AutoMapSystem][BACKEND_STALL_DIAG] 超过 60s 无新数据: frames_popped=%d processed_no=%d livo_cloud=%d livo_odom=%d ingress=%zu queue=0. "
                                "若 livo 计数不再增加→ROS2 回调未交付(检查 Executor/线程/QoS); ingress>0→feeder 可能卡住; 见 docs/LOG_ANALYSIS_RUN_20260310_151141.md",
                                popped, processed, livo_cloud, livo_odom, ingress_sz);
                        }
                        s_last_heartbeat_livo_cloud = livo_cloud;
                        s_last_heartbeat_livo_odom  = livo_odom;
                        s_last_stall_check_wall     = now_wall;
                    }
                }
                continue;  // 超时且队列空但未达空闲阈值，仅重新等待
            }

            qs_after_wait = frame_queue_.size();
            f = std::move(frame_queue_.front());
            frame_queue_.pop();
            frame_queue_not_full_cv_.notify_one();  // 背压：腾出空间，唤醒可能等待的生产者
        }
        const int frame_no = backend_cloud_frames_processed_.fetch_add(1) + 1;
        size_t qs_after_pop = 0;
        { std::lock_guard<std::mutex> lk(frame_queue_mutex_); qs_after_pop = frame_queue_.size(); }

        // 周期性心跳：每 30s 打一条，便于在合并日志中确认后端存活（grep BACKEND HEARTBEAT）
        {
            auto now_wall = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now_wall - s_last_backend_heartbeat_wall).count() >= BACKEND_HEARTBEAT_INTERVAL_SEC) {
                s_last_backend_heartbeat_wall = now_wall;
                const int processed_so_far = backend_frames_actually_processed_.load(std::memory_order_acquire);
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][BACKEND][HEARTBEAT] frames_popped=%d processed_no=%d queue=%zu last_ts=%.3f (grep BACKEND HEARTBEAT 可确认后端存活)",
                    frame_no, processed_so_far, qs_after_pop, f.ts);
            }
        }

        try {
        // 每隔 process_every_n 帧才处理一帧（减轻后端负载）；队列仍每帧弹出；降级时 override 增大以减负
        const int pevery = (process_every_n_override_.load(std::memory_order_acquire) > 0)
            ? process_every_n_override_.load(std::memory_order_acquire) : process_every_n_config;
        const bool will_process = ((frame_no - 1) % pevery == 0);
        if (frame_no <= 50) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][POP] frame_no=%d ts=%.3f action=%s queue_after=%zu (grep BACKEND POP 可追踪每帧弹出)",
                        frame_no, f.ts, will_process ? "process" : "skip", qs_after_pop);
        }
        if (!will_process) {
            if (frame_no > 50 && (frame_no % 500 == 0)) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][SKIP] frame_no=%d ts=%.3f (process_every_n=%d)", frame_no, f.ts, pevery);
            }
            continue;
        }
        const int processed_no = backend_frames_actually_processed_.fetch_add(1) + 1;
        // 🔧 DEBUG: 在处理帧之前打印日志
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] about_to_process frame_no=%d processed_no=%d",
                    frame_no, processed_no);
        if (frame_no <= 20) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] popped frame_no=%d ts=%.3f queue_after_pop=%zu (wait saw %zu) session_id=%lu",
                        frame_no, f.ts, qs_after_pop, qs_after_wait, current_session_id_);
        }
        // 强化日志：崩溃时 grep CRASH_CONTEXT 最后一行即当前处理的 frame/session，便于精准定位
        if (processed_no <= 35 || processed_no % 100 == 0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][CRASH_CONTEXT] step=backend_worker_processing session_id=%lu frame_no=%d processed_no=%d ts=%.3f (grep CRASH_CONTEXT to locate crash)",
                        current_session_id_, frame_no, processed_no, f.ts);
        }
        // 🔧 DEBUG: 在获取 keyframe_mutex 之前打印日志
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] about_to_lock keyframe_mutex processed_no=%d", processed_no);

        // 按时间戳从缓存对齐 pose/cov 和 kfinfo，不阻塞生产者
        Pose3d pose = Pose3d::Identity();
        Mat66d cov  = Mat66d::Identity() * 1e-4;
        if (!odomCacheGet(f.ts, pose, cov)) {
            no_odom_skip_count++;
            if (no_odom_skip_count <= 15) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][DIAG] no odom in cache for ts=%.3f frame_no=%d skip #%d (odom not yet arrived or ts mismatch)", f.ts, frame_no, no_odom_skip_count);
                RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=skip reason=no_odom_in_cache frame_no=%d ts=%.3f pts=%zu (精准定位: 缓存无对应位姿)", frame_no, f.ts, f.cloud ? f.cloud->size() : 0u);
            } else {
                rclcpp::Clock::SharedPtr clk = get_clock();
                if (clk) RCLCPP_WARN_THROTTLE(get_logger(), *clk, 2000,
                    "[AutoMapSystem][BACKEND] no odom in cache for ts=%.3f frame_no=%d, skip (align by ts) total_skips=%d", f.ts, frame_no, no_odom_skip_count);
            }
            continue;
        }
        LivoKeyFrameInfo kfinfo_copy;
        if (!kfinfoCacheGet(f.ts, kfinfo_copy)) {
            std::lock_guard<std::mutex> lk(data_mutex_);
            kfinfo_copy = last_livo_info_;
        }
        kf_manager_.updateLivoInfo(kfinfo_copy);
        if (processed_no <= 5) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] tryCreateKeyFrame entered session_id=%lu processed_no=%d frame_no=%d ts=%.3f pts=%zu", current_session_id_, processed_no, frame_no, f.ts, f.cloud ? f.cloud->size() : 0u);
        }
        // 若前端发布的是世界系点云（fast_livo /cloud_registered），先转为 body 系再建 KF，否则全局图会因双重变换而错乱
        CloudXYZIPtr cloud_for_kf = f.cloud;
        if (cloud_frame == "world" && f.cloud && !f.cloud->empty()) {
            cloud_for_kf = transformWorldToBody(f.cloud, pose);
            if (processed_no == 1) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] first frame: cloud converted world→body pts=%zu pose=[%.2f,%.2f,%.2f]",
                            cloud_for_kf->size(), pose.translation().x(), pose.translation().y(), pose.translation().z());
            }
        }
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=tryCreateKeyFrame_enter processed_no=%d ts=%.3f (grep BACKEND STEP 精准定位卡住阶段)",
                    processed_no, f.ts);
        try {
            std::lock_guard<std::mutex> lk(keyframe_mutex_);
            tryCreateKeyFrame(f.ts, pose, cov, cloud_for_kf, &kfinfo_copy,
                (f.cloud_ds && !f.cloud_ds->empty()) ? &f.cloud_ds : nullptr);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker session_id=%lu frame_no=%d ts=%.3f: %s", current_session_id_, frame_no, f.ts, e.what());
            RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=fail reason=exception session_id=%lu frame_no=%d ts=%.3f step_where=tryCreateKeyFrame what=%s",
                       current_session_id_, frame_no, f.ts, e.what());
            ErrorMonitor::instance().recordException(e, errors::LIVO_ODOMETRY_FAILED);
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker session_id=%lu frame_no=%d ts=%.3f: unknown", current_session_id_, frame_no, f.ts);
            RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=fail reason=unknown_exception session_id=%lu frame_no=%d ts=%.3f step_where=tryCreateKeyFrame",
                       current_session_id_, frame_no, f.ts);
            ErrorMonitor::instance().recordError(ErrorDetail(errors::UNKNOWN_ERROR, "backend worker unknown exception"));
        }
        const double duration_ms = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=tryCreateKeyFrame_exit processed_no=%d duration_ms=%.1f (grep BACKEND STEP 定位卡住)",
                    processed_no, duration_ms);
        if (processed_no % 100 == 0 || duration_ms > 5000.0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] frame_processed_done frame_no=%d processed_no=%d ts=%.3f duration_ms=%.1f (grep to locate last completed frame when stuck)",
                        frame_no, processed_no, f.ts, duration_ms);
        }
        if (duration_ms > 5000.0 && duration_ms <= 30000.0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][SLOW] processed_no=%d ts=%.3f duration_ms=%.1f (>5s, 可能 ISAM2/回环/GPS 批次；若持续见 STUCK)",
                        processed_no, f.ts, duration_ms);
        }
        // 强化日志：每帧处理时间超过 60 秒时告警，便于检测 commitAndUpdate 卡死
        // 超过 60 秒则强制重置 ISAM2 恢复系统
        if (duration_ms > 60000.0) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][STUCK] processed_no=%d ts=%.3f duration_ms=%.1f (>60s) - forcing ISAM2 reset!",
                        processed_no, f.ts, duration_ms);
            isam2_optimizer_.reset();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STUCK] ISAM2 reset complete after stuck detection");
        } else if (duration_ms > 30000.0) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][STUCK] processed_no=%d ts=%.3f duration_ms=%.1f (>30s) - possible commitAndUpdate deadlock!",
                        processed_no, f.ts, duration_ms);
        }
        const int map_interval = ConfigManager::instance().backendPublishGlobalMapEveryNProcessed();
        if ((processed_no <= 3 || duration_ms > 200.0) && processed_no % map_interval != 0) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] worker processed #%d ts=%.3f duration_ms=%.1f", processed_no, f.ts, duration_ms);
        } else if (processed_no % map_interval == 0 || duration_ms > 200.0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker processed #%d ts=%.3f duration_ms=%.1f", processed_no, f.ts, duration_ms);
        }
        // 数据触发：投递到 status/viz 线程，不阻塞后端
        if (processed_no <= 1 || processed_no % 10 == 0) {
            status_publish_pending_.store(true, std::memory_order_release);
            status_pub_cv_.notify_one();
        }
        if (processed_no % 50 == 0) {
            data_flow_publish_pending_.store(true, std::memory_order_release);
            status_pub_cv_.notify_one();
        }
        if (processed_no % map_interval == 0) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] processed_no=%d step=publishGlobalMap request (async, every_n=%d)", processed_no, map_interval);
            map_publish_pending_.store(true);
            map_publish_cv_.notify_one();
        }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker round session_id=%lu frame_no=%d: %s", current_session_id_, frame_no, e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker round session_id=%lu frame_no=%d: unknown", current_session_id_, frame_no);
        }
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker thread exiting (next: destructor will join and call loop_detector_.stop())");
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=backend_worker_exited");
}

void AutoMapSystem::mapPublishLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_mappub");
#endif
    map_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 初始心跳
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        map_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 更新心跳
        std::unique_lock<std::mutex> lock(map_publish_mutex_);
        const bool map_woke = map_publish_cv_.wait_for(lock, std::chrono::seconds(5), [this] {
            return shutdown_requested_.load(std::memory_order_acquire) || map_publish_pending_.load(std::memory_order_acquire);
        });
        if (shutdown_requested_.load(std::memory_order_acquire)) break;
        if (!map_woke) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP_PUB] wait_done reason=timeout (5s，无 keyframe 或 shutdown 时定期唤醒，grep MAP_PUB 可定位)");
        }
        if (!map_publish_pending_.exchange(false, std::memory_order_acq_rel)) continue;
        lock.unlock();  // 释放锁后再执行耗时 publishGlobalMap，不阻塞 backend 的 keyframe_mutex_
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap async step=enter");
        try {
            publishGlobalMap();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap async failed: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap async failed: unknown");
        }
        const double ms = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] step=publishGlobalMap done (async) map_publish_ms=%.0f", ms);
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] mapPublishLoop exiting");
}

void AutoMapSystem::loopOptThreadLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_loopopt");
#endif
    loop_opt_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 初始心跳
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        loop_opt_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 更新心跳
        LoopConstraint::Ptr lc;
        {
            std::unique_lock<std::mutex> lock(loop_opt_mutex_);
            const bool loop_woke = loop_opt_cv_.wait_for(lock, std::chrono::seconds(5), [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !loop_factor_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!loop_woke && loop_factor_queue_.empty()) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][LOOP_OPT] wait_done reason=timeout (5s，无回环或 shutdown 时定期唤醒，grep LOOP_OPT 可定位)");
            }
            if (loop_factor_queue_.empty()) continue;
            lc = std::move(loop_factor_queue_.front());
            loop_factor_queue_.pop();
        }
        loop_opt_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 更新心跳
        if (!lc) continue;
        state_ = SystemState::LOOP_CLOSING;
        try {
            auto result = isam2_optimizer_.addLoopFactor(
                lc->submap_i, lc->submap_j, lc->delta_T, lc->information);
            if (result.success) {
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][LOOP] async optimized nodes_updated=%d elapsed=%.1fms final_rmse=%.4f",
                    result.nodes_updated, result.elapsed_ms, result.final_rmse);
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=loop_factor_added success=1 sm_i=%d sm_j=%d nodes_updated=%d",
                           lc->submap_i, lc->submap_j, result.nodes_updated);
            } else {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][LOOP][EXCEPTION] addLoopFactor failed sm_i=%d sm_j=%d", lc->submap_i, lc->submap_j);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][LOOP] loop_opt_thread addLoopFactor exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][LOOP] loop_opt_thread addLoopFactor unknown exception");
        }
        state_ = SystemState::MAPPING;
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP] loopOptThreadLoop exiting");
}

void AutoMapSystem::vizThreadLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_viz");
#endif
    viz_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 初始心跳
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        viz_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 更新心跳
        CloudXYZIPtr cloud;
        {
            std::unique_lock<std::mutex> lock(viz_mutex_);
            viz_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !viz_cloud_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (viz_cloud_queue_.empty()) continue;
            cloud = std::move(viz_cloud_queue_.front());
            viz_cloud_queue_.pop();
        }
        try {
            if (cloud && !cloud->empty())
                rviz_publisher_.publishCurrentCloud(cloud);
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][VIZ] publishCurrentCloud: %s", e.what());
        } catch (...) {}
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][VIZ] vizThreadLoop exiting");
}

void AutoMapSystem::statusPublisherLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_status");
#endif
    status_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 初始心跳
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        status_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);  // 更新心跳
        {
            std::unique_lock<std::mutex> lock(status_pub_mutex_);
            status_pub_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return shutdown_requested_.load(std::memory_order_acquire)
                    || status_publish_pending_.load(std::memory_order_acquire)
                    || data_flow_publish_pending_.load(std::memory_order_acquire);
            });
        }
        if (shutdown_requested_.load(std::memory_order_acquire)) break;
        if (status_publish_pending_.exchange(false, std::memory_order_acq_rel)) {
            try { publishStatus(); } catch (const std::exception& e) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][STATUS] publishStatus: %s", e.what());
            } catch (...) {}
        }
        if (data_flow_publish_pending_.exchange(false, std::memory_order_acq_rel)) {
            try { publishDataFlowSummary(); } catch (const std::exception& e) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][STATUS] publishDataFlowSummary: %s", e.what());
            } catch (...) {}
        }
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][STATUS] statusPublisherLoop exiting");
}

CloudXYZIPtr AutoMapSystem::transformWorldToBody(const CloudXYZIPtr& world_cloud, const Pose3d& T_w_b) const {
    if (!world_cloud || world_cloud->empty()) return world_cloud;
    Pose3d T_b_w = T_w_b.inverse();
    Eigen::Affine3f T_b_w_f;
    T_b_w_f.matrix() = T_b_w.matrix().cast<float>();
    CloudXYZIPtr body(new CloudXYZI);
    pcl::transformPointCloud(*world_cloud, *body, T_b_w_f);
    return body;
}

void AutoMapSystem::tryCreateKeyFrame(double ts) {
    Pose3d cur_pose;
    CloudXYZIPtr cur_cloud;
    Mat66d cur_cov;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        cur_pose  = last_odom_pose_;
        cur_cloud = last_cloud_;
        cur_cov   = last_cov_;
    }
    tryCreateKeyFrame(ts, cur_pose, cur_cov, cur_cloud, nullptr);
}

void AutoMapSystem::tryCreateKeyFrame(double ts, const Pose3d& pose, const Mat66d& cov, const CloudXYZIPtr& cloud,
                                      const LivoKeyFrameInfo* optional_livo_info,
                                      const CloudXYZIPtr* optional_cloud_ds) {
    const Pose3d& cur_pose = pose;
    const Mat66d& cur_cov  = cov;
    const CloudXYZIPtr& cur_cloud = cloud;
    const double odom_ts = ts;
    LivoKeyFrameInfo livo_info;
    if (optional_livo_info)
        livo_info = *optional_livo_info;
    else
        { std::lock_guard<std::mutex> lk(data_mutex_); livo_info = last_livo_info_; }

    // 空点云直接丢弃，不参与关键帧决策
    if (!cur_cloud || cur_cloud->empty()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][FRAME] ts=%.3f result=discard_empty (no KF decision)", ts);
        RCLCPP_INFO(get_logger(), "[TRACE] step=kf_decision result=skip reason=cloud_empty ts=%.3f pts=0 (精准定位: 点云为空)", ts);
        static int empty_skip_count = 0;
        if (++empty_skip_count <= 3 || empty_skip_count % 100 == 0) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][KF] cloud_ts=%.3f cloud empty, discard (no KF decision)", ts);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=cloud_empty_discard cloud_ts=%.3f odom_ts=%.3f (throttled)", ts, odom_ts);
        }
        return;
    }

    // RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][FRAME] process ts=%.3f pts=%zu odom_ts=%.3f (KF decision)",
    //             ts, cur_cloud->size(), odom_ts);

    using KfClock = std::chrono::steady_clock;
    const auto kf_t0 = KfClock::now();

    if (!kf_manager_.shouldCreateKeyFrame(cur_pose, ts)) {
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][FRAME] ts=%.3f result=skip_no_kf (dist/rot/interval)", ts);
        static int reject_count = 0;
        if (++reject_count >= 100) {
            reject_count = 0;
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PIPELINE] event=kf_candidate_rejected (throttled) ts=%.3f", ts);
        }
        return;
    }

    const double odom_cloud_dt = std::abs(ts - odom_ts);
    if (odom_ts >= 0.0 && odom_cloud_dt > 0.15) {
        rclcpp::Clock::SharedPtr clk = get_clock();
        if (clk) RCLCPP_WARN_THROTTLE(get_logger(), *clk, 5000,
            "[AutoMapSystem][KF] odom_ts=%.3f cloud_ts=%.3f dt=%.3fs (expected <0.15s)", odom_ts, ts, odom_cloud_dt);
    }

    AUTOMAP_TIMED_SCOPE(MOD, "CreateKeyFrame", 50.0);

    float ds_res = static_cast<float>(ConfigManager::instance().submapMatchRes());
    CloudXYZIPtr cloud_ds;
    if (optional_cloud_ds && *optional_cloud_ds && !(*optional_cloud_ds)->empty()) {
        cloud_ds = *optional_cloud_ds;
    } else {
        cloud_ds = utils::voxelDownsample(cur_cloud, ds_res);
    }
    if (!cloud_ds) cloud_ds = std::make_shared<CloudXYZI>();
    auto t_after_voxel = KfClock::now();
    double ms_voxel = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_voxel - kf_t0).count();
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP] ts=%.3f step=voxel_ds done ms=%.1f ds_pts=%zu", ts, ms_voxel, cloud_ds->size());

    GPSMeasurement gps;
    bool has_gps = false;
    // 优先按位置匹配：找与当前关键帧位置最近的 GPS，用其时间戳做插值；无对齐或未命中时回退到时间戳匹配
    auto gps_opt = gps_manager_.queryByNearestPosition(cur_pose.translation());
    if (!gps_opt)
        gps_opt = gps_manager_.queryByTimestampEnhanced(ts);
    if (gps_opt) {
        gps     = *gps_opt;
        has_gps = gps.is_valid;
        // 【GPS_DIAG】记录为何 has_gps=0 的原因，便于定位问题
        if (!has_gps) {
            RCLCPP_WARN(get_logger(),
                "[GPS_KF_BIND] ts=%.3f found_gps=true but is_valid=false: "
                "hdop=%.2f quality=%d(MEDIUM=2) dt=%.3fs gps_ts=%.3f reason=quality_below_medium",
                ts, gps_opt->hdop, static_cast<int>(gps_opt->quality),
                std::abs(gps_opt->timestamp - ts), gps_opt->timestamp);
        } else {
            RCLCPP_DEBUG(get_logger(),
                "[GPS_KF_BIND] ts=%.3f found_gps=true is_valid=true: "
                "hdop=%.2f quality=%d dt=%.3fs",
                ts, gps_opt->hdop, static_cast<int>(gps_opt->quality),
                std::abs(gps_opt->timestamp - ts));
        }
    } else {
        // 【增强日志】添加 GPS 窗口、max_dt、时间关系，便于精确分析未匹配原因
        double first_gps_ts = gps_manager_.getFirstGpsTimestamp();
        double last_gps_ts = gps_manager_.getLastGpsTimestamp();
        double max_dt_s = gps_manager_.getKeyframeMatchWindowS();
        double delay_to_first_gps = (first_gps_ts > 0.0) ? (ts - first_gps_ts) : 0.0;
        double gap_to_window = (first_gps_ts > 0.0 && last_gps_ts > 0.0)
            ? (ts < first_gps_ts ? first_gps_ts - ts : (ts > last_gps_ts ? ts - last_gps_ts : 0.0))
            : 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
            "[GPS_KF_BIND] ts=%.3f found_gps=false window_size=%zu max_dt_s=%.2f gps_ts_range=[%.3f, %.3f] delay_to_first_gps=%.3fs gap_to_window=%.3fs "
            "(no GPS within max_dt_s of odom_ts; odom before first_gps or after last_gps or nearest_gps_dt>max_dt)",
            ts, gps_manager_.getGpsWindowSize(), max_dt_s, first_gps_ts, last_gps_ts, delay_to_first_gps, gap_to_window);
    }
    auto t_after_gps = KfClock::now();
    double ms_gps = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_gps - t_after_voxel).count();
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP] ts=%.3f step=gps_query done ms=%.1f", ts, ms_gps);

    // 精准定位模糊：关键帧点云过少会导致全局图稀疏/糊
    constexpr size_t kSparseKeyframeThreshold = 500;
    if (cur_cloud->size() < kSparseKeyframeThreshold) {
        RCLCPP_WARN(get_logger(), "[GLOBAL_MAP_BLUR] sparse_keyframe kf_pts=%zu (threshold=%zu) ts=%.3f → 全局图可能稀疏，检查前端 filter_size_surf/盲区",
                    cur_cloud->size(), kSparseKeyframeThreshold, ts);
    }

    auto kf = kf_manager_.createKeyFrame(
        cur_pose, cur_cov, ts,
        cur_cloud, cloud_ds,
        gps, has_gps, current_session_id_);
    auto t_after_create_kf = KfClock::now();
    double ms_create_kf = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_create_kf - t_after_gps).count();
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP] ts=%.3f step=createKeyFrame done kf_id=%lu", ts, kf->id);

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=addKeyFrame_enter ts=%.3f kf_id=%lu sm_id=%d (grep BACKEND STEP 定位卡住)",
                ts, kf->id, kf->submap_id);
    submap_manager_.addKeyFrame(kf);
    auto t_after_add_submap = KfClock::now();
    double ms_add_submap = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_add_submap - t_after_create_kf).count();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=addKeyFrame_exit ts=%.3f duration_ms=%.1f (grep BACKEND STEP 定位卡住)", ts, ms_add_submap);

    // ─────────────────────────────────────────────────────────────────────────
    // 子图内回环检测（新增）
    // ─────────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=intra_loop_enter ts=%.3f (grep BACKEND STEP 定位卡住)", ts);
    auto t_before_intra_loop = KfClock::now();
    {
        auto active_sm = submap_manager_.getActiveSubmap();
        if (active_sm && active_sm->id >= 0) {
            int query_idx = static_cast<int>(active_sm->keyframes.size()) - 1;
            if (query_idx >= 0) {
                RCLCPP_INFO(get_logger(),
                    "[INTRA_LOOP][TRIGGER] submap_id=%d kf_idx=%d ts=%.3f (checking intra-submap loop)",
                    active_sm->id, query_idx, ts);
                auto loops = loop_detector_.detectIntraSubmapLoop(active_sm, query_idx);
                if (!loops.empty()) {
                    RCLCPP_INFO(get_logger(),
                        "[INTRA_LOOP][RESULT] submap_id=%d kf_idx=%d detected=%zu loops (batch commit)",
                        active_sm->id, query_idx, loops.size());
                    // 子图内回环：仅加入 pending，本帧末尾一次 commit，避免每条回环一次 commitAndUpdate 导致 30s+ 卡住
                    for (const auto& lc : loops) {
                        // P3: 使用 MAX_KF_PER_SUBMAP 编码关键帧节点ID
                        int node_i = lc->submap_i * MAX_KF_PER_SUBMAP + lc->keyframe_i;
                        int node_j = lc->submap_j * MAX_KF_PER_SUBMAP + lc->keyframe_j;
                        const auto& kf_i = active_sm->keyframes[lc->keyframe_i];
                        const auto& kf_j = active_sm->keyframes[lc->keyframe_j];
                        if (kf_i && kf_j) {
                            isam2_optimizer_.addSubMapNode(node_i, kf_i->T_w_b, false);
                            isam2_optimizer_.addSubMapNode(node_j, kf_j->T_w_b, false);
                            RCLCPP_INFO(get_logger(),
                                "[INTRA_LOOP][ADD_FACTOR] submap=%d kf_i=%d kf_j=%d node_i=%d node_j=%d (deferred)",
                                lc->submap_i, lc->keyframe_i, lc->keyframe_j, node_i, node_j);
                            isam2_optimizer_.addLoopFactorDeferred(node_i, node_j, lc->delta_T, lc->information);
                        }
                    }
                    // 本帧所有子图内回环因子一次提交，避免 N 次 commitAndUpdate
                    isam2_optimizer_.forceUpdate();
                }
            }
        }
    }
    auto t_after_intra_loop = KfClock::now();
    double ms_intra_loop = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_intra_loop - t_before_intra_loop).count();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=intra_loop_exit ts=%.3f duration_ms=%.1f (grep BACKEND STEP 定位卡住)", ts, ms_intra_loop);
    if (ms_intra_loop > 100.0) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP_TIMING] ts=%.3f step=intra_loop_ms=%.1f (grep KF_STEP_TIMING)", ts, ms_intra_loop);
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=gps_factor_enter ts=%.3f (grep BACKEND STEP 定位卡住)", ts);
    auto t_before_gps_factor = KfClock::now();
    if (gps_aligned_ && has_gps && kf->submap_id >= 0) {
        Eigen::Vector3d pos_map = gps_manager_.enu_to_map(gps.position_enu);
        Eigen::Matrix3d cov = gps.covariance;
        if (cov.norm() < 1e-6 || !std::isfinite(cov(0, 0)))
            cov = Eigen::Matrix3d::Identity() * 1.0;
        RCLCPP_INFO(get_logger(),
            "[GPS_FACTOR_ADDED] kf_id=%lu sm_id=%d pos=[%.2f,%.2f,%.2f] hdop=%.2f (精准分析: 每添加一次 GPS 约束打印)",
            kf->id, kf->submap_id, pos_map.x(), pos_map.y(), pos_map.z(), gps.hdop);
        if (ConfigManager::instance().asyncIsam2Update()) {
            IncrementalOptimizer::OptimTask t;
            t.type = IncrementalOptimizer::OptimTaskType::GPS_FACTOR;
            t.from_id = kf->submap_id;
            t.gps_pos = pos_map;
            t.gps_cov = cov;
            isam2_optimizer_.enqueueOptTask(t);
        } else {
            isam2_optimizer_.addGPSFactor(kf->submap_id, pos_map, cov);
        }
    }
    auto t_after_gps_factor = KfClock::now();
    double ms_gps_factor = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_gps_factor - t_before_gps_factor).count();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=gps_factor_exit ts=%.3f duration_ms=%.1f (grep BACKEND STEP 定位卡住)", ts, ms_gps_factor);
    if (ms_gps_factor > 100.0) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP_TIMING] ts=%.3f step=gps_factor_ms=%.1f (grep KF_STEP_TIMING)", ts, ms_gps_factor);
    }

    // 投递到 viz 线程，不阻塞；有界队列满时丢弃最旧
    {
        std::lock_guard<std::mutex> lk(viz_mutex_);
        if (viz_cloud_queue_.size() >= kVizQueueMaxSize)
            viz_cloud_queue_.pop();
        viz_cloud_queue_.push(cur_cloud);
        viz_cv_.notify_one();
    }

    double ms_total = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(KfClock::now() - kf_t0).count();
    // 分步耗时便于定位卡点：voxel/gps_query/createKf/addKf/intra_loop/gps_factor 等
    if (ms_total > 500.0 || (kf->id <= 3)) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][BACKEND][KF_STEP_TIMING] ts=%.3f kf_id=%lu voxel=%.1f gps=%.1f create=%.1f add=%.1f intra_loop=%.1f gps_fac=%.1f total=%.1f (grep KF_STEP_TIMING 定位慢步骤)",
            ts, kf->id, ms_voxel, ms_gps, ms_create_kf, ms_add_submap, ms_intra_loop, ms_gps_factor, ms_total);
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][KF] created kf_id=%lu sm_id=%d ts=%.3f pts=%zu has_gps=%d total_ms=%.1f",
        kf->id, kf->submap_id, ts, cur_cloud->size(), has_gps ? 1 : 0, ms_total);
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][FRAME] ts=%.3f result=kf_created kf_id=%lu sm_id=%d", ts, kf->id, kf->submap_id);
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PIPELINE] event=kf_created kf_id=%lu sm_id=%d", kf->id, kf->submap_id);
    ALOG_DEBUG(MOD, "KF#{} created: pts={} ds_pts={} has_gps={} livo_degen={}",
               kf->id, cur_cloud->size(), cloud_ds->size(),
               has_gps, livo_info.is_degenerate);
}

// ─────────────────────────────────────────────────────────────────────────────
// 子图冻结处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onSubmapFrozen(const SubMap::Ptr& submap) {
    if (!submap) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][SM] onSubmapFrozen: null submap ignored");
        return;
    }
    frozen_submap_count_++;
    const double ax = submap->pose_w_anchor.translation().x();
    const double ay = submap->pose_w_anchor.translation().y();
    const double az = submap->pose_w_anchor.translation().z();
    (void)ax; (void)ay; (void)az;  // for optional RCLCPP_INFO below
    // RCLCPP_INFO(get_logger(),
    //     "[AutoMapSystem][SM] frozen sm_id=%d kf_count=%zu t=[%.3f,%.3f] dist=%.2fm anchor=[%.2f,%.2f,%.2f] total_frozen=%d",
    //     submap->id, submap->keyframes.size(), submap->t_start, submap->t_end,
    //     submap->spatial_extent_m, ax, ay, az, frozen_submap_count_);
    // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sm_frozen sm_id=%d kfs=%zu total_frozen=%d", submap->id, submap->keyframes.size(), frozen_submap_count_);
    // ALOG_INFO(MOD, "SM#{} FROZEN: kf={} dist={:.1f}m total_frozen={}",
    //           submap->id, submap->keyframes.size(),
    //           submap->spatial_extent_m, frozen_submap_count_);

    // 添加子图节点到 iSAM2
    bool is_first = (isam2_optimizer_.nodeCount() == 0);
    isam2_optimizer_.addSubMapNode(submap->id, submap->pose_w_anchor, is_first);

    // 添加里程计因子（与前一个子图之间）
    auto all_sm = submap_manager_.getFrozenSubmaps();
    if (all_sm.size() >= 2) {
        const auto& prev = all_sm[all_sm.size() - 2];
        if (!prev) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][SM] onSubmapFrozen: prev submap null, skip odom factor sm_id=%d", submap->id);
        } else {
            Pose3d rel = prev->pose_w_anchor_optimized.inverse() * submap->pose_w_anchor;
            Mat66d info = computeOdomInfoMatrix(prev, submap, rel);
            isam2_optimizer_.addOdomFactor(prev->id, submap->id, rel, info);
        }
    }

    // 关键修复：添加节点和因子后立即 forceUpdate，确保节点被提交到 iSAM2 图
    // 避免 GPS 对齐时因 node_count=0 导致所有 GPS 因子被跳过
    auto update_result = isam2_optimizer_.forceUpdate();

    // 增强诊断日志：显示更多信息用于定位问题
    int current_node_count = isam2_optimizer_.nodeCount();
    size_t pending_vals = isam2_optimizer_.pendingValuesCount();
    size_t pending_facts = isam2_optimizer_.pendingFactorsCount();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][SM] onSubmapFrozen: sm_id=%d forceUpdate success=%d nodes=%d "
        "nodeCount=%d pending_values=%zu pending_factors=%zu",
        submap->id, update_result.success, update_result.nodes_updated,
        current_node_count, pending_vals, pending_facts);

    // 如果 forceUpdate 返回 0 节点，说明 GTSAM 单节点延迟问题
    // 需要添加 workaround：手动将节点标记为存在于 current_estimate_
    if (update_result.nodes_updated == 0 && current_node_count > 0) {
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][SM] WARNING: sm_id=%d forceUpdate returned 0 nodes (GTSAM single-node defer issue), "
            "applying workaround to mark pending values as estimated",
            submap->id);
        isam2_optimizer_.markPendingValuesAsEstimated();

        // workaround 后再次尝试 forceUpdate
        auto retry_result = isam2_optimizer_.forceUpdate();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][SM] onSubmapFrozen: sm_id=%d after workaround retry success=%d nodes=%d",
            submap->id, retry_result.success, retry_result.nodes_updated);
    }

    // 建图精度日志：子图冻结时的几何与锚定帧不确定性
    // 修复: 添加NaN检查，防止协方差矩阵元素为NaN时导致sqrt产生NaN
    if (!submap->keyframes.empty()) {
        const KeyFrame::Ptr& anchor_kf = submap->keyframes.front();
        if (anchor_kf) {
            const Mat66d& anchor_cov = anchor_kf->covariance;
            double a_px = std::isfinite(anchor_cov(3, 3)) ? std::sqrt(std::max(0.0, anchor_cov(3, 3))) : 0.0;
            double a_py = std::isfinite(anchor_cov(4, 4)) ? std::sqrt(std::max(0.0, anchor_cov(4, 4))) : 0.0;
            double a_pz = std::isfinite(anchor_cov(5, 5)) ? std::sqrt(std::max(0.0, anchor_cov(5, 5))) : 0.0;
            RCLCPP_INFO(get_logger(),
                "[PRECISION][SUBMAP] frozen sm_id=%d kf_count=%zu extent_m=%.2f anchor_pos_std_xyz=[%.4f,%.4f,%.4f] total_frozen=%d",
                submap->id, submap->keyframes.size(), submap->spatial_extent_m, a_px, a_py, a_pz, frozen_submap_count_);
        } else {
            RCLCPP_INFO(get_logger(),
                "[PRECISION][SUBMAP] frozen sm_id=%d kf_count=%zu extent_m=%.2f (anchor kf null) total_frozen=%d",
                submap->id, submap->keyframes.size(), submap->spatial_extent_m, frozen_submap_count_);
        }
    } else {
        RCLCPP_INFO(get_logger(),
            "[PRECISION][SUBMAP] frozen sm_id=%d kf_count=0 extent_m=%.2f total_frozen=%d",
            submap->id, submap->spatial_extent_m, frozen_submap_count_);
    }

    // 提交到回环检测器
    loop_detector_.addSubmap(submap);

    // ─────────────────────────────────────────────────────────────────────────
    // 子图内回环检测：冻结时准备描述子数据库（新增）
    // ─────────────────────────────────────────────────────────────────────────
    {
        RCLCPP_INFO(get_logger(),
            "[INTRA_LOOP][PREPARE_SUBMAP] submap_id=%d keyframes=%zu (preparing descriptors on freeze)",
            submap->id, submap->keyframes.size());
        loop_detector_.prepareIntraSubmapDescriptors(submap);
    }

    // 延迟 GPS 补偿：子图冻结时注册，GPS 对齐后或回环优化后补偿
    if (gps_compensator_ && gps_compensator_->isEnabled()) {
        gps_compensator_->registerSubmap(submap);
    }

    // HBA 仅在建图结束时触发一次，子图冻结时不再周期触发（见 sensor_idle / finish_mapping）
    // 数据触发：子图冻结后异步发布全局地图，不阻塞后端
    map_publish_pending_.store(true);
    map_publish_cv_.notify_one();
}

// ─────────────────────────────────────────────────────────────────────────────
// 回环检测处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onLoopDetected(const LoopConstraint::Ptr& lc) {
    if (!lc) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][LOOP] onLoopDetected: null constraint ignored");
        return;
    }
    {
        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
        loop_constraints_.push_back(lc);
        if (loop_constraints_.size() > 500) loop_constraints_.erase(loop_constraints_.begin());
    }
    state_ = SystemState::LOOP_CLOSING;
    const double tx = lc->delta_T.translation().x();
    const double ty = lc->delta_T.translation().y();
    const double tz = lc->delta_T.translation().z();
    const double info_norm = lc->information.norm();
    // 同子图同节点回环无效：后端图节点按 submap_id，sm_i==sm_j 即 from==to，Between 退化
    if (lc->submap_i == lc->submap_j) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP] skip same-submap loop sm_i=sm_j=%d (invalid Between factor; quality filter)", lc->submap_i);
        state_ = SystemState::MAPPING;
        return;
    }
    // 回环质量：相对平移过小视为 trivial 约束，不加入图（可配置 loop_closure.teaser.min_relative_translation_m）
    const double trans_norm = std::sqrt(tx*tx + ty*ty + tz*tz);
    const double min_trans_m = ConfigManager::instance().loopMinRelativeTranslationM();
    if (min_trans_m > 0 && trans_norm < min_trans_m) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP] skip trivial loop sm_i=%d sm_j=%d trans_norm=%.3fm < %.2fm (quality filter)",
            lc->submap_i, lc->submap_j, trans_norm, min_trans_m);
        state_ = SystemState::MAPPING;
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][LOOP] detected sm_i=%d sm_j=%d score=%.3f inlier=%.3f rmse=%.3f trans=[%.2f,%.2f,%.2f] info_norm=%.2f (enqueue for async iSAM2)",
        lc->submap_i, lc->submap_j, lc->overlap_score, lc->inlier_ratio, lc->rmse, tx, ty, tz, info_norm);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=loop_detected sm_i=%d sm_j=%d score=%.3f rmse=%.3f", lc->submap_i, lc->submap_j, lc->overlap_score, lc->rmse);
    ALOG_INFO(MOD, "Loop detected: SM#{} ↔ SM#{} score={:.3f} trans=[{:.2f},{:.2f},{:.2f}] (enqueue)",
              lc->submap_i, lc->submap_j, lc->overlap_score, tx, ty, tz);

    // 增强诊断：记录当前 ISAM2 状态
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][LOOP][DIAG] before enqueue: nodeCount=%d factorCount=%d pendingValues=%zu pendingFactors=%zu",
        isam2_optimizer_.nodeCount(), isam2_optimizer_.factorCount(),
        isam2_optimizer_.pendingValuesCount(), isam2_optimizer_.pendingFactorsCount());

    // performance.async_isam2_update: 直接入队 iSAM2 异步执行，不经过 loop_opt_thread
    if (ConfigManager::instance().asyncIsam2Update()) {
        IncrementalOptimizer::OptimTask t;
        t.type = IncrementalOptimizer::OptimTaskType::LOOP_FACTOR;
        t.from_id = lc->submap_i;
        t.to_id = lc->submap_j;
        t.rel_pose = lc->delta_T;
        t.info_matrix = lc->information;
        isam2_optimizer_.enqueueOptTask(t);

        // 增强诊断：记录入队后状态
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][LOOP][DIAG] async enqueue done: queue_depth=%zu",
            isam2_optimizer_.getQueueDepth());

        state_ = SystemState::MAPPING;
        return;
    }

    // 入队由 loop_opt_thread_ 执行 addLoopFactor，避免阻塞 match_worker；有界队列+超时防死锁
    {
        std::unique_lock<std::mutex> lk(loop_opt_mutex_);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        bool enqueued = false;
        while (loop_factor_queue_.size() >= kMaxLoopFactorQueueSize && !shutdown_requested_.load(std::memory_order_acquire)) {
            if (loop_opt_cv_.wait_until(lk, deadline, [this] {
                return loop_factor_queue_.size() < kMaxLoopFactorQueueSize || shutdown_requested_.load(std::memory_order_acquire);
            })) {
                enqueued = true;
                break;
            }
            break;  // 超时退出循环
        }
        if (loop_factor_queue_.size() < kMaxLoopFactorQueueSize) {
            loop_factor_queue_.push(lc);
            enqueued = true;
        }
        if (!enqueued) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][LOOP] loop factor queue full (max=%zu), dropping loop sm_i=%d sm_j=%d (apply sync fallback)", kMaxLoopFactorQueueSize, lc->submap_i, lc->submap_j);
            lk.unlock();
            auto result = isam2_optimizer_.addLoopFactor(lc->submap_i, lc->submap_j, lc->delta_T, lc->information);

            // 增强诊断：记录同步执行结果
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][LOOP] sync fallback: sm_i=%d sm_j=%d success=%d nodes_updated=%d elapsed=%.1fms",
                lc->submap_i, lc->submap_j, result.success ? 1 : 0, result.nodes_updated, result.elapsed_ms);

            state_ = SystemState::MAPPING;
            return;
        }
        loop_opt_cv_.notify_one();
    }
    state_ = SystemState::MAPPING;  // 先恢复 MAPPING，实际优化在 loop_opt_thread 中完成
}

// ─────────────────────────────────────────────────────────────────────────────
// 位姿更新（来自 iSAM2）
// ─────────────────────────────────────────────────────────────────────────────
// P3 修复：区分子图级节点和关键帧级节点
// 使用数值范围来区分节点类型：
//   - 子图级: 0 <= id < MAX_KF_PER_SUBMAP
//   - 关键帧级: id >= MAX_KF_PER_SUBMAP
// 这样避免了原方案使用数值>=10000判断导致的潜在冲突问题

void AutoMapSystem::onPoseUpdated(const std::unordered_map<int, Pose3d>& poses) {
    const size_t n = poses.size();
    int first_id = -1;
    double first_x = 0, first_y = 0, first_z = 0;
    
    // P3: 区分子图级节点('s')和关键帧级节点('k')
    // 由于 GTSAM Symbol 使用字符区分，这里通过解析 node_id 的范围来判断
    // 子图级: 0 <= id < MAX_KF_PER_SUBMAP (子图数量有限)
    // 关键帧级: id >= MAX_KF_PER_SUBMAP (子图ID * MAX_KF_PER_SUBMAP + keyframe_index)
    std::vector<std::pair<int, Pose3d>> submap_poses;  // 子图级节点
    std::vector<std::pair<int, Pose3d>> kf_poses;      // 关键帧级节点
    
    for (const auto& [node_id, pose] : poses) {
        // 判断是子图级还是关键帧级
        if (node_id >= MAX_KF_PER_SUBMAP) {
            kf_poses.emplace_back(node_id, pose);
        } else {
            submap_poses.emplace_back(node_id, pose);
        }
    }
    
    // P3 诊断日志：记录两种节点的数量
    if (n > 0) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][POSE][SPLIT] total=%zu submap_nodes=%zu kf_nodes=%zu",
            n, submap_poses.size(), kf_poses.size());
    }
    
    // 处理子图级节点：更新子图锚点
    for (const auto& [sm_id, pose] : submap_poses) {
        if (first_id < 0) {
            first_id = sm_id;
            first_x = pose.translation().x();
            first_y = pose.translation().y();
            first_z = pose.translation().z();
        }
        submap_manager_.updateSubmapPose(sm_id, pose);
    }
    
    // P3: 处理关键帧级节点：更新活跃子图中对应关键帧的 T_w_b_optimized
    // 关键帧级节点 ID = submap_id * MAX_KF_PER_SUBMAP + keyframe_index
    // 注意：iSAM2 返回的关键帧级节点的 Pose 是优化后的全局位姿（世界坐标系）
    // 应该直接使用 kf_pose 作为 T_w_b_optimized
    if (!kf_poses.empty()) {
        auto active_sm = submap_manager_.getActiveSubmap();
        if (active_sm && active_sm->state == SubMapState::ACTIVE) {
            // 更新活跃子图中对应关键帧的 T_w_b_optimized
            int updated_kf_count = 0;
            for (const auto& [node_id, kf_pose] : kf_poses) {
                // 解析 node_id 获取 submap_id 和 keyframe_index
                int kf_sm_id = node_id / MAX_KF_PER_SUBMAP;
                int kf_idx = node_id % MAX_KF_PER_SUBMAP;
                
                // 只处理活跃子图的关键帧
                if (kf_sm_id == active_sm->id && kf_idx < static_cast<int>(active_sm->keyframes.size())) {
                    auto& kf = active_sm->keyframes[kf_idx];
                    if (kf) {
                        // 直接使用 iSAM2 返回的优化位姿（全局坐标）
                        kf->T_w_b_optimized = kf_pose;
                        updated_kf_count++;
                    }
                }
            }
            if (updated_kf_count > 0) {
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][POSE][KF] updated %d keyframes in active submap %d (from iSAM2 kf-nodes, global pose)",
                    updated_kf_count, active_sm->id);
            }
        } else {
            RCLCPP_DEBUG(get_logger(),
                "[AutoMapSystem][POSE][KF] no active submap, skip kf-node updates");
        }
    }
    
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][POSE] updated count=%zu first_sm_id=%d pos=[%.2f,%.2f,%.2f]",
        n, first_id, first_x, first_y, first_z);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=pose_updated count=%zu first_sm_id=%d", n, first_id);

    // ========== 实时更新 GPS 显示 ==========
    // 每次 iSAM2 优化后，实时更新 GPS 标记位置，使用 KeyFrame 优化位姿
    if (gps_aligned_) {
        try {
            auto all_sm = submap_manager_.getAllSubmaps();

            // SubMap 级别 GPS 标记 - 使用优化后的 anchor 位姿
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            for (const auto& sm : all_sm) {
                if (!sm || !sm->has_valid_gps) continue;
                gps_positions_map_for_submaps.push_back(sm->pose_w_anchor_optimized.translation());
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
            }

            // KeyFrame 级别 GPS 位置 - 使用每个 KeyFrame 的优化位姿
            std::vector<Eigen::Vector3d> gps_positions_map;
            for (const auto& sm : all_sm) {
                if (!sm) continue;
                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->has_valid_gps) continue;
                    gps_positions_map.push_back(kf->T_w_b_optimized.translation());
                }
            }
            if (!gps_positions_map.empty()) {
                rviz_publisher_.publishGPSPositionsInMap(gps_positions_map);
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][POSE][GPS] update exception: %s", e.what());
        } catch (...) {}
    }

    // ✅ P3 修复：发布优化后轨迹（按 submap_id 排序，保证 Path 在 RViz 中按顺序连线）
    opt_path_.header.stamp    = now();
    opt_path_.header.frame_id = "map";
    opt_path_.poses.clear();
    
    // P3：将 unordered_map 转为 sorted vector，按 submap_id 升序
    // 确保 RViz 中轨迹按子图顺序连接，不会"乱跳"
    std::vector<std::pair<int, Pose3d>> sorted(poses.begin(), poses.end());
    std::sort(sorted.begin(), sorted.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    for (const auto& [sm_id, pose] : sorted) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = opt_path_.header;
        ps.pose.position.x = pose.translation().x();
        ps.pose.position.y = pose.translation().y();
        ps.pose.position.z = pose.translation().z();
        Eigen::Quaterniond q(pose.rotation());
        ps.pose.orientation.w = q.w(); ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z();
        opt_path_.poses.push_back(ps);
    }
    if (opt_path_pub_) {
        try { opt_path_pub_->publish(opt_path_); } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][POSE] opt_path publish: %s", e.what());
        } catch (...) {}
        pub_opt_path_count_++;
    }

    try {
        auto all_sm = submap_manager_.getAllSubmaps();
        rviz_publisher_.publishOptimizedPath(all_sm);
        rviz_publisher_.publishKeyframePoses(collectKeyframesFromSubmaps(all_sm));
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishOptimizedPath: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishOptimizedPath: unknown exception");
    }

}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onHBADone(const HBAResult& result) {
    if (!result.success) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][HBA][EXCEPTION] optimization failed");
        RCLCPP_INFO(get_logger(), "[TRACE] step=hba_done result=fail reason=optimization_failed (精准定位: 见 [HBA][BACKEND] 或 stderr 中 HBA failed 详情)");
        return;
    }
    const size_t pose_count = result.optimized_poses.size();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][HBA] done success=1 MME=%.4f poses=%zu iter_layer=%d elapsed=%.1fms",
        result.final_mme, pose_count, result.iterations_per_layer, result.elapsed_ms);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=hba_done MME=%.4f poses=%zu elapsed=%.0fms", result.final_mme, pose_count, result.elapsed_ms);

    auto all_sm = submap_manager_.getFrozenSubmaps();
    // 在写回 HBA 结果之前采样 iSAM2 当前估计，用于分离度诊断（pose_w_anchor 为冻结初值，非 iSAM2 估计）
    std::unordered_map<int, Pose3d> isam2_poses_before_hba;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        isam2_poses_before_hba[sm->id] = isam2_optimizer_.getPose(sm->id);
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][HBA] isam2_poses_sampled=%zu (before updateAllFromHBA, for separation metric)",
        isam2_poses_before_hba.size());

    // 将 HBA 优化结果写回所有子图的关键帧（用于显示和全局图构建）
    submap_manager_.updateAllFromHBA(result);

    try {
        rviz_publisher_.publishHBAResult(result);
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishHBAResult: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishHBAResult: unknown exception");
    }

    // ✅ P2 修复：HBA 与 iSAM2 两轨架构说明与诊断
    // ─────────────────────────────────────────────────────────────────────────────
    // 设计：AutoMap Pro 采用"两轨并行优化"以平衡精度与性能
    //   - iSAM2 轨：增量优化，快速（O(1) 摊销），用于因子图约束
    //   - HBA 轨：离线批量优化，精准（O(k²)），用于显示/全局图/导出
    // 
    // 当前行为：HBA 完成后尝试同步到 iSAM2，但 iSAM2 的 addSubMapNode 有重复检查
    // 导致同步失败 → iSAM2 保持独立估计（略滞后但可接受）
    //
    // 坐标系一致性：✅ 两轨均在同一世界系（camera_init），无混系问题
    // 
    // 使用指导：
    //   - 显示/导出：使用 HBA 结果（当前做法 ✅）
    //   - 新因子约束：基于 iSAM2 估计（快速约束）
    //   - 诊断：定期检查两轨位姿差值（见下，使用 updateAllFromHBA 前采样的 iSAM2 估计）
    // ─────────────────────────────────────────────────────────────────────────────
    
    // 位姿/点云已更新：刷新 RViz 优化轨迹与关键帧位姿，便于实时看到 HBA 结果
    // ========== HBA 完成后更新 GPS 显示 ==========
    if (gps_aligned_) {
        try {
            // SubMap 级别 GPS 标记 - 使用优化后的 anchor 位姿
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            for (const auto& sm : all_sm) {
                if (!sm || !sm->has_valid_gps) continue;
                gps_positions_map_for_submaps.push_back(sm->pose_w_anchor_optimized.translation());
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
            }

            // KeyFrame 级别 GPS 位置 - 使用每个 KeyFrame 的优化位姿
            std::vector<Eigen::Vector3d> gps_positions_map;
            for (const auto& sm : all_sm) {
                if (!sm) continue;
                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->has_valid_gps) continue;
                    gps_positions_map.push_back(kf->T_w_b_optimized.translation());
                }
            }
            if (!gps_positions_map.empty()) {
                rviz_publisher_.publishGPSPositionsInMap(gps_positions_map);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][HBA][GPS] update exception: %s", e.what());
        } catch (...) {}
    }

    try {
        rviz_publisher_.publishOptimizedPath(all_sm);
        rviz_publisher_.publishKeyframePoses(collectKeyframesFromSubmaps(all_sm));
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] HBA viz refresh: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] HBA viz refresh: unknown exception");
    }
    
    // 诊断：计算 HBA 与 iSAM2 的位姿分离程度（iSAM2 为 updateAllFromHBA 前的当前估计）
    double max_drift = 0.0;
    double sum_drift = 0.0;
    const double kDriftTraceThreshold = 0.05;  // 单子图 drift > 5cm 时打一条 trace 便于定位
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        Pose3d hba_pose = sm->pose_w_anchor_optimized;  // 已为 HBA 结果
        auto it = isam2_poses_before_hba.find(sm->id);
        Pose3d isam2_pose = (it != isam2_poses_before_hba.end()) ? it->second : Pose3d::Identity();
        double drift = (hba_pose.translation() - isam2_pose.translation()).norm();
        max_drift = std::max(max_drift, drift);
        sum_drift += drift;
        if (drift > kDriftTraceThreshold) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][HBA] separation sm_id=%d drift=%.3fm (grep BACKEND HBA separation 定位大偏差子图)",
                sm->id, drift);
        }
    }
    
    double avg_drift = all_sm.empty() ? 0.0 : sum_drift / all_sm.size();
    
    if (max_drift > 0.1) {  // 10cm 阈值
        ALOG_WARN(MOD,
            "P2 HBA-iSAM2 separation: max_drift={:.3f}m, avg_drift={:.3f}m\n"
            "  HBA 优化后，iSAM2 仍保持独立估计（设计选择：优先性能）\n"
            "  两轨坐标系一致（无混系问题），但因子图估计滞后\n"
            "  建议：若精度关键，可在此处调用 iSAM2 强制同步（可选），参见代码注释",
            max_drift, avg_drift);
        METRICS_GAUGE_SET(metrics::HBA_ISAM2_SEPARATION_M, max_drift);
    } else {
        ALOG_DEBUG(MOD, "P2 HBA-iSAM2 drift acceptable: max={:.3f}m", max_drift);
    }
    
    // 尝试同步 iSAM2：使用新添加的 updateSubMapNodePose 方法强制更新
    int synced_count = 0;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        isam2_optimizer_.updateSubMapNodePose(sm->id, sm->pose_w_anchor_optimized);
        synced_count++;
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][HBA] synced %d nodes to iSAM2 (using updateSubMapNodePose)", synced_count);
    
    ALOG_INFO(MOD,
        "P2: HBA done, iSAM2 estimated (separate tracks). "
        "Displays use HBA results; factor graph uses iSAM2 estimates");
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=hba_isam2_status "
        "hba_poses=%zu isam2_nodes=%zu separation_m=%.3f", 
        pose_count, all_sm.size(), max_drift);
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS 对齐完成处理（延迟对齐核心）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onGPSAligned(const GPSAlignResult& result) {
    if (!result.success) return;

    gps_aligned_ = true;
    try {
        auto all_sm = submap_manager_.getAllSubmaps();
        rviz_publisher_.publishGPSAlignment(result, all_sm);

        // ========== 修复：使用 KeyFrame 优化位姿显示 GPS ==========
        // 原始方案：显示的是 GPS 原始位置 (enu_to_map(gps.position_enu))
        // 新方案：显示的是 KeyFrame 优化后的位姿 (T_w_b_optimized)
        // 这样 GPS 标记会跟随后端优化移动，与优化轨迹一致

        // 1. SubMap 级别的 GPS 标记和轨迹 - 使用优化后的 anchor 位姿
        std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
        for (const auto& sm : all_sm) {
            if (!sm || !sm->has_valid_gps) continue;
            // 使用 SubMap anchor 的优化位姿（而不是原始 GPS 位置）
            gps_positions_map_for_submaps.push_back(sm->pose_w_anchor_optimized.translation());
        }
        if (!gps_positions_map_for_submaps.empty()) {
            rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
            rviz_publisher_.publishGPSTrajectory(all_sm, gps_positions_map_for_submaps, true);
        }

        // 2. KeyFrame 级别的 GPS 位置 - 使用每个 KeyFrame 的优化位姿
        std::vector<Eigen::Vector3d> gps_positions_map;
        for (const auto& sm : all_sm) {
            if (!sm) continue;
            for (const auto& kf : sm->keyframes) {
                if (!kf || !kf->has_valid_gps) continue;
                // 使用 KeyFrame 的优化位姿（而不是原始 GPS ENU 位置）
                gps_positions_map.push_back(kf->T_w_b_optimized.translation());
            }
        }
        if (!gps_positions_map.empty())
            rviz_publisher_.publishGPSPositionsInMap(gps_positions_map);
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishGPSAlignment: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishGPSAlignment: unknown exception");
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][GPS] aligned rmse_m=%.2f matched=%d R_diag=[%.2f,%.2f,%.2f] t=[%.2f,%.2f,%.2f]",
        result.rmse_m, result.matched_points,
        result.R_gps_lidar(0,0), result.R_gps_lidar(1,1), result.R_gps_lidar(2,2),
        result.t_gps_lidar.x(), result.t_gps_lidar.y(), result.t_gps_lidar.z());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_aligned rmse_m=%.3f matched=%d", result.rmse_m, result.matched_points);

    // 批量为历史子图补充 GPS 因子（gps.add_constraints_on_align=false 时跳过，仅 HBA 用 GPS，用于隔离）
    addBatchGPSFactors();

    // 仅更新 HBA 的 GPS 对齐状态，不在此处触发 HBA；整段建图只在结束时做一次 HBA（sensor_idle / finish_mapping）
    hba_optimizer_.setGPSAlignedState(result);
}

void AutoMapSystem::addBatchGPSFactors() {
    if (gps_batch_added_) return;
    gps_batch_added_ = true;

    // gps.add_constraints_on_align=false 时不向 ISAM2 添加 GPS 因子（仅 HBA 用 GPS，用于隔离双路 GTSAM）
    if (!ConfigManager::instance().gpsAddConstraintsOnAlign()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] skipped (gps.add_constraints_on_align=false, ISAM2 no GPS)");
        return;
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] enter (grep GPS_BATCH 可追踪批量 GPS 因子与 waitForPendingTasks)");

    auto all_sm = submap_manager_.getFrozenSubmaps();
    int added = 0;
    int historical_bound = 0;
    
    // 1. 首先处理已有 GPS 的子图（原有逻辑）
    // ✅ 修复：无论 async_isam2_update 设置如何，都使用同步模式添加 GPS 因子
    // 避免在 GPS 批量处理时触发多线程竞态导致 SIGSEGV
    {
        // 先等待所有待处理任务完成
        isam2_optimizer_.waitForPendingTasks();

        // 关键修复：对齐后先 forceUpdate，确保首节点（及已有 pending）已提交，避免 addGPSFactor(sm_id=0) 报 "node not in graph"
        // 再根据需要 flush 与 GPS 同 key 的 pending，避免 Prior+GPS 同批 update 触发 GTSAM double free
        isam2_optimizer_.forceUpdate();
        if (isam2_optimizer_.hasPendingFactorsOrValues()) {
            RCLCPP_INFO(get_logger(), "[GPS_BATCH][DIAG] flush pending before adding GPS (avoid Prior+GPS same key in one update)");
            isam2_optimizer_.forceUpdate();
        }

        for (const auto& sm : all_sm) {
            if (!sm || !sm->has_valid_gps) continue;
            auto pos_map = gps_manager_.enu_to_map(sm->gps_center);
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1.0;
            isam2_optimizer_.addGPSFactor(sm->id, pos_map, cov);
            added++;
        }
        if (added > 0) {
            isam2_optimizer_.forceUpdate();
        }
        // Flush deferred GPS factors (e.g. sm_id=0 when node was not in graph earlier) after first forceUpdate may have committed nodes
        isam2_optimizer_.forceUpdate();
        if (added > 0) {
            RCLCPP_INFO(get_logger(), "[GPS_BATCH][DIAG] phase=existing_sync added=%d (forced sync to avoid SIGSEGV)", added);
        }
    }

    // 2. 【新增】为 GPS 延迟到达场景下的历史关键帧补充 GPS 约束
    // 收集所有没有 GPS 的子图的时间戳（用子图起始时间查询 GPS）
    std::vector<std::pair<double, int>> kf_without_gps;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        if (!sm->has_valid_gps && sm->t_start > 0) {
            kf_without_gps.emplace_back(sm->t_start, sm->id);
        }
    }
    
    if (!kf_without_gps.empty()) {
        RCLCPP_INFO(get_logger(), 
            "[AutoMapSystem][GPS_HISTORICAL] attempting to bind %zu submaps without GPS",
            kf_without_gps.size());
        
        // 调用 GPSManager 为历史帧查找 GPS 绑定
        auto bindings = gps_manager_.getHistoricalGPSBindings(kf_without_gps, 0.5);
        
        // 历史绑定位置必须转为 map 系（addGPSFactor 约束的是 map 系），此处已在对齐成功回调内故 enu_to_map 有效
        std::vector<IncrementalOptimizer::GPSFactorItem> hist_items;
        for (const auto& [submap_id, gps_meas] : bindings) {
            if (!gps_meas.is_valid) continue;
            hist_items.push_back({ submap_id, gps_manager_.enu_to_map(gps_meas.position_enu), gps_meas.covariance });
            historical_bound++;
        }
        
        if (historical_bound > 0) {
            // ✅ 修复：强制使用同步模式添加历史 GPS 因子，避免多线程竞态导致 SIGSEGV
            RCLCPP_INFO(get_logger(), "[GPS_BATCH][DIAG] phase=historical_sync added=%d (forced sync to avoid SIGSEGV)", historical_bound);
            isam2_optimizer_.waitForPendingTasks();
            if (isam2_optimizer_.hasPendingFactorsOrValues()) {
                RCLCPP_INFO(get_logger(), "[GPS_BATCH][DIAG] flush pending before historical GPS");
                isam2_optimizer_.forceUpdate();
            }
            for (const auto& f : hist_items)
                isam2_optimizer_.addGPSFactor(f.sm_id, f.pos, f.cov);
            isam2_optimizer_.forceUpdate();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][GPS_HISTORICAL] successfully bound %d/%zu submaps (sync mode)",
                historical_bound, kf_without_gps.size());
        }
    }

    int total_added = added + historical_bound;
    if (total_added > 0) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][GPS] batch factors added: existing_gps=%d historical=%d total=%d frozen=%zu",
            added, historical_bound, total_added, all_sm.size());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_batch_factors_added count=%d", total_added);
    }
}

void AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA() {
    // 设计约束：必须先完成后端 GTSAM（iSAM2）优化提交并释放 GTSAM 状态，再启动 HBA 做一次最终全局优化，以保证全局一致性。
    // 原因：(1) 后端与 HBA 共用 GTSAM，不能同时持有；(2) HBA 需要基于当前已提交的因子图做 batch PGO；(3) 先 flush 再 HBA 可避免 double free 与状态不一致。
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][HBA][DESIGN] 必须先完成后端 iSAM2 提交并释放 GTSAM，再启动 HBA 做最终全局优化以保证全局一致性（flush 完成后再 trigger HBA）");
    size_t qd = isam2_optimizer_.getQueueDepth();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][HBA][TRACE] step=ensureBackendCompletedAndFlushBeforeHBA_enter queue_depth=%zu (崩溃时 grep TRACE 定位)",
        qd);
    RCLCPP_INFO(get_logger(),
        "[CRASH_CONTEXT] step=ensureBackend_enter queue_depth=%zu (崩溃时最后一条 CRASH_CONTEXT 即上一成功步骤)",
        qd);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][HBA] ensureBackendCompletedAndFlushBeforeHBA: wait for backend idle then flush");
    RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=ensureBackend_waitForPendingTasks_enter");
    isam2_optimizer_.waitForPendingTasks();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][HBA][TRACE] step=waitForPendingTasks_done");
    RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=ensureBackend_waitForPendingTasks_done");
    bool has_pending = isam2_optimizer_.hasPendingFactorsOrValues();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][HBA][TRACE] step=ensureBackend_has_pending has_pending=%d (1=将调用 forceUpdate)",
        has_pending ? 1 : 0);
    RCLCPP_INFO(get_logger(),
        "[CRASH_CONTEXT] step=ensureBackend_has_pending_done has_pending=%d", has_pending ? 1 : 0);
    if (has_pending) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][HBA] flush backend pending before HBA (release GTSAM state) has_pending=1 → calling forceUpdate (若崩溃在 forceUpdate 内见 ISAM2_DIAG TRACE)");
        RCLCPP_INFO(get_logger(),
            "[CRASH_CONTEXT] step=ensureBackend_before_forceUpdate (若崩溃则发生在 forceUpdate/commitAndUpdate 内，见 ISAM2_DIAG CRASH_CONTEXT)");
        isam2_optimizer_.forceUpdate();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][HBA][TRACE] step=forceUpdate_after_flush_done");
        RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=ensureBackend_after_forceUpdate");
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=backend_flushed_before_hba (HBA will use GTSAM after backend release)");
    RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=ensureBackend_exit");
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishStatus() {
    try {
        const int kf_count = submap_manager_.keyframeCount();
        const int sm_count = submap_manager_.submapCount();
        const int loop_ok  = loop_detector_.loopDetectedCount();
        const int hba_trig = hba_optimizer_.triggerCount();
        const bool hba_busy = hba_optimizer_.isRunning();
        const double last_ts = livo_bridge_.lastOdomTs();

        automap_pro::msg::MappingStatusMsg msg;
        rclcpp::Time ts;
        try { ts = now(); } catch (...) { ts = rclcpp::Time(0); }
        msg.header.stamp  = ts;
        msg.state         = stateToString(state_.load());
        msg.session_id    = current_session_id_;
        msg.keyframe_count = kf_count;
        msg.submap_count  = sm_count;
        msg.gps_aligned   = gps_aligned_;
        msg.gps_alignment_score = gps_manager_.alignResult().rmse_m > 0 ?
            static_cast<float>(1.0 / gps_manager_.alignResult().rmse_m) : 0.0f;
        if (status_pub_) {
            status_pub_->publish(msg);
            pub_status_count_++;
        }

        // 每秒输出后端状态到终端，便于精准分析建图/GPS/回环
        const size_t hba_queue = hba_optimizer_.queueDepth();
        const size_t gps_win = gps_manager_.getGpsWindowSize();
        size_t frame_q = 0;
        { std::lock_guard<std::mutex> lk(frame_queue_mutex_); frame_q = frame_queue_.size(); }
        const int force_drops = backpressure_force_drop_count_.load(std::memory_order_relaxed);
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][BACKEND] state=%s kf=%d sm=%d loop=%d gps_win=%zu frame_queue=%zu force_drops=%d | [HBA] trig=%d busy=%d queue=%zu last_ts=%.1f",
            msg.state.c_str(), kf_count, sm_count, loop_ok, gps_win, frame_q, force_drops, hba_trig, hba_busy ? 1 : 0, hba_queue, last_ts);

        if (++status_publish_count_ >= 5) {
            status_publish_count_ = 0;
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][PIPELINE] event=heartbeat state=%s kf=%d sm=%d gps=%d gps_win=%zu",
                msg.state.c_str(), kf_count, sm_count, gps_aligned_ ? 1 : 0, gps_win);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][STATUS] publishStatus exception: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][STATUS] publishStatus unknown exception");
    }
}

void AutoMapSystem::publishGlobalMap() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=enter");
    const float voxel_size = map_voxel_size_;
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=using_cached_voxel voxel_size=%.3f", voxel_size);
    size_t pts = 0;
    try {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_enter");
        CloudXYZIPtr global;
        if (ConfigManager::instance().asyncGlobalMapBuild()) {
            global = submap_manager_.buildGlobalMapAsync(voxel_size).get();
        } else {
            global = submap_manager_.buildGlobalMap(voxel_size);
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_done pts=%zu", global ? global->size() : 0u);
        if (!global || global->empty()) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP] global map empty, skip publish");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=map_publish_skipped reason=empty");
            return;
        }
        pts = global->size();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=map_published points=%zu voxel=%.3f", pts, voxel_size);
        // [GLOBAL_MAP_DIAG] 发布环节：确认 frame_id 与点数，便于与 RViz Fixed Frame 对照
        const std::string map_frame_id = "map";
        RCLCPP_INFO(get_logger(), "[GLOBAL_MAP_DIAG] publish /automap/global_map frame_id=%s pts=%zu (RViz Fixed Frame 需与此一致)",
            map_frame_id.c_str(), pts);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=toROSMsg_enter pts=%zu", pts);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*global, cloud_msg);
        cloud_msg.header.stamp    = now();
        cloud_msg.header.frame_id = map_frame_id;
        if (global_map_pub_) {
            try { global_map_pub_->publish(cloud_msg); } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][MAP] global_map publish: %s", e.what());
            } catch (...) {}
            pub_map_count_++;
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=global_map_pub_done");

        // 综合可视化：子图边界/框/连接图、回环、GPS、坐标轴
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=rviz_global_map_enter");
        rviz_publisher_.publishGlobalMap(global);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=rviz_global_map_done");
        auto all_sm = submap_manager_.getAllSubmaps();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=getAllSubmaps_done sm_count=%zu", all_sm.size());
        rviz_publisher_.publishSubmapBoundaries(all_sm);
        rviz_publisher_.publishSubmapBoundingBoxes(all_sm);
        rviz_publisher_.publishSubmapGraph(all_sm);
        rviz_publisher_.publishOptimizedPath(all_sm);
        rviz_publisher_.publishKeyframePoses(collectKeyframesFromSubmaps(all_sm));
        // GPS 约束：仅在对齐后使用 map 系位置，避免未对齐时把 ENU 标成 map
        // ========== 修复：使用 KeyFrame 优化位姿而非原始 GPS 位置 ==========
        {
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            if (gps_aligned_) {
                for (const auto& sm : all_sm) {
                    if (!sm || !sm->has_valid_gps) continue;
                    // 使用 SubMap anchor 的优化位姿（而不是原始 GPS ENU 位置）
                    gps_positions_map_for_submaps.push_back(sm->pose_w_anchor_optimized.translation());
                }
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
                rviz_publisher_.publishGPSTrajectory(all_sm, gps_positions_map_for_submaps, true, "map");
            } else {
                rviz_publisher_.publishGPSMarkers(all_sm);
                rviz_publisher_.publishGPSTrajectory(all_sm, true);  // raw 使用 gps_center(ENU)，frame_id=enu
            }
        }
        // 真实 GPS 位置（按关键帧）转换到地图系后发布（仅对齐后有效）
        // ========== 修复：使用 KeyFrame 优化位姿而非原始 GPS 位置 ==========
        if (gps_aligned_) {
            std::vector<Eigen::Vector3d> gps_positions_map;
            for (const auto& sm : all_sm) {
                if (!sm) continue;
                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->has_valid_gps) continue;
                    // 使用 KeyFrame 的优化位姿（而不是原始 GPS ENU 位置）
                    gps_positions_map.push_back(kf->T_w_b_optimized.translation());
                }
            }
            if (!gps_positions_map.empty())
                rviz_publisher_.publishGPSPositionsInMap(gps_positions_map);
        }
        {
            std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
            rviz_publisher_.publishLoopMarkers(loop_constraints_, all_sm);
        }
        // 因子图可视化：由当前子图 + 回环约束构建临时 PoseGraph（过滤 null 避免 sorted.front() 空指针崩溃）
        std::vector<SubMap::Ptr> sorted;
        for (const auto& s : all_sm) if (s) sorted.push_back(s);
        std::sort(sorted.begin(), sorted.end(),
            [](const SubMap::Ptr& a, const SubMap::Ptr& b) { return a->id < b->id; });
        if (!sorted.empty()) {
            PoseGraph graph;
            const int first_id = sorted.front()->id;
            for (const auto& sm : sorted) {
                GraphNode n;
                n.id = sm->id;
                n.pose = sm->pose_w_anchor_optimized;
                n.pose_opt = sm->pose_w_anchor_optimized;
                n.fixed = (sm->id == first_id);
                graph.addNode(n);
            }
            for (size_t i = 0; i + 1 < sorted.size(); ++i) {
                GraphEdge e;
                e.from = sorted[i]->id;
                e.to = sorted[i + 1]->id;
                e.type = EdgeType::ODOM;
                graph.addEdge(e);
            }
            {
                std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
                for (const auto& lc : loop_constraints_) {
                    if (!lc || !graph.hasNode(lc->submap_i) || !graph.hasNode(lc->submap_j)) continue;
                    GraphEdge e;
                    e.from = lc->submap_i;
                    e.to = lc->submap_j;
                    e.type = EdgeType::LOOP;
                    graph.addEdge(e);
                }
            }
            for (const auto& sm : sorted) {
                if (!sm || !sm->has_valid_gps || !graph.hasNode(sm->id)) continue;
                GraphEdge e;
                e.from = sm->id;
                e.to = -1;  // 一元因子
                e.type = EdgeType::GPS;
                graph.addEdge(e);
            }
            rviz_publisher_.publishFactorGraph(graph);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=factor_graph_done");
        }

        Pose3d base_pose;
        { std::lock_guard<std::mutex> lk(data_mutex_); base_pose = last_odom_pose_; }
        rviz_publisher_.publishCoordinateFrames(base_pose, base_pose);

        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=done points=%zu voxel=%.3f", pts, voxel_size);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap failed: %s", e.what());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=map_publish_failed reason=exception");
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap failed: unknown exception");
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=map_publish_failed reason=unknown");
    }
}

std::vector<KeyFrame::Ptr> AutoMapSystem::collectKeyframesFromSubmaps(const std::vector<SubMap::Ptr>& submaps) {
    std::vector<KeyFrame::Ptr> out;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (kf) out.push_back(kf);
        }
    }
    std::sort(out.begin(), out.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });
    return out;
}

void AutoMapSystem::publishDataFlowSummary() {
    // 在定时器内做 path 裁剪，避免在 onOdometry 回调中做 O(n) erase 阻塞
    if (odom_path_.poses.size() > 10000) {
        const size_t to_trim = odom_path_.poses.size() - 5000;
        odom_path_.poses.erase(odom_path_.poses.begin(), odom_path_.poses.begin() + static_cast<ptrdiff_t>(to_trim));
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][DATA_FLOW] odom_path trimmed to %zu poses", odom_path_.poses.size());
    }
    const int odom_recv  = livo_bridge_.odomCount();
    const int cloud_recv = livo_bridge_.cloudCount();
    const int empty_cloud = livo_bridge_.emptyCloudCount();
    const double last_odom_ts  = livo_bridge_.lastOdomTs();
    const double last_cloud_ts = livo_bridge_.lastCloudTs();
    const int backend_frames = backend_cloud_frames_processed_.load();
    size_t frame_queue_size = 0;
    const int queue_dropped = frame_queue_dropped_.load();
    size_t odom_cache_sz = 0, kfinfo_cache_sz = 0;
    { std::lock_guard<std::mutex> lk(frame_queue_mutex_); frame_queue_size = frame_queue_.size(); }
    { std::lock_guard<std::mutex> lk(odom_cache_mutex_); odom_cache_sz = odom_cache_.size(); }
    { std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_); kfinfo_cache_sz = kfinfo_cache_.size(); }
    const int kf_created  = kf_manager_.keyframeCount();
    const int sm_count   = submap_manager_.submapCount();
    const size_t loop_db = loop_detector_.dbSize();
    const size_t loop_q  = loop_detector_.queueSize();
    const int loop_ok    = loop_detector_.loopDetectedCount();
    const int hba_trig   = hba_optimizer_.triggerCount();
    RCLCPP_DEBUG(get_logger(),
        "[AutoMapSystem][DATA_FLOW] recv: odom=%d cloud=%d kf=%d sm=%d | loop: db=%zu detected=%d",
        odom_recv, cloud_recv, kf_created, sm_count, loop_db, loop_ok);
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PIPELINE] event=data_flow kf=%d sm=%d loop_ok=%d", kf_created, sm_count, loop_ok);

    if (queue_dropped > 0) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][DIAG] frame_queue full, total_dropped=%d (worker slower than recv, consider tuning or higher CPU)", queue_dropped);
    }
    if (cloud_recv == 1 && odom_recv > 1) {
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][DIAG] Only 1 cloud received so far (odom=%d). Likely cause: fast_livo not publishing more /cloud_registered (LIO output empty → '[ LIO ]: No point!!!'). Check fast_livo log and lid_topic/preprocess.",
            odom_recv);
    } else if (cloud_recv == 1 && odom_recv == 1) {
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][DIAG] Only 1 odom and 1 cloud so far. Check bag play rate, lid_topic/imu_topic, and fast_livo '[ LIO ]: No point!!!' or 'get imu at time' logs.");
    } else if (odom_recv >= 5 && cloud_recv > 0 && cloud_recv < odom_recv / 2) {
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][DIAG] backend received cloud=%d vs odom=%d (backend receiving far fewer clouds). Check: 1) [LivoBridge][RECV] delta_recv_ms (large gap=executor not delivering); 2) [fast_livo][PUB] publish() duration (>>10ms=blocking); 3) cloud QoS KeepLast(100) and RELIABLE on both sides.",
            cloud_recv, odom_recv);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 服务处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::handleSaveMap(
    const std::shared_ptr<automap_pro::srv::SaveMap::Request> req,
    std::shared_ptr<automap_pro::srv::SaveMap::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] SaveMap to %s", req->output_dir.c_str());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_start output_dir=%s", req->output_dir.c_str());
    state_ = SystemState::SAVING;
    try {
        saveMapToFiles(req->output_dir);
        res->success     = true;
        res->output_path = req->output_dir;
        res->message     = "Map saved successfully";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_done output_dir=%s success=1", req->output_dir.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][EXCEPTION] SaveMap failed: %s", e.what());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_done output_dir=%s success=0 error=%s", req->output_dir.c_str(), e.what());
        res->success = false;
        res->message = e.what();
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][EXCEPTION] SaveMap failed: unknown exception");
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_done output_dir=%s success=0 error=unknown", req->output_dir.c_str());
        res->success = false;
        res->message = "unknown exception";
    }
    state_ = SystemState::MAPPING;
}

void AutoMapSystem::handleGetStatus(
    const std::shared_ptr<automap_pro::srv::GetStatus::Request>,
    std::shared_ptr<automap_pro::srv::GetStatus::Response> res)
{
    res->state         = stateToString(state_.load());
    res->session_id    = current_session_id_;
    res->keyframe_count = submap_manager_.keyframeCount();
    res->submap_count  = submap_manager_.submapCount();
    res->gps_aligned   = gps_aligned_;
}

void AutoMapSystem::handleTriggerHBA(
    const std::shared_ptr<automap_pro::srv::TriggerHBA::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerHBA::Response> res)
{
    auto all = submap_manager_.getAllSubmaps();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Triggered HBA (wait=%d)", req->wait_for_result);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=trigger_hba_called wait=%d submaps=%zu", req->wait_for_result ? 1 : 0, all.size());
    ensureBackendCompletedAndFlushBeforeHBA();
    hba_optimizer_.triggerAsync(all, req->wait_for_result);
    res->success = true;
    res->message = "HBA triggered";
}

void AutoMapSystem::handleTriggerOptimize(
    const std::shared_ptr<automap_pro::srv::TriggerOptimize::Request>,
    std::shared_ptr<automap_pro::srv::TriggerOptimize::Response> res)
{
    auto t0 = std::chrono::steady_clock::now();
    auto result = isam2_optimizer_.forceUpdate();
    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    res->success      = result.success;
    res->elapsed_seconds = elapsed;
    res->nodes_updated = result.nodes_updated;
}

void AutoMapSystem::handleTriggerGpsAlign(
    const std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_align_triggered force=%d", req->force ? 1 : 0);
    if (req->force) gps_manager_.triggerRealign();
    // 等待对齐完成（最多 5 秒）
    for (int i = 0; i < 50; ++i) {
        if (gps_manager_.isAligned()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const auto& r = gps_manager_.alignResult();
    res->success       = r.success;
    res->alignment_rmse_m = r.rmse_m;
    res->message = r.success ? "GPS aligned" : "GPS alignment failed";
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_align_service_done success=%d rmse_m=%.3f", r.success ? 1 : 0, r.rmse_m);
    for (int i = 0; i < 9; ++i) res->r_gps_lidar[i] = r.R_gps_lidar(i/3, i%3);
    for (int i = 0; i < 3; ++i) res->t_gps_lidar[i] = r.t_gps_lidar[i];
}

void AutoMapSystem::handleLoadSession(
    const std::shared_ptr<automap_pro::srv::LoadSession::Request> req,
    std::shared_ptr<automap_pro::srv::LoadSession::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loading session from %s", req->session_dir.c_str());
    int loaded = 0;
    try {
        if (req->session_dir.empty()) {
            res->success = false;
            res->message = "session_dir is empty";
            res->submaps_loaded = 0;
            res->descriptors_loaded = 0;
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession: session_dir is empty");
            return;
        }
        for (auto& entry : fs::directory_iterator(req->session_dir)) {
            if (!entry.is_directory()) continue;
            std::string name = entry.path().filename().string();
            if (name.find("submap_") != 0) continue;

            int sm_id = 0;
            try {
                sm_id = std::stoi(name.substr(7));
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem] LoadSession: skip invalid dir '%s': %s", name.c_str(), e.what());
                continue;
            }
            SubMap::Ptr sm;
            if (submap_manager_.loadArchivedSubmap(req->session_dir, sm_id, sm)) {
                if (sm) {
                    sm->session_id = req->session_id;
                    loop_detector_.addToDatabase(sm);
                    loaded++;
                }
            }
        }
        res->success          = (loaded > 0);
        res->submaps_loaded   = loaded;
        res->descriptors_loaded = loaded;
        res->message          = std::to_string(loaded) + " submaps loaded";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loaded %d submaps from session %lu",
                    loaded, req->session_id);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=session_loaded dir=%s submaps=%d success=%d", req->session_dir.c_str(), loaded, res->success ? 1 : 0);
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession filesystem error: %s", e.what());
        res->success = false;
        res->submaps_loaded = 0;
        res->descriptors_loaded = 0;
        res->message = std::string("filesystem error: ") + e.what();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession exception: %s", e.what());
        res->success = false;
        res->submaps_loaded = 0;
        res->descriptors_loaded = 0;
        res->message = std::string("exception: ") + e.what();
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession unknown exception");
        res->success = false;
        res->submaps_loaded = 0;
        res->descriptors_loaded = 0;
        res->message = "unknown exception";
    }
}

void AutoMapSystem::handleFinishMapping(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (sensor_idle_finish_triggered_.exchange(true, std::memory_order_acq_rel)) {
        res->success = true;
        res->message = "finish_mapping already triggered";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] finish_mapping: already done, ignoring");
        return;
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_service (final HBA + save + shutdown)");
    RCLCPP_INFO(get_logger(),
        "[CRASH_CONTEXT] step=finish_mapping_enter (崩溃时最后一条 CRASH_CONTEXT 即上一成功步骤)");
    try {
        if (submap_manager_.submapCount() > 0) {
            // 强制冻结当前活跃子图，使最后一子图进入 iSAM2 因子图，保证 submap 数量与图节点一致
            submap_manager_.forceFreezeActiveSubmapForFinish();
            auto all = submap_manager_.getAllSubmaps();
            RCLCPP_INFO(get_logger(),
                "[CRASH_CONTEXT] step=finish_mapping_before_ensureBackend submaps=%zu (若崩溃则发生在 ensureBackend 或 forceUpdate/commitAndUpdate 内)",
                all.size());
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][HBA][TRACE] step=finish_mapping_ensureBackend_enter");
            ensureBackendCompletedAndFlushBeforeHBA();
            RCLCPP_INFO(get_logger(),
                "[CRASH_CONTEXT] step=finish_mapping_after_ensureBackend");
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][HBA][TRACE] step=finish_mapping_ensureBackend_done");
            if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnFinish()) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_final_hba_enter submaps=%zu", all.size());
                RCLCPP_INFO(get_logger(),
                    "[CRASH_CONTEXT] step=finish_mapping_before_hba_trigger submaps=%zu", all.size());
                hba_optimizer_.triggerAsync(all, true);
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_final_hba_done");
                RCLCPP_INFO(get_logger(),
                    "[CRASH_CONTEXT] step=finish_mapping_after_hba_trigger");
            }
            std::string out_dir = getOutputDir();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_save_enter output_dir=%s", out_dir.c_str());
            RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=finish_mapping_before_save output_dir=%s", out_dir.c_str());
            saveMapToFiles(out_dir);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_save_done output_dir=%s", out_dir.c_str());
            RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=finish_mapping_after_save");
        }
        res->success = true;
        res->message = "Map saved, requesting shutdown";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] finish_mapping: requesting context shutdown (end mapping)");
        RCLCPP_INFO(get_logger(), "[CRASH_CONTEXT] step=finish_mapping_success_before_shutdown");
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        RCLCPP_INFO(get_logger(),
            "[CRASH_CONTEXT] step=finish_mapping_caught_exception exception=%s (上一 CRASH_CONTEXT step 即崩溃前最后成功步骤)",
            e.what());
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] finish_mapping failed: %s", e.what());
        res->success = false;
        res->message = std::string("exception: ") + e.what();
    } catch (...) {
        RCLCPP_INFO(get_logger(),
            "[CRASH_CONTEXT] step=finish_mapping_caught_unknown_exception (上一 CRASH_CONTEXT step 即崩溃前最后成功步骤)");
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] finish_mapping failed: unknown exception");
        res->success = false;
        res->message = "unknown exception";
    }
}

// ✅ 修复：动态计算里程计信息矩阵（根据子图质量）
Mat66d AutoMapSystem::computeOdomInfoMatrix(
    const SubMap::Ptr& prev,
    const SubMap::Ptr& curr,
    const Pose3d& rel) const
{
    if (!prev || !curr) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem] computeOdomInfoMatrix: null prev or curr, return default");
        Mat66d def = Mat66d::Identity() * 1e-2;
        return def;
    }
    // 基础置信度（根据子图质量）
    double base_info = 10.0;
    
    // 1. 根据关键帧数量调整（关键帧越多，置信度越高）
    double kf_factor = std::min(2.0, (double)curr->keyframes.size() / 10.0);
    
    // 2. 根据空间距离调整（距离越远，置信度越低）
    double dist = (curr->pose_w_anchor.translation() - prev->pose_w_anchor.translation()).norm();
    double dist_factor = std::max(0.1, std::exp(-dist / 50.0));  // 50米衰减到0.1
    
    // 3. 根据时间间隔调整（时间越长，置信度越低）
    double dt = curr->t_start - prev->t_end;
    double time_factor = std::max(0.1, std::exp(-dt / 60.0));  // 60秒衰减到0.1
    
    // 4. 根据点云质量调整（检查退化标志）
    double quality_factor = 1.0;
    for (const auto& kf : curr->keyframes) {
        if (!kf) continue;
        if (kf->livo_info.is_degenerate) {
            quality_factor *= 0.5;  // 有退化则降低置信度
            break;
        }
    }
    
    // 综合调整因子
    double combined_factor = kf_factor * dist_factor * time_factor * quality_factor;
    
    // 计算平移和旋转信息权重
    double trans_info = base_info * combined_factor * 10.0;   // 平移信息
    double rot_info = base_info * combined_factor * 100.0;    // 旋转信息（通常更准确）
    
    // 构建信息矩阵
    Mat66d info = Mat66d::Zero();
    info.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * trans_info;  // 平移部分
    info.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * rot_info;   // 旋转部分
    
    ALOG_DEBUG(MOD, "OdomInfo: kf={:.1f} dist={:.1f} dt={:.1f} quality={:.1f} -> factor={:.2f}",
               kf_factor, dist_factor, time_factor, quality_factor, combined_factor);
    
    return info;
}

// ─────────────────────────────────────────────────────────────────────────────
// 轨迹对比记录（每帧位姿 + GPS，便于脚本绘图分析建图精度）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::ensureTrajectoryLogDir() {
    if (!trajectory_session_id_.empty()) return;
    try {
        fs::create_directories(trajectory_log_dir_);
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm buf;
#if defined(_WIN32) || defined(_WIN64)
        std::tm* ptm = std::localtime(&t);
#else
        std::tm* ptm = ::localtime_r(&t, &buf);
#endif
        if (ptm) {
            std::ostringstream oss;
            oss << std::put_time(ptm, "%Y%m%d_%H%M%S");
            trajectory_session_id_ = oss.str();
        } else {
            trajectory_session_id_ = std::to_string(current_session_id_);
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] trajectory log dir=%s session_id=%s",
                    trajectory_log_dir_.c_str(), trajectory_session_id_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] create dir failed: %s", e.what());
        trajectory_session_id_ = "default";
    }
}

void AutoMapSystem::writeTrajectoryOdom(double ts, const Pose3d& pose, const Mat66d& cov) {
    std::lock_guard<std::mutex> lk(trajectory_log_mutex_);
    ensureTrajectoryLogDir();
    if (!trajectory_odom_file_.is_open()) {
        std::string path = trajectory_log_dir_ + "/trajectory_odom_" + trajectory_session_id_ + ".csv";
        trajectory_odom_file_.open(path, std::ios::out);
        if (trajectory_odom_file_.is_open()) {
            trajectory_odom_file_ << "timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid,gps_hdop,gps_quality\n";
            trajectory_odom_file_.flush();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] opened %s (with GPS columns). If CSV has no GPS: grep -E 'LivoBridge\\[GPS\\]|GPS_DIAG|TRAJ_LOG no GPS' in logs.",
                path.c_str());
        }
    }
    if (!trajectory_odom_file_.is_open()) return;

    constexpr double kTrajectoryLogGpsMaxDt = 1.0;

    // 诊断：首行轨迹打一次，便于与 GPS 时间范围对比
    static std::atomic<uint32_t> traj_row_count{0};
    uint32_t row = traj_row_count++;
    if (row == 0) {
        size_t gps_sz = gps_manager_.getGpsWindowSize();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] First trajectory row: odom_ts=%.3f (GPS match window=%.1fs; odom ts from LivoBridge /aft_mapped_to_init) gps_window_size=%zu",
            ts, kTrajectoryLogGpsMaxDt, gps_sz);
        if (gps_sz == 0) {
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] GPS window empty at first row. If bag has GPS: grep 'LivoBridge\\[GPS\\]' and 'GPS_DIAG' in logs; after ~45s see '[LivoBridge][GPS_DIAG] Still 0 NavSatFix' if topic not received.");
        }
        if (!gps_manager_.isAligned()) {
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] GPS not aligned! gps_frame will be ENU, not map. Check: good_samples=%d (need=%d) accumulated_dist_m=%.1f (min=%.1f) state=%d",
                gps_manager_.getGoodSampleCount(), gps_manager_.getGoodSamplesNeeded(),
                gps_manager_.getAccumulatedDistM(), gps_manager_.getMinAlignDistM(), static_cast<int>(gps_manager_.state()));
        }
    }
    // 周期性未对齐诊断（每 200 行打一次），便于长时间运行仍可定位
    if (!gps_manager_.isAligned() && row > 0 && (row % 200 == 0)) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] still not aligned at row=%u: good_samples=%d need=%d dist_m=%.1f min_dist=%.1f state=%d",
            row, gps_manager_.getGoodSampleCount(), gps_manager_.getGoodSamplesNeeded(),
            gps_manager_.getAccumulatedDistM(), gps_manager_.getMinAlignDistM(), static_cast<int>(gps_manager_.state()));
    }

    // 查询当前时间戳对应的GPS数据；有匹配即记录（不要求 quality>=MEDIUM），便于分析；gps_valid 表示是否达到 MEDIUM 及以上
    // 使用 1.0s 时间窗：1Hz GPS + 10Hz odom 时，0.5s 窗会导致每 5 行无匹配（间隔内 odom 与前后 GPS 均 >0.5s），1.0s 可覆盖整秒内 odom
    double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
    double gps_hdop = 0.0;
    int gps_quality = 0;  // 0=INVALID 1=LOW 2=MEDIUM 3=HIGH 4=EXCELLENT
    bool gps_valid = false;
    std::string gps_frame_str = "none";

    auto gps_opt = gps_manager_.queryByTimestampForLog(ts, kTrajectoryLogGpsMaxDt);
    if (gps_opt) {
        // 使用 enu_to_map_with_frame 保证位置与坐标系标签一致（未对齐时为 enu，已对齐为 map）
        auto [pos, frame] = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
        gps_x = pos.x();
        gps_y = pos.y();
        gps_z = pos.z();
        gps_frame_str = frame;
        gps_hdop = gps_opt->hdop;
        gps_quality = static_cast<int>(gps_opt->quality);
        gps_valid = gps_opt->is_valid;  // quality >= MEDIUM
        // 首行且未对齐时说明坐标系含义，便于分析轨迹对比
        if (row == 0 && frame == "enu") {
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] CSV: trajectory (x,y,z)=map frame; gps_x/gps_y/gps_z=enu (not aligned). Use gps_frame column; after align both in map.");
        }
    } else {
        // 无时间匹配：打少量诊断日志
        static std::atomic<uint32_t> traj_no_gps_count{0};
        uint32_t no_gps = traj_no_gps_count++;
        size_t gps_window_size = gps_manager_.getGpsWindowSize();
        const char* reason = gps_window_size == 0 ? "no GPS data received (LivoBridge onGPS never called)" : "no match within 1.0s";
        if (no_gps < 5 || (no_gps > 0 && no_gps % 500 == 0)) {
            double gps_min = 0.0, gps_max = 0.0;
            bool has_range = gps_manager_.getGpsWindowTimeRange(&gps_min, &gps_max);
            if (gps_window_size == 0 && no_gps < 5) {
                if (has_range) {
                    RCLCPP_WARN(get_logger(),
                        "[AutoMapSystem][TRAJ_LOG] no GPS for odom ts=%.3f gps_window_size=%zu gps_ts_range=[%.3f, %.3f] (reason: %s).",
                        ts, gps_window_size, gps_min, gps_max, reason);
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[AutoMapSystem][TRAJ_LOG] no GPS for odom ts=%.3f gps_window_size=%zu (reason: %s). Check bag has topic and NavSatFix.",
                        ts, gps_window_size, reason);
                }
            } else {
                RCLCPP_DEBUG(get_logger(),
                    "[AutoMapSystem][TRAJ_LOG] no GPS for odom ts=%.3f gps_window_size=%zu (reason: %s).",
                    ts, gps_window_size, reason);
            }
        }
    }

    Eigen::Quaterniond q(pose.rotation());
    double px = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
    double py = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
    double pz = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
    trajectory_odom_file_ << std::fixed << std::setprecision(6)
        << ts << ","
        << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
        << px << "," << py << "," << pz << ","
        << gps_x << "," << gps_y << "," << gps_z << ","
        << gps_frame_str << "," << (gps_valid ? "1" : "0") << ","
        << gps_hdop << "," << gps_quality << "\n";
    trajectory_odom_file_.flush();
}

void AutoMapSystem::writeTrajectoryOdomAfterMapping(const std::string& output_dir) {
    if (output_dir.empty()) return;
    auto all_sm = submap_manager_.getAllSubmaps();
    std::vector<std::pair<double, Pose3d>> kf_poses;
    std::vector<Mat66d> kf_covs;
    // 与 keyframe_poses.pcd / trajectory_tum.txt 完全一致：仅使用 T_w_b_optimized，不做 fallback，
    // 保证 CSV 中 (x,y,z) 与 PCD 及 RViz 优化轨迹同源，轨迹与 GPS 列才能重合对比
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            const Pose3d& T = kf->T_w_b_optimized;
            kf_poses.emplace_back(kf->timestamp, T);
            kf_covs.push_back(kf->covariance);
        }
    }
    if (kf_poses.empty()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] writeTrajectoryOdomAfterMapping: no keyframes, skip");
        return;
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][TRAJ_LOG] writing trajectory_odom at save (keyframe+GPS, map frame); same frame as keyframe_poses.pcd / gps_positions_map.pcd — use this file in save dir for trajectory-GPS comparison.");
    if (!gps_manager_.isAligned()) {
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] GPS not aligned: gps_x/gps_y/gps_z will be in ENU frame; trajectory vs GPS may not coincide in plot. Consider triggering GPS align before save if needed.");
    }
    const std::string filename = "trajectory_odom_" + trajectory_session_id_ + ".csv";
    const std::string path_primary = output_dir + "/" + filename;
    const bool also_to_log_dir = !trajectory_log_dir_.empty() && trajectory_log_dir_ != output_dir;

    std::ofstream out(path_primary);
    if (!out.is_open()) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] failed to open %s", path_primary.c_str());
        return;
    }
    std::ofstream out_log;
    if (also_to_log_dir) {
        fs::create_directories(trajectory_log_dir_);
        const std::string path_log = trajectory_log_dir_ + "/" + filename;
        out_log.open(path_log);
        if (!out_log.is_open()) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] failed to open log dir copy %s (will only write to output_dir)", path_log.c_str());
        }
    }

    const std::string header = "timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid,gps_hdop,gps_quality\n";
    out << header;
    if (out_log.is_open()) out_log << header;

    constexpr double kGpsMaxDt = 1.0;
    for (size_t i = 0; i < kf_poses.size(); ++i) {
        const double ts = kf_poses[i].first;
        const Pose3d& pose = kf_poses[i].second;
        const Mat66d& cov = kf_covs[i];
        double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0, gps_hdop = 0.0;
        int gps_quality = 0;
        bool gps_valid = false;
        std::string gps_frame_str = "none";
        auto gps_opt = gps_manager_.queryByTimestampForLog(ts, kGpsMaxDt);
        if (gps_opt) {
            auto [pos, frame] = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
            gps_x = pos.x(); gps_y = pos.y(); gps_z = pos.z();
            gps_frame_str = frame;
            gps_hdop = gps_opt->hdop;
            gps_quality = static_cast<int>(gps_opt->quality);
            gps_valid = gps_opt->is_valid;
        }
        Eigen::Quaterniond q(pose.rotation());
        double px = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
        double py = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
        double pz = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
        std::ostringstream line;
        line << std::fixed << std::setprecision(6)
            << ts << ","
            << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
            << px << "," << py << "," << pz << ","
            << gps_x << "," << gps_y << "," << gps_z << ","
            << gps_frame_str << "," << (gps_valid ? "1" : "0") << ","
            << gps_hdop << "," << gps_quality << "\n";
        const std::string line_str = line.str();
        out << line_str;
        if (out_log.is_open()) out_log << line_str;
    }
    out.close();
    if (out_log.is_open()) {
        out_log.close();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] wrote trajectory_odom (keyframe+GPS, map frame) to %s and %s (%zu rows). For trajectory-GPS comparison use the file in save dir (same as keyframe_poses.pcd).",
            path_primary.c_str(), (trajectory_log_dir_ + "/" + filename).c_str(), kf_poses.size());
    } else {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] wrote trajectory_odom (keyframe+GPS, map frame) to %s (%zu rows). Use this file for trajectory-GPS comparison (same frame as keyframe_poses.pcd).",
            path_primary.c_str(), kf_poses.size());
    }
}

void AutoMapSystem::onGPSMeasurementForLog(double ts, const Eigen::Vector3d& pos_enu) {
    if (!trajectory_log_enabled_) return;
    std::lock_guard<std::mutex> lk(trajectory_log_mutex_);
    ensureTrajectoryLogDir();
    if (!trajectory_gps_file_.is_open()) {
        std::string path = trajectory_log_dir_ + "/trajectory_gps_" + trajectory_session_id_ + ".csv";
        trajectory_gps_file_.open(path, std::ios::out);
        if (trajectory_gps_file_.is_open()) {
            trajectory_gps_file_ << "timestamp,x,y,z,frame,pitch,roll,yaw,attitude_source,velocity,attitude_valid\n";
            trajectory_gps_file_.flush();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] opened %s (with attitude columns)", path.c_str());
        }
    }
    if (!trajectory_gps_file_.is_open()) return;
    auto m_opt = gps_manager_.queryByTimestampForLog(ts, 0.1);
    if (!m_opt) return;
    const GPSMeasurement& m = *m_opt;
    Eigen::Vector3d pos = m.position_enu;
    const char* frame = gps_manager_.isAligned() ? "map" : "enu";
    const AttitudeEstimate& att = m.attitude;
    trajectory_gps_file_ << std::fixed << std::setprecision(6)
        << m.timestamp << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << frame << ","
        << att.pitch << "," << att.roll << "," << att.yaw << ","
        << static_cast<int>(att.source) << "," << att.velocity_horizontal << ","
        << (att.is_valid ? "1" : "0") << "\n";
    trajectory_gps_file_.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// 地图保存
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoMapSystem::getOutputDir() const {
    if (!output_dir_override_.empty()) return output_dir_override_;
    return ConfigManager::instance().outputDir();
}

void AutoMapSystem::saveMapToFiles(const std::string& output_dir) {
    try {
        if (output_dir.empty()) {
            throw std::invalid_argument("saveMapToFiles: output_dir is empty");
        }
        fs::create_directories(output_dir);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_writing output_dir=%s", output_dir.c_str());

        const float voxel_size = map_voxel_size_;
        CloudXYZIPtr global = ConfigManager::instance().asyncGlobalMapBuild()
            ? submap_manager_.buildGlobalMapAsync(voxel_size).get()
            : submap_manager_.buildGlobalMap(voxel_size);
        size_t pcd_points = 0;
        if (global && !global->empty()) {
            std::string pcd_path = output_dir + "/global_map.pcd";
            pcl::io::savePCDFileBinary(pcd_path, *global);
            pcd_points = global->size();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Saved %s (%zu points)",
                        pcd_path.c_str(), global->size());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_pcd path=%s points=%zu", pcd_path.c_str(), pcd_points);
        }

        // 保存 TUM 轨迹（优化后）与关键帧位置 PCD
        std::string tum_path = output_dir + "/trajectory_tum.txt";
        std::ofstream tum_file(tum_path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr kf_poses_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        kf_poses_cloud->reserve(4096);
        auto all_sm = submap_manager_.getAllSubmaps();
        size_t trajectory_poses = 0;
        for (const auto& sm : all_sm) {
            if (!sm) continue;
            for (const auto& kf : sm->keyframes) {
                if (!kf) continue;
                const auto& T = kf->T_w_b_optimized;
                Eigen::Quaterniond q(T.rotation());
                tum_file << std::fixed << std::setprecision(6)
                         << kf->timestamp << " "
                         << T.translation().x() << " "
                         << T.translation().y() << " "
                         << T.translation().z() << " "
                         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
                trajectory_poses++;
                // 关键帧位置写入 PCD：xyz=位姿平移，intensity=关键帧 id（便于区分）
                pcl::PointXYZI pt;
                pt.x = static_cast<float>(T.translation().x());
                pt.y = static_cast<float>(T.translation().y());
                pt.z = static_cast<float>(T.translation().z());
                pt.intensity = static_cast<float>(kf->id);
                kf_poses_cloud->push_back(pt);
            }
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_trajectory path=%s poses=%zu", tum_path.c_str(), trajectory_poses);

        // 建图完成后写 trajectory_odom CSV：关键帧位姿 + 最终地图系下 GPS，保证与 plot_trajectory_compare 对比时轨迹与 GPS 重合
        if (trajectory_log_enabled_) {
            writeTrajectoryOdomAfterMapping(output_dir);
        }

        if (!kf_poses_cloud->empty()) {
            std::string kf_pcd_path = output_dir + "/keyframe_poses.pcd";
            if (pcl::io::savePCDFileBinary(kf_pcd_path, *kf_poses_cloud) == 0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem] Saved keyframe poses PCD: %s (%zu keyframes)",
                            kf_pcd_path.c_str(), kf_poses_cloud->size());
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_keyframe_pcd path=%s keyframes=%zu",
                            kf_pcd_path.c_str(), kf_poses_cloud->size());
            } else {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem] Failed to save keyframe poses PCD: %s", kf_pcd_path.c_str());
            }
        }

        // 建图结束时保存地图坐标系下的 GPS 位置（PCD 格式，与 keyframe_poses.pcd 同目录）
        auto gps_map_positions = gps_manager_.getGpsPositionsInMapFrame();
        if (!gps_map_positions.empty()) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr gps_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            gps_cloud->reserve(gps_map_positions.size());
            for (size_t i = 0; i < gps_map_positions.size(); ++i) {
                pcl::PointXYZI pt;
                pt.x = static_cast<float>(gps_map_positions[i].second.x());
                pt.y = static_cast<float>(gps_map_positions[i].second.y());
                pt.z = static_cast<float>(gps_map_positions[i].second.z());
                pt.intensity = static_cast<float>(i);
                gps_cloud->push_back(pt);
            }
            std::string gps_pcd_path = output_dir + "/gps_positions_map.pcd";
            if (pcl::io::savePCDFileBinary(gps_pcd_path, *gps_cloud) == 0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem] Saved GPS positions (map frame) PCD: %s (%zu points)",
                            gps_pcd_path.c_str(), gps_cloud->size());
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_gps_positions_pcd path=%s points=%zu",
                            gps_pcd_path.c_str(), gps_cloud->size());
            } else {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem] Failed to save GPS positions PCD: %s", gps_pcd_path.c_str());
            }
        } else {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=skip_gps_positions_pcd reason=no_aligned_gps");
        }

        // 归档子图（用于下次增量建图加载）
        std::string session_dir = output_dir + "/session_" + std::to_string(current_session_id_);
        fs::create_directories(session_dir);
        for (auto& sm : all_sm) {
            submap_manager_.archiveSubmap(sm, session_dir);
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Session archived to %s", session_dir.c_str());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_session dir=%s submaps=%zu", session_dir.c_str(), all_sm.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] saveMapToFiles failed: %s", e.what());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_failed reason=exception");
        throw;
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] saveMapToFiles failed: unknown exception");
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_failed reason=unknown");
        throw;
    }
}

std::string AutoMapSystem::stateToString(SystemState s) const {
    switch (s) {
        case SystemState::IDLE:         return "IDLE";
        case SystemState::INITIALIZING: return "INITIALIZING";
        case SystemState::MAPPING:      return "MAPPING";
        case SystemState::LOOP_CLOSING: return "LOOP_CLOSING";
        case SystemState::OPTIMIZING:   return "OPTIMIZING";
        case SystemState::SAVING:       return "SAVING";
        case SystemState::ERROR:        return "ERROR";
        default:                        return "UNKNOWN";
    }
}

// ============================================================================
// V2: 线程心跳监控
// ============================================================================

void AutoMapSystem::checkThreadHeartbeats() {
    const auto now = std::chrono::steady_clock::now();
    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    struct ThreadHealth {
        std::string name;
        int64_t last_heartbeat_ms;
        std::string status;  // "OK", "WARN", "ERROR"
    };
    
    std::vector<ThreadHealth> healths = {
        {"feeder",       feeder_heartbeat_ts_ms_.load(),       "OK"},
        {"backend",      backend_heartbeat_ts_ms_.load(),      "OK"},
        {"map_pub",      map_pub_heartbeat_ts_ms_.load(),      "OK"},
        {"loop_opt",     loop_opt_heartbeat_ts_ms_.load(),     "OK"},
        {"viz",          viz_heartbeat_ts_ms_.load(),          "OK"},
        {"status_pub",   status_pub_heartbeat_ts_ms_.load(),   "OK"},
    };
    
    bool has_warn = false;
    bool has_error = false;
    std::stringstream warn_ss, error_ss;
    
    for (auto& h : healths) {
        if (h.last_heartbeat_ms == 0) {
            // 尚未开始心跳，可能是系统刚启动
            h.status = "INIT";
            continue;
        }
        int64_t elapsed_ms = now_ms - h.last_heartbeat_ms;
        if (elapsed_ms > kHeartbeatErrorThresholdMs) {
            h.status = "ERROR";
            has_error = true;
            error_ss << h.name << "(" << (elapsed_ms/1000) << "s) ";
        } else if (elapsed_ms > kHeartbeatWarnThresholdMs) {
            h.status = "WARN";
            has_warn = true;
            warn_ss << h.name << "(" << (elapsed_ms/1000) << "s) ";
        }
    }
    
    // 输出心跳状态汇总
    if (has_error) {
        RCLCPP_ERROR(get_logger(), 
            "[AutoMapSystem][HEARTBEAT] CRITICAL: threads stuck: %s", 
            error_ss.str().c_str());
    } else if (has_warn) {
        RCLCPP_WARN(get_logger(), 
            "[AutoMapSystem][HEARTBEAT] WARNING: slow threads: %s", 
            warn_ss.str().c_str());
    }
    
    // 周期性输出健康状态（每30秒一次，即使正常）
    static auto last_health_log = now;
    double sec_since_last = std::chrono::duration<double>(now - last_health_log).count();
    if (sec_since_last >= 30.0) {
        std::stringstream ss;
        ss << "[AutoMapSystem][HEARTBEAT] thread health: ";
        for (const auto& h : healths) {
            ss << h.name << "=" << h.status << " ";
        }
        ss << "| ingress_q=" << [this]() { 
            std::lock_guard<std::mutex> lk(ingress_mutex_); 
            return ingress_queue_.size(); 
        }();
        ss << " frame_q=" << [this]() { 
            std::lock_guard<std::mutex> lk(frame_queue_mutex_); 
            return frame_queue_.size(); 
        }();
        ss << " backend_frames=" << backend_cloud_frames_processed_.load();
        ss << " kf_count=" << kf_manager_.keyframeCount();
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
        last_health_log = now;
    }
}

} // namespace automap_pro
