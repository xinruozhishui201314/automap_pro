// 模块1: 构造与初始化
// 包含: AutoMapSystem 构造函数、析构函数、loadConfigAndInit、setupModules、deferredSetupModules、setupPublishers、setupServices、setupTimers
// 注意: nowMs() 辅助函数定义在 tasks_and_utils.cpp 中 (AutoMapSystem 成员函数)

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/config/thread_config.h"
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

constexpr int MAX_KF_PER_SUBMAP = 100000;

// ─────────────────────────────────────────────────────────────────────────────
// 构造与初始化
// ─────────────────────────────────────────────────────────────────────────────
AutoMapSystem::AutoMapSystem(const rclcpp::NodeOptions& options)
    : rclcpp::Node("automap_system", options)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 0: Constructor entered");

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

    deferred_init_timer_ = create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() { this->deferredSetupModules(); });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 6: deferred setupModules scheduled (will run after first spin)");

    feeder_thread_ = std::thread(&AutoMapSystem::feederLoop, this);
    backend_worker_ = std::thread(&AutoMapSystem::backendWorkerLoop, this);
    map_publish_thread_ = std::thread(&AutoMapSystem::mapPublishLoop, this);
    // 注意：loopOptThreadLoop 已删除，回环约束直接在 onLoopDetected 中处理
    gps_worker_thread_ = std::thread(&AutoMapSystem::gpsWorkerLoop, this);
    opt_worker_thread_ = std::thread(&AutoMapSystem::optWorkerLoop, this);
    loop_trigger_thread_ = std::thread(&AutoMapSystem::loopTriggerThreadLoop, this);
    intra_loop_worker_thread_ = std::thread(&AutoMapSystem::intraLoopWorkerLoop, this);
    gps_align_thread_ = std::thread(&AutoMapSystem::gpsAlignWorkerLoop, this);
    // 注意：viz_thread_ 已删除
    status_publisher_thread_ = std::thread(&AutoMapSystem::statusPublisherLoop, this);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 7: all worker threads started");
}

AutoMapSystem::~AutoMapSystem() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=1] destructor entered, requesting backend exit");
    shutdown_requested_.store(true, std::memory_order_release);

    // 唤醒所有等待中的条件变量
    frame_processor_.stop();
    map_publish_cv_.notify_all();
    gps_queue_cv_.notify_all();
    loop_trigger_cv_.notify_all();
    intra_loop_task_cv_.notify_all();
    gps_align_cv_.notify_all();
    opt_task_cv_.notify_all();
    status_pub_cv_.notify_all();

    // 逐个 join 线程
    if (feeder_thread_.joinable()) {
        feeder_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2a] feeder thread joined");
    }
    if (backend_worker_.joinable()) {
        backend_worker_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2b] backend worker joined");
    }
    if (map_publish_thread_.joinable()) {
        map_publish_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2c] map_publish thread joined");
    }
    if (gps_worker_thread_.joinable()) {
        gps_worker_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2d] gps_worker thread joined");
    }
    if (loop_trigger_thread_.joinable()) {
        loop_trigger_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2e] loop_trigger thread joined");
    }
    if (intra_loop_worker_thread_.joinable()) {
        intra_loop_worker_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2f] intra_loop_worker thread joined");
    }
    if (gps_align_thread_.joinable()) {
        gps_align_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2g] gps_align thread joined");
    }
    if (opt_worker_thread_.joinable()) {
        opt_worker_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2h] opt_worker thread joined");
    }
    if (status_publisher_thread_.joinable()) {
        status_publisher_thread_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=2i] status_publisher thread joined");
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=3] calling loop_detector_.stop()");
    loop_detector_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=4] loop_detector_.stop() done");

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=4a] calling submap_manager_.stop()");
    submap_manager_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=4b] submap_manager_.stop() done");

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=4c] calling HealthMonitor::instance().stop()");
    HealthMonitor::instance().stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=4d] HealthMonitor stop() done");

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

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=6] hba_optimizer_.stop() enter");
    if (ConfigManager::instance().hbaOnFinish()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_skip_final_hba (map already saved, exit quickly)");
    }
    hba_optimizer_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=7] hba_optimizer_.stop() done");

    isam2_optimizer_.clearForShutdown();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=8] isam2_optimizer_.clearForShutdown() done (prior_noise_ already released)");

    // 关闭轨迹日志文件（确保数据写入磁盘）
    if (trajectory_odom_file_.is_open()) {
        trajectory_odom_file_.flush();
        trajectory_odom_file_.close();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=8a] trajectory_odom_file closed");
    }
    if (trajectory_gps_file_.is_open()) {
        trajectory_gps_file_.flush();
        trajectory_gps_file_.close();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=8b] trajectory_gps_file closed");
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SHUTDOWN][step=9] Destructor body done. Member destructors next: RvizPublisher -> HBAOptimizer -> IncrementalOptimizer -> LoopDetector -> SubMapManager -> ...");
}

void AutoMapSystem::loadConfigAndInit() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][CONFIG] Declaring parameter config_file");
    declare_parameter("config_file", "");
    declare_parameter("output_dir", std::string(""));
    declare_parameter("trajectory_log_enable", true);
    declare_parameter("trajectory_log_after_mapping_only", true);
    declare_parameter("trajectory_log_dir", std::string(""));
    declare_parameter("vtk_viewer_after_hba", true);
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
        const bool path_looks_m2dgr = (config_path.find("M2DGR") != std::string::npos);
        if (path_looks_m2dgr && gps_topic_from_config_ == "/gps/fix") {
            gps_topic_from_config_ = "/ublox/fix";
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem][CONFIG][GPS_DIAG] M2DGR config detected but sensor.gps.topic was default; using /ublox/fix (match bag topic). Check YAML sensor.gps.topic if this is unexpected.");
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] sensor.gps.enabled=%s sensor.gps.topic=%s (if trajectory CSV has no GPS, ensure enabled=true and bag publishes this topic)",
                    gps_enabled_from_config_ ? "true" : "false", gps_topic_from_config_.c_str());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_DIAG] config_path=%s gps_topic=%s (M2DGR bag: /ublox/fix; grep LivoBridge[GPS] for first message)",
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
    vtk_viewer_after_hba_ = get_parameter("vtk_viewer_after_hba").as_bool();
    if (trajectory_log_dir_.empty()) {
        const char* env_dir = std::getenv("AUTOMAP_LOG_DIR");
        trajectory_log_dir_ = env_dir ? env_dir : "logs";
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] trajectory_log_enable=%d trajectory_log_after_mapping_only=%d trajectory_log_dir=%s vtk_viewer_after_hba=%d",
                trajectory_log_enabled_ ? 1 : 0, trajectory_log_after_mapping_only_ ? 1 : 0, trajectory_log_dir_.c_str(), vtk_viewer_after_hba_ ? 1 : 0);
    if (trajectory_log_enabled_ && trajectory_log_after_mapping_only_) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][CONFIG] trajectory_odom will be written only at save (keyframe+GPS, map frame); use the CSV in save output_dir for trajectory-GPS comparison.");
    }

    const size_t max_frame_queue_size = ConfigManager::instance().frameQueueMaxSize();
    const size_t max_ingress_queue_size = ConfigManager::instance().ingressQueueMaxSize();

    // 初始化 ThreadConfig 单例
    ThreadConfig::mutableInstance().max_frame_queue_size = max_frame_queue_size;
    ThreadConfig::mutableInstance().max_ingress_queue_size = max_ingress_queue_size;
    ThreadConfig::mutableInstance().max_opt_task_queue_size = kMaxOptTaskQueueSize;
    ThreadConfig::mutableInstance().max_loop_trigger_queue_size = kMaxLoopTriggerQueueSize;
    ThreadConfig::mutableInstance().max_gps_queue_size = kMaxGPSQueueSize;

    // 初始化 FrameProcessor
    frame_processor_.init(max_ingress_queue_size, max_frame_queue_size);
    frame_processor_.setShutdownFlag(&shutdown_requested_);

    // 初始化 TaskDispatcher（使用外部队列）
    task_dispatcher_ = std::make_unique<TaskDispatcher>(&opt_task_queue_, &opt_task_mutex_, &opt_task_cv_, kMaxOptTaskQueueSize);

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] frame_queue_max_size=%zu ingress_queue_max_size=%zu sensor_idle_timeout_sec=%.1f (back-pressure in feeder thread, no drop)",
                max_frame_queue_size, max_ingress_queue_size, ConfigManager::instance().sensorIdleTimeoutSec());
}

void AutoMapSystem::setupModules() {
}

void AutoMapSystem::deferredSetupModules() {
    if (deferred_init_timer_) {
        deferred_init_timer_->cancel();
        deferred_init_timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 1: deferredSetupModules entered (shared_from_this now valid)");

    if (!ErrorMonitor::instance().isRunning()) {
        ErrorMonitor::instance().start();
    }

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

    current_session_id_ = static_cast<uint64_t>(
        std::chrono::system_clock::now().time_since_epoch().count());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 2: session_id=%lu", current_session_id_);

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 3a: init SubMapManager");
    submap_manager_.init(shared_from_this());
    submap_manager_.startNewSession(current_session_id_);
    submap_manager_.registerSubmapFrozenCallback(
        [this](const SubMap::Ptr& sm) { onSubmapFrozen(sm); });

    rviz_publisher_.init(shared_from_this());
    rviz_publisher_.setFrameId("map");
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 3b: SubMapManager inited and session started");

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 4a: init LoopDetector");
    loop_detector_.init(shared_from_this());
    loop_detector_.registerLoopCallback(
        [this](const LoopConstraint::Ptr& lc) { onLoopDetected(lc); });
    loop_detector_.start();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 4b: LoopDetector inited and started");

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 5: register iSAM2 pose callback");
    isam2_optimizer_.registerPoseUpdateCallback(
        [this](const std::unordered_map<int, Pose3d>& poses) {
            onPoseUpdated(poses);
        });

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

    gps_manager_.applyConfig();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7: register GPSManager callbacks");
    gps_manager_.registerAlignCallback(
        [this](const GPSAlignResult& r) {
            onGPSAligned(r);
        });
    gps_manager_.registerMeasurementLogCallback(
        [this](double ts, const Eigen::Vector3d& pos_enu) { onGPSMeasurementForLog(ts, pos_enu); });
    gps_manager_.setAlignScheduler([]() { /* 仅标记 pending */ });

    gps_manager_.registerGpsFactorCallback(
        [this](double ts, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov) {
            if (!gps_aligned_.load()) return;
            auto submaps = submap_manager_.getFrozenSubmaps();
            if (submaps.empty()) return;

            int best_id = -1;
            double best_dt = 1e9;
            double best_dist = 1e9;
            const double max_bind_dt = 30.0;
            const double max_bind_dist = 100.0;

            for (const auto& sm : submaps) {
                double dt_end = std::abs(sm->t_end - ts);
                double dt_start = std::abs(sm->t_start - ts);
                double dt = std::min(dt_end, dt_start);
                Eigen::Vector3d sm_center = sm->pose_w_anchor.translation();
                double spatial_dist = (pos - sm_center).norm();

                if (dt < best_dt && spatial_dist < max_bind_dist) {
                    best_dt = dt;
                    best_dist = spatial_dist;
                    best_id = sm->id;
                } else if (dt < best_dt * 0.5 && spatial_dist < best_dist) {
                    best_dt = dt;
                    best_dist = spatial_dist;
                    best_id = sm->id;
                }
            }

            if (best_id >= 0 && best_dt <= max_bind_dt) {
                RCLCPP_DEBUG(get_logger(),
                    "[AutoMapSystem][GPS_BIND] sm_id=%d dt=%.2fs dist=%.2fm (thresh: dt=%.0fs dist=%.0fm)",
                    best_id, best_dt, best_dist, max_bind_dt, max_bind_dist);
                
                // 由于系统已全球化，需要将 GPSManager 传出的 local pos 转换到全局 ENU 坐标系
                // 如果未对齐，R/t 为 Identity，转换后仍为 pos_enu
                Eigen::Vector3d pos_global;
                {
                    std::lock_guard<std::mutex> lk(gps_transform_mutex_);
                    pos_global = gps_transform_R_ * pos + gps_transform_t_;
                }

                // 始终通过 TaskDispatcher 投递任务
                if (task_dispatcher_) {
                    task_dispatcher_->submitGPSFactor(best_id, pos_global, cov);
                }
            } else {
                RCLCPP_DEBUG(get_logger(),
                    "[AutoMapSystem][GPS_BIND] No suitable submap: best_dt=%.2fs best_dist=%.2fm (thresh: dt=%.0fs dist=%.0fm)",
                    best_dt, best_dist, max_bind_dt, max_bind_dist);
            }
        });

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
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7b: AttitudeEstimator created, IMU topic=%s, gps.has_attitude=%s",
                   imu_topic.c_str(), cfg.gpsHasAttitude() ? "true" : "false");
    }

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
    RCLCPP_INFO(get_logger(), "=== AutoMapSystem READY (session_id=%lu) ===", current_session_id_);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] state=READY session_id=%lu (grep PIPELINE 可实时查看建图各环节)", current_session_id_);
}

void AutoMapSystem::setupPublishers() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PUB] Creating publishers");
    // 🔧 V2 修复：删除冗余发布者，统一使用 rviz_publisher_ 管理可视化 topic
    // odom_path_pub_, opt_path_pub_, global_map_pub_ 已移至 rviz_publisher_
    status_pub_ = create_publisher<automap_pro::msg::MappingStatusMsg>("/automap/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][TOPIC] publish: /automap/status (others via rviz_publisher_)");
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
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][TIMER] No wall timers for status/map/data_flow (data-triggered in backend worker)");

    heartbeat_monitor_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        [this]() { this->checkThreadHeartbeats(); });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][TIMER] Heartbeat monitor timer started (5s interval)");
}

}  // namespace automap_pro
