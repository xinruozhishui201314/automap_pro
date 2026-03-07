#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/backend/pose_graph.h"
#define MOD "AutoMapSystem"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;
namespace automap_pro {

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

    // 提前启动 backend worker，避免 0ms 定时器因 executor 繁忙未及时触发导致队列只增不消
    backend_worker_ = std::thread(&AutoMapSystem::backendWorkerLoop, this);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INIT] Step 7: backend worker thread started in ctor (waiting on empty queue until LivoBridge feeds)");
}

AutoMapSystem::~AutoMapSystem() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Shutting down...");

    shutdown_requested_.store(true, std::memory_order_release);
    frame_queue_cv_.notify_all();
    if (backend_worker_.joinable()) {
        backend_worker_.join();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Backend worker joined");
    }
    loop_detector_.stop();

    // 优先保存点云地图，确保 Ctrl+C 后一定能输出（此前先 stop HBA 再 triggerAsync(wait=true) 导致
    // 工作线程已退出、队列无人消费而永久阻塞，进程被 SIGKILL，地图从未保存）
    if (submap_manager_.submapCount() > 0) {
        try {
            std::string out_dir = getOutputDir();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save output_dir=%s submaps=%d", out_dir.c_str(), submap_manager_.submapCount());
            saveMapToFiles(out_dir);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Auto-saved backend map to %s on shutdown", out_dir.c_str());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save_done success=1");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Auto-save map on shutdown failed: %s", e.what());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save_done success=0 error=%s", e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Auto-save map on shutdown failed: unknown exception");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_auto_save_done success=0 error=unknown");
        }
    }

    // 关闭时不再触发最终 HBA，避免 stop() 等待工作线程跑完长时间优化导致进程被 SIGKILL、地图已在上方保存
    if (ConfigManager::instance().hbaOnFinish()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=shutdown_skip_final_hba (map already saved, exit quickly)");
    }

    hba_optimizer_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Shutdown complete.");
}

void AutoMapSystem::loadConfigAndInit() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][CONFIG] Declaring parameter config_file");
    declare_parameter("config_file", "");
    declare_parameter("output_dir", std::string(""));
    declare_parameter("trajectory_log_enable", true);
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
    } else {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][CONFIG] No config file specified, using defaults");
    }
    trajectory_log_enabled_ = get_parameter("trajectory_log_enable").as_bool();
    trajectory_log_dir_    = get_parameter("trajectory_log_dir").as_string();
    if (trajectory_log_dir_.empty()) {
        const char* env_dir = std::getenv("AUTOMAP_LOG_DIR");
        trajectory_log_dir_ = env_dir ? env_dir : "logs";
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] trajectory_log_enable=%d trajectory_log_dir=%s",
                trajectory_log_enabled_ ? 1 : 0, trajectory_log_dir_.c_str());
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

    // iSAM2 优化器
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 5: register iSAM2 pose callback");
    isam2_optimizer_.registerPoseUpdateCallback(
        [this](const std::unordered_map<int, Pose3d>& poses) { onPoseUpdated(poses); });

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

    // GPS 管理器
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7: register GPSManager callbacks");
    gps_manager_.registerAlignCallback(
        [this](const GPSAlignResult& r) { onGPSAligned(r); });
    gps_manager_.registerMeasurementLogCallback(
        [this](double ts, const Eigen::Vector3d& pos_enu) { onGPSMeasurementForLog(ts, pos_enu); });
    gps_manager_.registerGpsFactorCallback(
        [this](double ts, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov) {
            if (!gps_aligned_) return;
            auto submaps = submap_manager_.getFrozenSubmaps();
            if (submaps.empty()) return;
            int best_id = -1;
            double best_dt = 5.0;
            for (const auto& sm : submaps) {
                double dt = std::abs(sm->t_end - ts);
                if (dt < best_dt) { best_dt = dt; best_id = sm->id; }
            }
            if (best_id >= 0) {
                isam2_optimizer_.addGPSFactor(best_id, pos, cov);
            }
        });

    // LivoBridge（最后初始化，开始接收数据）
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 8a: init LivoBridge");
    livo_bridge_.init(shared_from_this());
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
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 10: backend worker thread started here (queue_max=%zu)", kMaxFrameQueueSize);
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
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][SRV] All 7 services created (save_map, get_status, trigger_hba, trigger_optimize, trigger_gps_align, load_session)");
}

void AutoMapSystem::setupTimers() {
    // 不再使用周期定时器，避免阻塞 executor 线程。status/map/data_flow 由 backendWorkerLoop 数据触发：
    // 每处理一帧后按帧数触发 publishStatus(每约10帧)、publishDataFlowSummary(每约50帧)、publishGlobalMap(每约100帧)
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][TIMER] No wall timers for status/map/data_flow (data-triggered in backend worker)");
}

// ─────────────────────────────────────────────────────────────────────────────
// 数据流：里程计 → 关键帧 → 子图（后端接收入口统一打 [BACKEND][RECV] 便于 grep 追踪）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    static std::atomic<int> backend_odom_recv_count{0};
    const int seq = backend_odom_recv_count.fetch_add(1) + 1;
    if (seq <= 5 || seq % 500 == 0) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][BACKEND][RECV] odom #%d ts=%.3f pos=[%.2f,%.2f,%.2f] (backend entry)",
            seq, ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
        double pos_std_x = std::sqrt(std::max(0.0, cov(3, 3)));
        double pos_std_y = std::sqrt(std::max(0.0, cov(4, 4)));
        double pos_std_z = std::sqrt(std::max(0.0, cov(5, 5)));
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

    // 轨迹对比记录：每帧位姿写入 CSV，便于与 GPS 曲线对比分析建图精度
    if (trajectory_log_enabled_) writeTrajectoryOdom(ts, pose, cov);

    // 关键帧由 onCloud 触发，保证 pose 与 cloud 同帧（fast_livo 先发 odom 再发 cloud，收到 cloud 时 last_odom_pose_ 已是本帧）
}

void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud) {
    const size_t pts = cloud ? cloud->size() : 0u;
    (void)pts;  // for optional RCLCPP_INFO below
    // RCLCPP_INFO(get_logger(),
    //     "[AutoMapSystem][BACKEND][RECV] cloud entry ts=%.3f pts=%zu → queue (callback returns immediately)",
    //     ts, pts);
    if (!first_cloud_logged_.exchange(true)) {
        // RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA] First cloud received ts=%.3f pts=%zu", ts, pts);
        // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=first_cloud ts=%.3f pts=%zu", ts, pts);
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
    {
        std::lock_guard<std::mutex> lk(frame_queue_mutex_);
        if (frame_queue_.size() >= kMaxFrameQueueSize) {
            frame_queue_.pop();
            frame_queue_dropped_++;
            // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            //     "[AutoMapSystem][BACKEND] frame queue full (%zu), drop oldest (total_dropped=%d)", kMaxFrameQueueSize, frame_queue_dropped_.load());
        }
        frame_queue_.push(std::move(f));
    }
    size_t qs = 0;
    { std::lock_guard<std::mutex> lk(frame_queue_mutex_); qs = frame_queue_.size(); }
    frame_queue_cv_.notify_one();
    {
        static std::atomic<bool> first_frame_queued_logged{false};
        if (!first_frame_queued_logged.exchange(true)) {
            // RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] first frame in queue ts=%.3f pts=%zu, notify_one (worker should wake)", ts, pts);
        }
    }
    if (qs <= 3 || qs % 50 == 0) {
        // RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][RECV] cloud ts=%.3f pts=%zu queued, queue_size=%zu", ts, pts, qs);
    }
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
    // 找 ts <= cloud_ts 的最近一条（从后往前）
    const OdomCacheEntry* best = nullptr;
    for (auto it = odom_cache_.rbegin(); it != odom_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) best = &odom_cache_.front();
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
    const KFinfoCacheEntry* best = nullptr;
    for (auto it = kfinfo_cache_.rbegin(); it != kfinfo_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) best = &kfinfo_cache_.front();
    out_info = best->info;
    return true;
}

void AutoMapSystem::backendWorkerLoop() {
    // RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker thread running, entering wait for first frame (queue empty until LivoBridge feeds)");
    const std::string cloud_frame = ConfigManager::instance().frontendCloudFrame();
    // RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][CONFIG] frontend_cloud_frame=%s (fast_livo /cloud_registered is world; use world to avoid global map double-transform)",
    //             cloud_frame.c_str());
    static thread_local int no_odom_skip_count = 0;
    const auto wait_chunk = std::chrono::milliseconds(2000);
    const double idle_timeout_sec = ConfigManager::instance().sensorIdleTimeoutSec();
    const bool auto_finish = ConfigManager::instance().autoFinishOnSensorIdle();

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        FrameToProcess f;
        size_t qs_after_wait = 0;
        bool woke_by_data = false;
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            woke_by_data = frame_queue_cv_.wait_for(lock, wait_chunk, [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !frame_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;

            if (!woke_by_data && frame_queue_.empty()) {
                // 超时且队列为空：检查是否传感器空闲超时，触发最终处理并结束建图
                auto now = std::chrono::steady_clock::now();
                double idle_sec = std::chrono::duration<double>(now - last_sensor_data_wall_time_).count();
                if (auto_finish && !sensor_idle_finish_triggered_.load(std::memory_order_acquire) &&
                    first_cloud_logged_.load(std::memory_order_acquire) && idle_sec >= idle_timeout_sec) {
                    sensor_idle_finish_triggered_.store(true, std::memory_order_release);
                    lock.unlock();
                    // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=sensor_idle_timeout idle_sec=%.1f timeout=%.1f → final HBA + save + shutdown", idle_sec, idle_timeout_sec);
                    try {
                        if (submap_manager_.submapCount() > 0) {
                            if (ConfigManager::instance().hbaOnFinish()) {
                                auto all = submap_manager_.getAllSubmaps();
                                // RCLCPP_INFO(get_logger(), "[AutoMapSystem] Sensor idle: triggering final HBA (wait=true) submaps=%zu", all.size());
                                hba_optimizer_.triggerAsync(all, true);
                                // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=final_hba_done");
                            }
                            std::string out_dir = getOutputDir();
                            // RCLCPP_INFO(get_logger(), "[AutoMapSystem] Sensor idle: saving map to %s", out_dir.c_str());
                            saveMapToFiles(out_dir);
                            // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=auto_finish_save_done output_dir=%s", out_dir.c_str());
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
                continue;  // 超时且队列空但未达空闲阈值，仅重新等待
            }

            qs_after_wait = frame_queue_.size();
            f = std::move(frame_queue_.front());
            frame_queue_.pop();
        }
        const int frame_no = backend_cloud_frames_processed_.fetch_add(1) + 1;
        size_t qs_after_pop = 0;
        { std::lock_guard<std::mutex> lk(frame_queue_mutex_); qs_after_pop = frame_queue_.size(); }
        if (frame_no <= 20) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] popped frame_no=%d ts=%.3f queue_after_pop=%zu (wait saw %zu)", frame_no, f.ts, qs_after_pop, qs_after_wait);
        }
        // 按时间戳从缓存对齐 pose/cov 和 kfinfo，不阻塞生产者
        Pose3d pose = Pose3d::Identity();
        Mat66d cov  = Mat66d::Identity() * 1e-4;
        if (!odomCacheGet(f.ts, pose, cov)) {
            no_odom_skip_count++;
            if (no_odom_skip_count <= 15) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][DIAG] no odom in cache for ts=%.3f frame_no=%d skip #%d (odom not yet arrived or ts mismatch)", f.ts, frame_no, no_odom_skip_count);
                RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=skip reason=no_odom_in_cache frame_no=%d ts=%.3f pts=%zu (精准定位: 缓存无对应位姿)", frame_no, f.ts, f.cloud ? f.cloud->size() : 0u);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
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
        if (frame_no <= 5) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] tryCreateKeyFrame entered frame_no=%d ts=%.3f pts=%zu", frame_no, f.ts, f.cloud ? f.cloud->size() : 0u);
        }
        // 若前端发布的是世界系点云（fast_livo /cloud_registered），先转为 body 系再建 KF，否则全局图会因双重变换而错乱
        CloudXYZIPtr cloud_for_kf = f.cloud;
        if (cloud_frame == "world" && f.cloud && !f.cloud->empty()) {
            cloud_for_kf = transformWorldToBody(f.cloud, pose);
            if (frame_no == 1) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] first frame: cloud converted world→body pts=%zu pose=[%.2f,%.2f,%.2f]",
                            cloud_for_kf->size(), pose.translation().x(), pose.translation().y(), pose.translation().z());
            }
        }
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        try {
            tryCreateKeyFrame(f.ts, pose, cov, cloud_for_kf, &kfinfo_copy);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker frame_no=%d ts=%.3f: %s", frame_no, f.ts, e.what());
            RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=fail reason=exception frame_no=%d ts=%.3f step_where=tryCreateKeyFrame what=%s",
                       frame_no, f.ts, e.what());
            ErrorMonitor::instance().recordException(e, errors::LIVO_ODOMETRY_FAILED);
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker frame_no=%d ts=%.3f: unknown", frame_no, f.ts);
            RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=fail reason=unknown_exception frame_no=%d ts=%.3f step_where=tryCreateKeyFrame",
                       frame_no, f.ts);
            ErrorMonitor::instance().recordError(ErrorDetail(errors::UNKNOWN_ERROR, "backend worker unknown exception"));
        }
        const double duration_ms = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count();
        if (frame_no <= 5 || frame_no % 100 == 0 || duration_ms > 200.0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker processed #%d ts=%.3f duration_ms=%.1f", frame_no, f.ts, duration_ms);
        }
        // 数据触发：按处理帧数发布 status / 数据流汇总 / 全局地图，不再使用定时器
        if (frame_no <= 1 || frame_no % 10 == 0) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] frame_no=%d step=publishStatus enter", frame_no);
            publishStatus();
        }
        if (frame_no % 50 == 0) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] frame_no=%d step=publishDataFlowSummary enter", frame_no);
            publishDataFlowSummary();
        }
        if (frame_no % 100 == 0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] frame_no=%d step=publishGlobalMap enter (last log before map publish)", frame_no);
            publishGlobalMap();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] frame_no=%d step=publishGlobalMap done", frame_no);
        }
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker thread exiting");
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
                                      const LivoKeyFrameInfo* optional_livo_info) {
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
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][FRAME] ts=%.3f result=skip_no_kf (dist/rot/interval)", ts);
        RCLCPP_INFO(get_logger(), "[TRACE] step=kf_decision result=skip reason=shouldCreateKeyFrame_false ts=%.3f pts=%zu (精准定位: 距离/旋转/间隔未达阈值)", ts, cur_cloud->size());
        static int reject_count = 0;
        if (++reject_count >= 60) {
            reject_count = 0;
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=kf_candidate_rejected cloud_ts=%.3f reason=dist/rot/interval (throttled)", ts);
        }
        return;
    }

    const double odom_cloud_dt = std::abs(ts - odom_ts);
    if (odom_ts >= 0.0 && odom_cloud_dt > 0.15) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[AutoMapSystem][KF] odom_ts=%.3f cloud_ts=%.3f dt=%.3fs (expected <0.15s)", odom_ts, ts, odom_cloud_dt);
    }

    AUTOMAP_TIMED_SCOPE(MOD, "CreateKeyFrame", 50.0);

    float ds_res = static_cast<float>(ConfigManager::instance().submapMatchRes());
    CloudXYZIPtr cloud_ds = utils::voxelDownsample(cur_cloud, ds_res);
    if (!cloud_ds) cloud_ds = std::make_shared<CloudXYZI>();
    auto t_after_voxel = KfClock::now();
    double ms_voxel = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_voxel - kf_t0).count();
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP] ts=%.3f step=voxel_ds done ms=%.1f ds_pts=%zu", ts, ms_voxel, cloud_ds->size());

    GPSMeasurement gps;
    bool has_gps = false;
    auto gps_opt = gps_manager_.queryByTimestamp(ts);
    if (gps_opt) {
        gps     = *gps_opt;
        has_gps = gps.is_valid;
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
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP] ts=%.3f step=createKeyFrame done ms=%.1f kf_id=%lu", ts, ms_create_kf, kf->id);

    submap_manager_.addKeyFrame(kf);
    auto t_after_add_submap = KfClock::now();
    double ms_add_submap = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(t_after_add_submap - t_after_create_kf).count();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][KF_STEP] ts=%.3f step=addKeyFrame done ms=%.1f sm_id=%d (if >>500ms worker blocked)", ts, ms_add_submap, kf->submap_id);

    if (gps_aligned_ && has_gps && kf->submap_id >= 0) {
        Eigen::Vector3d pos_map = gps_manager_.enu_to_map(gps.position_enu);
        Eigen::Matrix3d cov = gps.covariance;
        if (cov.norm() < 1e-6 || !std::isfinite(cov(0, 0)))
            cov = Eigen::Matrix3d::Identity() * 1.0;
        isam2_optimizer_.addGPSFactor(kf->submap_id, pos_map, cov);
    }

    try { rviz_publisher_.publishCurrentCloud(cur_cloud); } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishCurrentCloud: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishCurrentCloud: unknown exception");
    }

    double ms_total = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(KfClock::now() - kf_t0).count();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][KF] created kf_id=%lu sm_id=%d cloud_ts=%.3f odom_ts=%.3f pts=%zu ds_pts=%zu has_gps=%d degen=%d total_ms=%.1f",
        kf->id, kf->submap_id, ts, odom_ts, cur_cloud->size(), cloud_ds->size(),
        has_gps ? 1 : 0, livo_info.is_degenerate ? 1 : 0, ms_total);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][FRAME] ts=%.3f result=kf_created kf_id=%lu sm_id=%d pts=%zu (tryCreateKeyFrame total_ms=%.1f)",
                ts, kf->id, kf->submap_id, cur_cloud->size(), ms_total);
    RCLCPP_INFO(get_logger(), "[TRACE] step=kf_decision result=ok reason=kf_created kf_id=%lu sm_id=%d ts=%.3f pts=%zu total_ms=%.1f",
                kf->id, kf->submap_id, ts, cur_cloud->size(), ms_total);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=kf_created kf_id=%lu sm_id=%d cloud_ts=%.3f odom_ts=%.3f pts=%zu",
        kf->id, kf->submap_id, ts, odom_ts, cur_cloud->size());
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
    // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=isam2_node_added sm_id=%d is_first=%d node_count=%d", submap->id, is_first ? 1 : 0, isam2_optimizer_.nodeCount());

    // 添加里程计因子（与前一个子图之间）
    auto all_sm = submap_manager_.getFrozenSubmaps();
    if (all_sm.size() >= 2) {
        const auto& prev = all_sm[all_sm.size() - 2];
        Pose3d rel = prev->pose_w_anchor_optimized.inverse() * submap->pose_w_anchor;
        
        // ✅ 修复：动态计算信息矩阵（根据子图质量）
        Mat66d info = computeOdomInfoMatrix(prev, submap, rel);
        
        isam2_optimizer_.addOdomFactor(prev->id, submap->id, rel, info);
        // RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][SM] odom factor prev_sm=%d cur_sm=%d info_norm=%.2e",
        //              prev->id, submap->id, info.norm());
        // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=odom_factor_added prev_sm=%d cur_sm=%d", prev->id, submap->id);
    }

    // 建图精度日志：子图冻结时的几何与锚定帧不确定性
    if (!submap->keyframes.empty()) {
        const Mat66d& anchor_cov = submap->keyframes.front()->covariance;
        double a_px = std::sqrt(std::max(0.0, anchor_cov(3, 3)));
        double a_py = std::sqrt(std::max(0.0, anchor_cov(4, 4)));
        double a_pz = std::sqrt(std::max(0.0, anchor_cov(5, 5)));
        RCLCPP_INFO(get_logger(),
            "[PRECISION][SUBMAP] frozen sm_id=%d kf_count=%zu extent_m=%.2f anchor_pos_std_xyz=[%.4f,%.4f,%.4f] total_frozen=%d",
            submap->id, submap->keyframes.size(), submap->spatial_extent_m, a_px, a_py, a_pz, frozen_submap_count_);
    } else {
        RCLCPP_INFO(get_logger(),
            "[PRECISION][SUBMAP] frozen sm_id=%d kf_count=0 extent_m=%.2f total_frozen=%d",
            submap->id, submap->spatial_extent_m, frozen_submap_count_);
    }

    // 提交到回环检测器
    loop_detector_.addSubmap(submap);

    // 检查 HBA 周期触发
    const int hba_trigger = ConfigManager::instance().hbaTriggerSubmaps();
    if (frozen_submap_count_ % hba_trigger == 0) {
        auto all = submap_manager_.getAllSubmaps();
        hba_optimizer_.triggerAsync(all, false);
        // RCLCPP_INFO(get_logger(), "[AutoMapSystem][SM] HBA trigger (frozen_count=%d mod %d) submaps=%zu",
        //            frozen_submap_count_, hba_trigger, all.size());
        // RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=hba_trigger frozen=%d submaps=%zu", frozen_submap_count_, all.size());
    }
    // 数据触发：子图冻结后立即发布全局地图
    publishGlobalMap();
}

// ─────────────────────────────────────────────────────────────────────────────
// 回环检测处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onLoopDetected(const LoopConstraint::Ptr& lc) {
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
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][LOOP] detected sm_i=%d sm_j=%d score=%.3f inlier=%.3f rmse=%.3f trans=[%.2f,%.2f,%.2f] info_norm=%.2f",
        lc->submap_i, lc->submap_j, lc->overlap_score, lc->inlier_ratio, lc->rmse, tx, ty, tz, info_norm);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=loop_detected sm_i=%d sm_j=%d score=%.3f rmse=%.3f", lc->submap_i, lc->submap_j, lc->overlap_score, lc->rmse);
    ALOG_INFO(MOD, "Loop detected: SM#{} ↔ SM#{} score={:.3f} trans=[{:.2f},{:.2f},{:.2f}]",
              lc->submap_i, lc->submap_j, lc->overlap_score, tx, ty, tz);

    auto result = isam2_optimizer_.addLoopFactor(
        lc->submap_i, lc->submap_j, lc->delta_T, lc->information);

    if (result.success) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][LOOP] optimized nodes_updated=%d elapsed=%.1fms final_rmse=%.4f",
            result.nodes_updated, result.elapsed_ms, result.final_rmse);
        RCLCPP_INFO(get_logger(), "[TRACE] step=loop_factor_add result=ok sm_i=%d sm_j=%d nodes_updated=%d rmse=%.4f elapsed_ms=%.1f",
                   lc->submap_i, lc->submap_j, result.nodes_updated, result.final_rmse, result.elapsed_ms);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=loop_factor_added success=1 sm_i=%d sm_j=%d nodes_updated=%d rmse=%.4f", lc->submap_i, lc->submap_j, result.nodes_updated, result.final_rmse);
    } else {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][LOOP][EXCEPTION] addLoopFactor failed sm_i=%d sm_j=%d", lc->submap_i, lc->submap_j);
        RCLCPP_INFO(get_logger(), "[TRACE] step=loop_factor_add result=fail reason=isam2_node_missing_or_exception sm_i=%d sm_j=%d (精准定位: 见上文 [IncrementalOptimizer][EXCEPTION])",
                   lc->submap_i, lc->submap_j);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=loop_factor_added success=0 sm_i=%d sm_j=%d", lc->submap_i, lc->submap_j);
    }

    state_ = SystemState::MAPPING;
}

// ─────────────────────────────────────────────────────────────────────────────
// 位姿更新（来自 iSAM2）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onPoseUpdated(const std::unordered_map<int, Pose3d>& poses) {
    const size_t n = poses.size();
    int first_id = -1;
    double first_x = 0, first_y = 0, first_z = 0;
    for (const auto& [sm_id, pose] : poses) {
        if (first_id < 0) {
            first_id = sm_id;
            first_x = pose.translation().x();
            first_y = pose.translation().y();
            first_z = pose.translation().z();
        }
        submap_manager_.updateSubmapPose(sm_id, pose);
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][POSE] updated count=%zu first_sm_id=%d pos=[%.2f,%.2f,%.2f]",
        n, first_id, first_x, first_y, first_z);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=pose_updated count=%zu first_sm_id=%d", n, first_id);

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
    opt_path_pub_->publish(opt_path_);
    pub_opt_path_count_++;

    try {
        auto all_sm = submap_manager_.getAllSubmaps();
        rviz_publisher_.publishOptimizedPath(all_sm);
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
    //   - 诊断：定期检查两轨位姿差值（见下）
    // ─────────────────────────────────────────────────────────────────────────────
    
    auto all_sm = submap_manager_.getFrozenSubmaps();
    
    // 诊断：计算 HBA 与 iSAM2 的位姿分离程度
    double max_drift = 0.0;
    double sum_drift = 0.0;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        
        // 获取 HBA 优化结果（新）
        Pose3d hba_pose = sm->pose_w_anchor_optimized;
        
        // 获取 iSAM2 当前估计（旧，可能滞后）
        Pose3d isam2_pose = sm->pose_w_anchor;  // iSAM2 的位姿存储在 pose_w_anchor（初始化后不变）
        
        // 计算欧氏距离
        double drift = (hba_pose.translation() - isam2_pose.translation()).norm();
        max_drift = std::max(max_drift, drift);
        sum_drift += drift;
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
    
    // 尝试同步 iSAM2（近似方法，精确方法需重建因子图）
    // 注意：这里 addSubMapNode 会因为节点已存在而被拒绝，但保留代码以备未来 GTSAM 版本升级
    for (const auto& sm : all_sm) {
        isam2_optimizer_.addSubMapNode(sm->id, sm->pose_w_anchor_optimized, false);
    }
    
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

    // 批量为历史子图补充 GPS 因子（对齐前积累的 GPS 数据）
    addBatchGPSFactors();

    // 通知 HBA 优化器（带 GPS 约束的全局优化）
    auto all = submap_manager_.getAllSubmaps();
    hba_optimizer_.onGPSAligned(result, all);
}

void AutoMapSystem::addBatchGPSFactors() {
    if (gps_batch_added_) return;
    gps_batch_added_ = true;

    auto all_sm = submap_manager_.getFrozenSubmaps();
    int added = 0;
    for (const auto& sm : all_sm) {
        if (!sm->has_valid_gps) continue;
        // GPS中心已经是 ENU 坐标
        auto pos_map = gps_manager_.enu_to_map(sm->gps_center);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1.0;  // 1m 精度
        isam2_optimizer_.addGPSFactor(sm->id, pos_map, cov);
        added++;
    }

    if (added > 0) {
        isam2_optimizer_.forceUpdate();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][GPS] batch factors added submaps=%d total_frozen=%zu", added, all_sm.size());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_batch_factors_added count=%d", added);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishStatus() {
    const int kf_count = submap_manager_.keyframeCount();
    const int sm_count = submap_manager_.submapCount();
    const int loop_ok  = loop_detector_.loopDetectedCount();
    const int hba_trig = hba_optimizer_.triggerCount();
    const bool hba_busy = hba_optimizer_.isRunning();
    const double last_ts = livo_bridge_.lastOdomTs();

    automap_pro::msg::MappingStatusMsg msg;
    msg.header.stamp  = now();
    msg.state         = stateToString(state_.load());
    msg.session_id    = current_session_id_;
    msg.keyframe_count = kf_count;
    msg.submap_count  = sm_count;
    msg.gps_aligned   = gps_aligned_;
    msg.gps_alignment_score = gps_manager_.alignResult().rmse_m > 0 ?
        static_cast<float>(1.0 / gps_manager_.alignResult().rmse_m) : 0.0f;
    status_pub_->publish(msg);
    pub_status_count_++;

    // 每秒输出后端状态到终端，便于观察建图与优化是否正常
    const size_t hba_queue = hba_optimizer_.queueDepth();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][BACKEND] state=%s kf=%d sm=%d loop=%d | [HBA] trig=%d busy=%d queue=%zu last_ts=%.1f",
        msg.state.c_str(), kf_count, sm_count, loop_ok, hba_trig, hba_busy ? 1 : 0, hba_queue, last_ts);

    if (++status_publish_count_ >= 5) {
        status_publish_count_ = 0;
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][PIPELINE] event=heartbeat state=%s kf=%d sm=%d gps=%d",
            msg.state.c_str(), kf_count, sm_count, gps_aligned_ ? 1 : 0);
    }
}

void AutoMapSystem::publishGlobalMap() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=enter");
    const float voxel_size = map_voxel_size_;
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=using_cached_voxel voxel_size=%.3f", voxel_size);
    size_t pts = 0;
    try {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_enter");
        auto global = submap_manager_.buildGlobalMap(voxel_size);
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
        global_map_pub_->publish(cloud_msg);
        pub_map_count_++;
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
        // GPS 约束：子图中心 + 约束线（子图中心 -> 地图系 GPS），一次发布；并发布 GPS 轨迹 Path
        {
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            for (const auto& sm : all_sm) {
                if (!sm || !sm->has_valid_gps) continue;
                gps_positions_map_for_submaps.push_back(gps_manager_.enu_to_map(sm->gps_center));
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
                rviz_publisher_.publishGPSTrajectory(all_sm, gps_positions_map_for_submaps, true);
            } else {
                rviz_publisher_.publishGPSMarkers(all_sm);
                rviz_publisher_.publishGPSTrajectory(all_sm, true);
            }
        }
        // 真实 GPS 位置（按关键帧）转换到地图系后发布（供 RViz 显示）
        {
            std::vector<Eigen::Vector3d> gps_positions_map;
            for (const auto& sm : all_sm) {
                if (!sm) continue;
                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->has_valid_gps) continue;
                    gps_positions_map.push_back(gps_manager_.enu_to_map(kf->gps.position_enu));
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
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][DATA_FLOW] recv: odom=%d cloud=%d empty_cloud=%d last_odom_ts=%.1f last_cloud_ts=%.1f | cache: odom=%zu kfinfo=%zu | frame_queue=%zu dropped=%d backend_frames=%d kf=%d sm=%d | loop: db=%zu queue=%zu detected=%d | hba_trig=%d | pub: odom_path=%d opt_path=%d map=%d status=%d",
        odom_recv, cloud_recv, empty_cloud, last_odom_ts, last_cloud_ts, odom_cache_sz, kfinfo_cache_sz, frame_queue_size, queue_dropped, backend_frames, kf_created, sm_count, loop_db, loop_q, loop_ok, hba_trig,
        pub_odom_path_count_.load(), pub_opt_path_count_.load(), pub_map_count_.load(), pub_status_count_.load());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=data_flow odom=%d cloud=%d queue=%zu dropped=%d backend_frames=%d kf=%d sm=%d loop_ok=%d hba_trig=%d last_odom_ts=%.1f last_cloud_ts=%.1f",
        odom_recv, cloud_recv, frame_queue_size, queue_dropped, backend_frames, kf_created, sm_count, loop_ok, hba_trig, last_odom_ts, last_cloud_ts);

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

// ✅ 修复：动态计算里程计信息矩阵（根据子图质量）
Mat66d AutoMapSystem::computeOdomInfoMatrix(
    const SubMap::Ptr& prev,
    const SubMap::Ptr& curr,
    const Pose3d& rel) const
{
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
            trajectory_odom_file_ << "timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z\n";
            trajectory_odom_file_.flush();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] opened %s", path.c_str());
        }
    }
    if (!trajectory_odom_file_.is_open()) return;
    Eigen::Quaterniond q(pose.rotation());
    double px = std::sqrt(std::max(0.0, cov(3, 3)));
    double py = std::sqrt(std::max(0.0, cov(4, 4)));
    double pz = std::sqrt(std::max(0.0, cov(5, 5)));
    trajectory_odom_file_ << std::fixed << std::setprecision(6)
        << ts << ","
        << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
        << px << "," << py << "," << pz << "\n";
    trajectory_odom_file_.flush();
}

void AutoMapSystem::onGPSMeasurementForLog(double ts, const Eigen::Vector3d& pos_enu) {
    if (!trajectory_log_enabled_) return;
    std::lock_guard<std::mutex> lk(trajectory_log_mutex_);
    ensureTrajectoryLogDir();
    if (!trajectory_gps_file_.is_open()) {
        std::string path = trajectory_log_dir_ + "/trajectory_gps_" + trajectory_session_id_ + ".csv";
        trajectory_gps_file_.open(path, std::ios::out);
        if (trajectory_gps_file_.is_open()) {
            trajectory_gps_file_ << "timestamp,x,y,z,frame\n";
            trajectory_gps_file_.flush();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] opened %s", path.c_str());
        }
    }
    if (!trajectory_gps_file_.is_open()) return;
    Eigen::Vector3d pos = gps_manager_.isAligned() ? gps_manager_.enu_to_map(pos_enu) : pos_enu;
    const char* frame = gps_manager_.isAligned() ? "map" : "enu";
    trajectory_gps_file_ << std::fixed << std::setprecision(6)
        << ts << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << frame << "\n";
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
        auto global = submap_manager_.buildGlobalMap(voxel_size);
        size_t pcd_points = 0;
        if (global && !global->empty()) {
            std::string pcd_path = output_dir + "/global_map.pcd";
            pcl::io::savePCDFileBinary(pcd_path, *global);
            pcd_points = global->size();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Saved %s (%zu points)",
                        pcd_path.c_str(), global->size());
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_pcd path=%s points=%zu", pcd_path.c_str(), pcd_points);
        }

        // 保存 TUM 轨迹（优化后）
        std::string tum_path = output_dir + "/trajectory_tum.txt";
        std::ofstream tum_file(tum_path);
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
            }
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_trajectory path=%s poses=%zu", tum_path.c_str(), trajectory_poses);

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

} // namespace automap_pro
