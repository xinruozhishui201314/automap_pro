#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/backend/pose_graph.h"
#define MOD "AutoMapSystem"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <chrono>

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
}

AutoMapSystem::~AutoMapSystem() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Shutting down...");
    
    // 设置关闭请求标志，通知所有异步任务
    shutdown_requested_.store(true, std::memory_order_release);
    
    // 先停止所有异步任务（给它们时间完成清理）
    loop_detector_.stop();
    hba_optimizer_.stop();
    
    // 等待异步任务完成（100ms缓冲）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 建图结束时触发最终 HBA 优化（如果需要）
    if (ConfigManager::instance().hbaOnFinish()) {
        try {
            auto all_submaps = submap_manager_.getAllSubmaps();
            hba_optimizer_.triggerAsync(all_submaps, /*wait=*/true);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), 
                "[AutoMapSystem] Final HBA failed: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), 
                "[AutoMapSystem] Final HBA failed with unknown exception");
        }
    }
    
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Shutdown complete.");
}

void AutoMapSystem::loadConfigAndInit() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][CONFIG] Declaring parameter config_file");
    declare_parameter("config_file", "");
    std::string config_path = get_parameter("config_file").as_string();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] config_file param=%s", config_path.empty() ? "(empty)" : config_path.c_str());
    if (!config_path.empty()) {
        ConfigManager::instance().load(config_path);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][CONFIG] Config loaded from %s", config_path.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][CONFIG] No config file specified, using defaults");
    }
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

    const auto& cfg = ConfigManager::instance();

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
        [weak_this = weak_from_this()](const HBAResult& result) {
            auto shared_this = weak_this.lock();
            if (!shared_this || shared_this->shutdown_requested_.load(std::memory_order_acquire)) {
                return;
            }
            shared_this->onHBADone(result);
        });
    hba_optimizer_.start();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 6c: HBAOptimizer started");

    // GPS 管理器
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][DEFERRED] Step 7: register GPSManager callbacks");
    gps_manager_.registerAlignCallback(
        [this](const GPSAlignResult& r) { onGPSAligned(r); });
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
    RCLCPP_INFO(get_logger(), "=== AutoMapSystem READY (session_id=%lu) ===", current_session_id_);
}

void AutoMapSystem::setupPublishers() {
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PUB] Creating publishers");
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    auto map_qos = rclcpp::SensorDataQoS();
    odom_path_pub_  = create_publisher<nav_msgs::msg::Path>("/automap/odom_path", path_qos);
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PUB] /automap/odom_path");
    opt_path_pub_   = create_publisher<nav_msgs::msg::Path>("/automap/optimized_path", path_qos);
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PUB] /automap/optimized_path");
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/automap/global_map", map_qos);
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][PUB] /automap/global_map");
    status_pub_     = create_publisher<automap_pro::msg::MappingStatusMsg>("/automap/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PUB] All 4 publishers created (odom_path, opt_path, global_map, status)");
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
    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][TIMER] Creating wall timers");
    status_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this] { publishStatus(); });
    map_pub_timer_ = create_wall_timer(
        std::chrono::seconds(10), [this] { publishGlobalMap(); });
    data_flow_timer_ = create_wall_timer(
        std::chrono::seconds(15), [this] { publishDataFlowSummary(); });
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][TIMER] status_timer=1Hz, map_pub_timer=0.1Hz, data_flow=1/15Hz");
}

// ─────────────────────────────────────────────────────────────────────────────
// 数据流：里程计 → 关键帧 → 子图
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    if (!first_odom_logged_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA] First odometry received ts=%.3f", ts);
    }
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_odom_pose_ = pose;
        last_odom_ts_   = ts;
        last_cov_       = cov;
    }

    // 更新 GPS 关键帧轨迹（用于对齐）
    gps_manager_.addKeyFramePose(ts, pose);

    // 里程计路径
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
    if (odom_path_.poses.size() > 10000) odom_path_.poses.erase(odom_path_.poses.begin());
    odom_path_pub_->publish(odom_path_);
    pub_odom_path_count_++;

    // 尝试创建关键帧
    tryCreateKeyFrame(ts);
}

void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud) {
    if (!first_cloud_logged_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA] First cloud received ts=%.3f points=%zu", ts, cloud ? cloud->size() : 0u);
    }
    std::lock_guard<std::mutex> lk(data_mutex_);
    last_cloud_ = cloud;
}

void AutoMapSystem::onKFInfo(const LivoKeyFrameInfo& info) {
    last_livo_info_ = info;
    kf_manager_.updateLivoInfo(info);
}

void AutoMapSystem::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats) {
    gps_manager_.addGPSMeasurement(ts, lat, lon, alt, hdop, sats);
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

    if (!kf_manager_.shouldCreateKeyFrame(cur_pose, ts)) return;
    if (!cur_cloud || cur_cloud->empty()) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][KF] ts=%.3f cloud empty, skip", ts);
        ALOG_WARN(MOD, "KF candidate at ts={:.3f}: cloud empty, skipped", ts);
        return;
    }

    AUTOMAP_TIMED_SCOPE(MOD, "CreateKeyFrame", 50.0);

    // 下采样点云（用于回环匹配）
    CloudXYZIPtr cloud_ds(new CloudXYZI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cur_cloud);
    float ds_res = static_cast<float>(ConfigManager::instance().submapMatchRes());
    vg.setLeafSize(ds_res, ds_res, ds_res);
    vg.filter(*cloud_ds);

    // 获取 GPS 测量
    GPSMeasurement gps;
    bool has_gps = false;
    auto gps_opt = gps_manager_.queryByTimestamp(ts);
    if (gps_opt) {
        gps     = *gps_opt;
        has_gps = gps.is_valid;
    }

    // 创建关键帧
    auto kf = kf_manager_.createKeyFrame(
        cur_pose, cur_cov, ts,
        cur_cloud, cloud_ds,
        gps, has_gps, current_session_id_);

    // 添加到子图（触发子图切分判断）
    submap_manager_.addKeyFrame(kf);

    // GPS 对齐后，每帧带有效 GPS 的关键帧都向 iSAM2 添加 GPS 因子（持续约束）
    if (gps_aligned_ && has_gps && kf->submap_id >= 0) {
        Eigen::Vector3d pos_map = gps_manager_.enu_to_map(gps.position_enu);
        Eigen::Matrix3d cov = gps.covariance;
        if (cov.norm() < 1e-6 || !std::isfinite(cov(0, 0)))
            cov = Eigen::Matrix3d::Identity() * 1.0;
        isam2_optimizer_.addGPSFactor(kf->submap_id, pos_map, cov);
    }

    // 发布当前帧点云到 RViz（/automap/current_cloud）
    try { rviz_publisher_.publishCurrentCloud(cur_cloud); } catch (const std::exception&) {}

    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][KF] created kf_id=%d sm_id=%d ts=%.3f pts=%zu ds_pts=%zu has_gps=%d degen=%d",
        kf->id, kf->submap_id, ts, cur_cloud->size(), cloud_ds->size(),
        has_gps ? 1 : 0, last_livo_info_.is_degenerate ? 1 : 0);
    ALOG_DEBUG(MOD, "KF#{} created: pts={} ds_pts={} has_gps={} livo_degen={}",
               kf->id, cur_cloud->size(), cloud_ds->size(),
               has_gps, last_livo_info_.is_degenerate);
}

// ─────────────────────────────────────────────────────────────────────────────
// 子图冻结处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onSubmapFrozen(const SubMap::Ptr& submap) {
    frozen_submap_count_++;
    const double ax = submap->pose_w_anchor.translation().x();
    const double ay = submap->pose_w_anchor.translation().y();
    const double az = submap->pose_w_anchor.translation().z();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][SM] frozen sm_id=%d kf_count=%zu t=[%.3f,%.3f] dist=%.2fm anchor=[%.2f,%.2f,%.2f] total_frozen=%d",
        submap->id, submap->keyframes.size(), submap->t_start, submap->t_end,
        submap->spatial_extent_m, ax, ay, az, frozen_submap_count_);
    ALOG_INFO(MOD, "SM#{} FROZEN: kf={} dist={:.1f}m total_frozen={}",
              submap->id, submap->keyframes.size(),
              submap->spatial_extent_m, frozen_submap_count_);

    // 添加子图节点到 iSAM2
    bool is_first = (isam2_optimizer_.nodeCount() == 0);
    isam2_optimizer_.addSubMapNode(submap->id, submap->pose_w_anchor, is_first);

    // 添加里程计因子（与前一个子图之间）
    auto all_sm = submap_manager_.getFrozenSubmaps();
    if (all_sm.size() >= 2) {
        const auto& prev = all_sm[all_sm.size() - 2];
        Pose3d rel = prev->pose_w_anchor_optimized.inverse() * submap->pose_w_anchor;
        
        // ✅ 修复：动态计算信息矩阵（根据子图质量）
        Mat66d info = computeOdomInfoMatrix(prev, submap, rel);
        
        isam2_optimizer_.addOdomFactor(prev->id, submap->id, rel, info);
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][SM] odom factor prev_sm=%d cur_sm=%d info_norm=%.2e",
                     prev->id, submap->id, info.norm());
    }

    // 提交到回环检测器
    loop_detector_.addSubmap(submap);

    // 检查 HBA 周期触发
    const int hba_trigger = ConfigManager::instance().hbaTriggerSubmaps();
    if (frozen_submap_count_ % hba_trigger == 0) {
        auto all = submap_manager_.getAllSubmaps();
        hba_optimizer_.triggerAsync(all, false);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SM] HBA trigger (frozen_count=%d mod %d) submaps=%zu",
                   frozen_submap_count_, hba_trigger, all.size());
    }
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
    ALOG_INFO(MOD, "Loop detected: SM#{} ↔ SM#{} score={:.3f} trans=[{:.2f},{:.2f},{:.2f}]",
              lc->submap_i, lc->submap_j, lc->overlap_score, tx, ty, tz);

    auto result = isam2_optimizer_.addLoopFactor(
        lc->submap_i, lc->submap_j, lc->delta_T, lc->information);

    if (result.success) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][LOOP] optimized nodes_updated=%d elapsed=%.1fms final_rmse=%.4f",
            result.nodes_updated, result.elapsed_ms, result.final_rmse);
    } else {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][LOOP] addLoopFactor failed");
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

    // 发布优化后轨迹
    opt_path_.header.stamp    = now();
    opt_path_.header.frame_id = "map";
    opt_path_.poses.clear();
    for (const auto& [sm_id, pose] : poses) {
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
    } catch (const std::exception&) { /* ignore */ }
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onHBADone(const HBAResult& result) {
    if (!result.success) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][HBA] optimization failed");
        return;
    }
    const size_t pose_count = result.optimized_poses.size();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][HBA] done success=1 MME=%.4f poses=%zu iter_layer=%d elapsed=%.1fms",
        result.final_mme, pose_count, result.iterations_per_layer, result.elapsed_ms);

    // 将 HBA 优化结果写回所有子图的关键帧
    submap_manager_.updateAllFromHBA(result);

    try {
        rviz_publisher_.publishHBAResult(result);
    } catch (const std::exception&) { /* ignore */ }

    // 同步更新 iSAM2 线性化点（重新提交优化后位姿作为先验）
    auto all_sm = submap_manager_.getFrozenSubmaps();
    for (const auto& sm : all_sm) {
        // 重设 iSAM2 节点位姿（通过添加强先验覆盖）
        // 注意：这是一种近似方法，精确方法需要重建整个因子图
        isam2_optimizer_.addSubMapNode(sm->id, sm->pose_w_anchor_optimized, false);
    }
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
    } catch (const std::exception&) { /* ignore */ }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][GPS] aligned rmse_m=%.2f matched=%d R_diag=[%.2f,%.2f,%.2f] t=[%.2f,%.2f,%.2f]",
        result.rmse_m, result.matched_points,
        result.R_gps_lidar(0,0), result.R_gps_lidar(1,1), result.R_gps_lidar(2,2),
        result.t_gps_lidar.x(), result.t_gps_lidar.y(), result.t_gps_lidar.z());

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
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishStatus() {
    const int kf_count = submap_manager_.keyframeCount();
    const int sm_count = submap_manager_.submapCount();
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

    if (++status_publish_count_ >= 10) {
        status_publish_count_ = 0;
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][STATUS] state=%s kf=%d sm=%d gps_aligned=%d",
            msg.state.c_str(), kf_count, sm_count, gps_aligned_ ? 1 : 0);
    }
}

void AutoMapSystem::publishGlobalMap() {
    float voxel_size = static_cast<float>(ConfigManager::instance().mapVoxelSize());
    auto global = submap_manager_.buildGlobalMap(voxel_size);
    if (!global || global->empty()) {
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP] global map empty, skip publish");
        return;
    }
    const size_t pts = global->size();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*global, cloud_msg);
    cloud_msg.header.stamp    = now();
    cloud_msg.header.frame_id = "map";
    global_map_pub_->publish(cloud_msg);
    pub_map_count_++;

    // 综合可视化：子图边界/框/连接图、回环、GPS、坐标轴
    try {
        rviz_publisher_.publishGlobalMap(global);
        auto all_sm = submap_manager_.getAllSubmaps();
        rviz_publisher_.publishSubmapBoundaries(all_sm);
        rviz_publisher_.publishSubmapBoundingBoxes(all_sm);
        rviz_publisher_.publishSubmapGraph(all_sm);
        rviz_publisher_.publishOptimizedPath(all_sm);
        rviz_publisher_.publishGPSMarkers(all_sm);
        {
            std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
            if (!loop_constraints_.empty())
                rviz_publisher_.publishLoopMarkers(loop_constraints_, all_sm);
        }
        // 因子图可视化：由当前子图 + 回环约束构建临时 PoseGraph
        if (all_sm.size() >= 1) {
            PoseGraph graph;
            std::vector<SubMap::Ptr> sorted = all_sm;
            std::sort(sorted.begin(), sorted.end(),
                [](const SubMap::Ptr& a, const SubMap::Ptr& b) { return a->id < b->id; });
            for (const auto& sm : sorted) {
                if (!sm) continue;
                GraphNode n;
                n.id = sm->id;
                n.pose = sm->pose_w_anchor_optimized;
                n.pose_opt = sm->pose_w_anchor_optimized;
                n.fixed = (sm->id == sorted.front()->id);
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
        }

        Pose3d base_pose;
        { std::lock_guard<std::mutex> lk(data_mutex_); base_pose = last_odom_pose_; }
        rviz_publisher_.publishCoordinateFrames(base_pose, base_pose);
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP] rviz publish skip: %s", e.what());
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] published points=%zu voxel=%.3f", pts, voxel_size);
}

void AutoMapSystem::publishDataFlowSummary() {
    const int odom_recv  = livo_bridge_.odomCount();
    const int cloud_recv = livo_bridge_.cloudCount();
    const double last_ts = livo_bridge_.lastOdomTs();
    const int kf_created  = kf_manager_.keyframeCount();
    const int sm_count   = submap_manager_.submapCount();
    const int kf_total   = submap_manager_.keyframeCount();
    const size_t loop_db = loop_detector_.dbSize();
    const size_t loop_q  = loop_detector_.queueSize();
    const int loop_ok    = loop_detector_.loopDetectedCount();
    const int hba_trig   = hba_optimizer_.triggerCount();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][DATA_FLOW] recv: odom=%d cloud=%d last_ts=%.1f | kf=%d sm=%d | loop: db=%zu queue=%zu detected=%d | hba_trig=%d | pub: odom_path=%d opt_path=%d map=%d status=%d",
        odom_recv, cloud_recv, last_ts, kf_created, sm_count, loop_db, loop_q, loop_ok, hba_trig,
        pub_odom_path_count_.load(), pub_opt_path_count_.load(), pub_map_count_.load(), pub_status_count_.load());
}

// ─────────────────────────────────────────────────────────────────────────────
// 服务处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::handleSaveMap(
    const std::shared_ptr<automap_pro::srv::SaveMap::Request> req,
    std::shared_ptr<automap_pro::srv::SaveMap::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] SaveMap to %s", req->output_dir.c_str());
    state_ = SystemState::SAVING;
    try {
        saveMapToFiles(req->output_dir);
        res->success     = true;
        res->output_path = req->output_dir;
        res->message     = "Map saved successfully";
    } catch (const std::exception& e) {
        res->success = false;
        res->message = e.what();
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
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Triggered HBA (wait=%d)", req->wait_for_result);
    auto all = submap_manager_.getAllSubmaps();
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
    for (int i = 0; i < 9; ++i) res->r_gps_lidar[i] = r.R_gps_lidar(i/3, i%3);
    for (int i = 0; i < 3; ++i) res->t_gps_lidar[i] = r.t_gps_lidar[i];
}

void AutoMapSystem::handleLoadSession(
    const std::shared_ptr<automap_pro::srv::LoadSession::Request> req,
    std::shared_ptr<automap_pro::srv::LoadSession::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loading session from %s", req->session_dir.c_str());
    // 增量式建图：加载历史子图的描述子到回环检测数据库
    int loaded = 0;
    for (auto& entry : fs::directory_iterator(req->session_dir)) {
        if (!entry.is_directory()) continue;
        std::string name = entry.path().filename().string();
        if (name.find("submap_") != 0) continue;

        int sm_id = std::stoi(name.substr(7));
        SubMap::Ptr sm;
        if (submap_manager_.loadArchivedSubmap(req->session_dir, sm_id, sm)) {
            sm->session_id = req->session_id;
            loop_detector_.addToDatabase(sm);
            loaded++;
        }
    }
    res->success          = (loaded > 0);
    res->submaps_loaded   = loaded;
    res->descriptors_loaded = loaded;
    res->message          = std::to_string(loaded) + " submaps loaded";
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loaded %d submaps from session %lu",
                loaded, req->session_id);
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
// 地图保存
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::saveMapToFiles(const std::string& output_dir) {
    fs::create_directories(output_dir);

    // 保存全局 PCD
    float voxel_size = static_cast<float>(ConfigManager::instance().mapVoxelSize());
    auto global = submap_manager_.buildGlobalMap(voxel_size);
    if (global && !global->empty()) {
        std::string pcd_path = output_dir + "/global_map.pcd";
        pcl::io::savePCDFileBinary(pcd_path, *global);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Saved %s (%zu points)",
                    pcd_path.c_str(), global->size());
    }

    // 保存 TUM 轨迹（优化后）
    std::ofstream tum_file(output_dir + "/trajectory_tum.txt");
    auto all_sm = submap_manager_.getAllSubmaps();
    for (const auto& sm : all_sm) {
        for (const auto& kf : sm->keyframes) {
            const auto& T = kf->T_w_b_optimized;
            Eigen::Quaterniond q(T.rotation());
            tum_file << std::fixed << std::setprecision(6)
                     << kf->timestamp << " "
                     << T.translation().x() << " "
                     << T.translation().y() << " "
                     << T.translation().z() << " "
                     << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
    }

    // 归档子图（用于下次增量建图加载）
    std::string session_dir = output_dir + "/session_" + std::to_string(current_session_id_);
    fs::create_directories(session_dir);
    for (auto& sm : all_sm) {
        submap_manager_.archiveSubmap(sm, session_dir);
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Session archived to %s", session_dir.c_str());
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
