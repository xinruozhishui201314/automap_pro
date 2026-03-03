#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
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
    // 初始化日志系统（最先执行）
    const char* log_dir_env = std::getenv("AUTOMAP_LOG_DIR");
    const char* log_lvl_env = std::getenv("AUTOMAP_LOG_LEVEL");
    std::string log_dir = log_dir_env ? log_dir_env : "/tmp/automap_logs";
    std::string log_lvl = log_lvl_env ? log_lvl_env : "info";
    automap_pro::Logger::instance().init(log_dir, log_lvl);

    ALOG_INFO(MOD, "=== AutoMapSystem v2.0 starting ===");
    ALOG_INFO(MOD, "Build type={} log_dir={} log_level={}", CMAKE_BUILD_TYPE_STR, log_dir, log_lvl);
    RCLCPP_INFO(get_logger(), "=== AutoMapSystem v2.0 starting ===");
    loadConfigAndInit();
    setupModules();
    setupPublishers();
    setupServices();
    setupTimers();
    state_ = SystemState::MAPPING;
    RCLCPP_INFO(get_logger(), "=== AutoMapSystem READY ===");
}

AutoMapSystem::~AutoMapSystem() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Shutting down...");
    // 建图结束时触发最终 HBA 优化
    if (ConfigManager::instance().hbaOnFinish()) {
        auto all_submaps = submap_manager_.getAllSubmaps();
        hba_optimizer_.triggerAsync(all_submaps, /*wait=*/true);
    }
    loop_detector_.stop();
    hba_optimizer_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Shutdown complete.");
}

void AutoMapSystem::loadConfigAndInit() {
    // 从 ROS2 参数加载配置文件路径
    declare_parameter("config_file", "");
    std::string config_path = get_parameter("config_file").as_string();
    if (!config_path.empty()) {
        ConfigManager::instance().load(config_path);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Config loaded from %s", config_path.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem] No config file specified, using defaults");
    }
}

void AutoMapSystem::setupModules() {
    const auto& cfg = ConfigManager::instance();

    // 分配新 session ID
    current_session_id_ = static_cast<uint64_t>(
        std::chrono::system_clock::now().time_since_epoch().count());

    // 子图管理器
    submap_manager_.init(shared_from_this());
    submap_manager_.startNewSession(current_session_id_);
    submap_manager_.registerSubmapFrozenCallback(
        [this](const SubMap::Ptr& sm) { onSubmapFrozen(sm); });

    // 回环检测器
    loop_detector_.init(shared_from_this());
    loop_detector_.registerLoopCallback(
        [this](const LoopConstraint::Ptr& lc) { onLoopDetected(lc); });
    loop_detector_.start();

    // iSAM2 优化器
    isam2_optimizer_.registerPoseUpdateCallback(
        [this](const std::unordered_map<int, Pose3d>& poses) { onPoseUpdated(poses); });

    // HBA 优化器
    hba_optimizer_.init();
    hba_optimizer_.registerDoneCallback(
        [this](const HBAResult& result) { onHBADone(result); });
    hba_optimizer_.start();

    // GPS 管理器
    gps_manager_.registerAlignCallback(
        [this](const GPSAlignResult& r) { onGPSAligned(r); });
    gps_manager_.registerGpsFactorCallback(
        [this](double ts, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov) {
            // 仅对齐后才添加 GPS 因子
            if (!gps_aligned_) return;
            // 找最近时间的子图节点
            auto submaps = submap_manager_.getFrozenSubmaps();
            if (submaps.empty()) return;
            // 找最近时间戳的子图
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

    RCLCPP_INFO(get_logger(), "[AutoMapSystem] All modules initialized (session_id=%lu)",
                current_session_id_);
}

void AutoMapSystem::setupPublishers() {
    odom_path_pub_  = create_publisher<nav_msgs::msg::Path>("/automap/odom_path", 1);
    opt_path_pub_   = create_publisher<nav_msgs::msg::Path>("/automap/optimized_path", 1);
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/automap/global_map", 1);
    status_pub_     = create_publisher<automap_pro::msg::MappingStatusMsg>("/automap/status", 10);
}

void AutoMapSystem::setupServices() {
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
}

void AutoMapSystem::setupTimers() {
    // 状态发布（1Hz）
    status_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this] { publishStatus(); });
    // 全局地图发布（0.1Hz，10s一次，避免带宽压力）
    map_pub_timer_ = create_wall_timer(
        std::chrono::seconds(10), [this] { publishGlobalMap(); });
}

// ─────────────────────────────────────────────────────────────────────────────
// 数据流：里程计 → 关键帧 → 子图
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
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

    // 尝试创建关键帧
    tryCreateKeyFrame(ts);
}

void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud) {
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

    ALOG_DEBUG(MOD, "KF#{} created: pts={} ds_pts={} has_gps={} livo_degen={}",
               kf->id, cur_cloud->size(), cloud_ds->size(),
               has_gps, last_livo_info_.is_degenerate);

    // 添加到子图（触发子图切分判断）
    submap_manager_.addKeyFrame(kf);
}

// ─────────────────────────────────────────────────────────────────────────────
// 子图冻结处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onSubmapFrozen(const SubMap::Ptr& submap) {
    frozen_submap_count_++;
    ALOG_INFO(MOD, "SM#{} FROZEN: kf={} dist={:.1f}m total_frozen={}",
              submap->id, submap->keyframes.size(),
              submap->spatial_extent_m, frozen_submap_count_);
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem] Submap %d frozen (kf=%zu, dist=%.1fm) total=%d",
        submap->id, submap->keyframes.size(), submap->spatial_extent_m, frozen_submap_count_);

    // 添加子图节点到 iSAM2
    bool is_first = (isam2_optimizer_.nodeCount() == 0);
    isam2_optimizer_.addSubMapNode(submap->id, submap->pose_w_anchor, is_first);

    // 添加里程计因子（与前一个子图之间）
    auto all_sm = submap_manager_.getFrozenSubmaps();
    if (all_sm.size() >= 2) {
        const auto& prev = all_sm[all_sm.size() - 2];
        Pose3d rel = prev->pose_w_anchor_optimized.inverse() * submap->pose_w_anchor;
        Mat66d info = Mat66d::Identity() * 10.0;  // 里程计置信度
        isam2_optimizer_.addOdomFactor(prev->id, submap->id, rel, info);
    }

    // 提交到回环检测器
    loop_detector_.addSubmap(submap);

    // 检查 HBA 周期触发
    if (frozen_submap_count_ % ConfigManager::instance().hbaTriggerSubmaps() == 0) {
        auto all = submap_manager_.getAllSubmaps();
        hba_optimizer_.triggerAsync(all, false);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 回环检测处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onLoopDetected(const LoopConstraint::Ptr& lc) {
    state_ = SystemState::LOOP_CLOSING;
    ALOG_INFO(MOD, "Loop detected: SM#{} ↔ SM#{} score={:.3f} trans=[{:.2f},{:.2f},{:.2f}]",
              lc->submap_i, lc->submap_j, lc->overlap_score,
              lc->delta_T.translation().x(),
              lc->delta_T.translation().y(),
              lc->delta_T.translation().z());

    auto result = isam2_optimizer_.addLoopFactor(
        lc->submap_i, lc->submap_j, lc->delta_T, lc->information);

    if (result.success) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem] Loop optimized: %d submaps updated in %.1fms",
            result.nodes_updated, result.elapsed_ms);
    }

    state_ = SystemState::MAPPING;
}

// ─────────────────────────────────────────────────────────────────────────────
// 位姿更新（来自 iSAM2）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onPoseUpdated(const std::unordered_map<int, Pose3d>& poses) {
    for (const auto& [sm_id, pose] : poses) {
        submap_manager_.updateSubmapPose(sm_id, pose);
    }

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
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onHBADone(const HBAResult& result) {
    if (!result.success) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem] HBA optimization failed");
        return;
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem] HBA done: MME=%.4f elapsed=%.1fms",
        result.final_mme, result.elapsed_ms);

    // 将 HBA 优化结果写回所有子图的关键帧
    submap_manager_.updateAllFromHBA(result);

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
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem] GPS ALIGNED! RMSE=%.2fm matched=%d R=[%.2f,%.2f,%.2f]",
        result.rmse_m, result.matched_points,
        result.R_gps_lidar(0,0), result.R_gps_lidar(1,1), result.R_gps_lidar(2,2));

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
            "[AutoMapSystem] Batch GPS factors added: %d submaps", added);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishStatus() {
    automap_pro::msg::MappingStatusMsg msg;
    msg.header.stamp  = now();
    msg.state         = stateToString(state_.load());
    msg.session_id    = current_session_id_;
    msg.keyframe_count = submap_manager_.keyframeCount();
    msg.submap_count  = submap_manager_.submapCount();
    msg.gps_aligned   = gps_aligned_;
    msg.gps_alignment_score = gps_manager_.alignResult().rmse_m > 0 ?
        static_cast<float>(1.0 / gps_manager_.alignResult().rmse_m) : 0.0f;
    status_pub_->publish(msg);
}

void AutoMapSystem::publishGlobalMap() {
    float voxel_size = static_cast<float>(ConfigManager::instance().mapVoxelSize());
    auto global = submap_manager_.buildGlobalMap(voxel_size);
    if (!global || global->empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*global, cloud_msg);
    cloud_msg.header.stamp    = now();
    cloud_msg.header.frame_id = "map";
    global_map_pub_->publish(cloud_msg);
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
