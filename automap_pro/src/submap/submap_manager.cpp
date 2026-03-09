#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/error_code.h"
#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/utils.h"
#define MOD "SubMapMgr"

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <automap_pro/msg/sub_map_event_msg.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace automap_pro {

SubMapManager::SubMapManager() {
    const auto& cfg = ConfigManager::instance();
    max_kf_       = cfg.submapMaxKF();
    max_spatial_  = cfg.submapMaxSpatial();
    max_temporal_ = cfg.submapMaxTemporal();
    match_res_    = cfg.submapMatchRes();
    merge_res_    = cfg.submapMergeRes();
    
    // ✅ P1 修复：配置检查
    bool retain_cloud = cfg.retainCloudBody();
    bool allow_archival = cfg.allowCloudArchival();
    
    if (!retain_cloud && allow_archival) {
        SLOG_WARN(MOD, 
            "⚠️  CONFIGURATION WARNING: retain_cloud_body=false AND allow_cloud_archival=true\n"
            "  This may cause buildGlobalMap to use merged_cloud (旧世界系)\n"
            "  Recommendation: Set retain_cloud_body=true to ensure main path is always available");
    } else if (!retain_cloud) {
        SLOG_WARN(MOD,
            "⚠️  retain_cloud_body=false: Keyframe point clouds will not be preserved\n"
            "  If this is intentional for memory savings, ensure allow_cloud_archival=false\n"
            "  Otherwise, recommend setting retain_cloud_body=true");
    } else {
        SLOG_INFO(MOD, "✅ retain_cloud_body=true: Main path (buildGlobalMap via T_w_b_optimized) will be used");
    }
}

SubMapManager::~SubMapManager() {
    freeze_post_running_.store(false);
    freeze_post_cv_.notify_all();
    if (freeze_post_thread_.joinable())
        freeze_post_thread_.join();
}

void SubMapManager::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    event_pub_ = node->create_publisher<automap_pro::msg::SubMapEventMsg>(
        "/automap/submap_event", 50);
    RCLCPP_INFO(node->get_logger(), "[SubMapMgr][TOPIC] publish: /automap/submap_event");
    freeze_post_thread_ = std::thread(&SubMapManager::freezePostProcessLoop, this);
}

void SubMapManager::startNewSession(uint64_t session_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    current_session_id_ = session_id;
    active_submap_ = nullptr;  // 新 session 重新开始子图
}

void SubMapManager::addKeyFrame(const KeyFrame::Ptr& kf) {
    SLOG_START_SPAN(MOD, "add_keyframe");
    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][ADD_KF_STEP] addKeyFrame enter (worker holds no other AutoMap lock)");

    if (!kf) {
        auto err = ErrorDetail(errors::KEYFRAME_CREATE_FAILED, "addKeyFrame: null keyframe");
        err.context().operation = "addKeyFrame";
        err.context().file = __FILE__;
        err.context().line = __LINE__;
        SLOG_ERROR_CODE(MOD, static_cast<uint32_t>(err.code()), err.message());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapManager] addKeyFrame: null keyframe rejected");
        ErrorMonitor::instance().recordError(err);
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        return;
    }

    std::unique_lock<std::mutex> lk(mutex_);

    METRIC_TIMED_SCOPE(metrics::POINTCLOUD_PROCESS_TIME_MS);

    try {
        kf->id         = kf_id_counter_++;
        kf->session_id = current_session_id_;

        // 如果没有活跃子图，创建一个
        if (!active_submap_) {
            active_submap_ = createNewSubmap(kf);
            if (!active_submap_) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapMgr][ADD_KF_STEP] createNewSubmap returned null, skip addKeyFrame");
                return;
            }
            submaps_.push_back(active_submap_);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][ADD_KF_STEP] created new submap sm_id=%d (first KF)", active_submap_->id);
            // 建图精度分析：子图锚定帧位姿协方差(1σ)
            const Mat66d& cov = kf->covariance;
            double pos_std_x = std::sqrt(std::max(0.0, cov(3, 3)));
            double pos_std_y = std::sqrt(std::max(0.0, cov(4, 4)));
            double pos_std_z = std::sqrt(std::max(0.0, cov(5, 5)));
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[PRECISION][SUBMAP] created sm_id=%d kf_id=%lu pos_std_xyz=[%.4f,%.4f,%.4f]m",
                active_submap_->id, kf->id, pos_std_x, pos_std_y, pos_std_z);
            SLOG_INFO(MOD, "Created new submap: id={}, session_id={}", 
                      active_submap_->id, current_session_id_);
            
            // 记录指标
            METRICS_INCREMENT(metrics::SUBMAPS_CREATED);
        }

        // 添加关键帧到子图
        kf->submap_id = active_submap_->id;
        active_submap_->keyframes.push_back(kf);
        active_submap_->t_end = kf->timestamp;

        // 更新锚定位姿（第一帧）
        if (active_submap_->keyframes.size() == 1) {
            active_submap_->pose_w_anchor           = kf->T_w_b;
            active_submap_->pose_w_anchor_optimized = kf->T_w_b;
            kf->is_anchor = true;
        }

        // 更新 GPS 中心（所有有效 GPS 的平均值）
        if (kf->has_valid_gps) {
            updateGPSGravityCenter(kf);
        }

        // 合并点云（带内存检查）
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][ADD_KF_STEP] before mergeCloudToSubmap kf_id=%lu pts=%zu", kf->id, kf->cloud_body ? kf->cloud_body->size() : 0u);
        mergeCloudToSubmap(active_submap_, kf);
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][ADD_KF_STEP] after mergeCloudToSubmap sm_id=%d merged_pts=%zu", active_submap_->id, active_submap_->merged_cloud ? active_submap_->merged_cloud->size() : 0u);

        // 更新空间范围（最近帧到锚定帧的最大距离）
        double dist = (kf->T_w_b.translation() -
                       active_submap_->pose_w_anchor.translation()).norm();
        active_submap_->spatial_extent_m = std::max(active_submap_->spatial_extent_m, dist);

        // 结构化日志：调试信息
        SLOG_DEBUG(MOD, "KF processed: id={}, sm_id={}, kf_count={}, dist={:.2f}m",
                     kf->id, active_submap_->id, active_submap_->keyframes.size(),
                     active_submap_->spatial_extent_m);

        // 更新健康检查：队列大小（禁止在持 mutex_ 时调用 getFrozenSubmaps()，否则同锁重入死锁）
        {
            size_t frozen_count = 0;
            for (const auto& s : submaps_) {
                if (s->state == SubMapState::FROZEN || s->state == SubMapState::OPTIMIZED)
                    frozen_count++;
            }
            HEALTH_UPDATE_QUEUE("submap", frozen_count);
        }

        if (isFull(active_submap_)) {
            const int sm_id = active_submap_->id;
            const size_t kf_count = active_submap_->keyframes.size();
            const double dist = active_submap_->spatial_extent_m;
            SubMap::Ptr to_freeze = active_submap_;
            active_submap_ = nullptr;

            SLOG_INFO(MOD, "SubMap FULL: id={}, kf={}, dist={:.1f}m → freezing",
                       sm_id, kf_count, dist);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][ADD_KF_STEP] isFull=true sm_id=%d kf_count=%zu → unlock before freeze (avoid deadlock)", sm_id, kf_count);

            lk.unlock();
            try {
                freezeActiveSubmap(to_freeze);
            } catch (const std::exception& e) {
                auto err = ErrorDetail::fromException(e, errors::SUBMAP_STATE_INVALID);
                SLOG_ERROR_CODE(MOD, static_cast<uint32_t>(err.code()),
                              fmt::format("Failed to freeze submap #{}: {}", sm_id, e.what()));
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[SubMapManager][EXCEPTION] Failed to freeze submap #%d: %s", sm_id, e.what());
                METRICS_INCREMENT(metrics::ERRORS_TOTAL);
            }
            lk.lock();
        }

    } catch (const std::exception& e) {
        // 使用错误码系统
        auto err = ErrorDetail::fromException(e, errors::SUBMAP_MERGE_FAILED);
        err.context().operation = "addKeyFrame";
        err.context().file = __FILE__;
        err.context().line = __LINE__;
        err.context().function = __func__;
        err.addSuggestion(RecoverySuggestion{
            "Check point cloud data validity",
            "Data should contain valid XYZ values",
            2, false
        });
        err.setRetryable(true, 3, 200);

        SLOG_ERROR_CODE(MOD, static_cast<uint32_t>(err.code()), err.message());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[SubMapManager][EXCEPTION] addKeyFrame: %s", err.message().c_str());

        // 记录错误指标
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);

        // 发布错误事件
        publishErrorEvent(active_submap_ ? active_submap_->id : 0, err);
    }
    
    SLOG_END_SPAN();
    METRICS_INCREMENT(metrics::KEYFRAMES_CREATED);
    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][ADD_KF_STEP] addKeyFrame exit kf_id=%lu", kf->id);
}

bool SubMapManager::isFull(const SubMap::Ptr& sm) const {
    if ((int)sm->keyframes.size() >= max_kf_)         return true;
    if (sm->spatial_extent_m >= max_spatial_)         return true;
    if (!sm->keyframes.empty()) {
        double dt = sm->t_end - sm->t_start;
        if (dt >= max_temporal_)                      return true;
    }
    return false;
}

void SubMapManager::freezeActiveSubmap() {
    SubMap::Ptr sm;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        sm = active_submap_;
    }
    if (sm && sm->state == SubMapState::ACTIVE)
        freezeActiveSubmap(sm);
    else if (sm)
        SLOG_WARN(MOD, "No active submap to freeze (state={})", static_cast<int>(sm->state));
}

void SubMapManager::freezeActiveSubmap(const SubMap::Ptr& sm) {
    if (!sm || sm->state != SubMapState::ACTIVE) {
        SLOG_WARN(MOD, "freezeActiveSubmap(sm) invalid: state={}",
                   sm ? static_cast<int>(sm->state) : -1);
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][FREEZE_STEP] enter freeze sm_id=%d (async post-process)", sm->id);

    SLOG_START_SPAN(MOD, "freeze_submap");

    try {
        sm->state = SubMapState::FROZEN;
        publishEvent(sm, "FROZEN");
        METRICS_INCREMENT(metrics::SUBMAPS_FROZEN);
        SLOG_EVENT(MOD, "submap_frozen", "SubMap #{} state=FROZEN (post-process enqueue)", sm->id);

        bool enqueued = false;
        {
            std::unique_lock<std::mutex> lk(freeze_post_mutex_);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (freeze_post_queue_.size() >= kMaxFreezePostQueueSize && freeze_post_running_.load()) {
                if (freeze_post_cv_.wait_until(lk, deadline, [this] {
                    return freeze_post_queue_.size() < kMaxFreezePostQueueSize || !freeze_post_running_.load();
                }))
                    break;
                break;
            }
            if (freeze_post_queue_.size() < kMaxFreezePostQueueSize && freeze_post_running_.load()) {
                freeze_post_queue_.push(sm);
                enqueued = true;
                freeze_post_cv_.notify_one();
            }
        }
        if (enqueued) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[SubMapMgr][FREEZE_STEP] exit freeze sm_id=%d (enqueued)", sm->id);
            SLOG_END_SPAN();
            return;
        }

        // 队列满或超时：同步执行 voxel + 回调，避免阻塞/死锁
        RCLCPP_WARN(rclcpp::get_logger("automap_system"), "[SubMapMgr][FREEZE_STEP] queue full or timeout, sync fallback sm_id=%d", sm->id);
        if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            CloudXYZIPtr ds = utils::voxelDownsample(sm->merged_cloud, static_cast<float>(match_res_));
            if (!ds || ds->empty()) ds = sm->merged_cloud;
            sm->downsampled_cloud = ds;
            METRICS_HISTOGRAM_OBSERVE(metrics::POINTCLOUD_SIZE, static_cast<double>(sm->merged_cloud->size()));
        }
        for (auto& cb : frozen_cbs_) cb(sm);
        HEALTH_UPDATE_QUEUE("submap", getFrozenSubmaps().size());
        HEALTH_UPDATE_QUEUE("loop", 0);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[SubMapMgr][FREEZE_STEP] exit freeze sm_id=%d (sync)", sm->id);
    } catch (const std::exception& e) {
        auto error = ErrorDetail(errors::SUBMAP_STATE_INVALID, fmt::format("Failed to freeze submap #{}: {}", sm->id, e.what()));
        error.context().operation = "freezeActiveSubmap";
        error.context().file = __FILE__;
        error.context().line = __LINE__;
        error.context().function = __func__;
        SLOG_ERROR_CODE(MOD, static_cast<uint32_t>(error.code()), error.message());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapManager][EXCEPTION] freezeActiveSubmap: %s", error.message().c_str());
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        publishErrorEvent(sm->id, error);
    }
    SLOG_END_SPAN();
}

void SubMapManager::freezePostProcessLoop() {
    while (freeze_post_running_.load()) {
        SubMap::Ptr sm;
        {
            std::unique_lock<std::mutex> lk(freeze_post_mutex_);
            freeze_post_cv_.wait(lk, [this] {
                return !freeze_post_queue_.empty() || !freeze_post_running_.load();
            });
            if (!freeze_post_running_.load() && freeze_post_queue_.empty()) break;
            if (freeze_post_queue_.empty()) continue;
            sm = std::move(freeze_post_queue_.front());
            freeze_post_queue_.pop();
        }
        if (!sm) continue;
        try {
            if (sm->merged_cloud && !sm->merged_cloud->empty()) {
                CloudXYZIPtr ds = utils::voxelDownsample(sm->merged_cloud, static_cast<float>(match_res_));
                if (!ds || ds->empty()) ds = sm->merged_cloud;
                sm->downsampled_cloud = ds;
                METRICS_HISTOGRAM_OBSERVE(metrics::POINTCLOUD_SIZE, static_cast<double>(sm->merged_cloud->size()));
            }
            for (auto& cb : frozen_cbs_) cb(sm);
            HEALTH_UPDATE_QUEUE("submap", getFrozenSubmaps().size());
            HEALTH_UPDATE_QUEUE("loop", 0);
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"), "[SubMapMgr][FREEZE_POST] sm_id=%d downsampled_cloud done", sm->id);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapManager][FREEZE_POST] sm_id=%d exception: %s", sm ? sm->id : -1, e.what());
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapManager][FREEZE_POST] sm_id=%d unknown exception", sm ? sm->id : -1);
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        }
    }
}

SubMap::Ptr SubMapManager::createNewSubmap(const KeyFrame::Ptr& first_kf) {
    if (!first_kf) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapManager] createNewSubmap: first_kf is null");
        return nullptr;
    }
    auto sm = std::make_shared<SubMap>();
    sm->id          = submap_id_counter_++;
    sm->session_id  = current_session_id_;
    sm->state       = SubMapState::ACTIVE;
    sm->t_start     = first_kf->timestamp;
    sm->t_end       = first_kf->timestamp;
    sm->merged_cloud = std::make_shared<CloudXYZI>();
    publishEvent(sm, "CREATED");
    RCLCPP_DEBUG(node_->get_logger(), "[SubMapMgr][DATA] createNewSubmap sm_id=%d session=%lu", sm->id, sm->session_id);
    return sm;
}

namespace {
constexpr size_t kDownsampleThreshold = 200000;
}

void SubMapManager::mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const {
    if (!kf->cloud_body || kf->cloud_body->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu has null/empty cloud_body, skip merge", 
            kf->id);
        return;
    }

    // 合并前先检查是否需要降采样（避免累积；使用 utils::voxelDownsample 避免 PCL 整数溢出崩溃）
    if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
        size_t old_pts = sm->merged_cloud->size();
        CloudXYZIPtr temp = utils::voxelDownsample(sm->merged_cloud, static_cast<float>(merge_res_));
        if (temp && !temp->empty()) {
            size_t new_pts = temp->size();
            sm->merged_cloud.swap(temp);
            ALOG_DEBUG(MOD, "SM#{} pre-merge downsample: {} -> {} pts",
                       sm->id, old_pts, new_pts);
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[GLOBAL_MAP_DIAG] SM#%d pre-merge downsample: %zu → %zu pts", sm->id, old_pts, new_pts);
        }
    }

    // 将 body 系点云变换到世界系（注意：此处始终使用 T_w_b，未优化位姿；优化后 buildGlobalMap 主路径用 T_w_b_optimized 从 cloud_body 重算）
    CloudXYZIPtr world_cloud = getCloudFromPool();
    Eigen::Affine3f T_wf;
    T_wf.matrix() = kf->T_w_b.cast<float>().matrix();
    
    try {
        pcl::transformPointCloud(*kf->cloud_body, *world_cloud, T_wf);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu transform failed: %s", kf->id, e.what());
        return;
    }

    if (world_cloud->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu transform resulted in empty cloud", kf->id);
        return;
    }

    size_t world_cloud_size = world_cloud->size();

    // 合并点云
    if (!sm->merged_cloud || sm->merged_cloud->empty()) {
        sm->merged_cloud = std::make_shared<CloudXYZI>(*world_cloud);
    } else {
        size_t old_size = sm->merged_cloud->size();
        sm->merged_cloud->reserve(old_size + world_cloud->size());
        for (const auto& pt : world_cloud->points) {
            sm->merged_cloud->push_back(pt);
        }
    }

    // [GLOBAL_MAP_DIAG] 精准定位：合并时使用的位姿与点数（merged_cloud 始终用 T_w_b，若后续优化未重投影则与轨迹不一致）
    const Eigen::Vector3d t = kf->T_w_b.translation();
    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
        "[GLOBAL_MAP_DIAG] ✓ merge sm_id=%d kf_id=%lu: body_pts=%zu → world_pts=%zu, T_w_b=[%.2f,%.2f,%.2f], merged_total=%zu",
        sm->id, kf->id, kf->cloud_body->size(), world_cloud_size, t.x(), t.y(), t.z(), sm->merged_cloud->size());

    // 合并后超过阈值则降采样（使用 utils::voxelDownsample 避免 PCL 整数溢出崩溃）
    if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
        size_t before_downsample = sm->merged_cloud->size();
        CloudXYZIPtr temp = utils::voxelDownsample(sm->merged_cloud, static_cast<float>(merge_res_));
        if (temp && !temp->empty()) {
            size_t after_downsample = temp->size();
            // ✅ 修复：检查降采样后点数，防止降采样失败或点数仍然过大
            if (after_downsample > 10000000) {  // 超过1000万点则强制清空
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[GLOBAL_MAP_DIAG] SM#%d post-merge downsample result too large (%zu pts), clearing to avoid memory issue",
                    sm->id, after_downsample);
                sm->merged_cloud->clear();
                sm->merged_cloud->points.shrink_to_fit();
            } else {
                sm->merged_cloud.swap(temp);
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[GLOBAL_MAP_DIAG] SM#%d post-merge downsample: %zu → %zu pts (threshold=%.0f)",
                    sm->id, before_downsample, after_downsample, static_cast<float>(kDownsampleThreshold));
            }
        } else {
            // ✅ 修复：降采样失败则清空，避免累积过多点
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[GLOBAL_MAP_DIAG] SM#%d post-merge downsample failed (returned empty), clearing merged cloud",
                sm->id);
            sm->merged_cloud->clear();
            sm->merged_cloud->points.shrink_to_fit();
        }
    }
}

void SubMapManager::updateSubmapPose(int submap_id, const Pose3d& new_pose) {
    // 结构化日志：开始Span
    SLOG_START_SPAN(MOD, "update_submap_pose");
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    bool updated = false;
    double max_translation_diff = 0.0;
    double max_rotation_diff = 0.0;

    for (auto& sm : submaps_) {
        if (sm->id != submap_id) continue;

        // 验证状态转换合法性
        if (sm->state != SubMapState::FROZEN &&
            sm->state != SubMapState::OPTIMIZED) {
            auto error = ErrorDetail(
                errors::SUBMAP_STATE_INVALID,
                "Invalid state transition: can only update pose in FROZEN or OPTIMIZED state"
            );
            error.context().operation = "updateSubmapPose";
            error.context().metadata["from_state"] = std::to_string(static_cast<int>(sm->state));
            error.context().metadata["to_state"] = "OPTIMIZED";
            
            SLOG_WARN(MOD, "Invalid pose update for SM#{}: state={}",
                        sm->id, static_cast<int>(sm->state));
            
            METRICS_INCREMENT(metrics::WARNINGS_TOTAL);
            continue;
        }

        Pose3d old_anchor = sm->pose_w_anchor_optimized;
        sm->pose_w_anchor_optimized = new_pose;
        sm->state = SubMapState::OPTIMIZED;

        // 计算位姿增量
        double translation_diff = (new_pose.translation() - old_anchor.translation()).norm();
        double rotation_diff = Eigen::AngleAxisd(
            new_pose.rotation().inverse() * old_anchor.rotation()).angle();

        max_translation_diff = std::max(max_translation_diff, translation_diff);
        max_rotation_diff = std::max(max_rotation_diff, rotation_diff);

        // 记录最大差异的指标
        METRICS_GAUGE_SET(metrics::LOOP_RMSE_METERS, max_translation_diff);

        SLOG_DEBUG(MOD, "SM#{} pose updated: trans={:.3f}m rot={:.1f}°",
                     sm->id, translation_diff,
                     rotation_diff * 180.0 / M_PI);

        // 更新所有关键帧的优化位姿
        Pose3d delta = new_pose * old_anchor.inverse();
        for (auto& kf : sm->keyframes) {
            kf->T_w_b_optimized = delta * kf->T_w_b;
        }

        publishEvent(sm, "OPTIMIZED");
        SLOG_EVENT(MOD, "submap_optimized", 
                   "SubMap #{} optimized (dt_trans={:.3f}m, dt_rot={:.1f}°)",
                   sm->id, max_translation_diff, max_rotation_diff * 180.0 / M_PI);

        updated = true;
        break;
    }

    if (updated) {
        METRICS_INCREMENT(metrics::OPTIMIZATIONS_RUN);
    }

    // 结构化日志：结束Span
    SLOG_END_SPAN();
}

void SubMapManager::updateAllFromHBA(const HBAResult& result) {
    if (!result.success || result.optimized_poses.empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    // HBA 已按时间序写回各关键帧 T_w_b_optimized，此处仅同步子图锚定位姿
    for (auto& sm : submaps_) {
        if (!sm || sm->keyframes.empty()) continue;
        KeyFrame::Ptr anchor = sm->keyframes.front();
        if (!anchor) continue;
        sm->pose_w_anchor_optimized = anchor->T_w_b_optimized;
        if (sm->state == SubMapState::FROZEN || sm->state == SubMapState::OPTIMIZED)
            sm->state = SubMapState::OPTIMIZED;
    }
}

// ── 查询接口实现（头文件声明，此前未实现会导致 undefined symbol）────────────────────
SubMap::Ptr SubMapManager::getActiveSubmap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return active_submap_;
}

SubMap::Ptr SubMapManager::getSubmap(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& sm : submaps_) {
        if (sm->id == id) return sm;
    }
    return nullptr;
}

std::vector<SubMap::Ptr> SubMapManager::getAllSubmaps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return submaps_;
}

std::vector<SubMap::Ptr> SubMapManager::getFrozenSubmaps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<SubMap::Ptr> out;
    for (const auto& sm : submaps_) {
        if (sm->state == SubMapState::FROZEN || sm->state == SubMapState::OPTIMIZED)
            out.push_back(sm);
    }
    return out;
}

int SubMapManager::submapCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(submaps_.size());
}

int SubMapManager::keyframeCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    int n = 0;
    for (const auto& sm : submaps_) {
        n += static_cast<int>(sm->keyframes.size());
    }
    return n;
}

// 单子图点云点数上限，避免单子图过大导致 PCL/内存异常
static constexpr size_t kMaxPointsPerSubmap = 5000000u;
// 合并时总点数上限：提高以尽量包含所有子图，最终由体素下采样控制发布量；超大场景可增大 map_voxel_size
static constexpr size_t kMaxCombinedPoints  = 100000000u;

namespace {
// 计算点云包围盒（用于诊断日志），采样最多 50000 点避免大云过慢
void cloudBbox(const CloudXYZI& cloud, float& minx, float& miny, float& minz, float& maxx, float& maxy, float& maxz, size_t& out_count) {
    minx = miny = minz = 1e9f; maxx = maxy = maxz = -1e9f;
    out_count = 0;
    const size_t step = std::max<size_t>(1u, cloud.size() / 50000u);
    for (size_t i = 0; i < cloud.size(); i += step) {
        const auto& p = cloud.points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        minz = std::min(minz, p.z); maxz = std::max(maxz, p.z);
        ++out_count;
    }
}
}  // namespace

CloudXYZIPtr SubMapManager::buildGlobalMap(float voxel_size) const {
    const unsigned tid = automap_pro::logThreadId();
    // 使用全局 logger 名称，避免 backend 线程中解引用 node_（可能析构顺序导致 use-after-free）
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMap_enter voxel_size={:.3f}", tid, voxel_size);
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] buildGlobalMap enter voxel_size=%.3f (grep GLOBAL_MAP_DIAG 可精准定位各环节)", voxel_size);
    std::lock_guard<std::mutex> lk(mutex_);
    const size_t num_submaps = submaps_.size();
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMap_locked submaps={}", tid, num_submaps);
    RCLCPP_INFO(log, "[AutoMapSystem][MAP] buildGlobalMap step=locked_done submaps=%zu", num_submaps);
    CloudXYZIPtr combined = std::make_shared<CloudXYZI>();
    // 使用优化后位姿从关键帧重算全局图，避免“位姿已优化、点云仍为旧世界系”导致的杂乱（见 docs/GLOBAL_MAP_MESSY_ANALYSIS.md）
    CloudXYZIPtr world_tmp = std::make_shared<CloudXYZI>();
    bool hit_limit = false;
    bool used_fallback_path = false;  // 用于 [GLOBAL_MAP_BLUR] 精准定位
    size_t kf_used_total = 0;
    size_t kf_skipped_null = 0;
    size_t kf_skipped_empty = 0;
    size_t kf_fallback_unopt = 0;
    int subs_with_kf = 0;
    size_t total_pts_before_transform = 0;
    
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] ┌─ 主路径: 从关键帧重算（使用 T_w_b_optimized）");
    
    for (size_t idx = 0; idx < num_submaps && !hit_limit; ++idx) {
        const auto& sm = submaps_[idx];
        if (!sm) {
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] submap[%zu] is null, skip", idx);
            continue;
        }
        
        size_t sm_pts = 0;
        size_t sm_kf_count = 0;
        size_t sm_kf_valid = 0;
        
        RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: %zu keyframes, pose_w_anchor_optimized=[%.2f,%.2f,%.2f]",
            sm->id, sm->keyframes.size(),
            sm->pose_w_anchor_optimized.translation().x(),
            sm->pose_w_anchor_optimized.translation().y(),
            sm->pose_w_anchor_optimized.translation().z());
        
        for (const auto& kf : sm->keyframes) {
            sm_kf_count++;
            
            if (!kf) {
                kf_skipped_null++;
                RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ │ kf[%zu] is null", sm_kf_count - 1);
                continue;
            }
            
            if (!kf->cloud_body) {
                kf_skipped_empty++;
                RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ │ kf_id=%lu cloud_body is null", kf->id);
                continue;
            }
            
            if (kf->cloud_body->empty()) {
                kf_skipped_empty++;
                RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ │ kf_id=%lu cloud_body is empty", kf->id);
                continue;
            }
            
            sm_kf_valid++;
            // 【关键】选择位姿：优先用优化位姿，若未初始化则用原始位姿
            Pose3d T_w_b = kf->T_w_b_optimized;
            bool using_optimized = true;
            
            // 检测是否为 Identity 且原始位姿非 Identity
            if (T_w_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) && 
                !kf->T_w_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6)) {
                T_w_b = kf->T_w_b;
                using_optimized = false;
                kf_fallback_unopt++;
                
                RCLCPP_WARN(log,
                    "[GLOBAL_MAP_DIAG] │ │ kf_id=%lu sm_id=%d T_w_b_optimized=Identity → using T_w_b (unoptimized)", 
                    kf->id, sm->id);
            }
            
            // 变换点云到世界系
            Eigen::Affine3f T_wf;
            T_wf.matrix() = T_w_b.cast<float>().matrix();
            world_tmp->clear();
            
            try {
                pcl::transformPointCloud(*kf->cloud_body, *world_tmp, T_wf);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(log, "[GLOBAL_MAP_DIAG] │ │ kf_id=%lu transform failed: %s", kf->id, e.what());
                continue;
            }
            
            if (world_tmp->empty()) {
                RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ │ kf_id=%lu transform resulted in empty cloud", kf->id);
                continue;
            }
            
            size_t add_size = world_tmp->size();
            total_pts_before_transform += kf->cloud_body->size();
            
            // 检查是否会超过点数上限
            if (combined->size() + add_size > kMaxCombinedPoints) {
                ALOG_WARN(MOD, "buildGlobalMap: combined would exceed {} pts, stop adding (current={}, trying to add={})", 
                    kMaxCombinedPoints, combined->size(), add_size);
                RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] │ │ ⚠️  LIMIT: combined %zu + %zu > %zu, stop here",
                    combined->size(), add_size, kMaxCombinedPoints);
                hit_limit = true;
                break;
            }
            
            // 合并到全局点云
            combined->reserve(combined->size() + add_size);
            for (const auto& pt : world_tmp->points) {
                combined->push_back(pt);
            }
            
            sm_pts += add_size;
            kf_used_total++;
            
            // DEBUG：每个关键帧的贡献
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ │ ✓ kf_id=%lu body_pts=%zu → world_pts=%zu [opt=%d] t=[%.2f,%.2f,%.2f]",
                kf->id, kf->cloud_body->size(), add_size, using_optimized ? 1 : 0,
                T_w_b.translation().x(), T_w_b.translation().y(), T_w_b.translation().z());
        }
        
        if (sm_kf_valid > 0) {
            subs_with_kf++;
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: %zu/%zu keyframes used, contributed %zu pts",
                sm->id, sm_kf_valid, sm_kf_count, sm_pts);
        }
    }
    
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] └─ 主路径完成：");
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] path=from_kf submaps_with_kf=%d kf_used=%zu combined_pts=%zu",
        subs_with_kf, kf_used_total, combined->size());
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG]   • 统计: kf_skipped_null=%zu, kf_skipped_empty=%zu, kf_fallback_unopt=%zu",
        kf_skipped_null, kf_skipped_empty, kf_fallback_unopt);
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG]   • 输入点数 (body系): %zu, 输出点数 (world系): %zu",
        total_pts_before_transform, combined->size());
    // 若主路径无关键帧点云，记录警告并尝试回退
    if (combined->empty()) {
        const auto& cfg = ConfigManager::instance();
        
        if (cfg.retainCloudBody()) {
            SLOG_ERROR(MOD, 
                "🔴 P1 FALLBACK DETECTED: No keyframe clouds found despite retain_cloud_body=true!\n"
                "   Statistics: kf_skipped_null=%zu, kf_skipped_empty=%zu, num_submaps=%zu\n"
                "   This may indicate:\n"
                "   1. All keyframes have been archived/deleted (unexpected)\n"
                "   2. Memory pressure triggered cloud_body cleanup anyway\n"
                "   3. All keyframe clouds are geometrically empty\n"
                "   Attempting fallback to merged_cloud (which uses OLD world coordinate system)\n"
                "   ⚠️  Result: global_map may NOT align with optimized trajectory",
                kf_skipped_null, kf_skipped_empty, num_submaps);
            METRICS_INCREMENT(metrics::FALLBACK_TO_MERGED_CLOUD);
        } else {
            SLOG_WARN(MOD,
                "⚠️  P1 EXPECTED FALLBACK: retain_cloud_body=false → No keyframe clouds available\n"
                "   kf_skipped: null=%zu, empty=%zu\n"
                "   Using merged_cloud (built with T_w_b, not T_w_b_optimized)\n"
                "   After optimization, this may cause misalignment with trajectory\n"
                "   Recommendation: Set retain_cloud_body=true if precision is critical",
                kf_skipped_null, kf_skipped_empty);
            METRICS_INCREMENT(metrics::FALLBACK_TO_MERGED_CLOUD);
        }

        used_fallback_path = true;  // 供 [GLOBAL_MAP_BLUR] 汇总判断
        RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] ┌─ 回退路径: 拼接 merged_cloud (旧世界系，可能不准确)");
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] path=fallback_merged_cloud (⚠️  if shown, global_map may be misaligned with optimized trajectory)");
        
        // 回退：拼接各子图的 merged_cloud
        for (const auto& sm : submaps_) {
            if (!sm) continue;
            if (!sm->merged_cloud || sm->merged_cloud->empty()) {
                RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: merged_cloud is null/empty, skip", sm->id);
                continue;
            }
            
            size_t add_size = sm->merged_cloud->size();
            
            if (add_size > kMaxPointsPerSubmap) {
                RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] │ SM#%d: merged_pts=%zu > max=%zu, skip",
                    sm->id, add_size, kMaxPointsPerSubmap);
                continue;
            }
            
            if (combined->size() + add_size > kMaxCombinedPoints) {
                RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] │ SM#%d: would exceed limit, stop fallback", sm->id);
                break;
            }
            
            combined->reserve(combined->size() + add_size);
            for (const auto& pt : sm->merged_cloud->points) {
                combined->push_back(pt);
            }
            
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: added %zu pts from merged_cloud (built with T_w_b, not optimized)",
                sm->id, add_size);
        }
        
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] └─ 回退完成: combined_pts=%zu", combined->size());
    }
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMap_merge_done combined={}", tid, combined->size());
    
    // 计算并记录包围盒
    if (!combined->empty()) {
        float minx, miny, minz, maxx, maxy, maxz;
        size_t bbox_pts;
        cloudBbox(*combined, minx, miny, minz, maxx, maxy, maxz, bbox_pts);
        
        double bbox_volume = (maxx - minx) * (maxy - miny) * (maxz - minz);
        double bbox_diagonal = std::sqrt(
            (maxx-minx)*(maxx-minx) + (maxy-miny)*(maxy-miny) + (maxz-minz)*(maxz-minz));
        
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] combined_pts=%zu bbox=[%.2f,%.2f,%.2f]→[%.2f,%.2f,%.2f]",
            combined->size(), minx, miny, minz, maxx, maxy, maxz);
        RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG]   bbox_volume=%.2f m³, bbox_diagonal=%.2f m, bbox_sampled=%zu pts",
            bbox_volume, bbox_diagonal, bbox_pts);
    } else {
        RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] combined cloud is empty! No points to return");
        return combined;
    }
    
    // 体素下采样
    if (combined->empty()) {
        RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] combined is empty after all steps, returning empty");
        return combined;
    }
    
    try {
        float vs = std::max(voxel_size, utils::kMinVoxelLeafSize);
        if (voxel_size <= 0.0f) {
            SLOG_WARN(MOD, "buildGlobalMap: voxel_size={} <= 0, returning combined without downsample", voxel_size);
            RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] voxel_size <= 0, skip downsampling");
            return utils::sanitizePointCloudForVoxel(combined, 1e6f);
        }
        
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] downsampling: voxel_size=%.3f, input_pts=%zu", vs, combined->size());
        CloudXYZIPtr out = utils::voxelDownsampleChunked(combined, vs, 50.0f);
        ALOG_INFO(MOD, "[tid={}] step=buildGlobalMap_exit out={}", tid, out ? out->size() : 0u);
        
        if (out && !out->empty()) {
            float minx, miny, minz, maxx, maxy, maxz;
            size_t bbox_pts;
            cloudBbox(*out, minx, miny, minz, maxx, maxy, maxz, bbox_pts);
            
            RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] after_downsample out_pts=%zu bbox=[%.2f,%.2f,%.2f]→[%.2f,%.2f,%.2f]",
                out->size(), minx, miny, minz, maxx, maxy, maxz);
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG]   compression_ratio=%.1f%% (combined_pts %zu → out_pts %zu)",
                100.0 * out->size() / std::max(size_t(1), combined->size()), combined->size(), out->size());
            
            RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] ════════════════════════════════════════════════════════");
            RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] buildGlobalMap SUCCESS: %zu points → %zu after downsample", 
                combined->size(), out->size());
            RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] ════════════════════════════════════════════════════════");
            // 精准定位模糊问题：单行汇总，grep GLOBAL_MAP_BLUR 即可
            {
                const double comp_pct = (combined->empty()) ? 0.0 : (100.0 * static_cast<double>(out->size()) / static_cast<double>(combined->size()));
                const bool blur_risk = used_fallback_path || (kf_fallback_unopt > 0u) || (comp_pct < 5.0) || (vs > 0.3f);
                RCLCPP_INFO(log, "[GLOBAL_MAP_BLUR] path=%s kf_unopt=%zu voxel=%.3f combined=%zu out=%zu comp_pct=%.1f%% blur_risk=%s",
                    used_fallback_path ? "fallback" : "from_kf", kf_fallback_unopt, vs, combined->size(), out->size(), comp_pct, blur_risk ? "yes" : "no");
                if (blur_risk) {
                    RCLCPP_WARN(log,
                        "[GLOBAL_MAP_BLUR] 存在模糊风险: %s%s%s%s → 见 docs/GLOBAL_MAP_BLUR_ANALYSIS.md",
                        used_fallback_path ? "path=fallback " : "",
                        (kf_fallback_unopt > 0u) ? "未优化位姿 " : "",
                        (comp_pct < 5.0) ? "下采样过狠 " : "",
                        (vs > 0.3f) ? "体素过大 " : "");
                }
            }
            return out;
        } else if (out && out->empty()) {
            SLOG_WARN(MOD, "buildGlobalMap: voxelDownsampleChunked returned empty, returning sanitized combined");
            RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] downsampling resulted in empty, returning sanitized combined");
            return utils::sanitizePointCloudForVoxel(combined, 1e6f);
        } else {
            SLOG_WARN(MOD, "buildGlobalMap: voxelDownsampleChunked returned null");
            RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] downsampling returned null");
            return utils::sanitizePointCloudForVoxel(combined, 1e6f);
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "buildGlobalMap exception during downsampling: {}", e.what());
        RCLCPP_ERROR(log, "[GLOBAL_MAP_DIAG] ❌ buildGlobalMap exception during downsampling: %s", e.what());
        return utils::sanitizePointCloudForVoxel(combined, 1e6f);
    } catch (...) {
        ALOG_ERROR(MOD, "buildGlobalMap unknown exception");
        RCLCPP_ERROR(log, "[GLOBAL_MAP_DIAG] ❌ buildGlobalMap unknown exception");
        return utils::sanitizePointCloudForVoxel(combined, 1e6f);
    }
}

// ─────────────────────────────────────────────────────────────────────
// 持久化：archiveSubmap / loadArchivedSubmap（修复 undefined symbol）
// ─────────────────────────────────────────────────────────────────────

bool SubMapManager::archiveSubmap(const SubMap::Ptr& submap, const std::string& dir) {
    if (!submap) return false;
    std::string subdir = dir + "/submap_" + std::to_string(submap->id);
    try {
        fs::create_directories(subdir);

        json meta;
        meta["id"] = submap->id;
        meta["session_id"] = submap->session_id;
        meta["t_start"] = submap->t_start;
        meta["t_end"] = submap->t_end;
        meta["spatial_extent_m"] = submap->spatial_extent_m;
        meta["has_descriptor"] = submap->has_descriptor;
        meta["has_valid_gps"] = submap->has_valid_gps;

        const Pose3d& T = submap->pose_w_anchor_optimized;
        Eigen::Quaterniond q(T.rotation());
        meta["anchor_pose"] = {
            {"px", T.translation().x()}, {"py", T.translation().y()}, {"pz", T.translation().z()},
            {"qx", q.x()}, {"qy", q.y()}, {"qz", q.z()}, {"qw", q.w()}
        };
        if (submap->has_valid_gps) {
            meta["gps_center"] = {submap->gps_center.x(), submap->gps_center.y(), submap->gps_center.z()};
        }
        if (submap->has_descriptor && submap->overlap_descriptor.size() > 0) {
            meta["descriptor"] = std::vector<float>(submap->overlap_descriptor.data(),
                submap->overlap_descriptor.data() + submap->overlap_descriptor.size());
        }

        std::ofstream ofs(subdir + "/submap_meta.json");
        ofs << meta.dump(2);

        if (submap->downsampled_cloud && !submap->downsampled_cloud->empty()) {
            pcl::io::savePCDFileBinary(subdir + "/downsampled_cloud.pcd", *submap->downsampled_cloud);
        }
        submap->state = SubMapState::ARCHIVED;
        return true;
    } catch (const std::exception& e) {
        SLOG_ERROR(MOD, "archiveSubmap sm_id={} failed: {}", submap->id, e.what());
        return false;
    }
}

bool SubMapManager::loadArchivedSubmap(const std::string& dir, int submap_id, SubMap::Ptr& out) {
    std::string subdir = dir + "/submap_" + std::to_string(submap_id);
    std::string meta_path = subdir + "/submap_meta.json";
    if (!fs::exists(meta_path)) return false;
    try {
        std::ifstream ifs(meta_path);
        json meta;
        ifs >> meta;

        auto sm = std::make_shared<SubMap>();
        sm->id = meta.value("id", -1);
        sm->session_id = meta.value("session_id", 0ULL);
        sm->t_start = meta.value("t_start", 0.0);
        sm->t_end = meta.value("t_end", 0.0);
        sm->spatial_extent_m = meta.value("spatial_extent_m", 0.0);
        sm->has_descriptor = meta.value("has_descriptor", false);
        sm->has_valid_gps = meta.value("has_valid_gps", false);
        sm->state = SubMapState::ARCHIVED;

        if (meta.contains("anchor_pose")) {
            const auto& ap = meta["anchor_pose"];
            Eigen::Quaterniond q(ap.value("qw", 1.0), ap.value("qx", 0.0), ap.value("qy", 0.0), ap.value("qz", 0.0));
            sm->pose_w_anchor_optimized = sm->pose_w_anchor = Pose3d::Identity();
            sm->pose_w_anchor_optimized.linear() = q.toRotationMatrix();
            sm->pose_w_anchor_optimized.translation() << ap.value("px", 0.0), ap.value("py", 0.0), ap.value("pz", 0.0);
            sm->pose_w_anchor = sm->pose_w_anchor_optimized;
        }
        if (meta.contains("gps_center") && meta["gps_center"].is_array() && meta["gps_center"].size() >= 3) {
            sm->gps_center << meta["gps_center"][0], meta["gps_center"][1], meta["gps_center"][2];
        }
        if (meta.contains("descriptor") && meta["descriptor"].is_array()) {
            std::vector<float> d = meta["descriptor"].get<std::vector<float>>();
            if (d.size() == 256) {
                sm->overlap_descriptor = Eigen::VectorXf::Map(d.data(), 256);
                sm->overlap_descriptor_norm = sm->overlap_descriptor.norm();
            }
        }

        std::string pcd_path = subdir + "/downsampled_cloud.pcd";
        if (fs::exists(pcd_path)) {
            sm->downsampled_cloud = std::make_shared<CloudXYZI>();
            if (pcl::io::loadPCDFile(pcd_path, *sm->downsampled_cloud) == 0) {
                /* loaded */
            } else {
                sm->downsampled_cloud->clear();
            }
        }
        if (!sm->downsampled_cloud) sm->downsampled_cloud = std::make_shared<CloudXYZI>();

        out = sm;
        return true;
    } catch (const std::exception& e) {
        SLOG_ERROR(MOD, "loadArchivedSubmap sm_id={} failed: {}", submap_id, e.what());
        return false;
    }
}

// ─────────────────────────────────────────────────────────────────────
// 工程化辅助函数实现
// ─────────────────────────────────────────────────────────────────────

void SubMapManager::updateGPSGravityCenter(const KeyFrame::Ptr& kf) {
    if (!kf->has_valid_gps || !active_submap_) {
        return;
    }

    size_t gps_count = 0;
    Eigen::Vector3d gps_sum = Eigen::Vector3d::Zero();

    SLOG_DEBUG(MOD, "Calculating GPS center for SM#{} ({} frames)", 
                 active_submap_->id, active_submap_->keyframes.size());

    for (const auto& f : active_submap_->keyframes) {
        if (f->has_valid_gps) {
            gps_sum += f->gps.position_enu;
            gps_count++;
        }
    }

    if (gps_count > 0) {
        active_submap_->gps_center = gps_sum / gps_count;
        active_submap_->has_valid_gps = true;
        
        SLOG_DEBUG(MOD, "Updated GPS center for SM#{} ({} GPS fixes): ({:.3f}, {:.3f}, {:.3f})",
                     active_submap_->id, gps_count,
                     active_submap_->gps_center.x(),
                     active_submap_->gps_center.y(),
                     active_submap_->gps_center.z());
    }
}

void SubMapManager::publishEvent(const SubMap::Ptr& sm, const std::string& event) {
    if (!event_pub_ || !sm) return;

    auto msg = std::make_shared<automap_pro::msg::SubMapEventMsg>();
    msg->header.stamp = node_->now();
    msg->submap_id = sm->id;
    msg->session_id = sm->session_id;
    msg->event_type = event;
    msg->keyframe_count = static_cast<int>(sm->keyframes.size());
    msg->spatial_extent_m = sm->spatial_extent_m;
    msg->has_valid_gps = sm->has_valid_gps;

    const Pose3d& T = sm->pose_w_anchor_optimized;
    Eigen::Quaterniond q(T.rotation());
    msg->anchor_pose.position.x = T.translation().x();
    msg->anchor_pose.position.y = T.translation().y();
    msg->anchor_pose.position.z = T.translation().z();
    msg->anchor_pose.orientation.x = q.x();
    msg->anchor_pose.orientation.y = q.y();
    msg->anchor_pose.orientation.z = q.z();
    msg->anchor_pose.orientation.w = q.w();

    event_pub_->publish(*msg);
}

void SubMapManager::publishErrorEvent(int submap_id, const ErrorDetail& error) {
    if (!event_pub_) return;

    auto msg = std::make_shared<automap_pro::msg::SubMapEventMsg>();
    msg->header.stamp = node_->now();
    msg->submap_id = submap_id;
    msg->session_id = current_session_id_;
    msg->event_type = fmt::format("error_0x{:08X}", static_cast<uint32_t>(error.code()));
    msg->keyframe_count = 0;
    msg->spatial_extent_m = 0.0;
    msg->has_valid_gps = false;
    
    event_pub_->publish(*msg);
    
    SLOG_EVENT(MOD, "submap_error", "sm_id={}, code=0x{:08X}, msg={}",
               submap_id, static_cast<uint32_t>(error.code()),
               error.message());
}

// ── 异步构建实现（P0 优化）──────────────────────────────────────────

std::future<CloudXYZIPtr> SubMapManager::buildGlobalMapAsync(float voxel_size) const {
    return std::async(std::launch::async, [this, voxel_size]() {
        std::vector<SubMap::Ptr> submaps_copy;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            submaps_copy = submaps_;
        }
        return buildGlobalMapInternal(submaps_copy, voxel_size);
    });
}

CloudXYZIPtr SubMapManager::buildGlobalMapInternal(
    const std::vector<SubMap::Ptr>& submaps_copy, float voxel_size) const {
    const unsigned tid = automap_pro::logThreadId();
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_enter voxel_size={:.3f}", tid, voxel_size);

    CloudXYZIPtr combined = std::make_shared<CloudXYZI>();
    CloudXYZIPtr world_tmp = std::make_shared<CloudXYZI>();

    for (const auto& sm : submaps_copy) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
            Pose3d T_w_b = kf->T_w_b_optimized;
            if (T_w_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) &&
                !kf->T_w_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6)) {
                T_w_b = kf->T_w_b;
            }
            Eigen::Affine3f T_wf;
            T_wf.matrix() = T_w_b.cast<float>().matrix();
            world_tmp->clear();
            try {
                pcl::transformPointCloud(*kf->cloud_body, *world_tmp, T_wf);
            } catch (...) { continue; }
            if (world_tmp->empty()) continue;
            if (combined->size() + world_tmp->size() > kMaxCombinedPoints) break;
            combined->reserve(combined->size() + world_tmp->size());
            for (const auto& pt : world_tmp->points)
                combined->push_back(pt);
        }
    }

    if (combined->empty()) return combined;
    float vs = std::max(voxel_size, utils::kMinVoxelLeafSize);
    if (voxel_size <= 0.0f) return combined;

    CloudXYZIPtr out = utils::voxelDownsampleChunked(combined, vs, 50.0f);
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_exit out={}", tid, out ? out->size() : 0u);
    return out ? out : combined;
}

}  // namespace automap_pro
