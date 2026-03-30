/**
 * @file submap/submap_manager.cpp
 * @brief 子图与会话实现。
 */
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/v3/pose_chain.hpp"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/landmark_id.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/error_code.h"
#include "automap_pro/core/error_monitor.h"
#include "automap_pro/core/utils.h"
#define MOD "SubMapMgr"

#include <algorithm>
#include <cassert>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <automap_pro/msg/sub_map_event_msg.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <limits>

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
    submap_merge_semantic_intensity_vote_ = cfg.submapMergeSemanticIntensityVote();
    merge_thread_count_ = cfg.backendSubmapMergeThreads();
    retain_cloud_body_ = cfg.retainCloudBody();
    map_statistical_filter_ = cfg.mapStatisticalFilter();
    map_stat_filter_mean_k_ = cfg.mapStatFilterMeanK();
    map_stat_filter_std_mul_ = cfg.mapStatFilterStdMul();
    submap_rebuild_thresh_trans_ = cfg.submapRebuildThreshTrans();
    submap_rebuild_thresh_rot_ = cfg.submapRebuildThreshRot();
    parallel_voxel_downsample_ = cfg.parallelVoxelDownsample();
    backend_verbose_trace_ = cfg.backendVerboseTrace();
    assoc_cyl_max_dist_xy_m_ = cfg.semanticAssocCylinderMaxDistXyM();
    assoc_cyl_max_dist_z_m_ = cfg.semanticAssocCylinderMaxDistZM();
    assoc_cyl_max_angle_deg_ = cfg.semanticAssocCylinderMaxAngleDeg();
    assoc_cyl_max_radius_diff_m_ = cfg.semanticAssocCylinderMaxRadiusDiffM();
    assoc_cyl_alpha_min_ = cfg.semanticAssocCylinderAlphaMin();
    assoc_cyl_alpha_max_ = cfg.semanticAssocCylinderAlphaMax();
    assoc_cyl_mahalanobis_gate_ = cfg.semanticAssocCylinderMahalanobisGate();
    assoc_cyl_min_confirmations_ = cfg.semanticAssocCylinderMinConfirmations();
    assoc_cyl_min_observability_ = cfg.semanticAssocCylinderMinObservability();
    assoc_cyl_duplicate_merge_dist_xy_m_ = cfg.semanticAssocDuplicateMergeDistXyM();
    assoc_cyl_duplicate_merge_max_angle_deg_ = cfg.semanticAssocDuplicateMergeMaxAngleDeg();
    assoc_cyl_protection_mode_enabled_ = cfg.semanticAssocProtectionModeEnabled();
    assoc_cyl_protect_trigger_tree_new_rate_pct_ = cfg.semanticAssocProtectionTriggerTreeNewRatePct();
    assoc_cyl_protect_recover_tree_new_rate_pct_ = cfg.semanticAssocProtectionRecoverTreeNewRatePct();
    assoc_cyl_protect_trigger_duplicate_density_ = cfg.semanticAssocProtectionTriggerDuplicateDensity();
    assoc_cyl_protect_recover_duplicate_density_ = cfg.semanticAssocProtectionRecoverDuplicateDensity();
    assoc_plane_max_angle_deg_ = cfg.semanticAssocPlaneMaxAngleDeg();
    assoc_plane_max_distance_diff_m_ = cfg.semanticAssocPlaneMaxDistanceDiffM();
    assoc_plane_max_tangent_offset_m_ = cfg.semanticAssocPlaneMaxTangentOffsetM();
    assoc_plane_alpha_min_ = cfg.semanticAssocPlaneAlphaMin();
    assoc_plane_alpha_max_ = cfg.semanticAssocPlaneAlphaMax();
    
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
        SLOG_INFO(MOD, "✅ retain_cloud_body=true: Main path (buildGlobalMap via T_map_b_optimized) will be used");
    }
}

SubMapManager::~SubMapManager() {
    stop();
}

void SubMapManager::stop() {
    if (merge_running_.load()) {
        merge_running_.store(false);
        merge_cv_.notify_all();
        for (auto& t : merge_threads_) {
            if (t.joinable()) t.join();
        }
        merge_threads_.clear();
    }
    if (freeze_post_running_.load()) {
        freeze_post_running_.store(false);
        freeze_post_cv_.notify_all();
        if (freeze_post_thread_.joinable())
            freeze_post_thread_.join();
    }
}

void SubMapManager::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    event_pub_ = node->create_publisher<automap_pro::msg::SubMapEventMsg>(
        "/automap/submap_event", 50);
    RCLCPP_INFO(node->get_logger(), "[SubMapMgr][TOPIC] publish: /automap/submap_event");
    
    // 启动可配置的合并线程（默认 8，建议 8~10）
    for (int i = 0; i < merge_thread_count_; ++i) {
        merge_threads_.emplace_back(&SubMapManager::mergeWorkerLoop, this);
    }
    RCLCPP_INFO(node->get_logger(), "[SubMapMgr] merge worker threads=%d", merge_thread_count_);
    
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

    // 增强关键帧有效性检查
    if (!kf->cloud_body || kf->cloud_body->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SubMapManager] addKeyFrame: empty cloud, rejected (kf_id=%lu)", kf->id);
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        return;
    }

    // 检查位姿有效性
    const auto& t = kf->T_odom_b.translation();
    const auto& R = kf->T_odom_b.rotation();
    if (!t.allFinite() || !R.allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[SubMapManager] addKeyFrame: invalid pose (non-finite), rejected (kf_id=%lu)", kf->id);
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        return;
    }

    std::unique_lock<std::mutex> lk(mutex_);

    METRIC_TIMED_SCOPE(metrics::POINTCLOUD_PROCESS_TIME_MS);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][STEP] step=addKeyFrame_enter kf_id=%lu file=%s line=%d",
        kf->id, __FILE__, __LINE__);

    try {
        // 🏛️ [架构加固] 坐标系一致性检查：如果当前帧坐标系与活跃子图不一致（如 GPS 对齐瞬间产生的在途帧），
        // 必须强制切分子图，确保单个子图内部坐标系语义严格统一，这是消除全局重影的物理屏障。
        if (active_submap_ && !active_submap_->keyframes.empty() && kf->pose_frame != active_submap_->pose_frame) {
            const int sm_id = active_submap_->id;
            const size_t kf_count = active_submap_->keyframes.size();
            SubMap::Ptr to_freeze = active_submap_;
            active_submap_ = nullptr;

            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][GHOST_GUARD] PoseFrame transition (sm_%d=%d, kf_%lu=%d), forcing split",
                sm_id, static_cast<int>(to_freeze->pose_frame), kf->id, static_cast<int>(kf->pose_frame));
            
            lk.unlock();
            freezeSubmap(to_freeze);
            lk.lock();
            // 锁回后 active_submap_ 仍为 nullptr，后续逻辑将为新系关键帧创建新子图
        }

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
            // 修复: 添加NaN检查，防止协方差矩阵元素为NaN时导致sqrt产生NaN
            const Mat66d& cov = kf->covariance;
            double pos_std_x = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
            double pos_std_y = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
            double pos_std_z = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
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
        kf->index_in_submap = static_cast<int>(active_submap_->keyframes.size()); // ✅ V2 修复：设置子图内索引
        active_submap_->keyframes.push_back(kf);
        active_submap_->t_end = kf->timestamp;

        // 更新锚定位姿（第一帧）
        if (active_submap_->keyframes.size() == 1) {
            active_submap_->pose_odom_anchor           = kf->T_odom_b;
            // 🏛️ [修复] 初始锚点应继承关键帧的优化位姿（可能已由 MappingModule 应用 GPS 对齐），
            // 否则会导致每一块新子图在优化结果返回前都产生位姿“回退”到 Odom 系的重影。
            active_submap_->pose_map_anchor_optimized = kf->T_map_b_optimized;
            active_submap_->pose_frame = kf->pose_frame; // 🏛️ [架构加固] 尊重关键帧自带的坐标系语义
            kf->is_anchor = true;
            kf->T_submap_kf = Pose3d::Identity();

            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[V3][POSE_DIAG] New Submap #%d Anchor: T_odom_anchor=[%.2f,%.2f,%.2f] T_map_anchor=[%.2f,%.2f,%.2f]",
                active_submap_->id,
                active_submap_->pose_odom_anchor.translation().x(), active_submap_->pose_odom_anchor.translation().y(), active_submap_->pose_odom_anchor.translation().z(),
                active_submap_->pose_map_anchor_optimized.translation().x(), active_submap_->pose_map_anchor_optimized.translation().y(), active_submap_->pose_map_anchor_optimized.translation().z());
        } else {
            kf->T_submap_kf = active_submap_->pose_odom_anchor.inverse() * kf->T_odom_b;
            
            // 🏛️ [修复] 即使尚未触发后端优化，也要保持子图内部位姿的一致性：
            // 新关键帧的优化位姿 = 当前子图锚点优化位姿 * 帧相对于锚点的位姿。
            // 这确保了在 iSAM2 异步更新期间，新加入的帧能立即继承子图已有的优化/对齐偏移。
            kf->T_map_b_optimized = active_submap_->pose_map_anchor_optimized * kf->T_submap_kf;
            kf->pose_frame = active_submap_->pose_frame; // 🏛️ [架构加固] 继承子图锚点的坐标系语义

            RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("automap_system"), *node()->get_clock(), 5000,
                "[V3][POSE_DIAG] Submap #%d KF #%lu: T_submap_kf=[%.2f,%.2f,%.2f] T_map_b_opt=[%.2f,%.2f,%.2f]",
                active_submap_->id, kf->id,
                kf->T_submap_kf.translation().x(), kf->T_submap_kf.translation().y(), kf->T_submap_kf.translation().z(),
                kf->T_map_b_optimized.translation().x(), kf->T_map_b_optimized.translation().y(), kf->T_map_b_optimized.translation().z());
        }

        // 更新 GPS 中心（所有有效 GPS 的平均值）
        if (kf->has_valid_gps) {
            updateGPSGravityCenter(kf);
        }

        // 🏛️ V3: 异步合并点云 (Async Merge Worker)
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][ASYNC_MERGE] enqueuing kf_id=%lu for sm_id=%d", kf->id, active_submap_->id);
        
        {
            std::lock_guard<std::mutex> lk_merge(merge_mutex_);
            if (merge_queue_.size() < kMaxMergeQueueSize) {
                // [RC-3 修复] 先递增计数再入队，确保 freezeActiveSubmap 等待时不会提前放行
                active_submap_->pending_merge_count.fetch_add(1, std::memory_order_relaxed);
                merge_queue_.push({active_submap_, kf});
                merge_cv_.notify_one();
            } else {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[SubMapMgr][ASYNC_MERGE] Queue full! Dropping merge for kf_id=%lu", kf->id);
            }
        }

        // 更新空间范围（最近帧到锚定帧的最大距离）
        double dist = (kf->T_odom_b.translation() -
                       active_submap_->pose_odom_anchor.translation()).norm();
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

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=addKeyFrame_before_freeze_check sm_id=%d kf_count=%zu file=%s line=%d",
            active_submap_->id, active_submap_->keyframes.size(), __FILE__, __LINE__);
        if (isFull(active_submap_)) {
            const int sm_id = active_submap_->id;
            const size_t kf_count = active_submap_->keyframes.size();
            const double dist = active_submap_->spatial_extent_m;
            SubMap::Ptr to_freeze = active_submap_;
            active_submap_ = nullptr;

            SLOG_INFO(MOD, "SubMap FULL: id={}, kf={}, dist={:.1f}m → freezing",
                       sm_id, kf_count, dist);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][STEP] step=freeze_enter sm_id=%d kf_count=%zu file=%s line=%d",
                sm_id, kf_count, __FILE__, __LINE__);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][ADD_KF_STEP] isFull=true sm_id=%d kf_count=%zu → unlock before freeze (avoid deadlock)", sm_id, kf_count);

            lk.unlock();
            try {
                freezeActiveSubmap(to_freeze);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SubMapMgr][STEP] step=freeze_exit sm_id=%d file=%s line=%d", sm_id, __FILE__, __LINE__);
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

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=addKeyFrame_exit kf_id=%lu sm_id=%d file=%s line=%d",
            kf->id, active_submap_ ? active_submap_->id : -1, __FILE__, __LINE__);
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
    bool detached_active = false;
    {
        // Keep submap state transition and active pointer handoff atomic to avoid
        // appending new keyframes into a just-frozen submap.
        std::lock_guard<std::mutex> lk(mutex_);
        if (!sm || sm->state != SubMapState::ACTIVE) {
            SLOG_WARN(MOD, "freezeActiveSubmap(sm) invalid: state={}",
                       sm ? static_cast<int>(sm->state) : -1);
            return;
        }
        sm->state = SubMapState::FROZEN;
        if (active_submap_ == sm) {
            active_submap_ = nullptr;
            detached_active = true;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][FREEZE_STEP] enter freeze sm_id=%d (async post-process, detached_active=%d)",
        sm->id, detached_active ? 1 : 0);

    SLOG_START_SPAN(MOD, "freeze_submap");

    // [RC-3 修复] 等待该子图所有挂起/执行中的 merge 任务完成，确保 merged_cloud 已填充。
    // 若不等待，freeze_post_queue_ 消费时 merged_cloud 仍为空，downsampled_cloud 无法建立，
    // 导致回环检测数据库始终为空（全程 NO_CAND，zero loop closures）。
    {
        const int pending = sm->pending_merge_count.load(std::memory_order_acquire);
        if (pending > 0) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][FREEZE_STEP][RC3] sm_id=%d waiting for %d pending merge tasks before freeze_post",
                sm->id, pending);
            std::unique_lock<std::mutex> lk_done(merge_mutex_);
            static constexpr int kMergeWaitSec = 5;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(kMergeWaitSec);
            bool ok = merge_done_cv_.wait_until(lk_done, deadline, [&sm] {
                return sm->pending_merge_count.load(std::memory_order_acquire) == 0;
            });
            if (!ok) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[SubMapMgr][FREEZE_STEP][RC3] sm_id=%d merge wait timeout (%ds), remaining=%d "
                    "(merged_cloud may be partial, downsampled_cloud may be incomplete)",
                    sm->id, kMergeWaitSec, sm->pending_merge_count.load());
            } else {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SubMapMgr][FREEZE_STEP][RC3] sm_id=%d all merge tasks done, merged_cloud ready",
                    sm->id);
            }
        }
    }

    try {
        publishEvent(sm, "FROZEN");
        METRICS_INCREMENT(metrics::SUBMAPS_FROZEN);
        SLOG_EVENT(MOD, "submap_frozen", "SubMap #{} state=FROZEN (post-process enqueue)", sm->id);

        bool enqueued = false;
        {
            // 缩短等待时间 3s→1s，避免 addKeyFrame 阶段长时间阻塞后端（见 BACKEND_STUCK 分析）
            static constexpr int kFreezePostWaitSec = 1;
            std::unique_lock<std::mutex> lk(freeze_post_mutex_);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(kFreezePostWaitSec);
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
            // merged_cloud 由 merge 路径用 T_map_b_optimized 投到地图世界系；freeze 到锚点系须用 pose_map_anchor_optimized^{-1}
            CloudXYZIPtr body_cloud(new CloudXYZI());
            const Pose3d& T_w_anchor = sm->pose_map_anchor_optimized.matrix().allFinite()
                ? sm->pose_map_anchor_optimized
                : sm->pose_odom_anchor;
            Eigen::Isometry3d T_anchor_w = T_w_anchor.inverse();
            pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
            
            CloudXYZIPtr ds = submap_merge_semantic_intensity_vote_
                ? utils::voxelDownsampleMajorityIntensity(body_cloud, static_cast<float>(match_res_),
                                                            parallel_voxel_downsample_)
                : utils::voxelDownsample(body_cloud, static_cast<float>(match_res_), parallel_voxel_downsample_);
            if (!ds || ds->empty()) ds = body_cloud;
            sm->downsampled_cloud = ds;
            METRICS_HISTOGRAM_OBSERVE(metrics::POINTCLOUD_SIZE, static_cast<double>(sm->merged_cloud->size()));
        }
        {
            std::vector<SubMapFrozenCallback> cbs_copy;
            { std::lock_guard<std::mutex> lk(frozen_cbs_mutex_); cbs_copy = frozen_cbs_; }
            for (auto& cb : cbs_copy) cb(sm);  // 锁外执行回调，禁止回调内调用 getFrozenSubmaps 等会获取 mutex_ 的接口
        }
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

void SubMapManager::forceFreezeActiveSubmapForFinish() {
    SubMap::Ptr sm;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        sm = active_submap_;
        if (!sm || sm->state != SubMapState::ACTIVE) {
            if (sm)
                RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                    "[SubMapMgr][FINISH_FREEZE] no active submap to force-freeze (state=%d)", static_cast<int>(sm->state));
            return;
        }
        sm->state = SubMapState::FROZEN;
        publishEvent(sm, "FROZEN");
        METRICS_INCREMENT(metrics::SUBMAPS_FROZEN);
        active_submap_ = nullptr;
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][FINISH_FREEZE] force-freeze sm_id=%d (sync, so last submap enters factor graph)", sm->id);

    // [RC-3 修复] 结束冻结也需等待 merge 任务完成，确保最后一个子图的 merged_cloud 已填充
    {
        const int pending = sm->pending_merge_count.load(std::memory_order_acquire);
        if (pending > 0) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][FINISH_FREEZE][RC3] sm_id=%d waiting for %d pending merge tasks",
                sm->id, pending);
            std::unique_lock<std::mutex> lk_done(merge_mutex_);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            merge_done_cv_.wait_until(lk_done, deadline, [&sm] {
                return sm->pending_merge_count.load(std::memory_order_acquire) == 0;
            });
        }
    }

    try {
        if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            CloudXYZIPtr body_cloud(new CloudXYZI());
            const Pose3d& T_w_anchor = sm->pose_map_anchor_optimized.matrix().allFinite()
                ? sm->pose_map_anchor_optimized
                : sm->pose_odom_anchor;
            Eigen::Isometry3d T_anchor_w = T_w_anchor.inverse();
            pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
            
            CloudXYZIPtr ds = submap_merge_semantic_intensity_vote_
                ? utils::voxelDownsampleMajorityIntensity(body_cloud, static_cast<float>(match_res_),
                                                            parallel_voxel_downsample_)
                : utils::voxelDownsample(body_cloud, static_cast<float>(match_res_), parallel_voxel_downsample_);
            if (!ds || ds->empty()) ds = body_cloud;
            sm->downsampled_cloud = ds;
            METRICS_HISTOGRAM_OBSERVE(metrics::POINTCLOUD_SIZE, static_cast<double>(sm->merged_cloud->size()));
        }
        std::vector<SubMapFrozenCallback> cbs_copy;
        { std::lock_guard<std::mutex> lk(frozen_cbs_mutex_); cbs_copy = frozen_cbs_; }
        for (auto& cb : cbs_copy) cb(sm);
        HEALTH_UPDATE_QUEUE("submap", getFrozenSubmaps().size());
        HEALTH_UPDATE_QUEUE("loop", 0);
    } catch (const std::exception& e) {
        auto error = ErrorDetail(errors::SUBMAP_STATE_INVALID, fmt::format("forceFreezeActiveSubmapForFinish sm_id={}: {}", sm->id, e.what()));
        error.context().operation = "forceFreezeActiveSubmapForFinish";
        error.context().file = __FILE__;
        error.context().line = __LINE__;
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), "[SubMapManager] forceFreezeActiveSubmapForFinish: %s", e.what());
        publishErrorEvent(sm->id, error);
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
    }
}

void SubMapManager::freezeSubmap(const SubMap::Ptr& sm) {
    freezeActiveSubmap(sm);
}

void SubMapManager::mergeWorkerLoop() {
    while (merge_running_.load()) {
        MergeTask task;
        {
            std::unique_lock<std::mutex> lk(merge_mutex_);
            merge_cv_.wait(lk, [this] {
                return !merge_queue_.empty() || !merge_running_.load();
            });
            if (!merge_running_.load() && merge_queue_.empty()) break;
            if (merge_queue_.empty()) continue;
            task = std::move(merge_queue_.front());
            merge_queue_.pop();
            active_merge_tasks_++; // 🏛️ [修复] 增加活动任务数
        }

        if (task.sm && task.kf) {
            try {
                // 💡 [V3 性能优化] 在专用线程执行耗时的 SOR 滤波和点云变换
                mergeCloudToSubmap(task.sm, task.kf);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"), 
                    "[SubMapMgr][ASYNC_MERGE] Exception: %s", e.what());
            }
            // [RC-3 修复] 任务完成（成功或失败）后递减计数并通知等待的 freezeActiveSubmap
            task.sm->pending_merge_count.fetch_sub(1, std::memory_order_release);
            {
                std::lock_guard<std::mutex> lk_done(merge_mutex_);
                merge_done_cv_.notify_all();
            }
        }
        active_merge_tasks_--; // 🏛️ [修复] 减少活动任务数
    }
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
                // [HBA_FIX] 关键修复：将点云变换到子图锚点（Anchor）局部坐标系再进行降采样
                // 原因：LoopDetector 得到的 res.T_tgt_src 被 HBAOptimizer 直接用作 BetweenFactor。
                // GTSAM 的 BetweenFactor(X1, X2, T12) 要求 T12 是相对于 X1 的局部变换（body frame）。
                // 如果 downsampled_cloud 是世界坐标系，TEASER 得到的将是世界系下的变换（通常接近 Identity），
                // 强制 BetweenFactor 为 Identity 会导致 HBA 将不同位置的子图强行拉到一起，导致点云全花。
                // merged_cloud 为地图世界系；锚点变换与 merge 使用的 pose_map_anchor_optimized 一致。
                CloudXYZIPtr body_cloud(new CloudXYZI());
                const Pose3d& T_w_anchor = sm->pose_map_anchor_optimized.matrix().allFinite()
                    ? sm->pose_map_anchor_optimized
                    : sm->pose_odom_anchor;
                Eigen::Isometry3d T_anchor_w = T_w_anchor.inverse();
                pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
                
                CloudXYZIPtr ds = submap_merge_semantic_intensity_vote_
                    ? utils::voxelDownsampleMajorityIntensity(body_cloud, static_cast<float>(match_res_),
                                                                parallel_voxel_downsample_)
                    : utils::voxelDownsample(body_cloud, static_cast<float>(match_res_), parallel_voxel_downsample_);
                if (!ds || ds->empty()) ds = body_cloud;
                sm->downsampled_cloud = ds;
                
                METRICS_HISTOGRAM_OBSERVE(metrics::POINTCLOUD_SIZE, static_cast<double>(sm->merged_cloud->size()));
                RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                    "[SubMapMgr][FREEZE_POST] sm_id=%d downsampled_cloud created in ANCHOR frame (pts=%zu)", 
                    sm->id, ds->size());
            }
            {
                std::vector<SubMapFrozenCallback> cbs_copy;
                { std::lock_guard<std::mutex> lk(frozen_cbs_mutex_); cbs_copy = frozen_cbs_; }
                for (auto& cb : cbs_copy) cb(sm);  // 锁外执行回调，禁止回调内调用 getFrozenSubmaps 等会获取 mutex_ 的接口
            }
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
    RCLCPP_DEBUG(node()->get_logger(), "[SubMapMgr][DATA] createNewSubmap sm_id=%d session=%lu", sm->id, sm->session_id);
    return sm;
}

namespace {
constexpr size_t kDownsampleThreshold = 200000;
}

CloudXYZIPtr SubMapManager::downsampleKeyframeBodyForMerging_(const KeyFrame::Ptr& kf) const {
    CloudXYZIPtr raw_cloud = kf->cloud_body;
    if (!raw_cloud || raw_cloud->empty()) return std::make_shared<CloudXYZI>();

    const CloudXYZIPtr sem =
        (submap_merge_semantic_intensity_vote_ && kf->cloud_semantic_labeled_body &&
         kf->cloud_semantic_labeled_body->size() == raw_cloud->size())
            ? kf->cloud_semantic_labeled_body
            : nullptr;

    const float merge_res_f = static_cast<float>(merge_res_);

    if (sem) {
        CloudXYZIPtr ds = utils::voxelDownsampleBodyWithSemanticLabels(raw_cloud, sem, merge_res_f);
        if (!ds || ds->empty()) ds = raw_cloud;
        if (map_statistical_filter_) {
            CloudXYZIPtr temp(new CloudXYZI());
            try {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                sor.setInputCloud(ds);
                sor.setMeanK(map_stat_filter_mean_k_);
                sor.setStddevMulThresh(map_stat_filter_std_mul_);
                sor.filter(*temp);
                if (!temp->empty()) return temp;
            } catch (const std::exception& e) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[SubMapMgr][CLARITY] SOR after semantic voxel failed kf_id=%lu: %s", kf->id, e.what());
            }
        }
        return ds;
    }

    CloudXYZIPtr filtered_kf_cloud = raw_cloud;
    if (map_statistical_filter_) {
        CloudXYZIPtr temp(new CloudXYZI());
        try {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
            sor.setInputCloud(raw_cloud);
            sor.setMeanK(map_stat_filter_mean_k_);
            sor.setStddevMulThresh(map_stat_filter_std_mul_);
            sor.filter(*temp);
            if (!temp->empty()) filtered_kf_cloud = temp;
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][CLARITY] SOR filter failed for kf_id=%lu: %s", kf->id, e.what());
        }
    }

    CloudXYZIPtr ds_kf_cloud(new CloudXYZI());
    {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(filtered_kf_cloud);
        vg.setLeafSize(merge_res_f, merge_res_f, merge_res_f);
        vg.filter(*ds_kf_cloud);
    }
    if (ds_kf_cloud->empty()) ds_kf_cloud = filtered_kf_cloud;
    return ds_kf_cloud;
}

void SubMapManager::mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const {
    if (!kf->cloud_body || kf->cloud_body->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu has null/empty cloud_body, skip merge", 
            kf->id);
        return;
    }

    // 🏛️ [P0 性能优化] 架构重构：耗时操作移出锁外，支持多线程并行
    CloudXYZIPtr raw_cloud = kf->cloud_body;
    Pose3d T_map_b_snapshot = kf->T_map_b_optimized;

    CloudXYZIPtr ds_kf_cloud = downsampleKeyframeBodyForMerging_(kf);
    if (!ds_kf_cloud || ds_kf_cloud->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu downsample empty, skip merge", kf->id);
        return;
    }

    // 变换到世界坐标系
    CloudXYZIPtr world_cloud(new CloudXYZI());
    Eigen::Affine3f T_wf;
    T_wf.matrix() = T_map_b_snapshot.cast<float>().matrix();
    try {
        pcl::transformPointCloud(*ds_kf_cloud, *world_cloud, T_wf);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu transform failed: %s", kf->id, e.what());
        return;
    }

    // 3. 极速合并 (Inside Lock)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        
        // 关键帧 cloud_body 必须始终保持为「原始完整点云」（与关键帧创建时一致）。
        // SOR 仅用于本函数内的体素合并路径（merged_cloud），不得写回 kf->cloud_body，
        // 否则 buildGlobalMap、语义模块、RViz current_cloud 等会丢失点数或偏离真实扫描。

        if (!sm->merged_cloud || sm->merged_cloud->empty()) {
            sm->merged_cloud = std::make_shared<CloudXYZI>(*world_cloud);
        } else {
            // 直接追加，因为每一帧都已经预降采样过了，merged_cloud 增长可控
            sm->merged_cloud->points.insert(sm->merged_cloud->points.end(),
                                            world_cloud->points.begin(), world_cloud->points.end());
        }
        
        // 偶尔检查是否需要对整个子图做一次全量降采样（例如点云极其稠密时）
        if (sm->merged_cloud->size() > kDownsampleThreshold * 4) {
            const float res = static_cast<float>(merge_res_);
            CloudXYZIPtr temp;
            if (submap_merge_semantic_intensity_vote_) {
                temp = utils::voxelDownsampleMajorityIntensity(sm->merged_cloud, res, parallel_voxel_downsample_);
            } else {
                temp.reset(new CloudXYZI());
                pcl::VoxelGrid<pcl::PointXYZI> vg;
                vg.setInputCloud(sm->merged_cloud);
                vg.setLeafSize(res, res, res);
                vg.filter(*temp);
            }
            if (temp && !temp->empty()) {
                sm->merged_cloud.swap(temp);
            }
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][P0_OPT] merge sm_id=%d kf_id=%lu: raw=%zu -> ds_body=%zu -> world_pts=%zu, merged_total=%zu",
        sm->id, kf->id, raw_cloud->size(), ds_kf_cloud->size(), world_cloud->size(),
        sm->merged_cloud ? sm->merged_cloud->size() : 0);

    // [GHOSTING_TRACE] 合并已用快照位姿 T_map_b_snapshot；与 odom 差异诊断
    if (backend_verbose_trace_) {
        const Eigen::Vector3d t = T_map_b_snapshot.translation();
        const Eigen::Vector3d t_odom = kf->T_odom_b.translation();
        const Eigen::Vector3d t_opt = kf->T_map_b_optimized.translation();
        double trans_diff = (t_odom - t_opt).norm();
        double yaw_odom = std::atan2(kf->T_odom_b.rotation()(1, 0), kf->T_odom_b.rotation()(0, 0)) * 180.0 / M_PI;
        double yaw_opt = std::atan2(kf->T_map_b_optimized.rotation()(1, 0), kf->T_map_b_optimized.rotation()(0, 0)) * 180.0 / M_PI;
        double yaw_diff = std::abs(yaw_odom - yaw_opt);
        if (yaw_diff > 180.0) yaw_diff = 360.0 - yaw_diff;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[GHOSTING_TRACE] mergeCloudToSubmap sm_id=%d kf_id=%lu pose_source=T_map_b_optimized merge_t=[%.2f,%.2f,%.2f] T_odom_b=[%.2f,%.2f,%.2f] odom_vs_opt_trans=%.3fm odom_vs_opt_yaw=%.2fdeg",
            sm->id, kf->id, t.x(), t.y(), t.z(), t_odom.x(), t_odom.y(), t_odom.z(), trans_diff, yaw_diff);
    }
}

void SubMapManager::updateSubmapPose(int submap_id, const Pose3d& new_pose, PoseFrame pose_frame, uint64_t alignment_epoch) {
    // 结构化日志：开始Span
    SLOG_START_SPAN(MOD, "update_submap_pose");
    
    std::unique_lock<std::mutex> lk(mutex_);
    
    bool updated = false;
    double max_translation_diff = 0.0;
    double max_rotation_diff = 0.0;

    // ========== [SUBMAP_POSE_UPDATE_DEBUG] 增加详细日志 ==========
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][SUBMAP_POSE_UPDATE_DEBUG] =======================================================");
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][SUBMAP_POSE_UPDATE_DEBUG] 开始更新子图%d的位姿... frame=%d epoch=%lu", 
        submap_id, static_cast<int>(pose_frame), alignment_epoch);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][SUBMAP_POSE_UPDATE_DEBUG] 新锚点位姿: pos=(%.2f, %.2f, %.2f), RPY=(%.1f, %.1f, %.1f)",
        new_pose.translation().x(), new_pose.translation().y(), new_pose.translation().z(),
        new_pose.rotation().eulerAngles(2,1,0).x() * 180.0 / M_PI,
        new_pose.rotation().eulerAngles(2,1,0).y() * 180.0 / M_PI,
        new_pose.rotation().eulerAngles(2,1,0).z() * 180.0 / M_PI);
    // ========== [SUBMAP_POSE_UPDATE_DEBUG] 结束 ==========

    for (auto& sm : submaps_) {
        if (sm->id != submap_id) continue;

        // 验证状态转换合法性
        if (sm->state != SubMapState::FROZEN &&
            sm->state != SubMapState::OPTIMIZED &&
            sm->state != SubMapState::ACTIVE) { // ✅ 修复：允许更新 ACTIVE 子图位姿，解决子图内回环重影
            continue;
        }

        Pose3d old_anchor = sm->pose_map_anchor_optimized;
        Pose3d delta = new_pose * old_anchor.inverse();

        sm->pose_map_anchor_optimized = new_pose;
        sm->pose_frame = pose_frame; // 🏛️ [架构加固] 使用参数传入的 frame
        if (alignment_epoch > 0) sm->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
        sm->state = SubMapState::OPTIMIZED;

        // 更新关键帧位姿
        for (auto& kf : sm->keyframes) {
            kf->T_map_b_optimized = sm->pose_map_anchor_optimized * kf->T_submap_kf;
            kf->pose_frame = pose_frame; // 🏛️ [架构加固] 继承子图锚点的坐标系语义
            if (alignment_epoch > 0) kf->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
        }

        // 🔧 [修复] 保持 merged_cloud 与轨迹同步：同步变换该子图的合并点云，避免重影
        if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            try {
                pcl::transformPointCloud(*sm->merged_cloud, *sm->merged_cloud, delta.matrix().cast<float>());
            } catch (...) {}
        }

        publishEvent(sm, "OPTIMIZED");
        updated = true;
        break;
    }

    if (updated) {
        METRICS_INCREMENT(metrics::OPTIMIZATIONS_RUN);
    }

    // 结构化日志：结束Span
    SLOG_END_SPAN();
}

void SubMapManager::batchUpdateSubmapPoses(const std::unordered_map<int, Pose3d>& updates, uint64_t version, PoseFrame pose_frame, uint64_t alignment_epoch) {
    if (updates.empty()) return;

    std::unique_lock<std::mutex> lk(mutex_);
    current_map_version_ = version; // 更新当前位姿版本
    double max_trans_diff = 0.0;
    double max_rot_diff = 0.0;
    int updated_count = 0;

    for (const auto& [id, new_pose] : updates) {
        for (auto& sm : submaps_) {
            if (sm->id == id) {
                Pose3d old_anchor = sm->pose_map_anchor_optimized;
                Pose3d delta = new_pose * old_anchor.inverse();

                sm->pose_map_anchor_optimized = new_pose;
                sm->pose_frame = pose_frame; // 🏛️ [架构加固] 尊重传入的坐标系语义
                if (alignment_epoch > 0) sm->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
                sm->state = SubMapState::OPTIMIZED;

                double trans_diff = (new_pose.translation() - old_anchor.translation()).norm();
                double rot_diff = Eigen::AngleAxisd(new_pose.rotation().inverse() * old_anchor.rotation()).angle();
                
                max_trans_diff = std::max(max_trans_diff, trans_diff);
                max_rot_diff = std::max(max_rot_diff, rot_diff);

                // 更新关键帧位姿
                for (auto& kf : sm->keyframes) {
                    kf->T_map_b_optimized = sm->pose_map_anchor_optimized * kf->T_submap_kf;
                    kf->pose_frame = pose_frame; // 🏛️ [架构加固] 继承子图锚点的坐标系语义
                    if (alignment_epoch > 0) kf->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
                }

                // 🔧 [修复] 保持 merged_cloud 与轨迹同步：同步变换该子图的合并点云，避免重影
                if (sm->merged_cloud && !sm->merged_cloud->empty()) {
                    try {
                        pcl::transformPointCloud(*sm->merged_cloud, *sm->merged_cloud, delta.matrix().cast<float>());
                    } catch (...) {}
                }

                updated_count++;
                break;
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[V3][POSE_DIAG] Batch updated %d submaps for version %lu: max_trans_diff=%.3fm max_rot_diff=%.2fdeg",
        updated_count, version, max_trans_diff, max_rot_diff * 180.0 / M_PI);

    const double kRebuildThresholdTrans = submap_rebuild_thresh_trans_;
    const double kRebuildThresholdRot = submap_rebuild_thresh_rot_;
    const double max_rot_deg = max_rot_diff * 180.0 / M_PI;

    if (max_trans_diff > kRebuildThresholdTrans || max_rot_deg > kRebuildThresholdRot) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][AUTO_REBUILD] version=%lu drift large (%.2fm, %.2fdeg) -> triggering full rebuild",
            version, max_trans_diff, max_rot_deg);
        merged_cloud_dirty_.store(true, std::memory_order_release); // [RC5/RC6] 标记即将重建
        lk.unlock();
        rebuildMergedCloudFromOptimizedPoses();
    }
}

void SubMapManager::batchUpdateKeyFramePoses(const std::unordered_map<uint64_t, Pose3d>& updates, uint64_t version, PoseFrame pose_frame, uint64_t alignment_epoch) {
    if (updates.empty()) return;
    std::unique_lock<std::mutex> lk(mutex_);
    current_map_version_ = version;
    size_t updated = 0;
    double z_diag_max_abs_dz = 0.0;
    double z_diag_max_anchor_mapz_minus_odomz = 0.0;
    
    // 🏛️ [架构加固] 记录哪些子图的锚点发生了变动，以便后续同步子图内其他帧并检查重影
    std::unordered_map<int, Pose3d> sm_anchor_updates;
    double max_jump_dist = 0.0;

    auto process_sm_list = [&](std::vector<SubMap::Ptr>& sms) {
        for (auto& sm : sms) {
            if (!sm) continue;
            bool sm_anchor_kf_updated = false;
            for (auto& kf : sm->keyframes) {
                if (!kf) continue;
                auto it = updates.find(kf->id);
                if (it != updates.end()) {
                    if (!it->second.matrix().allFinite()) continue;
                    
                    // 记录跳变距离用于监控
                    if (kf->is_anchor) {
                        double jump = (it->second.translation() - kf->T_map_b_optimized.translation()).norm();
                        max_jump_dist = std::max(max_jump_dist, jump);
                    }

                    const double old_map_z = kf->T_map_b_optimized.translation().z();
                    const double new_map_z = it->second.translation().z();
                    z_diag_max_abs_dz = std::max(z_diag_max_abs_dz, std::abs(new_map_z - old_map_z));
                    if (kf->is_anchor && kf->T_odom_b.matrix().allFinite()) {
                        z_diag_max_anchor_mapz_minus_odomz = std::max(
                            z_diag_max_anchor_mapz_minus_odomz,
                            std::abs(new_map_z - kf->T_odom_b.translation().z()));
                    }

                    kf->T_map_b_optimized = it->second;
                    kf->pose_frame = pose_frame; 
                    if (alignment_epoch > 0) kf->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
                    
                    if (kf->is_anchor) {
                        sm->pose_map_anchor_optimized = kf->T_map_b_optimized;
                        sm->pose_frame = pose_frame;
                        if (alignment_epoch > 0) sm->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
                        sm_anchor_kf_updated = true;
                    }
                    updated++;
                }
            }

            // 🏛️ [关键修复] 如果子图锚点更新了（通常是坐标系从 ODOM 切换到 MAP），
            // 必须重新传播位姿到该子图内所有关键帧，确保子图内部一致性，彻底消除重影。
            if (sm_anchor_kf_updated) {
                const Pose3d inv_a = sm->pose_map_anchor_optimized.inverse();
                if (inv_a.matrix().allFinite()) {
                    for (auto& kf : sm->keyframes) {
                        if (!kf) continue;
                        // 对于已经在 updates 里的帧，更新其相对锚点的位姿 T_submap_kf
                        if (updates.count(kf->id)) {
                            kf->T_submap_kf = inv_a * kf->T_map_b_optimized;
                        } else {
                            // 对于太新（还在途）不在更新列表里的帧，根据新锚点和旧相对位姿重算绝对位姿
                            kf->T_map_b_optimized = sm->pose_map_anchor_optimized * kf->T_submap_kf;
                            kf->pose_frame = pose_frame;
                            if (alignment_epoch > 0) kf->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
                        }
                    }
                }
                sm_anchor_updates[sm->id] = sm->pose_map_anchor_optimized;
            }
        }
    };

    process_sm_list(submaps_);
    // active_submap_ 通常已在 submaps_ 中，但为了健壮性单独检查一次
    if (active_submap_) {
        std::vector<SubMap::Ptr> active_list = {active_submap_};
        process_sm_list(active_list);
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
                 "[SubMapMgr][Gateway] batchUpdateKeyFramePoses: updated %zu kfs, %zu sm_anchors to version %lu (max_jump=%.3fm)", 
                 updated, sm_anchor_updates.size(), version, max_jump_dist);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[Z_DRIFT_DIAG] stage=isam2_batch map_ver=%lu updated_kfs=%zu max_abs_dz=%.4fm "
                "max_anchor_abs_mapz_minus_odomz=%.4fm (纯 yaw+tz=0 对齐后锚点应接近 0；偏大则查 GPS/回环/杆臂)",
                static_cast<unsigned long>(version), updated, z_diag_max_abs_dz, z_diag_max_anchor_mapz_minus_odomz);

    // 🏛️ [架构加固] 如果检测到巨大跳变（如 GPS 对齐），强制重建点云，避免 merged_cloud 污染
    if (max_jump_dist > submap_rebuild_thresh_trans_) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][GHOST_GUARD] Large jump (%.3fm) detected in KF update, triggering merged_cloud rebuild",
            max_jump_dist);
        merged_cloud_dirty_.store(true, std::memory_order_release); // [RC5/RC6] 标记即将重建
        lk.unlock();
        rebuildMergedCloudFromOptimizedPoses();
    }
}

std::vector<KeyFrame::Ptr> SubMapManager::collectKeyframesInHBAOrder() const {
    // 与 HBAOptimizer::collectKeyFramesFromSubmaps 完全一致：过滤 + 按 timestamp 排序 + 按 timestamp 去重，
    // 保证 updateAllFromHBA 写回时 result.optimized_poses[i] 对应第 i 个关键帧，避免顺序错位导致 PCD 严重重影。
    std::vector<KeyFrame::Ptr> kfs;
    for (const auto& sm : submaps_) {
        if (!sm || sm->keyframes.empty()) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            if (!kf->cloud_body || kf->cloud_body->empty()) continue;
            const auto& t = kf->T_odom_b.translation();
            const auto& R = kf->T_odom_b.rotation();
            if (!t.allFinite() || !R.allFinite()) continue;
            kfs.push_back(kf);
        }
    }
    std::sort(kfs.begin(), kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });
    if (kfs.size() > 1) {
        std::vector<KeyFrame::Ptr> unique_kfs;
        unique_kfs.push_back(kfs[0]);
        for (size_t i = 1; i < kfs.size(); ++i) {
            if (std::abs(kfs[i]->timestamp - unique_kfs.back()->timestamp) > 0.001)
                unique_kfs.push_back(kfs[i]);
        }
        kfs.swap(unique_kfs);
    }
    return kfs;
}

void SubMapManager::updateAllFromHBA(const HBAResult& result) {
    if (!result.success || result.optimized_poses.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][HBA_GHOSTING] updateAllFromHBA skip: success=%d poses=%zu (无写回则不会产生位姿双轨)",
            result.success ? 1 : 0, result.optimized_poses.size());
        return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    current_map_version_++; // HBA 写回后强制版本自增以触发地图重建
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    const double diag_ts = (node() && node()->get_clock()) ? node()->get_clock()->now().seconds() : 0.0;
    RCLCPP_INFO(log,
        "[SubMapMgr][GHOSTING_DIAG] updateAllFromHBA enter ts=%.3f optimized_poses=%zu submaps=%zu (HBA 写回全部 KF；若在某次 build 的 enter~exit 之间则可能重影，grep GHOSTING_DIAG 查时间线)",
        diag_ts, result.optimized_poses.size(), submaps_.size());

    // [PCD_GHOSTING_FIX] 按与 HBA 完全相同的顺序写回：HBA 使用 filter+sort(timestamp)+dedupe，此处用 collectKeyframesInHBAOrder 得到相同顺序，避免位姿与关键帧错位导致保存的 global_map.pcd 严重重影（见 docs/PCD_GHOSTING_VS_RVIZ_ANALYSIS）
    std::vector<KeyFrame::Ptr> kfs_in_hba_order = collectKeyframesInHBAOrder();
    RCLCPP_INFO(log,
        "[SubMapMgr][GHOSTING_DIAG] HBA writeback_enter ts=%.3f pose_count=%zu kfs_in_hba_order=%zu (写回顺序与 HBA 输出一致)",
        diag_ts, result.optimized_poses.size(), kfs_in_hba_order.size());
    // [PCD_GHOSTING_VERIFY] 写回顺序首尾：与 HBA_INPUT_ORDER 应对齐（first/last kf_id+ts 一致）；若不一致则写回错位，grep WRITEBACK_ORDER HBA_INPUT_ORDER 对照
    if (!kfs_in_hba_order.empty()) {
        const auto& wf = kfs_in_hba_order.front();
        const auto& wb = kfs_in_hba_order.back();
        RCLCPP_INFO(log,
            "[WRITEBACK_ORDER] first kf_id=%lu sm_id=%d ts=%.3f last kf_id=%lu sm_id=%d ts=%.3f count=%zu (与 HBA_INPUT_ORDER 应对齐)",
            wf->id, wf->submap_id, wf->timestamp, wb->id, wb->submap_id, wb->timestamp, kfs_in_hba_order.size());
    }
    const size_t n_poses = result.optimized_poses.size();
    const size_t n_kfs = kfs_in_hba_order.size();
    if (n_kfs != n_poses) {
        RCLCPP_ERROR(log,
            "[HBA_WRITEBACK_MISMATCH] 严重错误: kfs_in_hba_order=%zu optimized_poses=%zu 数量不一致，正常建图不应出现，程序即将退出",
            n_kfs, n_poses);
        std::abort();
    }
    // Z 漂移诊断：写回前采样首/中/尾 map.z，写回后与 odom.z 对比（grep Z_DRIFT_DIAG stage=hba_writeback）
    const size_t zi0 = 0;
    const size_t zi1 = (n_kfs > 2) ? (n_kfs / 2) : 0;
    const size_t zi2 = (n_kfs > 1) ? (n_kfs - 1) : 0;
    const double z_b0 = kfs_in_hba_order[zi0]->T_map_b_optimized.translation().z();
    const double z_b1 = kfs_in_hba_order[zi1]->T_map_b_optimized.translation().z();
    const double z_b2 = kfs_in_hba_order[zi2]->T_map_b_optimized.translation().z();

    for (size_t i = 0; i < n_poses; ++i) {
        // [HBA_GHOSTING_FIX] 直接覆盖为 HBA 优化后的绝对位姿
        kfs_in_hba_order[i]->T_map_b_optimized = result.optimized_poses[i];
        // [连续HBA跳变修复] 同步写入 T_map_b_hba，专用于下一轮 HBA 的初值读取。
        // iSAM2 的 batchUpdateKeyFramePoses 路径不会触碰此字段，因此即使 iSAM2 在两次
        // HBA 之间覆写了 T_map_b_optimized，下一轮 HBA 仍可从 T_map_b_hba 获得正确的
        // GPS-aligned 初值，防止旋转角度在连续 HBA 之间退化（见：GHOST_GUARD kf_id=105 18.97deg）。
        kfs_in_hba_order[i]->T_map_b_hba = result.optimized_poses[i];
        // [HBA初值修复] 标记 T_map_b_hba 已由 HBA 写入，使 poseForInitial 可安全使用该字段。
        kfs_in_hba_order[i]->hba_pose_valid = true;
        kfs_in_hba_order[i]->pose_frame = result.pose_frame; // 🏛️ [架构加固] 尊重 HBA 报告的坐标系语义
        
        // 🏛️ [修复] 同步更新对齐纪元，确保 buildGlobalMap 主路径不会因 alignment_epoch_limit 过滤掉这些已优化的帧
        if (result.alignment_epoch_snapshot > 0) {
            kfs_in_hba_order[i]->alignment_epoch = result.alignment_epoch_snapshot;
        }
    }

    {
        auto absdz = [](double a, double b) { return std::abs(a - b); };
        double max_abs_dz = absdz(kfs_in_hba_order[zi0]->T_map_b_optimized.translation().z(), z_b0);
        max_abs_dz = std::max(max_abs_dz, absdz(kfs_in_hba_order[zi1]->T_map_b_optimized.translation().z(), z_b1));
        max_abs_dz = std::max(max_abs_dz, absdz(kfs_in_hba_order[zi2]->T_map_b_optimized.translation().z(), z_b2));
        double max_m_minus_o = 0.0;
        for (size_t zix : {zi0, zi1, zi2}) {
            const auto& kf = kfs_in_hba_order[zix];
            if (kf && kf->T_odom_b.matrix().allFinite()) {
                max_m_minus_o = std::max(max_m_minus_o,
                    std::abs(kf->T_map_b_optimized.translation().z() - kf->T_odom_b.translation().z()));
            }
        }
        RCLCPP_INFO(log,
            "[Z_DRIFT_DIAG] stage=hba_writeback kfs=%zu sample_idx=[%zu,%zu,%zu] max_abs_dz_sample=%.4fm "
            "max_abs_mapz_minus_odomz_sample=%.4fm (对齐后垂直向应贴近 LIO；偏大查回环/GPS/Between-Z)",
            n_kfs, zi0, zi1, zi2, max_abs_dz, max_m_minus_o);
    }

    // [PCD_GHOSTING_VERIFY] ... (此处省略校验代码) ...

    double max_trans_diff = 0.0;
    double max_rot_diff = 0.0;

    // 同步子图锚定位姿并更新 T_submap_kf
    for (auto& sm : submaps_) {
        if (!sm || sm->keyframes.empty()) continue;
        KeyFrame::Ptr anchor = sm->keyframes.front();
        if (!anchor) continue;
        
        // ... (此处省略日志代码) ...

        sm->pose_map_anchor_optimized = anchor->T_map_b_optimized;
        sm->pose_frame = result.pose_frame; // 🏛️ [架构加固] 尊重 HBA 报告的坐标系语义
        if (result.alignment_epoch_snapshot > 0) {
            sm->alignment_epoch = result.alignment_epoch_snapshot;
        }
        sm->pose_odom_anchor = anchor->T_odom_b;

        // 🏛️ [架构重构] HBA 可能会改变子图内部结构，因此需要更新 T_submap_kf
        // 保证后续 ISAM2 的无 stateless 更新 (pose_map_anchor_optimized * T_submap_kf) 是准确的
        for (auto& kf : sm->keyframes) {
            if (kf) {
                kf->T_submap_kf = sm->pose_map_anchor_optimized.inverse() * kf->T_map_b_optimized;
                kf->pose_frame = result.pose_frame; // 🏛️ [架构加固] 继承子图锚点的坐标系语义
            }
        }

        const Pose3d delta_odom_opt = anchor->T_map_b_optimized * anchor->T_odom_b.inverse();
        max_trans_diff = std::max(max_trans_diff, delta_odom_opt.translation().norm());
        max_rot_diff = std::max(max_rot_diff, Eigen::AngleAxisd(delta_odom_opt.rotation()).angle());
    }

    RCLCPP_INFO(log, 
        "[SubMapMgr][HBA_DIAG] updateAllFromHBA done: submaps=%zu max_trans_diff=%.3fm max_rot_diff=%.2fdeg",
        submaps_.size(), max_trans_diff, max_rot_diff);
    const double exit_ts = (node() && node()->get_clock()) ? node()->get_clock()->now().seconds() : 0.0;
    RCLCPP_INFO(log,
        "[SubMapMgr][GHOSTING_DIAG] updateAllFromHBA exit ts=%.3f (写回与锚点同步已完成；pose_snapshot_taken 应不落在此 enter~exit 之间，否则存在竞态)",
        exit_ts);

    // HBA 已 bump current_map_version_；清空缓存避免同步/异步路径返回旧体素图（与 handleSaveMap 前 invalidate 互为补充）
    cached_global_map_.reset();
    last_build_map_version_ = std::numeric_limits<uint64_t>::max();
    RCLCPP_INFO(log,
        "[SubMapMgr][GHOSTING_DIAG] global_map cache cleared after HBA writeback map_version=%lu",
        static_cast<unsigned long>(current_map_version_));
}

void SubMapManager::applyGpsMapOriginToOdomKeyframes(const Eigen::Matrix3d& R_enu_to_map,
                                                     const Eigen::Vector3d& t_enu_to_map,
                                                     uint64_t map_version,
                                                     uint64_t alignment_epoch) {
    Pose3d T_mo = Pose3d::Identity();
    T_mo.linear() = R_enu_to_map;
    T_mo.translation() = t_enu_to_map;
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    if (!T_mo.matrix().allFinite()) {
        RCLCPP_WARN(log,
            "[SubMapMgr][GPS_ALIGN_BARRIER] skip: T_map_odom non-finite map_version=%lu",
            static_cast<unsigned long>(map_version));
        return;
    }

    size_t upgraded = 0;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        auto upgradeKeyframes = [&](const SubMap::Ptr& sm) {
            if (!sm) return;
            for (auto& kf : sm->keyframes) {
                if (!kf) continue;
                if (kf->pose_frame != PoseFrame::ODOM) continue;
                if (!kf->T_odom_b.matrix().allFinite()) continue;
                kf->T_map_b_optimized = v3::pose_chain::mapBodyFromOdomBody(T_mo, kf->T_odom_b);
                kf->pose_frame = PoseFrame::MAP;
                if (alignment_epoch > 0) kf->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
                ++upgraded;
            }
        };
        for (auto& sm : submaps_) upgradeKeyframes(sm);
        upgradeKeyframes(active_submap_);

        auto resyncSubmapRelative = [&](const SubMap::Ptr& sm) {
            if (!sm || sm->keyframes.empty()) return;
            KeyFrame::Ptr anchor = sm->keyframes.front();
            if (!anchor || !anchor->T_map_b_optimized.matrix().allFinite()) return;
            sm->pose_map_anchor_optimized = anchor->T_map_b_optimized;
            sm->pose_frame = PoseFrame::MAP;
            if (alignment_epoch > 0) sm->alignment_epoch = alignment_epoch; // 🏛️ [对齐纪元]
            const Pose3d inv_a = sm->pose_map_anchor_optimized.inverse();
            if (!inv_a.matrix().allFinite()) return;
            for (auto& kf : sm->keyframes) {
                if (!kf || !kf->T_map_b_optimized.matrix().allFinite()) continue;
                kf->T_submap_kf = inv_a * kf->T_map_b_optimized;
            }
        };
        for (auto& sm : submaps_) resyncSubmapRelative(sm);
        resyncSubmapRelative(active_submap_);

        current_map_version_ = map_version;
        cached_global_map_.reset();
        last_build_map_version_ = std::numeric_limits<uint64_t>::max();
        // [RC5/RC6] 在锁内标记 merged_cloud 即将失效，防止锁释放到重建完成之间的竞争窗口
        merged_cloud_dirty_.store(true, std::memory_order_release);
    }

    RCLCPP_INFO(log,
        "[SubMapMgr][GPS_ALIGN_BARRIER] upgraded_odom_kfs=%zu map_version=%lu -> rebuildMergedCloudFromOptimizedPoses",
        upgraded, static_cast<unsigned long>(map_version));
    rebuildMergedCloudFromOptimizedPoses();
}

void SubMapManager::refreshDownsampledCloudFromMergedInAnchorFrame_(SubMap::Ptr& sm) {
    if (!sm) return;
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    if (!sm->merged_cloud || sm->merged_cloud->empty()) {
        sm->downsampled_cloud = std::make_shared<CloudXYZI>();
        RCLCPP_DEBUG(log, "[SubMapMgr][REBUILD_DS] sm_id=%d merged empty -> downsampled cleared", sm->id);
        return;
    }
    CloudXYZIPtr body_cloud(new CloudXYZI());
    const Pose3d& T_w_anchor = sm->pose_map_anchor_optimized.matrix().allFinite()
        ? sm->pose_map_anchor_optimized
        : sm->pose_odom_anchor;
    Eigen::Isometry3d T_anchor_w = T_w_anchor.inverse();
    try {
        pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
    } catch (const std::exception& e) {
        RCLCPP_WARN(log, "[SubMapMgr][REBUILD_DS] sm_id=%d anchor transform failed: %s -> downsampled cleared", sm->id, e.what());
        sm->downsampled_cloud = std::make_shared<CloudXYZI>();
        return;
    } catch (...) {
        RCLCPP_WARN(log, "[SubMapMgr][REBUILD_DS] sm_id=%d anchor transform unknown exception -> downsampled cleared", sm->id);
        sm->downsampled_cloud = std::make_shared<CloudXYZI>();
        return;
    }
    CloudXYZIPtr ds = submap_merge_semantic_intensity_vote_
        ? utils::voxelDownsampleMajorityIntensity(body_cloud, static_cast<float>(match_res_),
                                                    parallel_voxel_downsample_)
        : utils::voxelDownsample(body_cloud, static_cast<float>(match_res_), parallel_voxel_downsample_);
    if (!ds || ds->empty()) ds = body_cloud;
    sm->downsampled_cloud = ds;
    RCLCPP_INFO(log,
                "[SubMapMgr][REBUILD_DS] sm_id=%d downsampled_cloud refreshed in ANCHOR frame (pts=%zu, match_res=%.3f)",
                sm->id, ds->size(), match_res_);
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成后重建 merged_cloud：使用优化后的位姿重新构建点云
// 解决 merged_cloud 使用旧 T_odom_b 构建导致的点云重影问题
// ─────────────────────────────────────────────────────────────────────────────
void SubMapManager::rebuildMergedCloudFromOptimizedPoses() {
    std::lock_guard<std::mutex> lk(mutex_);
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    RCLCPP_INFO(log, "[SubMapMgr][HBA_GHOSTING] rebuildMergedCloudFromOptimizedPoses enter (必须仅用 T_map_b_optimized 变换 cloud_body，与 buildGlobalMap 主路径一致，避免重影)");
    RCLCPP_INFO(log, "[GHOSTING_SOURCE] merged_cloud rebuild_enter pose_source=T_map_b_optimized (HBA 写回后执行，完成后 fallback_merged_cloud 与主路径一致)");
    if (backend_verbose_trace_) {
        RCLCPP_INFO(log, "[GHOSTING_TRACE] rebuildMergedCloudFromOptimizedPoses ENTER submap_count=%zu (grep GHOSTING_TRACE 查重影证据链)",
            submaps_.size());
    }
    RCLCPP_INFO(log, "[SubMapMgr][REBUILD_MERGE] Starting rebuild merged_cloud with optimized poses...");

    for (auto& sm : submaps_) {
        if (!sm) continue;
        
        // 获取第一个关键帧的优化位姿作为子图锚点
        if (sm->keyframes.empty()) continue;
        KeyFrame::Ptr anchor = sm->keyframes.front();
        if (!anchor) continue;

        // 计算子图锚点的优化偏移
        Pose3d delta = anchor->T_map_b_optimized * anchor->T_odom_b.inverse();
        double delta_trans_norm = delta.translation().norm();
        double delta_rot_deg = Eigen::AngleAxisd(delta.rotation()).angle() * 180.0 / M_PI;
        const Eigen::Vector3d& anchor_t_odom = anchor->T_odom_b.translation();
        const Eigen::Vector3d& anchor_t_opt = anchor->T_map_b_optimized.translation();
        RCLCPP_INFO(log,
            "[SubMapMgr][POSE_DIAG] sm_id=%d rebuild 锚点: T_odom_b=[%.4f,%.4f,%.4f] T_map_b_optimized=[%.4f,%.4f,%.4f] delta_trans=%.4fm delta_rot=%.2fdeg",
            sm->id, anchor_t_odom.x(), anchor_t_odom.y(), anchor_t_odom.z(),
            anchor_t_opt.x(), anchor_t_opt.y(), anchor_t_opt.z(), delta_trans_norm, delta_rot_deg);

        // 重建 merged_cloud：先清空，然后用优化位姿重投影
        sm->merged_cloud = std::make_shared<CloudXYZI>();

        size_t kf_log_step = std::max<size_t>(1, sm->keyframes.size() / 3);
        size_t kf_idx = 0;
        for (const auto& kf : sm->keyframes) {
            if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;

            CloudXYZIPtr ds_body = downsampleKeyframeBodyForMerging_(kf);
            if (!ds_body || ds_body->empty()) continue;

            Eigen::Affine3f T_wf;
            T_wf.matrix() = kf->T_map_b_optimized.cast<float>().matrix();

            CloudXYZIPtr world_cloud = std::make_shared<CloudXYZI>();
            try {
                pcl::transformPointCloud(*ds_body, *world_cloud, T_wf);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(log, "[SubMapMgr][REBUILD_MERGE] kf_id=%lu transform failed: %s", kf->id, e.what());
                continue;
            }

            if (world_cloud->empty()) continue;

            // [POSE_DIAG] 抽样：记录用于变换的 T_map_b_optimized
            if (kf_idx % kf_log_step == 0) {
                const auto& t = kf->T_map_b_optimized.translation();
                double yaw_deg = std::atan2(kf->T_map_b_optimized.rotation()(1, 0), kf->T_map_b_optimized.rotation()(0, 0)) * 180.0 / M_PI;
                RCLCPP_INFO(log,
                    "[SubMapMgr][POSE_DIAG]   sm_id=%d kf_id=%lu 变换使用 T_map_b_optimized=[%.4f,%.4f,%.4f] yaw=%.2fdeg body_pts=%zu ds_body=%zu",
                    sm->id, kf->id, t.x(), t.y(), t.z(), yaw_deg, kf->cloud_body->size(), ds_body->size());
            }
            kf_idx++;

            size_t old_size = sm->merged_cloud->size();
            sm->merged_cloud->reserve(old_size + world_cloud->size());
            for (const auto& pt : world_cloud->points) {
                sm->merged_cloud->push_back(pt);
            }
        }

        // 降采样以控制大小
        if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
            CloudXYZIPtr ds = submap_merge_semantic_intensity_vote_
                ? utils::voxelDownsampleMajorityIntensity(sm->merged_cloud, static_cast<float>(merge_res_),
                                                            parallel_voxel_downsample_)
                : utils::voxelDownsample(sm->merged_cloud, static_cast<float>(merge_res_), parallel_voxel_downsample_);
            if (ds && !ds->empty()) {
                sm->merged_cloud->swap(*ds);
            }
        }

        RCLCPP_INFO(log, "[SubMapMgr][REBUILD_MERGE] sm_id=%d rebuilt merged_cloud: %zu points (anchor_delta: trans=%.3fm)",
            sm->id, sm->merged_cloud ? sm->merged_cloud->size() : 0,
            delta_trans_norm);

        // 与 LoopDetector 子图 TEASER 对齐：downsampled 必须对应当前 merged + pose_map_anchor_optimized（仅重建 merged 不刷新会导致初值/点云脱节）
        refreshDownsampledCloudFromMergedInAnchorFrame_(sm);
    }
    
    RCLCPP_INFO(log, "[SubMapMgr][REBUILD_MERGE] Done rebuilding merged_cloud for all submaps");
    const double done_ts = (node() && node()->get_clock()) ? node()->get_clock()->now().seconds() : 0.0;
    RCLCPP_INFO(log,
        "[SubMapMgr][GHOSTING_DIAG] rebuildMergedCloudFromOptimizedPoses done ts=%.3f (此后 buildGlobalMap 与 merged_cloud 均基于 T_map_b_optimized；grep GHOSTING_DIAG 查时间线)",
        done_ts);
    RCLCPP_INFO(log,
        "[GHOSTING_SOURCE] merged_cloud rebuild_done (fallback 路径若被使用将与 optimized_path 一致；若 build 在 rebuild_done 之前则可能用过旧 merged_cloud→重影)");
    if (backend_verbose_trace_) {
        RCLCPP_INFO(log, "[GHOSTING_TRACE] rebuildMergedCloudFromOptimizedPoses DONE (fallback 路径此后无重影；grep GHOSTING_TRACE 查证据链)");
    }
    // [RC5/RC6] 重建完成，cleared merged_cloud_dirty_ 标志，允许 buildGlobalMap fallback 路径使用
    merged_cloud_dirty_.store(false, std::memory_order_release);
}

void SubMapManager::syncOptimizedPosesFromLastHbaWriteback() {
    std::lock_guard<std::mutex> lk(mutex_);
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");

    size_t n_synced = 0;
    auto syncKeyframes = [&n_synced](const SubMap::Ptr& sm) {
        if (!sm) return;
        for (auto& kf : sm->keyframes) {
            if (!kf || !kf->hba_pose_valid) continue;
            kf->T_map_b_optimized = kf->T_map_b_hba;
            ++n_synced;
        }
    };
    for (auto& sm : submaps_) syncKeyframes(sm);
    syncKeyframes(active_submap_);

    if (n_synced == 0) {
        RCLCPP_INFO(log,
            "[SubMapMgr][SAVE_SYNC] syncOptimizedPosesFromLastHbaWriteback: no kf with hba_pose_valid, skip "
            "(global_map_final uses current T_map_b_optimized)");
        return;
    }

    auto resyncSubmapRelative = [](const SubMap::Ptr& sm) {
        if (!sm || sm->keyframes.empty()) return;
        KeyFrame::Ptr anchor = sm->keyframes.front();
        if (!anchor || !anchor->T_map_b_optimized.matrix().allFinite()) return;
        sm->pose_map_anchor_optimized = anchor->T_map_b_optimized;
        sm->pose_frame = anchor->pose_frame;
        const Pose3d inv_a = sm->pose_map_anchor_optimized.inverse();
        if (!inv_a.matrix().allFinite()) return;
        for (auto& kf : sm->keyframes) {
            if (!kf || !kf->T_map_b_optimized.matrix().allFinite()) continue;
            kf->T_submap_kf = inv_a * kf->T_map_b_optimized;
        }
    };
    for (auto& sm : submaps_) resyncSubmapRelative(sm);
    resyncSubmapRelative(active_submap_);

    ++current_map_version_;
    cached_global_map_.reset();
    last_build_map_version_ = std::numeric_limits<uint64_t>::max();
    merged_cloud_dirty_.store(true, std::memory_order_release);

    RCLCPP_INFO(log,
        "[SubMapMgr][SAVE_SYNC] syncOptimizedPosesFromLastHbaWriteback: kf_synced=%zu map_version=%lu "
        "(T_map_b_optimized<-T_map_b_hba; anchors/T_submap_kf resynced; cache invalidated for global_map_final)",
        n_synced, current_map_version_);
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

CloudXYZIPtr SubMapManager::buildGlobalMap(float voxel_size, uint64_t alignment_epoch_limit) const {
    const unsigned tid = automap_pro::logThreadId();
    // 使用全局 logger 名称，避免 backend 线程中解引用 node_（可能析构顺序导致 use-after-free）
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    
    std::unique_lock<std::mutex> lk(mutex_);
    
    // 🏛️ [P0 架构优化] 全局地图缓存检查
    // 如果关键帧总数、位姿版本号、体素大小、纪元限制均未变化，则直接返回缓存，避免高频冗余构建导致的性能卡顿
    size_t current_kf_total = 0;
    for (const auto& sm : submaps_) if(sm) current_kf_total += sm->keyframes.size();
    
    if (cached_global_map_ && !cached_global_map_->empty() &&
        last_build_kf_count_ == current_kf_total &&
        last_build_map_version_ == current_map_version_ &&
        std::abs(last_build_voxel_size_ - voxel_size) < 1e-4) {
        // 注意：缓存时也需要考虑 alignment_epoch_limit，如果 limit 变化，缓存失效
        // 但目前 buildGlobalMap 通常由可视化触发，limit 通常跟随 MapRegistry 变化，
        // 而 MapRegistry 变化会触发 invalidateGlobalMapCache 或 update current_map_version_。
        RCLCPP_DEBUG(log, "[SubMapMgr][CACHE] buildGlobalMap hit: kf=%zu ver=%lu voxel=%.3f (skip redundant build)",
                    current_kf_total, current_map_version_, voxel_size);
        return cached_global_map_;
    }

    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMap_enter voxel_size={:.3f} epoch_limit={}", tid, voxel_size, alignment_epoch_limit);
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG][HBA_GHOSTING] buildGlobalMap enter voxel_size=%.3f epoch_limit=%lu 主路径=从 kf->cloud_body 用 T_map_b_optimized 变换；T_map_b_optimized 未优化时直接退出程序（正常建图不应出现）", voxel_size, static_cast<unsigned long>(alignment_epoch_limit));
    
    const size_t num_submaps = submaps_.size();
    if (backend_verbose_trace_) {
        RCLCPP_INFO(log, "[GHOSTING_TRACE] buildGlobalMap ENTER voxel=%.3f submap_count=%zu (主路径=cloud_body+T_odom_b_opt 无重影；fallback=merged_cloud 若未rebuild则有重影)",
            voxel_size, num_submaps);
    }
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
    size_t kf_skipped_epoch = 0;
    // T_map_b_optimized 未优化时已 std::abort()，不会产生「跳过」统计
    int subs_with_kf = 0;
    size_t total_pts_before_transform = 0;
    
    // [GLOBAL_MAP_DIAG] 增强：记录各子图锚点的优化位姿状态
    for (const auto& sm : submaps_) {
        if (!sm) continue;

        // 🏛️ [对齐纪元] 子图级过滤
        if (alignment_epoch_limit > 0 && sm->alignment_epoch < alignment_epoch_limit) {
            kf_skipped_epoch += sm->keyframes.size();
            continue;
        }

        const auto& anchor_pose = sm->pose_map_anchor_optimized;
        RCLCPP_INFO(log, "[SubMapMgr][GLOBAL_MAP_DIAG] SM#%d anchor_pose: trans=[%.2f,%.2f,%.2f] state=%d kfs=%zu epoch=%lu",
            sm->id, anchor_pose.translation().x(), anchor_pose.translation().y(), anchor_pose.translation().z(),
            static_cast<int>(sm->state), sm->keyframes.size(), sm->alignment_epoch);
    }
    
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] ┌─ 主路径: 从关键帧重算（使用 T_map_b_optimized） epoch_limit=%lu", static_cast<unsigned long>(alignment_epoch_limit));
    
    for (size_t idx = 0; idx < num_submaps && !hit_limit; ++idx) {
        const auto& sm = submaps_[idx];
        if (!sm) {
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] submap[%zu] is null, skip", idx);
            continue;
        }

        // 🏛️ [对齐纪元] 子图级过滤（双重保险）
        if (alignment_epoch_limit > 0 && sm->alignment_epoch < alignment_epoch_limit) {
            continue;
        }
        
        size_t sm_pts = 0;
        size_t sm_kf_count = 0;
        size_t sm_kf_valid = 0;
        
        RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: %zu keyframes, pose_map_anchor_optimized=[%.2f,%.2f,%.2f] epoch=%lu",
            sm->id, sm->keyframes.size(),
            sm->pose_map_anchor_optimized.translation().x(),
            sm->pose_map_anchor_optimized.translation().y(),
            sm->pose_map_anchor_optimized.translation().z(),
            sm->alignment_epoch);
        
        for (const auto& kf : sm->keyframes) {
            sm_kf_count++;
            
            if (!kf) {
                kf_skipped_null++;
                RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ │ kf[%zu] is null", sm_kf_count - 1);
                continue;
            }

            // 🏛️ [对齐纪元] 关键帧级过滤
            if (alignment_epoch_limit > 0 && kf->alignment_epoch < alignment_epoch_limit) {
                kf_skipped_epoch++;
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
            // 【关键】T_map_b_optimized 未优化出结果时：正常建图不应出现，直接报严重错误并退出程序（不回退 T_odom_b）
            Pose3d T_odom_b_used = kf->T_map_b_optimized;
            if (T_odom_b_used.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) &&
                !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6)) {
                RCLCPP_ERROR(log,
                    "[T_map_b_optimized_UNOPT] 严重错误: kf_id=%lu sm_id=%d T_map_b_optimized=Identity 未优化出结果，正常建图不应出现，程序即将退出（请检查 HBA/写回是否覆盖该关键帧）",
                    kf->id, sm->id);
                std::abort();
            }

            // [POSE_DIAG] 主路径位姿使用：每子图首帧 + 每 20 帧抽样，记录使用的位姿来源与数值
            const bool using_optimized = true;
            const bool log_pose = (sm_kf_valid == 1) || (sm_kf_valid % 20 == 0);
            if (log_pose) {
                const auto& t_used = T_odom_b_used.translation();
                double yaw_used = std::atan2(T_odom_b_used.rotation()(1, 0), T_odom_b_used.rotation()(0, 0)) * 180.0 / M_PI;
                const auto& t_odom = kf->T_odom_b.translation();
                const auto& t_opt = kf->T_map_b_optimized.translation();
                double diff_odom_used = (t_used - t_odom).norm();
                double diff_opt_used = (t_used - t_opt).norm();
                RCLCPP_INFO(log,
                    "[SubMapMgr][POSE_DIAG] buildGlobalMap kf_id=%lu sm_id=%d pose_source=%s trans_used=[%.4f,%.4f,%.4f] yaw_used=%.2fdeg | T_odom_b=[%.4f,%.4f,%.4f] T_odom_b_opt=[%.4f,%.4f,%.4f] diff_to_odom=%.4fm diff_to_opt=%.4fm",
                    kf->id, sm->id, using_optimized ? "T_map_b_optimized" : "T_odom_b_fallback",
                    t_used.x(), t_used.y(), t_used.z(), yaw_used,
                    t_odom.x(), t_odom.y(), t_odom.z(), t_opt.x(), t_opt.y(), t_opt.z(), diff_odom_used, diff_opt_used);
            }

            // 变换点云到世界系
            Eigen::Affine3f T_wf;
            T_wf.matrix() = T_odom_b_used.cast<float>().matrix();
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
                T_odom_b_used.translation().x(), T_odom_b_used.translation().y(), T_odom_b_used.translation().z());
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
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG]   • 统计: kf_skipped_null=%zu, kf_skipped_empty=%zu, kf_skipped_epoch=%zu",
        kf_skipped_null, kf_skipped_empty, kf_skipped_epoch);
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG]   • 输入点数 (body系): %zu, 输出点数 (world系): %zu",
        total_pts_before_transform, combined->size());
    // 若主路径无关键帧点云，记录警告并尝试回退
    if (combined->empty()) {
        if (retain_cloud_body_) {
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
                "   Using merged_cloud (built with T_odom_b, not T_map_b_optimized)\n"
                "   After optimization, this may cause misalignment with trajectory\n"
                "   Recommendation: Set retain_cloud_body=true if precision is critical",
                kf_skipped_null, kf_skipped_empty);
            METRICS_INCREMENT(metrics::FALLBACK_TO_MERGED_CLOUD);
        }

        used_fallback_path = true;  // 供 [GLOBAL_MAP_BLUR] 汇总判断
        if (backend_verbose_trace_) {
            RCLCPP_INFO(log, "[GHOSTING_TRACE] buildGlobalMap PATH=fallback kf_skipped_null=%zu kf_skipped_empty=%zu (merged_cloud若未rebuild→重影风险；grep GHOSTING_TRACE)",
                kf_skipped_null, kf_skipped_empty);
        }
        RCLCPP_WARN(log, "[GHOSTING_RISK] buildGlobalMap_sync path=fallback_merged_cloud (主路径 combined 为空；merged_cloud 若未在 HBA 后 rebuild 则为 T_odom_b 系→重影；grep REBUILD_MERGE)");
        RCLCPP_WARN(log, "[GHOSTING_SOURCE] buildGlobalMap path=fallback_merged_cloud (主路径 combined 为空；merged_cloud 若未在 HBA 后 rebuild 则为 T_odom_b 系→与 optimized_path 重影，grep REBUILD_MERGE 查是否已重建)");
        RCLCPP_WARN(log, "[SubMapMgr][HBA_GHOSTING] buildGlobalMap 使用 fallback_merged_cloud：主路径 combined 为空，用各子图 merged_cloud 拼接；若 merged_cloud 未经 rebuildMergedCloudFromOptimizedPoses 则仍为 T_odom_b 世界系，与轨迹重影");
        RCLCPP_WARN(log, "[GLOBAL_MAP_DIAG] ┌─ 回退路径: 拼接 merged_cloud (旧世界系，可能不准确)");
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] path=fallback_merged_cloud (⚠️  if shown, global_map may be misaligned with optimized trajectory)");

        // [RC5/RC6] merged_cloud_dirty_ 为 true 表示重建尚未完成，用旧 merged_cloud 必然重影，直接跳过 fallback
        if (merged_cloud_dirty_.load(std::memory_order_acquire)) {
            RCLCPP_WARN(log,
                "[GHOSTING_GUARD] buildGlobalMap fallback_merged_cloud SKIPPED: merged_cloud_dirty_=true "
                "(rebuildMergedCloudFromOptimizedPoses 尚未完成，返回空云以防止重影; "
                "下一次 build 在 rebuild 完成后可正常拼接)");
            lk.unlock();
            return std::make_shared<CloudXYZI>(); // 返回空云，RViz 不更新，优于显示错位重影
        }
        
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
            
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: added %zu pts from merged_cloud (built with T_odom_b, not optimized)",
                sm->id, add_size);
        }
        
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] └─ 回退完成: combined_pts=%zu", combined->size());
    } else if (!combined->empty()) {
        if (backend_verbose_trace_) {
            RCLCPP_INFO(log, "[GHOSTING_TRACE] buildGlobalMap PATH=main combined_pts=%zu kf_used=%zu (pose=T_map_b_optimized 无重影)",
                combined->size(), kf_used_total);
        }
        RCLCPP_INFO(log,
            "[SubMapMgr][GHOSTING_DIAG] buildGlobalMap sync path=main combined_pts=%zu (位姿=现场读 T_map_b_optimized，与 optimized_path 同系；grep GHOSTING_DIAG)",
            combined->size());
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
                const bool blur_risk = used_fallback_path || (comp_pct < 5.0) || (vs > 0.3f);
                RCLCPP_INFO(log, "[GLOBAL_MAP_BLUR] path=%s voxel=%.3f combined=%zu out=%zu comp_pct=%.1f%% blur_risk=%s",
                    used_fallback_path ? "fallback" : "from_kf", vs, combined->size(), out->size(), comp_pct, blur_risk ? "yes" : "no");
                if (blur_risk) {
                    RCLCPP_WARN(log,
                        "[GLOBAL_MAP_BLUR] 存在模糊风险: %s%s%s → 见 docs/GLOBAL_MAP_BLUR_ANALYSIS.md",
                        used_fallback_path ? "path=fallback " : "",
                        (comp_pct < 5.0) ? "下采样过狠 " : "",
                        (vs > 0.3f) ? "体素过大 " : "");
                }
            }
            // [GHOSTING_RISK] 重影风险单行汇总（未优化 KF 会直接 abort，此处仅 path 风险）
            if (used_fallback_path) {
                RCLCPP_WARN(log, "[GHOSTING_RISK] buildGlobalMap_sync path=fallback_merged_cloud (grep GHOSTING_RISK)");
            } else {
                RCLCPP_INFO(log, "[GHOSTING_RISK] buildGlobalMap_sync path=from_kf (无重影风险)");
            }

            // 更新缓存
            cached_global_map_ = out;
            last_build_kf_count_ = current_kf_total;
            last_build_map_version_ = current_map_version_;
            last_build_voxel_size_ = voxel_size;

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
        meta["alignment_epoch"] = submap->alignment_epoch; // 🏛️ [对齐纪元] 持久化

        const Pose3d& T = submap->pose_map_anchor_optimized;
        Eigen::Quaterniond q(T.rotation());
        meta["anchor_pose"] = {
            {"px", T.translation().x()}, {"py", T.translation().y()}, {"pz", T.translation().z()},
            {"qx", q.x()}, {"qy", q.y()}, {"qz", q.z()}, {"qw", q.w()}
        };
        if (submap->has_valid_gps) {
            meta["gps_center"] = {submap->gps_center.x(), submap->gps_center.y(), submap->gps_center.z()};
        }

        // 🏛️ [产品化增强] 保存语义地标 (Landmarks)
        json landmarks_json = json::array();
        for (const auto& l : submap->landmarks) {
            if (!l) continue;
            json l_json;
            l_json["id"] = l->id;
            l_json["root"] = {l->root.x(), l->root.y(), l->root.z()};
            l_json["ray"] = {l->ray.x(), l->ray.y(), l->ray.z()};
            l_json["radius"] = l->radius;
            l_json["confidence"] = l->confidence;
            landmarks_json.push_back(l_json);
        }
        meta["landmarks"] = landmarks_json;
        SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][archiveSubmap] step=landmarks_saved sm_id={} count={}",
                   submap->id, landmarks_json.size());

        // ========== [增量建图] 保存子图内所有关键帧信息 ==========
        std::vector<uint64_t> kf_ids;
        std::string kf_dir = subdir + "/keyframes";
        fs::create_directories(kf_dir);
        
        for (const auto& kf : submap->keyframes) {
            if (!kf) continue;
            kf_ids.push_back(kf->id);
            
            json kf_meta;
            kf_meta["id"] = kf->id;
            kf_meta["ts"] = kf->timestamp;
            kf_meta["submap_id"] = kf->submap_id;
            kf_meta["alignment_epoch"] = kf->alignment_epoch; // 🏛️ [对齐纪元] 持久化
            
            // 位姿
            auto save_pose = [](const Pose3d& T) {
                Eigen::Quaterniond q(T.rotation());
                return json{
                    {"px", T.translation().x()}, {"py", T.translation().y()}, {"pz", T.translation().z()},
                    {"qx", q.x()}, {"qy", q.y()}, {"qz", q.z()}, {"qw", q.w()}
                };
            };
            kf_meta["T_odom_b"] = save_pose(kf->T_odom_b);
            kf_meta["T_map_b_optimized"] = save_pose(kf->T_map_b_optimized);
            kf_meta["T_submap_kf"] = save_pose(kf->T_submap_kf);
            
            // GPS (如果有)
            if (kf->has_valid_gps) {
                kf_meta["gps"] = {
                    {"lat", kf->gps.latitude}, {"lon", kf->gps.longitude}, {"alt", kf->gps.altitude},
                    {"hdop", kf->gps.hdop}
                };
            }

            // 保存关键帧元数据
            std::string kf_base = kf_dir + "/kf_" + std::to_string(kf->id);
            std::ofstream kf_ofs(kf_base + ".json");
            kf_ofs << kf_meta.dump(2);
            
            // 保存关键帧原始点云 (cloud_body)
            if (kf->cloud_body && !kf->cloud_body->empty()) {
                pcl::io::savePCDFileBinary(kf_base + ".pcd", *kf->cloud_body);
            }
        }
        meta["keyframe_ids"] = kf_ids;
        // ========================================================

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
        SLOG_INFO(MOD, "[SEMANTIC][SubMapMgr][archiveSubmap] step=ok sm_id={} dir={} landmarks={}",
                  submap->id, subdir, submap->landmarks.size());
        return true;
    } catch (const std::exception& e) {
        SLOG_ERROR(MOD, "[SEMANTIC][SubMapMgr][archiveSubmap] step=FAILED sm_id={} error={}", submap->id, e.what());
        return false;
    }
}

bool SubMapManager::loadArchivedSubmap(const std::string& dir, int submap_id, SubMap::Ptr& out) {
    std::string subdir = dir + "/submap_" + std::to_string(submap_id);
    std::string meta_path = subdir + "/submap_meta.json";
    if (!fs::exists(meta_path)) {
        SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][loadArchivedSubmap] step=skip reason=meta_not_found path={}", meta_path);
        return false;
    }
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
        sm->alignment_epoch = meta.value("alignment_epoch", 0ULL); // 🏛️ [对齐纪元] 加载
        sm->state = SubMapState::ARCHIVED;

        if (meta.contains("anchor_pose")) {
            const auto& ap = meta["anchor_pose"];
            Eigen::Quaterniond q(ap.value("qw", 1.0), ap.value("qx", 0.0), ap.value("qy", 0.0), ap.value("qz", 0.0));
            sm->pose_map_anchor_optimized = sm->pose_odom_anchor = Pose3d::Identity();
            sm->pose_map_anchor_optimized.linear() = q.toRotationMatrix();
            sm->pose_map_anchor_optimized.translation() << ap.value("px", 0.0), ap.value("py", 0.0), ap.value("pz", 0.0);
            sm->pose_odom_anchor = sm->pose_map_anchor_optimized;
            sm->pose_frame = PoseFrame::MAP;
        }
        if (meta.contains("gps_center") && meta["gps_center"].is_array() && meta["gps_center"].size() >= 3) {
            sm->gps_center << meta["gps_center"][0], meta["gps_center"][1], meta["gps_center"][2];
        }

        // 🏛️ [产品化增强] 加载语义地标 (Landmarks)
        if (meta.contains("landmarks") && meta["landmarks"].is_array()) {
            const auto& landmarks_arr = meta["landmarks"];
            for (const auto& l_json : landmarks_arr) {
                auto l = std::make_shared<CylinderLandmark>();
                l->id = l_json.value("id", 0ULL);
                l->submap_id = sm->id;
                if (l_json.contains("root") && l_json["root"].is_array() && l_json["root"].size() >= 3) {
                    l->root << l_json["root"][0], l_json["root"][1], l_json["root"][2];
                }
                if (l_json.contains("ray") && l_json["ray"].is_array() && l_json["ray"].size() >= 3) {
                    l->ray << l_json["ray"][0], l_json["ray"][1], l_json["ray"][2];
                }
                l->radius = l_json.value("radius", 0.1);
                l->confidence = l_json.value("confidence", 0.0);
                sm->landmarks.push_back(l);
            }
            uint64_t max_landmark_id = 0;
            for (const auto& lm : sm->landmarks) {
                if (lm && lm->id > max_landmark_id) {
                    max_landmark_id = lm->id;
                }
            }
            if (max_landmark_id > 0) {
                seedLandmarkIdSeq(max_landmark_id + 1);
            }
            SLOG_INFO(MOD, "[SEMANTIC][SubMapMgr][loadArchivedSubmap] step=landmarks_loaded sm_id={} count={}",
                      submap_id, sm->landmarks.size());
        } else {
            SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][loadArchivedSubmap] step=no_landmarks sm_id={} (metadata missing or empty)",
                       submap_id);
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

        // ========== [对齐纪元] 加载子图内所有关键帧 ==========
        if (meta.contains("keyframe_ids") && meta["keyframe_ids"].is_array()) {
            std::string kf_dir = subdir + "/keyframes";
            for (const auto& kf_id_json : meta["keyframe_ids"]) {
                uint64_t kf_id = kf_id_json.get<uint64_t>();
                std::string kf_base = kf_dir + "/kf_" + std::to_string(kf_id);
                std::string kf_meta_path = kf_base + ".json";
                if (!fs::exists(kf_meta_path)) continue;

                try {
                    std::ifstream kf_ifs(kf_meta_path);
                    json kf_meta;
                    kf_ifs >> kf_meta;

                    auto kf = std::make_shared<KeyFrame>();
                    kf->id = kf_meta.value("id", 0ULL);
                    kf->timestamp = kf_meta.value("ts", 0.0);
                    kf->submap_id = kf_meta.value("submap_id", sm->id);
                    kf->alignment_epoch = kf_meta.value("alignment_epoch", 0ULL); // 🏛️ [对齐纪元] 加载

                    auto load_pose = [](const json& p_json) {
                        Pose3d T = Pose3d::Identity();
                        if (p_json.is_object()) {
                            Eigen::Quaterniond q(p_json.value("qw", 1.0), p_json.value("qx", 0.0), 
                                                p_json.value("qy", 0.0), p_json.value("qz", 0.0));
                            T.linear() = q.toRotationMatrix();
                            T.translation() << p_json.value("px", 0.0), p_json.value("py", 0.0), p_json.value("pz", 0.0);
                        }
                        return T;
                    };

                    kf->T_odom_b = load_pose(kf_meta["T_odom_b"]);
                    kf->T_map_b_optimized = load_pose(kf_meta["T_map_b_optimized"]);
                    kf->T_submap_kf = load_pose(kf_meta["T_submap_kf"]);
                    kf->pose_frame = sm->pose_frame;

                    if (kf_meta.contains("gps")) {
                        const auto& g = kf_meta["gps"];
                        kf->gps.latitude = g.value("lat", 0.0);
                        kf->gps.longitude = g.value("lon", 0.0);
                        kf->gps.altitude = g.value("alt", 0.0);
                        kf->gps.hdop = g.value("hdop", 1.0);
                        kf->has_valid_gps = true;
                    }

                    std::string kf_pcd = kf_base + ".pcd";
                    if (fs::exists(kf_pcd)) {
                        kf->cloud_body = std::make_shared<CloudXYZI>();
                        pcl::io::loadPCDFile(kf_pcd, *kf->cloud_body);
                    }

                    sm->keyframes.push_back(kf);
                } catch (...) {
                    continue;
                }
            }
            SLOG_INFO(MOD, "[SubMapMgr][loadArchivedSubmap] sm_id={} loaded_keyframes={}", sm->id, sm->keyframes.size());
        }

        out = sm;
        SLOG_INFO(MOD, "[SEMANTIC][SubMapMgr][loadArchivedSubmap] step=ok sm_id={} landmarks={} dir={}",
                  submap_id, sm->landmarks.size(), subdir);
        return true;
    } catch (const std::exception& e) {
        SLOG_ERROR(MOD, "[SEMANTIC][SubMapMgr][loadArchivedSubmap] step=FAILED sm_id={} error={}", submap_id, e.what());
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
        active_submap_->gps_enu_pose = Pose3d::Identity();
        active_submap_->gps_enu_pose.translation() = active_submap_->gps_center;
        
        // 简单起见，使用最后一帧的 GPS 协方差作为子图的 GPS 协方差
        // 或者计算所有有效帧的平均协方差
        Eigen::Matrix3d cov_sum = Eigen::Matrix3d::Zero();
        for (const auto& f : active_submap_->keyframes) {
            if (f->has_valid_gps) {
                cov_sum += f->gps.covariance;
            }
        }
        active_submap_->gps_cov = cov_sum / static_cast<double>(gps_count);
        active_submap_->has_valid_gps = true;
        
        SLOG_DEBUG(MOD, "Updated GPS center for SM#{} ({} GPS fixes): ({:.3f}, {:.3f}, {:.3f})",
                     active_submap_->id, gps_count,
                     active_submap_->gps_center.x(),
                     active_submap_->gps_center.y(),
                     active_submap_->gps_center.z());
    }
}

void SubMapManager::associateLandmarks(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) {
    if (!sm || !kf) {
        SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][associateLandmarks] step=skip reason=sm_or_kf_null");
        return;
    }
    if (kf->landmarks.empty() && kf->plane_landmarks.empty()) {
        SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][associateLandmarks] step=skip kf_id={} sm_id={} reason=no_landmarks",
                   kf->id, sm->id);
        return;
    }

    SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][associateLandmarks] step=entry kf_id={} sm_id={} kf_trees={} sm_trees={} kf_planes={} sm_planes={}",
               kf->id, sm->id, kf->landmarks.size(), sm->landmarks.size(),
               kf->plane_landmarks.size(), sm->plane_landmarks.size());

    std::lock_guard<std::mutex> lk(mutex_);

    // T_submap_kf 是 KF 在 Submap frame 锚点下的位姿
    Pose3d T_s_kf = kf->T_submap_kf;
    if (!T_s_kf.matrix().allFinite()) {
        SLOG_WARN(MOD, "[SEMANTIC][SubMapMgr][associateLandmarks] step=skip reason=invalid_T_s_kf kf_id={} sm_id={}",
                  kf->id, sm->id);
        return;
    }

    size_t n_matched_tree = 0;
    size_t n_new_tree = 0;
    size_t n_protection_suppressed_tree = 0;
    double sm_min_x = std::numeric_limits<double>::infinity();
    double sm_min_y = std::numeric_limits<double>::infinity();
    double sm_max_x = -std::numeric_limits<double>::infinity();
    double sm_max_y = -std::numeric_limits<double>::infinity();
    for (const auto& l : sm->landmarks) {
        if (!l || !l->isValid()) continue;
        sm_min_x = std::min(sm_min_x, l->root.x());
        sm_min_y = std::min(sm_min_y, l->root.y());
        sm_max_x = std::max(sm_max_x, l->root.x());
        sm_max_y = std::max(sm_max_y, l->root.y());
    }
    const double sm_area = (std::isfinite(sm_min_x) && std::isfinite(sm_min_y) && std::isfinite(sm_max_x) && std::isfinite(sm_max_y))
        ? std::max(1e-3, (sm_max_x - sm_min_x) * (sm_max_y - sm_min_y))
        : 1.0;
    const double duplicate_density_now =
        static_cast<double>(assoc_tree_duplicate_merge_total_.load(std::memory_order_relaxed)) / sm_area;
    const uint64_t tree_m_curr = assoc_tree_matched_total_.load(std::memory_order_relaxed);
    const uint64_t tree_n_curr = assoc_tree_new_total_.load(std::memory_order_relaxed);
    const double tree_new_rate_curr = (tree_m_curr + tree_n_curr) > 0
        ? (100.0 * static_cast<double>(tree_n_curr) / static_cast<double>(tree_m_curr + tree_n_curr))
        : 0.0;
    if (assoc_cyl_protection_mode_enabled_) {
        if (!assoc_cyl_protection_mode_active_ &&
            (tree_new_rate_curr >= assoc_cyl_protect_trigger_tree_new_rate_pct_ ||
             duplicate_density_now >= assoc_cyl_protect_trigger_duplicate_density_)) {
            assoc_cyl_protection_mode_active_ = true;
            SLOG_WARN(MOD,
                      "[SEMANTIC][SubMapMgr][protection_mode] action=enter sm_id={} tree_new_rate={:.2f}% duplicate_density={:.4f}/m2",
                      sm->id, tree_new_rate_curr, duplicate_density_now);
        } else if (assoc_cyl_protection_mode_active_ &&
                   tree_new_rate_curr <= assoc_cyl_protect_recover_tree_new_rate_pct_ &&
                   duplicate_density_now <= assoc_cyl_protect_recover_duplicate_density_) {
            assoc_cyl_protection_mode_active_ = false;
            SLOG_INFO(MOD,
                      "[SEMANTIC][SubMapMgr][protection_mode] action=exit sm_id={} tree_new_rate={:.2f}% duplicate_density={:.4f}/m2",
                      sm->id, tree_new_rate_curr, duplicate_density_now);
        }
    }
    for (const auto& l_kf : kf->landmarks) {
        if (!l_kf) continue;
        
        // 将地标从 body 系转换到 submap 锚点系
        CylinderLandmark l_s;
        l_s.root = T_s_kf * l_kf->root;
        l_s.ray = (T_s_kf.rotation() * l_kf->ray).normalized();
        l_s.radius = l_kf->radius;
        l_s.confidence = l_kf->confidence;

        // 与子图中现有的地标进行关联（Mahalanobis-like probabilistic gating）
        bool associated = false;
        int matched_idx = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < sm->landmarks.size(); ++i) {
            auto& l_sm = sm->landmarks[i];
            if (!l_sm || !l_sm->isValid()) continue;

            const double dist_xy = (l_sm->root.head<2>() - l_s.root.head<2>()).norm();
            const double dist_z = std::abs(l_sm->root.z() - l_s.root.z());
            const double cos_theta = std::abs(l_sm->ray.dot(l_s.ray));
            const double angle = std::acos(std::min(1.0, cos_theta)) * 180.0 / M_PI;
            const double radius_diff = std::abs(l_sm->radius - l_s.radius);

            const double sx = std::max(1e-3, assoc_cyl_max_dist_xy_m_);
            const double sz = std::max(1e-3, assoc_cyl_max_dist_z_m_);
            const double sa = std::max(1e-3, assoc_cyl_max_angle_deg_);
            const double sr = std::max(1e-3, assoc_cyl_max_radius_diff_m_);
            const double maha_cost =
                (dist_xy * dist_xy) / (sx * sx) +
                (dist_z * dist_z) / (sz * sz) +
                (angle * angle) / (sa * sa) +
                (radius_diff * radius_diff) / (sr * sr);

            if (maha_cost >= assoc_cyl_mahalanobis_gate_) {
                continue;
            }
            if (maha_cost < best_cost) {
                best_cost = maha_cost;
                matched_idx = static_cast<int>(i);
                associated = true;
            }
        }

        if (associated && matched_idx >= 0) {
            auto& l_sm = sm->landmarks[static_cast<size_t>(matched_idx)];
            // 关联成功：加权平均更新地标参数
            double w_old = std::max(1e-3, l_sm->confidence);
            double w_new = std::max(1e-3, l_s.confidence);
            double total_w = w_old + w_new;
            double alpha = w_new / std::max(1e-3, total_w);
            alpha = std::clamp(alpha, assoc_cyl_alpha_min_, assoc_cyl_alpha_max_);

            l_sm->root = (1.0 - alpha) * l_sm->root + alpha * l_s.root;
            l_sm->ray = ((1.0 - alpha) * l_sm->ray + alpha * l_s.ray).normalized();
            l_sm->radius = (1.0 - alpha) * l_sm->radius + alpha * l_s.radius;
            l_sm->confidence = std::max(l_sm->confidence, l_s.confidence);
            l_sm->support_count += 1;
            const double geometric_consistency = 1.0 / (1.0 + best_cost);
            const double obs = 0.5 * std::clamp(l_s.confidence, 0.0, 1.0) + 0.5 * geometric_consistency;
            l_sm->observability_score = 0.9 * l_sm->observability_score + 0.1 * obs;

            if (static_cast<size_t>(matched_idx) < sm->landmarks.size()) {
                const auto& l_canon = sm->landmarks[static_cast<size_t>(matched_idx)];
                if (l_canon) {
                    l_kf->id = l_canon->id;
                }
            }
            l_kf->associated_idx = matched_idx;
            l_kf->submap_id = sm->id;
            n_matched_tree++;
        } else {
            // Hard guardrail: if a new observation is close to an existing tree under relaxed
            // duplicate criteria, force-attach instead of creating a new landmark. This prevents
            // "one real tree -> multiple map trees" from amplifying odometry drift.
            int relaxed_idx = -1;
            double relaxed_best = std::numeric_limits<double>::infinity();
            const double relaxed_xy = std::max(assoc_cyl_duplicate_merge_dist_xy_m_ * 1.8, assoc_cyl_max_dist_xy_m_);
            const double relaxed_angle_deg = std::max(assoc_cyl_duplicate_merge_max_angle_deg_ * 2.0, assoc_cyl_max_angle_deg_);
            for (size_t i = 0; i < sm->landmarks.size(); ++i) {
                const auto& l_exist = sm->landmarks[i];
                if (!l_exist || !l_exist->isValid()) continue;
                const double dxy = (l_exist->root.head<2>() - l_s.root.head<2>()).norm();
                if (dxy > relaxed_xy) continue;
                const double axis_cos = std::abs(l_exist->ray.normalized().dot(l_s.ray.normalized()));
                const double angle_deg = std::acos(std::min(1.0, axis_cos)) * 180.0 / M_PI;
                if (angle_deg > relaxed_angle_deg) continue;
                const double score = dxy + 0.05 * angle_deg;
                if (score < relaxed_best) {
                    relaxed_best = score;
                    relaxed_idx = static_cast<int>(i);
                }
            }
            if (relaxed_idx >= 0) {
                auto& l_exist = sm->landmarks[static_cast<size_t>(relaxed_idx)];
                if (l_exist) {
                    l_exist->support_count += 1;
                    l_exist->confidence = std::max(l_exist->confidence, l_s.confidence);
                    l_exist->observability_score = 0.95 * l_exist->observability_score + 0.05 * std::clamp(l_s.confidence, 0.0, 1.0);
                    l_kf->id = l_exist->id;
                    l_kf->associated_idx = relaxed_idx;
                    l_kf->submap_id = sm->id;
                    n_matched_tree++;
                    SLOG_DEBUG(MOD,
                               "[SEMANTIC][SubMapMgr][associateLandmarks] hard_guardrail_attach kf_id={} sm_id={} assoc_idx={} score={:.3f}",
                               kf->id, sm->id, relaxed_idx, relaxed_best);
                    continue;
                }
            }
            if (assoc_cyl_protection_mode_enabled_ && assoc_cyl_protection_mode_active_) {
                ++n_protection_suppressed_tree;
                SLOG_DEBUG(MOD,
                           "[SEMANTIC][SubMapMgr][protection_mode] suppress_new_tree kf_id={} sm_id={} reason=drift_risk",
                           kf->id, sm->id);
                continue;
            }
            // 未关联，作为新地标加入（id=进程唯一；associated_idx=子图内向量下标）
            if (l_kf->id == 0) {
                l_kf->id = allocateLandmarkId();
            }
            l_s.id = l_kf->id;
            l_s.support_count = 1;
            l_s.observability_score = std::clamp(l_s.confidence, 0.0, 1.0);
            auto new_l = std::make_shared<CylinderLandmark>(l_s);
            const int new_idx = static_cast<int>(sm->landmarks.size());
            l_kf->associated_idx = new_idx;
            l_kf->submap_id = sm->id;
            sm->landmarks.push_back(new_l);
            n_new_tree++;
        }
    }
    mergeDuplicateTreeLandmarks_(sm);

    // Plane landmarks association (body -> submap)
    size_t n_matched_plane = 0;
    size_t n_new_plane = 0;
    size_t n_protection_suppressed_plane = 0;
    for (const auto& p_kf : kf->plane_landmarks) {
        if (!p_kf || !p_kf->isValid()) continue;

        PlaneLandmark p_s;
        Eigen::Vector3d n_b = p_kf->normal;
        if (!n_b.allFinite() || n_b.norm() < 1e-6) continue;
        n_b.normalize();
        p_s.normal = (T_s_kf.rotation() * n_b).normalized();
        p_s.distance = p_kf->distance - p_s.normal.dot(T_s_kf.translation());
        p_s.confidence = p_kf->confidence;
        p_s.points = p_kf->points;
        // Propagate geometric extents into submap frame; they remain approximate
        // but are sufficient for backend gating (short/low planes like car bodies).
        p_s.vertical_span_m = p_kf->vertical_span_m;
        p_s.tangent_span_m = p_kf->tangent_span_m;

        // 计算观测平面在 submap 系下的质心，用于平面局部一致性约束
        Eigen::Vector3d centroid_s = Eigen::Vector3d::Zero();
        size_t centroid_cnt = 0;
        if (p_kf->points && !p_kf->points->empty()) {
            for (const auto& pt : p_kf->points->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                centroid_s += (T_s_kf * Eigen::Vector3d(pt.x, pt.y, pt.z));
                ++centroid_cnt;
            }
        }
        if (centroid_cnt == 0) {
            // 对于 n^T x + d = 0，取沿法向最近点 x0=-d*n 作为兜底参考点
            centroid_s = -p_s.distance * p_s.normal;
            centroid_cnt = 1;
        } else {
            centroid_s /= static_cast<double>(centroid_cnt);
        }

        bool associated = false;
        int matched_idx = -1;
        for (size_t i = 0; i < sm->plane_landmarks.size(); ++i) {
            auto& p_sm = sm->plane_landmarks[i];
            if (!p_sm || !p_sm->isValid()) continue;

            Eigen::Vector3d n_sm = p_sm->normal;
            if (!n_sm.allFinite() || n_sm.norm() < 1e-6) continue;
            n_sm.normalize();

            // 法向允许符号翻转（同一几何平面）
            double cos_theta = n_sm.dot(p_s.normal);
            double abs_cos = std::abs(cos_theta);
            double angle_deg = std::acos(std::min(1.0, abs_cos)) * 180.0 / M_PI;
            if (angle_deg >= assoc_plane_max_angle_deg_) continue;

            const double d_obs = (cos_theta >= 0.0) ? p_s.distance : -p_s.distance;
            const double d_diff = std::abs(p_sm->distance - d_obs);
            if (d_diff >= assoc_plane_max_distance_diff_m_) continue;

            // 子图已有平面的局部参考点
            Eigen::Vector3d sm_ref = Eigen::Vector3d::Zero();
            size_t sm_ref_cnt = 0;
            if (p_sm->points && !p_sm->points->empty()) {
                for (const auto& pt : p_sm->points->points) {
                    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                    sm_ref += Eigen::Vector3d(pt.x, pt.y, pt.z);
                    ++sm_ref_cnt;
                }
            }
            if (sm_ref_cnt == 0) sm_ref = -p_sm->distance * n_sm;
            else sm_ref /= static_cast<double>(sm_ref_cnt);

            // 投影到切平面后比较平面内平移，避免把平行但相距很远的结构误关联
            const Eigen::Vector3d d_vec = centroid_s - sm_ref;
            const Eigen::Vector3d d_tangent = d_vec - n_sm.dot(d_vec) * n_sm;
            if (d_tangent.norm() >= assoc_plane_max_tangent_offset_m_) continue;

            const double w_old = std::max(1e-3, p_sm->confidence);
            const double w_new = std::max(1e-3, p_s.confidence);
            const double total_w = w_old + w_new;
            double alpha = w_new / std::max(1e-6, total_w);
            alpha = std::clamp(alpha, assoc_plane_alpha_min_, assoc_plane_alpha_max_);

            Eigen::Vector3d n_obs = (cos_theta >= 0.0) ? p_s.normal : (-p_s.normal);
            p_sm->normal = ((1.0 - alpha) * n_sm + alpha * n_obs).normalized();
            p_sm->distance = (1.0 - alpha) * p_sm->distance + alpha * d_obs;
            p_sm->confidence = std::max(p_sm->confidence, p_s.confidence);
            p_sm->support_count += 1;
            p_sm->observability_score = 0.9 * p_sm->observability_score + 0.1 * std::clamp(p_s.confidence, 0.0, 1.0);
            if (!p_sm->points || p_sm->points->empty()) {
                p_sm->points = p_s.points;
            }

            matched_idx = static_cast<int>(i);
            associated = true;
            break;
        }

        if (associated && matched_idx >= 0) {
            if (static_cast<size_t>(matched_idx) < sm->plane_landmarks.size()) {
                const auto& p_canon = sm->plane_landmarks[static_cast<size_t>(matched_idx)];
                if (p_canon) {
                    p_kf->id = p_canon->id;
                }
            }
            p_kf->associated_idx = matched_idx;
            p_kf->submap_id = sm->id;
            n_matched_plane++;
        } else {
            // Hard guardrail (plane): relaxed attach before creating a new plane landmark.
            // This avoids fragmenting one stable wall/facade into many nearby landmarks.
            int relaxed_plane_idx = -1;
            double relaxed_plane_best = std::numeric_limits<double>::infinity();
            const double relaxed_angle_deg = std::max(assoc_plane_max_angle_deg_ * 1.8, assoc_plane_max_angle_deg_ + 2.0);
            const double relaxed_d_diff_m = std::max(assoc_plane_max_distance_diff_m_ * 1.8, assoc_plane_max_distance_diff_m_ + 0.15);
            const double relaxed_tangent_m = std::max(assoc_plane_max_tangent_offset_m_ * 1.8, assoc_plane_max_tangent_offset_m_ + 0.30);
            for (size_t i = 0; i < sm->plane_landmarks.size(); ++i) {
                const auto& p_exist = sm->plane_landmarks[i];
                if (!p_exist || !p_exist->isValid()) continue;

                Eigen::Vector3d n_sm = p_exist->normal;
                if (!n_sm.allFinite() || n_sm.norm() < 1e-6) continue;
                n_sm.normalize();

                const double cos_theta = n_sm.dot(p_s.normal);
                const double angle_deg = std::acos(std::min(1.0, std::abs(cos_theta))) * 180.0 / M_PI;
                if (angle_deg > relaxed_angle_deg) continue;

                const double d_obs = (cos_theta >= 0.0) ? p_s.distance : -p_s.distance;
                const double d_diff = std::abs(p_exist->distance - d_obs);
                if (d_diff > relaxed_d_diff_m) continue;

                Eigen::Vector3d sm_ref = Eigen::Vector3d::Zero();
                size_t sm_ref_cnt = 0;
                if (p_exist->points && !p_exist->points->empty()) {
                    for (const auto& pt : p_exist->points->points) {
                        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                        sm_ref += Eigen::Vector3d(pt.x, pt.y, pt.z);
                        ++sm_ref_cnt;
                    }
                }
                if (sm_ref_cnt == 0) sm_ref = -p_exist->distance * n_sm;
                else sm_ref /= static_cast<double>(sm_ref_cnt);

                const Eigen::Vector3d d_vec = centroid_s - sm_ref;
                const Eigen::Vector3d d_tangent = d_vec - n_sm.dot(d_vec) * n_sm;
                const double tangent_norm = d_tangent.norm();
                if (tangent_norm > relaxed_tangent_m) continue;

                const double score = d_diff + 0.05 * angle_deg + 0.5 * tangent_norm;
                if (score < relaxed_plane_best) {
                    relaxed_plane_best = score;
                    relaxed_plane_idx = static_cast<int>(i);
                }
            }
            if (relaxed_plane_idx >= 0) {
                auto& p_exist = sm->plane_landmarks[static_cast<size_t>(relaxed_plane_idx)];
                if (p_exist) {
                    p_exist->support_count += 1;
                    p_exist->confidence = std::max(p_exist->confidence, p_s.confidence);
                    p_exist->observability_score =
                        0.95 * p_exist->observability_score + 0.05 * std::clamp(p_s.confidence, 0.0, 1.0);
                    p_kf->id = p_exist->id;
                    p_kf->associated_idx = relaxed_plane_idx;
                    p_kf->submap_id = sm->id;
                    n_matched_plane++;
                    SLOG_DEBUG(MOD,
                               "[SEMANTIC][SubMapMgr][associateLandmarks] hard_guardrail_attach_plane kf_id={} sm_id={} assoc_idx={} score={:.3f}",
                               kf->id, sm->id, relaxed_plane_idx, relaxed_plane_best);
                    continue;
                }
            }
            if (assoc_cyl_protection_mode_enabled_ && assoc_cyl_protection_mode_active_) {
                ++n_protection_suppressed_plane;
                SLOG_DEBUG(MOD,
                           "[SEMANTIC][SubMapMgr][protection_mode] suppress_new_plane kf_id={} sm_id={} reason=drift_risk",
                           kf->id, sm->id);
                continue;
            }
            if (p_kf->id == 0) {
                p_kf->id = allocateLandmarkId();
            }
            p_s.id = p_kf->id;
            p_s.support_count = 1;
            p_s.observability_score = std::clamp(p_s.confidence, 0.0, 1.0);
            auto new_p = std::make_shared<PlaneLandmark>(p_s);
            const int new_idx = static_cast<int>(sm->plane_landmarks.size());
            p_kf->associated_idx = new_idx;
            p_kf->submap_id = sm->id;
            sm->plane_landmarks.push_back(new_p);
            n_new_plane++;
        }
    }

    assoc_tree_matched_total_.fetch_add(n_matched_tree, std::memory_order_relaxed);
    assoc_tree_new_total_.fetch_add(n_new_tree, std::memory_order_relaxed);
    assoc_plane_matched_total_.fetch_add(n_matched_plane, std::memory_order_relaxed);
    assoc_plane_new_total_.fetch_add(n_new_plane, std::memory_order_relaxed);

    const uint64_t tree_m = assoc_tree_matched_total_.load(std::memory_order_relaxed);
    const uint64_t tree_n = assoc_tree_new_total_.load(std::memory_order_relaxed);
    const uint64_t plane_m = assoc_plane_matched_total_.load(std::memory_order_relaxed);
    const uint64_t plane_n = assoc_plane_new_total_.load(std::memory_order_relaxed);
    const double tree_match_rate = (tree_m + tree_n) > 0 ? (100.0 * static_cast<double>(tree_m) / static_cast<double>(tree_m + tree_n)) : 0.0;
    const double tree_new_rate = (tree_m + tree_n) > 0 ? (100.0 * static_cast<double>(tree_n) / static_cast<double>(tree_m + tree_n)) : 0.0;
    const double plane_match_rate = (plane_m + plane_n) > 0 ? (100.0 * static_cast<double>(plane_m) / static_cast<double>(plane_m + plane_n)) : 0.0;
    const double plane_new_rate = (plane_m + plane_n) > 0 ? (100.0 * static_cast<double>(plane_n) / static_cast<double>(plane_m + plane_n)) : 0.0;

    SLOG_DEBUG(MOD, "[SEMANTIC][SubMapMgr][associateLandmarks] step=done kf_id={} sm_id={} tree_matched={} tree_new={} tree_suppressed_protection={} tree_total={} plane_matched={} plane_new={} plane_suppressed_protection={} plane_total={}",
               kf->id, sm->id, n_matched_tree, n_new_tree, n_protection_suppressed_tree, sm->landmarks.size(),
               n_matched_plane, n_new_plane, n_protection_suppressed_plane, sm->plane_landmarks.size());
    if ((tree_m + tree_n + plane_m + plane_n) % 50 == 0) {
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (const auto& l : sm->landmarks) {
            if (!l || !l->isValid()) continue;
            min_x = std::min(min_x, l->root.x());
            min_y = std::min(min_y, l->root.y());
            max_x = std::max(max_x, l->root.x());
            max_y = std::max(max_y, l->root.y());
        }
        const double area = (std::isfinite(min_x) && std::isfinite(min_y) && std::isfinite(max_x) && std::isfinite(max_y))
            ? std::max(1e-3, (max_x - min_x) * (max_y - min_y))
            : 1.0;
        const double duplicate_density = static_cast<double>(assoc_tree_duplicate_merge_total_.load(std::memory_order_relaxed)) / area;
        SLOG_INFO(MOD,
                  "[SEMANTIC][SubMapMgr][assoc_stats] tree(match/new)=({}/{}) tree_rates(match/new)=({:.1f}%/{:.1f}%) "
                  "plane(match/new)=({}/{}) plane_rates(match/new)=({:.1f}%/{:.1f}%) duplicate_density={:.4f}/m2 merged_total={} protection_mode_active={} suppressed_new_trees={} rule(tree_new_rate>35% && weak_loop_gps => drift_risk)",
                  tree_m, tree_n, tree_match_rate, tree_new_rate,
                  plane_m, plane_n, plane_match_rate, plane_new_rate,
                  duplicate_density,
                  assoc_tree_duplicate_merge_total_.load(std::memory_order_relaxed),
                  assoc_cyl_protection_mode_active_ ? 1 : 0,
                  n_protection_suppressed_tree);
    }
}

bool SubMapManager::isSemanticProtectionModeActive() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return assoc_cyl_protection_mode_enabled_ && assoc_cyl_protection_mode_active_;
}

void SubMapManager::mergeDuplicateTreeLandmarks_(SubMap::Ptr& sm) {
    if (!sm || sm->landmarks.size() < 2) {
        return;
    }
    const double max_xy = std::max(0.05, assoc_cyl_duplicate_merge_dist_xy_m_);
    const double cos_th = std::cos(std::max(1.0, assoc_cyl_duplicate_merge_max_angle_deg_) * M_PI / 180.0);
    size_t merged = 0;
    for (size_t i = 0; i < sm->landmarks.size(); ++i) {
        auto& a = sm->landmarks[i];
        if (!a || !a->isValid()) continue;
        for (size_t j = i + 1; j < sm->landmarks.size(); ++j) {
            auto& b = sm->landmarks[j];
            if (!b || !b->isValid()) continue;
            const double dxy = (a->root.head<2>() - b->root.head<2>()).norm();
            const double axis_cos = std::abs(a->ray.normalized().dot(b->ray.normalized()));
            if (dxy > max_xy || axis_cos < cos_th) {
                continue;
            }
            // Keep the more supported/observable landmark as canonical.
            auto keep_a = (a->support_count > b->support_count) ||
                          (a->support_count == b->support_count && a->observability_score >= b->observability_score);
            auto& keep = keep_a ? a : b;
            auto& drop = keep_a ? b : a;
            const double wa = std::max(1.0, static_cast<double>(keep->support_count));
            const double wb = std::max(1.0, static_cast<double>(drop->support_count));
            const double wsum = wa + wb;
            keep->root = (wa * keep->root + wb * drop->root) / wsum;
            keep->ray = (wa * keep->ray + wb * drop->ray).normalized();
            keep->radius = (wa * keep->radius + wb * drop->radius) / wsum;
            keep->confidence = std::max(keep->confidence, drop->confidence);
            keep->support_count += drop->support_count;
            keep->observability_score = std::max(keep->observability_score, drop->observability_score);
            drop.reset();
            ++merged;
        }
    }
    if (merged == 0) return;
    // Keep vector indices stable within current association pass.
    // Null entries are skipped by subsequent association/factor paths.
    assoc_tree_duplicate_merge_total_.fetch_add(static_cast<uint64_t>(merged), std::memory_order_relaxed);
}

void SubMapManager::publishEvent(const SubMap::Ptr& sm, const std::string& event) {
    if (!event_pub_ || !sm) return;

    auto msg = std::make_shared<automap_pro::msg::SubMapEventMsg>();
    msg->header.stamp = node()->now();
    msg->submap_id = sm->id;
    msg->session_id = sm->session_id;
    msg->event_type = event;
    msg->keyframe_count = static_cast<int>(sm->keyframes.size());
    msg->spatial_extent_m = sm->spatial_extent_m;
    msg->has_valid_gps = sm->has_valid_gps;

    const Pose3d& T = sm->pose_map_anchor_optimized;
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
    msg->header.stamp = node()->now();
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

// ── 异步构建实现（P0 优化 + 重影修复：位姿快照）────────────────────────────

namespace {
// 全局构建 ID，用于 GHOSTING_DIAG 串联同一次 build 的 snapshot/enter/exit，便于重影排查
std::atomic<uint64_t> g_build_global_map_id{0};
}  // namespace

void SubMapManager::invalidateGlobalMapCache() const {
    std::lock_guard<std::mutex> lk(mutex_);
    cached_global_map_.reset();
    last_build_map_version_ = std::numeric_limits<uint64_t>::max();
}

std::future<CloudXYZIPtr> SubMapManager::buildGlobalMapAsync(float voxel_size, uint64_t alignment_epoch_limit) const {
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    
    {
        std::lock_guard<std::mutex> lk(mutex_);
        size_t current_kf_total = 0;
        for (const auto& sm : submaps_) if(sm) current_kf_total += sm->keyframes.size();

        if (cached_global_map_ && !cached_global_map_->empty() &&
            last_build_kf_count_ == current_kf_total &&
            last_build_map_version_ == current_map_version_ &&
            std::abs(last_build_voxel_size_ - voxel_size) < 1e-4) {
            RCLCPP_DEBUG(log, "[SubMapMgr][CACHE] buildGlobalMapAsync hit: kf=%zu ver=%lu voxel=%.3f (return cached future)",
                        current_kf_total, current_map_version_, voxel_size);
            
            // 返回一个立即完成的 future
            std::promise<CloudXYZIPtr> promise;
            promise.set_value(cached_global_map_);
            return promise.get_future();
        }
    }

    return std::async(std::launch::async, [this, voxel_size, alignment_epoch_limit]() {
        const uint64_t build_id = ++g_build_global_map_id;
        const rclcpp::Logger log = rclcpp::get_logger("automap_system");
        
        // 持锁下快照 (cloud_body, pose)，避免异步任务中访问 sm->keyframes / kf->cloud_body 与后端并发修改导致 SIGSEGV（见 run full.log Thread automap_mappub SIGSEGV in buildGlobalMapInternal）
        std::vector<std::pair<CloudXYZIPtr, Pose3d>> cloud_pose_snapshot;
        size_t current_kf_total = 0;
        uint64_t current_ver = 0;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            current_ver = current_map_version_;
            cloud_pose_snapshot.reserve(512);
            for (const auto& sm : submaps_) {
                if (!sm) continue;

                // 🏛️ [对齐纪元] 子图级过滤
                if (alignment_epoch_limit > 0 && sm->alignment_epoch < alignment_epoch_limit) {
                    continue;
                }

                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;

                    // 🏛️ [对齐纪元] 关键帧级过滤
                    if (alignment_epoch_limit > 0 && kf->alignment_epoch < alignment_epoch_limit) {
                        continue;
                    }

                    const Pose3d& T = kf->T_map_b_optimized;
                    if (T.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) &&
                        !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6)) {
                        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                            "[T_map_b_optimized_UNOPT] 严重错误: build_id=%llu kf_id=%lu sm_id=%d T_map_b_optimized=Identity 未优化出结果，正常建图不应出现，程序即将退出（请检查 HBA/写回是否覆盖该关键帧）",
                            static_cast<unsigned long long>(build_id), kf->id, sm->id);
                        std::abort();
                    }
                    cloud_pose_snapshot.emplace_back(kf->cloud_body, T);
                }
            }
            current_kf_total = cloud_pose_snapshot.size();
        }
        
        const double snap_ts = (node() && node()->get_clock()) ? node()->get_clock()->now().seconds() : 0.0;
        if (!cloud_pose_snapshot.empty()) {
            const auto& first = cloud_pose_snapshot.front().second.translation();
            const auto& last = cloud_pose_snapshot.back().second.translation();
            RCLCPP_INFO(log,
                "[SubMapMgr][GHOSTING_DIAG] pose_snapshot_taken build_id=%llu ts=%.3f kf_count=%zu first_pos=[%.2f,%.2f,%.2f] last_pos=[%.2f,%.2f,%.2f] epoch_limit=%lu",
                static_cast<unsigned long long>(build_id), snap_ts, cloud_pose_snapshot.size(),
                first.x(), first.y(), first.z(), last.x(), last.y(), last.z(),
                static_cast<unsigned long>(alignment_epoch_limit));
            RCLCPP_INFO(log, "[GHOSTING_RISK] buildGlobalMap_async build_id=%llu path=snapshot pose_source=T_map_b_optimized_only (无重影风险)",
                static_cast<unsigned long long>(build_id));
        }
        
        CloudXYZIPtr res = buildGlobalMapInternalFromSnapshot(cloud_pose_snapshot, voxel_size, build_id);
        
        // 更新缓存
        if (res && !res->empty()) {
            std::lock_guard<std::mutex> lk(mutex_);
            cached_global_map_ = res;
            last_build_kf_count_ = current_kf_total;
            last_build_map_version_ = current_ver;
            last_build_voxel_size_ = voxel_size;
        }
        
        return res;
    });
}

CloudXYZIPtr SubMapManager::buildGlobalMapInternal(
    const std::vector<SubMap::Ptr>& submaps_copy,
    float voxel_size,
    const std::vector<Pose3d>* poses_snapshot,
    uint64_t build_id,
    uint64_t alignment_epoch_limit) const {
    const unsigned tid = automap_pro::logThreadId();
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    const bool use_snapshot = poses_snapshot && !poses_snapshot->empty();
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_enter voxel_size={:.3f} use_snapshot={} epoch_limit={}", 
              tid, voxel_size, use_snapshot ? "yes" : "no", alignment_epoch_limit);
    if (build_id != 0) {
        RCLCPP_INFO(log,
            "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_enter build_id=%llu use_snapshot=%s epoch_limit=%lu (grep GHOSTING_DIAG for timeline)",
            static_cast<unsigned long long>(build_id), use_snapshot ? "yes" : "no", 
            static_cast<unsigned long>(alignment_epoch_limit));
    }

    CloudXYZIPtr combined = std::make_shared<CloudXYZI>();
    CloudXYZIPtr world_tmp = std::make_shared<CloudXYZI>();
    size_t pose_idx = 0;
    size_t epoch_filtered = 0;

    for (const auto& sm : submaps_copy) {
        if (!sm) continue;
        
        // 🏛️ [对齐纪元] 子图级过滤：如果子图纪元落后且启用了过滤
        if (alignment_epoch_limit > 0 && sm->alignment_epoch < alignment_epoch_limit) {
            pose_idx += sm->keyframes.size(); // 必须推进索引以保持同步
            epoch_filtered += sm->keyframes.size();
            continue;
        }

        for (const auto& kf : sm->keyframes) {
            if (!kf || !kf->cloud_body || kf->cloud_body->empty()) {
                if (use_snapshot) pose_idx++;
                continue;
            }

            // 🏛️ [对齐纪元] 关键帧级过滤（双重保险）
            if (alignment_epoch_limit > 0 && kf->alignment_epoch < alignment_epoch_limit) {
                if (use_snapshot) pose_idx++;
                epoch_filtered++;
                continue;
            }

            Pose3d T_map_b;
            if (poses_snapshot && pose_idx < poses_snapshot->size()) {
                T_map_b = (*poses_snapshot)[pose_idx++];
            } else {
                T_map_b = kf->T_map_b_optimized;
                if (T_map_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) &&
                    !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6)) {
                    T_map_b = kf->T_odom_b;
                }
            }
            Eigen::Affine3f T_wf;
            T_wf.matrix() = T_map_b.cast<float>().matrix();
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

    // 🏛️ [架构优化] 在此处读取配置并显式向下传递，避免异步/OpenMP 线程内 ConfigManager 竞争导致 SIGSEGV
    const bool parallel = parallel_voxel_downsample_;
    CloudXYZIPtr out = utils::voxelDownsampleChunked(combined, vs, 50.0f, parallel);
    const size_t out_pts = out ? out->size() : combined->size();
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_exit out={} filtered_epoch={}", tid, out_pts, epoch_filtered);
    if (build_id != 0) {
        if (use_snapshot && poses_snapshot) {
            if (pose_idx != poses_snapshot->size()) {
                RCLCPP_WARN(log,
                    "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu snapshot_consumed=%zu snapshot_size=%zu epoch_filtered=%zu MISMATCH (possible ghosting or bug)",
                    static_cast<unsigned long long>(build_id), pose_idx, poses_snapshot->size(), epoch_filtered);
            } else {
                RCLCPP_INFO(log,
                    "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu pts=%zu snapshot_consumed=%zu snapshot_size=%zu epoch_filtered=%zu (grep GHOSTING_DIAG for timeline)",
                    static_cast<unsigned long long>(build_id), out_pts, pose_idx, poses_snapshot->size(), epoch_filtered);
            }
        } else {
            RCLCPP_INFO(log,
                "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu pts=%zu use_snapshot=no epoch_filtered=%zu",
                static_cast<unsigned long long>(build_id), out_pts, epoch_filtered);
        }
    }
    return out ? out : combined;
}

CloudXYZIPtr SubMapManager::buildGlobalMapInternalFromSnapshot(
    const std::vector<std::pair<CloudXYZIPtr, Pose3d>>& cloud_pose_snapshot,
    float voxel_size,
    uint64_t build_id) const {
    const unsigned tid = automap_pro::logThreadId();
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternalFromSnapshot_enter voxel_size={:.3f} snapshot_size={}", tid, voxel_size, cloud_pose_snapshot.size());
    if (build_id != 0) {
        RCLCPP_INFO(log,
            "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_enter build_id=%llu use_snapshot=yes (from cloud_pose_snapshot)",
            static_cast<unsigned long long>(build_id));
    }

    CloudXYZIPtr combined = std::make_shared<CloudXYZI>();
    CloudXYZIPtr world_tmp = std::make_shared<CloudXYZI>();

    double last_snapshot_ts = cloud_pose_snapshot.empty() ? 0.0 : 0.0; // 假定没有ts信息
    
    for (size_t i = 0; i < cloud_pose_snapshot.size(); ++i) {
        const auto& cp = cloud_pose_snapshot[i];
        const CloudXYZIPtr& cloud_body = cp.first;
        if (!cloud_body || cloud_body->empty()) continue;
        const Pose3d& T_map_b = cp.second;
        
        // 抽样记录位姿数值，用于核对是否包含 HBA 修正
        if (i % 100 == 0 || i == cloud_pose_snapshot.size() - 1) {
            ALOG_DEBUG(MOD, "[GHOSTING_DIAG] merging snapshot idx={} pos=[{:.3f},{:.3f},{:.3f}]", 
                      i, T_map_b.translation().x(), T_map_b.translation().y(), T_map_b.translation().z());
        }

        Eigen::Affine3f T_wf;
        T_wf.matrix() = T_map_b.cast<float>().matrix();
        world_tmp->clear();
        try {
            pcl::transformPointCloud(*cloud_body, *world_tmp, T_wf);
        } catch (...) { continue; }
        if (world_tmp->empty()) continue;
        if (combined->size() + world_tmp->size() > kMaxCombinedPoints) break;
        combined->reserve(combined->size() + world_tmp->size());
        for (const auto& pt : world_tmp->points)
            combined->push_back(pt);
    }

    if (combined->empty()) return combined;
    float vs = std::max(voxel_size, utils::kMinVoxelLeafSize);
    if (voxel_size <= 0.0f) return combined;

    // 🏛️ [架构优化] 在此处读取配置并显式向下传递，避免异步/OpenMP 线程内 ConfigManager 竞争导致 SIGSEGV
    const bool parallel = parallel_voxel_downsample_;
    CloudXYZIPtr out = utils::voxelDownsampleChunked(combined, vs, 50.0f, parallel);
    const size_t out_pts = out ? out->size() : combined->size();
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_exit out={}", tid, out_pts);
    if (build_id != 0) {
        if (!cloud_pose_snapshot.empty()) {
            const auto& map_last = cloud_pose_snapshot.back().second.translation();
            RCLCPP_INFO(log,
                "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu pts=%zu snapshot_size=%zu map_last_pos=[%.2f,%.2f,%.2f] (与 optimized_path last_pos 应一致)",
                static_cast<unsigned long long>(build_id), out_pts, cloud_pose_snapshot.size(), map_last.x(), map_last.y(), map_last.z());
        } else {
            RCLCPP_INFO(log,
                "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu pts=%zu snapshot_consumed=%zu snapshot_size=%zu (grep GHOSTING_DIAG for timeline)",
                static_cast<unsigned long long>(build_id), out_pts, cloud_pose_snapshot.size(), cloud_pose_snapshot.size());
        }
    }
    return out ? out : combined;
}

bool SubMapManager::needFreezeSubmap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!active_submap_) return false;
    return isFull(active_submap_);
}

SubMap::Ptr SubMapManager::freezeCurrentSubmap() {
    SubMap::Ptr sm = nullptr;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!active_submap_) return nullptr;
        sm = active_submap_;
        active_submap_ = nullptr;
    }
    
    if (sm) {
        freezeActiveSubmap(sm);
    }
    return sm;
}

}  // namespace automap_pro
