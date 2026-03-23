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
        if (merge_thread_.joinable())
            merge_thread_.join();
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
    merge_thread_ = std::thread(&SubMapManager::mergeWorkerLoop, this);
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
        // ✅ V2 修复：不再覆盖 kf->id 和 kf->session_id，尊重 Frontend (KeyFrameManager) 分配的原始 ID
        // kf->id         = kf_id_counter_++; 
        // kf->session_id = current_session_id_;

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
            // [HBA_FIX] 同步路径也应用锚点坐标系变换，防止回环拉花
            CloudXYZIPtr body_cloud(new CloudXYZI());
            Eigen::Isometry3d T_anchor_w = sm->pose_odom_anchor.inverse();
            pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
            
            CloudXYZIPtr ds = utils::voxelDownsample(body_cloud, static_cast<float>(match_res_));
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
    try {
        if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            // [HBA_FIX] 结束路径也应用锚点坐标系变换，防止回环拉花
            CloudXYZIPtr body_cloud(new CloudXYZI());
            Eigen::Isometry3d T_anchor_w = sm->pose_odom_anchor.inverse();
            pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
            
            CloudXYZIPtr ds = utils::voxelDownsample(body_cloud, static_cast<float>(match_res_));
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
        }

        if (task.sm && task.kf) {
            try {
                // 💡 [V3 性能优化] 在专用线程执行耗时的 SOR 滤波和点云变换
                mergeCloudToSubmap(task.sm, task.kf);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"), 
                    "[SubMapMgr][ASYNC_MERGE] Exception: %s", e.what());
            }
        }
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
                // 同时也解决了 OverlapTransformer/ScanContext 在世界坐标系（如 ENU 数千米外）下描述子失效的问题。
                CloudXYZIPtr body_cloud(new CloudXYZI());
                Eigen::Isometry3d T_anchor_w = sm->pose_odom_anchor.inverse();
                pcl::transformPointCloud(*sm->merged_cloud, *body_cloud, T_anchor_w.matrix().cast<float>());
                
                CloudXYZIPtr ds = utils::voxelDownsample(body_cloud, static_cast<float>(match_res_));
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

void SubMapManager::mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const {
    using Clock = std::chrono::steady_clock;
    constexpr int kMergeVoxelTimeoutMs = 15000;  // 子图合并体素超时，超时必记 [SubMapMgr][TIMEOUT]
    if (!kf->cloud_body || kf->cloud_body->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu has null/empty cloud_body, skip merge", 
            kf->id);
        return;
    }

    // 合并前先检查是否需要降采样（带超时；每步打 STEP+file+line 精确定位到行）
    if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
        size_t old_pts = sm->merged_cloud->size();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_pre_voxel_enter sm_id=%d kf_id=%lu pts=%zu file=%s line=%d",
            sm->id, kf->id, old_pts, __FILE__, __LINE__);
        auto t_pre = Clock::now();
        bool timed_out = false;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_pre_voxel_call_enter (下一行即 voxelDownsampleChunkedWithTimeout) file=%s line=%d", __FILE__, __LINE__);
        const float kMergeChunkSizeM = 50.0f;
        CloudXYZIPtr temp = utils::voxelDownsampleChunkedWithTimeout(
            sm->merged_cloud, static_cast<float>(merge_res_), kMergeChunkSizeM, kMergeVoxelTimeoutMs, &timed_out);
        double ms_pre = 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t_pre).count();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_pre_voxel_exit sm_id=%d duration_ms=%.1f timed_out=%d file=%s line=%d",
            sm->id, ms_pre, timed_out ? 1 : 0, __FILE__, __LINE__);
        if (timed_out) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][TIMEOUT] pre-merge voxelDownsample timed out sm_id=%d pts=%zu limit_ms=%d (建图时请重点关注)",
                sm->id, old_pts, kMergeVoxelTimeoutMs);
        }
        if (temp && !temp->empty()) {
            size_t new_pts = temp->size();
            sm->merged_cloud.swap(temp);
            ALOG_DEBUG(MOD, "SM#{} pre-merge downsample: {} -> {} pts",
                       sm->id, old_pts, new_pts);
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[GLOBAL_MAP_DIAG] SM#%d pre-merge downsample: %zu → %zu pts", sm->id, old_pts, new_pts);
        }
    }

    // 将 body 系点云变换到世界系
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][STEP] step=merge_transform_enter kf_id=%lu body_pts=%zu file=%s line=%d",
        kf->id, kf->cloud_body->size(), __FILE__, __LINE__);
    
    // 🔧 [修复] SOR 滤波：在合并前去除关键帧中的离群点，显著提升点云清晰度
    // 默认使用 MeanK=50, StdMul=1.0，抑制运动产生的拖影与噪声
    CloudXYZIPtr filtered_kf_cloud(new CloudXYZI());
    try {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(kf->cloud_body);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*filtered_kf_cloud);
        
        // 🔧 [修复] 提升全局图清晰度：将过滤后的点云写回关键帧 cloud_body
        // 否则 buildGlobalMap 与 saveMapToFiles 仍会使用包含离群点的原始点云
        if (!filtered_kf_cloud->empty()) {
            kf->cloud_body = filtered_kf_cloud;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][CLARITY] SOR filter kf_id=%lu: pts_remained=%zu",
            kf->id, filtered_kf_cloud->size());
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][CLARITY] SOR filter failed for kf_id=%lu: %s, using raw cloud",
            kf->id, e.what());
        filtered_kf_cloud = kf->cloud_body;
    }

    auto t_tf = Clock::now();
    CloudXYZIPtr world_cloud = getCloudFromPool();
    Eigen::Affine3f T_wf;

    // 🏛️ [修复] 线程安全与坐标系一致性：持有 mutex_ 保护 merged_cloud 免受 updateSubmapPose 并发修改
    // 且在此处现场读取 T_map_b_optimized（与全局图/轨迹一致），配合 MappingModule::updateGPSAlignment 的子图冻结逻辑
    std::lock_guard<std::mutex> lk(mutex_);

    // 与 buildGlobalMap 主路径一致：body→world 使用 T_map_b_optimized（创建时初值=T_odom_b），
    // 避免后端优化后仍用 T_odom_b 合并导致 merged_cloud 与 optimized_path/global_map 错位。
    T_wf.matrix() = kf->T_map_b_optimized.cast<float>().matrix();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][STEP] step=merge_transform_pcl_enter (下一行即 pcl::transformPointCloud) file=%s line=%d", __FILE__, __LINE__);
    try {
        pcl::transformPointCloud(*filtered_kf_cloud, *world_cloud, T_wf);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu transform failed: %s", kf->id, e.what());
        return;
    }
    double ms_tf = 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t_tf).count();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][STEP] step=merge_transform_exit kf_id=%lu duration_ms=%.1f world_pts=%zu file=%s line=%d",
        kf->id, ms_tf, world_cloud->size(), __FILE__, __LINE__);

    if (world_cloud->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GLOBAL_MAP_DIAG] mergeCloudToSubmap: kf_id=%lu transform resulted in empty cloud", kf->id);
        return;
    }

    size_t world_cloud_size = world_cloud->size();

    // 合并点云（append）
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][STEP] step=merge_append_enter kf_id=%lu sm_id=%d merged_before=%zu world_pts=%zu file=%s line=%d",
        kf->id, sm->id, sm->merged_cloud ? sm->merged_cloud->size() : 0u, world_cloud_size, __FILE__, __LINE__);
    auto t_append = Clock::now();
    if (!sm->merged_cloud || sm->merged_cloud->empty()) {
        sm->merged_cloud = std::make_shared<CloudXYZI>(*world_cloud);
    } else {
        size_t old_size = sm->merged_cloud->size();
        sm->merged_cloud->reserve(old_size + world_cloud->size());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_append_loop_enter count=%zu file=%s line=%d", world_cloud->size(), __FILE__, __LINE__);
        sm->merged_cloud->points.insert(sm->merged_cloud->points.end(),
                                        world_cloud->points.begin(), world_cloud->points.end());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_append_loop_exit file=%s line=%d", __FILE__, __LINE__);
    }
    double ms_append = 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t_append).count();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][STEP] step=merge_append_exit kf_id=%lu duration_ms=%.1f merged_total=%zu file=%s line=%d",
        kf->id, ms_append, sm->merged_cloud->size(), __FILE__, __LINE__);

    // [POSE_DIAG] 合并到子图时使用的位姿（与上方变换一致：T_map_b_optimized）
    const Eigen::Vector3d t = kf->T_map_b_optimized.translation();
    double yaw_deg = std::atan2(kf->T_map_b_optimized.rotation()(1, 0), kf->T_map_b_optimized.rotation()(0, 0)) * 180.0 / M_PI;
    RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
        "[GLOBAL_MAP_DIAG] ✓ merge sm_id=%d kf_id=%lu: body_pts=%zu → world_pts=%zu, T_map_b_opt=[%.2f,%.2f,%.2f], merged_total=%zu",
        sm->id, kf->id, kf->cloud_body->size(), world_cloud_size, t.x(), t.y(), t.z(), sm->merged_cloud->size());
    // [GHOSTING_TRACE] 合并已用 T_map_b_optimized；此处记录与纯里程计差异（应接近 0 若与后端一致）
    if (ConfigManager::instance().backendVerboseTrace()) {
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
    // 首帧合并到该子图时打 INFO（每子图一条）
    const bool is_first_merge = (sm->merged_cloud->size() == world_cloud_size);
    if (is_first_merge) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][POSE_DIAG] mergeCloudToSubmap sm_id=%d 首帧 kf_id=%lu T_map_b_optimized=[%.4f,%.4f,%.4f] yaw=%.2fdeg (与 buildGlobalMap 主路径一致)",
            sm->id, kf->id, t.x(), t.y(), t.z(), yaw_deg);
    }

    // 合并后超过阈值则降采样（带超时；打 STEP+file+line 精确定位到行）
    if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
        size_t before_downsample = sm->merged_cloud->size();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_post_voxel_enter sm_id=%d kf_id=%lu pts=%zu file=%s line=%d",
            sm->id, kf->id, before_downsample, __FILE__, __LINE__);
        auto t_post = Clock::now();
        bool timed_out = false;
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_post_voxel_call_enter (下一行即 voxelDownsampleChunkedWithTimeout) file=%s line=%d", __FILE__, __LINE__);
        const float kMergeChunkSizeM = 50.0f;
        CloudXYZIPtr temp = utils::voxelDownsampleChunkedWithTimeout(
            sm->merged_cloud, static_cast<float>(merge_res_), kMergeChunkSizeM, kMergeVoxelTimeoutMs, &timed_out);
        double ms_post = 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t_post).count();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][STEP] step=merge_post_voxel_exit sm_id=%d kf_id=%lu duration_ms=%.1f timed_out=%d file=%s line=%d",
            sm->id, kf->id, ms_post, timed_out ? 1 : 0, __FILE__, __LINE__);
        if (timed_out) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[SubMapMgr][TIMEOUT] post-merge voxelDownsample timed out sm_id=%d kf_id=%lu pts=%zu limit_ms=%d (建图时请重点关注)",
                sm->id, kf->id, before_downsample, kMergeVoxelTimeoutMs);
        }
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
    
    std::unique_lock<std::mutex> lk(mutex_);
    
    bool updated = false;
    double max_translation_diff = 0.0;
    double max_rotation_diff = 0.0;

    // ========== [SUBMAP_POSE_UPDATE_DEBUG] 增加详细日志 ==========
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][SUBMAP_POSE_UPDATE_DEBUG] =======================================================");
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[SubMapMgr][SUBMAP_POSE_UPDATE_DEBUG] 开始更新子图%d的位姿...", submap_id);
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
        sm->state = SubMapState::OPTIMIZED;

        // 更新关键帧位姿
        for (auto& kf : sm->keyframes) {
            kf->T_map_b_optimized = sm->pose_map_anchor_optimized * kf->T_submap_kf;
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

void SubMapManager::batchUpdateSubmapPoses(const std::unordered_map<int, Pose3d>& updates, uint64_t version) {
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
                sm->state = SubMapState::OPTIMIZED;

                double trans_diff = (new_pose.translation() - old_anchor.translation()).norm();
                double rot_diff = Eigen::AngleAxisd(new_pose.rotation().inverse() * old_anchor.rotation()).angle();
                
                max_trans_diff = std::max(max_trans_diff, trans_diff);
                max_rot_diff = std::max(max_rot_diff, rot_diff);

                // 更新关键帧位姿
                for (auto& kf : sm->keyframes) {
                    kf->T_map_b_optimized = sm->pose_map_anchor_optimized * kf->T_submap_kf;
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

    const double kRebuildThresholdTrans = ConfigManager::instance().submapRebuildThreshTrans();
    const double kRebuildThresholdRot = ConfigManager::instance().submapRebuildThreshRot();
    const double max_rot_deg = max_rot_diff * 180.0 / M_PI;

    if (max_trans_diff > kRebuildThresholdTrans || max_rot_deg > kRebuildThresholdRot) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SubMapMgr][AUTO_REBUILD] version=%lu drift large (%.2fm, %.2fdeg) -> triggering full rebuild",
            version, max_trans_diff, max_rot_deg);
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
    for (size_t i = 0; i < n_poses; ++i) {
        // [HBA_GHOSTING_FIX] 直接覆盖为 HBA 优化后的绝对位姿
        kfs_in_hba_order[i]->T_map_b_optimized = result.optimized_poses[i];
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
        sm->pose_odom_anchor = anchor->T_odom_b;

        // 🏛️ [架构重构] HBA 可能会改变子图内部结构，因此需要更新 T_submap_kf
        // 保证后续 ISAM2 的无 stateless 更新 (pose_map_anchor_optimized * T_submap_kf) 是准确的
        for (auto& kf : sm->keyframes) {
            if (kf) {
                kf->T_submap_kf = sm->pose_map_anchor_optimized.inverse() * kf->T_map_b_optimized;
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
    if (ConfigManager::instance().backendVerboseTrace()) {
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

            // 使用优化后的位姿
            Eigen::Affine3f T_wf;
            T_wf.matrix() = kf->T_map_b_optimized.cast<float>().matrix();

            CloudXYZIPtr world_cloud = std::make_shared<CloudXYZI>();
            try {
                pcl::transformPointCloud(*kf->cloud_body, *world_cloud, T_wf);
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
                    "[SubMapMgr][POSE_DIAG]   sm_id=%d kf_id=%lu 变换使用 T_map_b_optimized=[%.4f,%.4f,%.4f] yaw=%.2fdeg body_pts=%zu",
                    sm->id, kf->id, t.x(), t.y(), t.z(), yaw_deg, kf->cloud_body->size());
            }
            kf_idx++;

            // 合并点云
            size_t old_size = sm->merged_cloud->size();
            sm->merged_cloud->reserve(old_size + world_cloud->size());
            for (const auto& pt : world_cloud->points) {
                sm->merged_cloud->push_back(pt);
            }
        }

        // 降采样以控制大小
        if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
            CloudXYZIPtr ds = utils::voxelDownsample(sm->merged_cloud, static_cast<float>(merge_res_));
            if (ds && !ds->empty()) {
                sm->merged_cloud->swap(*ds);
            }
        }

        RCLCPP_INFO(log, "[SubMapMgr][REBUILD_MERGE] sm_id=%d rebuilt merged_cloud: %zu points (anchor_delta: trans=%.3fm)",
            sm->id, sm->merged_cloud ? sm->merged_cloud->size() : 0,
            delta_trans_norm);
    }
    
    RCLCPP_INFO(log, "[SubMapMgr][REBUILD_MERGE] Done rebuilding merged_cloud for all submaps");
    const double done_ts = (node() && node()->get_clock()) ? node()->get_clock()->now().seconds() : 0.0;
    RCLCPP_INFO(log,
        "[SubMapMgr][GHOSTING_DIAG] rebuildMergedCloudFromOptimizedPoses done ts=%.3f (此后 buildGlobalMap 与 merged_cloud 均基于 T_map_b_optimized；grep GHOSTING_DIAG 查时间线)",
        done_ts);
    RCLCPP_INFO(log,
        "[GHOSTING_SOURCE] merged_cloud rebuild_done (fallback 路径若被使用将与 optimized_path 一致；若 build 在 rebuild_done 之前则可能用过旧 merged_cloud→重影)");
    if (ConfigManager::instance().backendVerboseTrace()) {
        RCLCPP_INFO(log, "[GHOSTING_TRACE] rebuildMergedCloudFromOptimizedPoses DONE (fallback 路径此后无重影；grep GHOSTING_TRACE 查证据链)");
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
    
    std::unique_lock<std::mutex> lk(mutex_);
    
    // 🏛️ [P0 架构优化] 全局地图缓存检查
    // 如果关键帧总数、位姿版本号、体素大小均未变化，则直接返回缓存，避免高频冗余构建导致的性能卡顿
    size_t current_kf_total = 0;
    for (const auto& sm : submaps_) if(sm) current_kf_total += sm->keyframes.size();
    
    if (cached_global_map_ && !cached_global_map_->empty() &&
        last_build_kf_count_ == current_kf_total &&
        last_build_map_version_ == current_map_version_ &&
        std::abs(last_build_voxel_size_ - voxel_size) < 1e-4) {
        RCLCPP_DEBUG(log, "[SubMapMgr][CACHE] buildGlobalMap hit: kf=%zu ver=%lu voxel=%.3f (skip redundant build)",
                    current_kf_total, current_map_version_, voxel_size);
        return cached_global_map_;
    }

    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMap_enter voxel_size={:.3f}", tid, voxel_size);
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG][HBA_GHOSTING] buildGlobalMap enter voxel_size=%.3f 主路径=从 kf->cloud_body 用 T_map_b_optimized 变换；T_map_b_optimized 未优化时直接退出程序（正常建图不应出现）", voxel_size);
    
    const size_t num_submaps = submaps_.size();
    if (ConfigManager::instance().backendVerboseTrace()) {
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
    // T_map_b_optimized 未优化时已 std::abort()，不会产生「跳过」统计
    int subs_with_kf = 0;
    size_t total_pts_before_transform = 0;
    
    // [GLOBAL_MAP_DIAG] 增强：记录各子图锚点的优化位姿状态
    for (const auto& sm : submaps_) {
        if (!sm) continue;
        const auto& anchor_pose = sm->pose_map_anchor_optimized;
        RCLCPP_INFO(log, "[SubMapMgr][GLOBAL_MAP_DIAG] SM#%d anchor_pose: trans=[%.2f,%.2f,%.2f] state=%d kfs=%zu",
            sm->id, anchor_pose.translation().x(), anchor_pose.translation().y(), anchor_pose.translation().z(),
            static_cast<int>(sm->state), sm->keyframes.size());
    }
    
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] ┌─ 主路径: 从关键帧重算（使用 T_map_b_optimized）");
    
    for (size_t idx = 0; idx < num_submaps && !hit_limit; ++idx) {
        const auto& sm = submaps_[idx];
        if (!sm) {
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] submap[%zu] is null, skip", idx);
            continue;
        }
        
        size_t sm_pts = 0;
        size_t sm_kf_count = 0;
        size_t sm_kf_valid = 0;
        
        RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: %zu keyframes, pose_map_anchor_optimized=[%.2f,%.2f,%.2f]",
            sm->id, sm->keyframes.size(),
            sm->pose_map_anchor_optimized.translation().x(),
            sm->pose_map_anchor_optimized.translation().y(),
            sm->pose_map_anchor_optimized.translation().z());
        
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
    RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG]   • 统计: kf_skipped_null=%zu, kf_skipped_empty=%zu (未优化 KF 会直接 abort 不统计)",
        kf_skipped_null, kf_skipped_empty);
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
                "   Using merged_cloud (built with T_odom_b, not T_map_b_optimized)\n"
                "   After optimization, this may cause misalignment with trajectory\n"
                "   Recommendation: Set retain_cloud_body=true if precision is critical",
                kf_skipped_null, kf_skipped_empty);
            METRICS_INCREMENT(metrics::FALLBACK_TO_MERGED_CLOUD);
        }

        used_fallback_path = true;  // 供 [GLOBAL_MAP_BLUR] 汇总判断
        if (ConfigManager::instance().backendVerboseTrace()) {
            RCLCPP_INFO(log, "[GHOSTING_TRACE] buildGlobalMap PATH=fallback kf_skipped_null=%zu kf_skipped_empty=%zu (merged_cloud若未rebuild→重影风险；grep GHOSTING_TRACE)",
                kf_skipped_null, kf_skipped_empty);
        }
        RCLCPP_WARN(log, "[GHOSTING_RISK] buildGlobalMap_sync path=fallback_merged_cloud (主路径 combined 为空；merged_cloud 若未在 HBA 后 rebuild 则为 T_odom_b 系→重影；grep REBUILD_MERGE)");
        RCLCPP_WARN(log, "[GHOSTING_SOURCE] buildGlobalMap path=fallback_merged_cloud (主路径 combined 为空；merged_cloud 若未在 HBA 后 rebuild 则为 T_odom_b 系→与 optimized_path 重影，grep REBUILD_MERGE 查是否已重建)");
        RCLCPP_WARN(log, "[SubMapMgr][HBA_GHOSTING] buildGlobalMap 使用 fallback_merged_cloud：主路径 combined 为空，用各子图 merged_cloud 拼接；若 merged_cloud 未经 rebuildMergedCloudFromOptimizedPoses 则仍为 T_odom_b 世界系，与轨迹重影");
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
            
            RCLCPP_DEBUG(log, "[GLOBAL_MAP_DIAG] │ SM#%d: added %zu pts from merged_cloud (built with T_odom_b, not optimized)",
                sm->id, add_size);
        }
        
        RCLCPP_INFO(log, "[GLOBAL_MAP_DIAG] └─ 回退完成: combined_pts=%zu", combined->size());
    } else if (!combined->empty()) {
        if (ConfigManager::instance().backendVerboseTrace()) {
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

        const Pose3d& T = submap->pose_map_anchor_optimized;
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
            sm->pose_map_anchor_optimized = sm->pose_odom_anchor = Pose3d::Identity();
            sm->pose_map_anchor_optimized.linear() = q.toRotationMatrix();
            sm->pose_map_anchor_optimized.translation() << ap.value("px", 0.0), ap.value("py", 0.0), ap.value("pz", 0.0);
            sm->pose_odom_anchor = sm->pose_map_anchor_optimized;
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

std::future<CloudXYZIPtr> SubMapManager::buildGlobalMapAsync(float voxel_size) const {
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

    return std::async(std::launch::async, [this, voxel_size]() {
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
                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
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
                "[SubMapMgr][GHOSTING_DIAG] pose_snapshot_taken build_id=%llu ts=%.3f kf_count=%zu first_pos=[%.2f,%.2f,%.2f] last_pos=[%.2f,%.2f,%.2f]",
                static_cast<unsigned long long>(build_id), snap_ts, cloud_pose_snapshot.size(),
                first.x(), first.y(), first.z(), last.x(), last.y(), last.z());
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
    uint64_t build_id) const {
    const unsigned tid = automap_pro::logThreadId();
    const rclcpp::Logger log = rclcpp::get_logger("automap_system");
    const bool use_snapshot = poses_snapshot && !poses_snapshot->empty();
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_enter voxel_size={:.3f} use_snapshot={}", tid, voxel_size, use_snapshot ? "yes" : "no");
    if (build_id != 0) {
        RCLCPP_INFO(log,
            "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_enter build_id=%llu use_snapshot=%s (grep GHOSTING_DIAG for timeline)",
            static_cast<unsigned long long>(build_id), use_snapshot ? "yes" : "no");
    }

    CloudXYZIPtr combined = std::make_shared<CloudXYZI>();
    CloudXYZIPtr world_tmp = std::make_shared<CloudXYZI>();
    size_t pose_idx = 0;

    for (const auto& sm : submaps_copy) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
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
    const bool parallel = ConfigManager::instance().parallelVoxelDownsample();
    CloudXYZIPtr out = utils::voxelDownsampleChunked(combined, vs, 50.0f, parallel);
    const size_t out_pts = out ? out->size() : combined->size();
    ALOG_INFO(MOD, "[tid={}] step=buildGlobalMapInternal_exit out={}", tid, out_pts);
    if (build_id != 0) {
        if (use_snapshot && poses_snapshot) {
            if (pose_idx != poses_snapshot->size()) {
                RCLCPP_WARN(log,
                    "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu snapshot_consumed=%zu snapshot_size=%zu MISMATCH (possible ghosting or bug)",
                    static_cast<unsigned long long>(build_id), pose_idx, poses_snapshot->size());
            } else {
                RCLCPP_INFO(log,
                    "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu pts=%zu snapshot_consumed=%zu snapshot_size=%zu (grep GHOSTING_DIAG for timeline)",
                    static_cast<unsigned long long>(build_id), out_pts, pose_idx, poses_snapshot->size());
            }
        } else {
            RCLCPP_INFO(log,
                "[SubMapMgr][GHOSTING_DIAG] buildGlobalMapInternal_exit build_id=%llu pts=%zu use_snapshot=no",
                static_cast<unsigned long long>(build_id), out_pts);
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
    const bool parallel = ConfigManager::instance().parallelVoxelDownsample();
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
