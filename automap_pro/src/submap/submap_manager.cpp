#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/structured_logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/error_code.h"
#define MOD "SubMapMgr"

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
}

void SubMapManager::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    event_pub_ = node->create_publisher<automap_pro::msg::SubMapEventMsg>(
        "/automap/submap_event", 50);
    RCLCPP_INFO(node->get_logger(), "[SubMapMgr][TOPIC] publish: /automap/submap_event");
}

void SubMapManager::startNewSession(uint64_t session_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    current_session_id_ = session_id;
    active_submap_ = nullptr;  // 新 session 重新开始子图
}

void SubMapManager::addKeyFrame(const KeyFrame::Ptr& kf) {
    // 结构化日志：开始Span
    SLOG_START_SPAN(MOD, "add_keyframe");
    
    std::lock_guard<std::mutex> lk(mutex_);

    // 指标：点云处理计时
    METRIC_TIMED_SCOPE(metrics::POINTCLOUD_PROCESS_TIME_MS);

    try {
        // 分配全局唯一 KF ID
        kf->id         = kf_id_counter_++;
        kf->session_id = current_session_id_;

        // 如果没有活跃子图，创建一个
        if (!active_submap_) {
            active_submap_ = createNewSubmap(kf);
            submaps_.push_back(active_submap_);
            
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
        mergeCloudToSubmap(active_submap_, kf);

        // 更新空间范围（最近帧到锚定帧的最大距离）
        double dist = (kf->T_w_b.translation() -
                       active_submap_->pose_w_anchor.translation()).norm();
        active_submap_->spatial_extent_m = std::max(active_submap_->spatial_extent_m, dist);

        // 结构化日志：调试信息
        SLOG_DEBUG(MOD, "KF processed: id={}, sm_id={}, kf_count={}, dist={:.2f}m",
                     kf->id, active_submap_->id, active_submap_->keyframes.size(),
                     active_submap_->spatial_extent_m);

        // 更新健康检查：队列大小
        HEALTH_UPDATE_QUEUE("submap", getFrozenSubmaps().size());

        if (isFull(active_submap_)) {
            const int sm_id = active_submap_->id;
            const size_t kf_count = active_submap_->keyframes.size();
            const double dist = active_submap_->spatial_extent_m;
            const double t_start = active_submap_->t_start;
            const double t_end = active_submap_->t_end;
            
            SLOG_INFO(MOD, "SubMap FULL: id={}, kf={}, dist={:.1f}m → freezing",
                       sm_id, kf_count, dist);
            
            // 冻结子图（带错误处理）
            try {
                freezeActiveSubmap();
            } catch (const std::exception& e) {
                auto err = ErrorDetail::fromException(e, errors::SUBMAP_STATE_INVALID);
                SLOG_ERROR_CODE(MOD, static_cast<uint32_t>(err.code()),
                              fmt::format("Failed to freeze submap #{}: {}", sm_id, e.what()));
                METRICS_INCREMENT(metrics::ERRORS_TOTAL);
            }

            active_submap_ = nullptr;
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

        // 记录错误指标
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);

        // 发布错误事件
        publishErrorEvent(active_submap_ ? active_submap_->id : 0, err);
    }
    
    // 结构化日志：结束Span
    SLOG_END_SPAN();
    
    // 更新关键帧指标
    METRICS_INCREMENT(metrics::KEYFRAMES_CREATED);
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
    if (!active_submap_ || active_submap_->state != SubMapState::ACTIVE) {
        SLOG_WARN(MOD, "No active submap to freeze (state={})",
                   active_submap_ ? static_cast<int>(active_submap_->state) : -1);
        return;
    }

    // 结构化日志：开始Span
    SLOG_START_SPAN(MOD, "freeze_submap");
    
    // 计时：冻结操作（警告阈值500ms）
    SLOG_TIMED_SCOPE(MOD, fmt::format("freeze_submap#{}", active_submap_->id), 500.0);

    try {
        // 降采样点云（用于回环匹配）
        if (active_submap_->merged_cloud && !active_submap_->merged_cloud->empty()) {
            SLOG_TIMED_SCOPE(MOD, "downsample_submap", 200.0);
            
            CloudXYZIPtr ds(new CloudXYZI);
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(active_submap_->merged_cloud);
            vg.setLeafSize(match_res_, match_res_, match_res_);
            vg.filter(*ds);
            
            active_submap_->downsampled_cloud = ds;
            
            SLOG_DEBUG(MOD, "SM#{} downsampled: {} -> {} pts",
                         active_submap_->id, active_submap_->merged_cloud->size(), ds->size());
            
            // 记录指标
            METRICS_HISTOGRAM_OBSERVE(metrics::POINTCLOUD_SIZE, 
                                          static_cast<double>(active_submap_->merged_cloud->size()));
        }

        // 更新状态
        active_submap_->state = SubMapState::FROZEN;
        publishEvent(active_submap_, "FROZEN");
        
        SLOG_EVENT(MOD, "submap_frozen", 
                   "SubMap #{} frozen (kf_count={}, dist={:.1f}m, memory={}MB)",
                   active_submap_->id, active_submap_->keyframes.size(),
                   active_submap_->spatial_extent_m,
                   active_submap_->merged_cloud ? active_submap_->merged_cloud->size() * sizeof(pcl::PointXYZI) / (1024.0 * 1024.0) : 0.0);
        
        // 记录指标
        METRICS_INCREMENT(metrics::SUBMAPS_FROZEN);
        
        // 触发回调（→ LoopDetector, HBAOptimizer, IncrementalOptimizer）
        for (auto& cb : frozen_cbs_) {
            SLOG_DEBUG(MOD, "Calling frozen callback for SM#{}", active_submap_->id);
            cb(active_submap_);
        }
        
        // 更新健康检查：队列大小
        HEALTH_UPDATE_QUEUE("submap", getFrozenSubmaps().size());
        HEALTH_UPDATE_QUEUE("loop", 0);  // 假设 LoopDetector 也处理冻结子图

        int frozen_sm_id = active_submap_->id;
        active_submap_ = nullptr;

        ALOG_INFO(MOD, "[STATE] SubMap #{} frozen", frozen_sm_id);
        
    } catch (const std::exception& e) {
        // 使用错误码系统
        auto error = ErrorDetail(errors::SUBMAP_STATE_INVALID, 
                                 fmt::format("Failed to freeze submap #{}: {}", 
                                          active_submap_ ? active_submap_->id : 0, e.what()));
        error.context().operation = "freezeActiveSubmap";
        error.context().file = __FILE__;
        error.context().line = __LINE__;
        error.context().function = __func__;
        
        SLOG_ERROR_CODE(MOD, static_cast<uint32_t>(error.code()), error.message());
        
        // 记录错误指标
        METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        
        // 发布错误事件
        publishErrorEvent(active_submap_ ? active_submap_->id : 0, error);
    }
    
    // 结构化日志：结束Span
    SLOG_END_SPAN();
}

SubMap::Ptr SubMapManager::createNewSubmap(const KeyFrame::Ptr& first_kf) {
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
    if (!kf->cloud_body || kf->cloud_body->empty()) return;

    // 合并前先检查是否需要降采样（避免累积）
    if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
        CloudXYZIPtr temp = std::make_shared<CloudXYZI>();
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(sm->merged_cloud);
        vg.setLeafSize(merge_res_, merge_res_, merge_res_);
        vg.filter(*temp);
        size_t old_pts = sm->merged_cloud->size();
        size_t new_pts = temp->size();
        sm->merged_cloud.swap(temp);
        temp.reset();
        ALOG_DEBUG(MOD, "SM#{} pre-merge downsample: {} -> {} pts",
                   sm->id, old_pts, new_pts);
    }

    // 将 body 系点云变换到世界系
    CloudXYZIPtr world_cloud = getCloudFromPool();
    Eigen::Affine3f T_wf;
    T_wf.matrix() = kf->T_w_b.cast<float>().matrix();
    pcl::transformPointCloud(*kf->cloud_body, *world_cloud, T_wf);

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

    // 合并后超过阈值则降采样
    if (sm->merged_cloud && sm->merged_cloud->size() > kDownsampleThreshold) {
        pcl::VoxelGrid<pcl::PointXYZI>* vg = getVoxelGrid(merge_res_);
        vg->setInputCloud(sm->merged_cloud);
        CloudXYZIPtr temp = std::make_shared<CloudXYZI>();
        vg->filter(*temp);
        sm->merged_cloud.swap(temp);
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
        if (sm->keyframes.empty()) continue;
        KeyFrame::Ptr anchor = sm->keyframes.front();
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

CloudXYZIPtr SubMapManager::buildGlobalMap(float voxel_size) const {
    std::lock_guard<std::mutex> lk(mutex_);
    CloudXYZIPtr combined = std::make_shared<CloudXYZI>();
    for (const auto& sm : submaps_) {
        if (!sm->merged_cloud || sm->merged_cloud->empty()) continue;
        *combined += *sm->merged_cloud;
    }
    if (combined->empty()) return combined;
    CloudXYZIPtr out = std::make_shared<CloudXYZI>();
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(combined);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*out);
    return out;
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

}  // namespace automap_pro
