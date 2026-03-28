#pragma once

#include "automap_pro/v3/module_base.h"
#include <algorithm>
#include <cmath>
#include "automap_pro/visualization/rviz_publisher.h"
#include "automap_pro/core/config_manager.h"
#include <pcl/common/transforms.h>

#include <atomic>
#include <mutex>
#include <condition_variable>

namespace automap_pro::v3 {

/**
 * @brief 可视化模块 (Visualization Micro-Kernel Module)
 * 
 * 职责：
 * 1. 负责向 RViz 发布地图、轨迹、回环等可视化信息
 * 2. 监听 MapUpdateEvent 和 OptimizationResultEvent 自动刷新显示
 * 3. 异步处理点云合并与发布，避免阻塞核心路径
 */
class VisualizationModule : public ModuleBase {
public:
    VisualizationModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
        : ModuleBase("VisualizationModule", event_bus, map_registry), node_(node) {
        
        rviz_publisher_.init(node_);
        rviz_publisher_.setFrameId("map");

        // 订阅同步帧（用于显示当前点云：完整扫描，强度为原始值）
        // 语义着色与地标见 SemanticCloudEvent + publishEverything(semantic_landmarks)。
        // 🏛️ [P0 性能优化] 使用 onEventAsync 确保可视化点云变换不阻塞 FrontEndModule 发布线程
        onEventAsync<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
            // 与 global_map / optimized_path 一致：回环或 iSAM2 修正后 T_map_b_optimized ≠ T_odom_b，
            // 当前帧 /cloud_registered 仍在 LIO 世界系，需用「最新关键帧」上的 odom→优化 链式修正到 map。
            try {
                auto snapshot = map_registry_->getPoseSnapshot();
                CloudXYZIPtr cloud_map(new CloudXYZI());

                Pose3d T_map_odom = Pose3d::Identity();
                if (snapshot->gps_aligned) {
                    T_map_odom.linear()      = snapshot->R_enu_to_map;
                    T_map_odom.translation() = snapshot->t_enu_to_map;
                }

                // 🏛️ [架构契约] 确定基础变换语义
                Pose3d T_base_to_map = Pose3d::Identity();
                if (ev.pose_frame == PoseFrame::ODOM) {
                    T_base_to_map = T_map_odom;
                }

                KeyFrame::Ptr anchor_kf = map_registry_->getLatestKeyFrameByTimestamp();
                Pose3d T_world_to_map = T_base_to_map; // 默认使用基础契约转换
                if (anchor_kf) {
                    Pose3d T_k_opt = anchor_kf->T_map_b_optimized;
                    auto it = snapshot->keyframe_poses.find(anchor_kf->id);
                    if (it != snapshot->keyframe_poses.end()) {
                        T_k_opt = it->second;
                    }
                    
                    // 🏛️ [契约校核] 如果关键帧还是 ODOM 系但系统已对齐，执行自动补偿
                    if (anchor_kf->pose_frame == PoseFrame::ODOM && snapshot->gps_aligned) {
                        T_k_opt = T_map_odom * T_k_opt;
                    }

                    // p_map ≈ T_k_opt * inv(T_k_odom) * p_lio_world（帧间相对仍用前端里程计）
                    T_world_to_map = T_k_opt * anchor_kf->T_odom_b.inverse();
                }

                if (ConfigManager::instance().frontendCloudFrame() == "world") {
                    pcl::transformPointCloud(*ev.cloud, *cloud_map, T_world_to_map.matrix().cast<float>());
                } else {
                    Pose3d T_map_body = T_world_to_map * ev.T_odom_b;
                    pcl::transformPointCloud(*ev.cloud, *cloud_map, T_map_body.matrix().cast<float>());
                }
                rviz_publisher_.publishCurrentCloud(cloud_map);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[V3][DIAG] step=SyncedFrame_cloud EXCEPTION: %s fallback=raw", e.what());
                rviz_publisher_.publishCurrentCloud(ev.cloud);
            }
        });

        // 🏛️ 已移除同步回调中的复杂逻辑，全部改由 publishEverything 使用快照处理
        onEventAsync<GPSAlignedEvent>([this](const GPSAlignedEvent& /*ev*/) {
            // 仅触发刷新，实际对齐逻辑已由 MapRegistry 快照化
            requestRefresh();
        });

        // 订阅地图变更事件
        onEventAsync<MapUpdateEvent>([this](const MapUpdateEvent& /*ev*/) {
            requestRefresh();
        });

        // 订阅语义点云：与输入同点数（强度为类别 id，RViz 中映射为 RGB），用于「在完整几何上叠加语义」；
        // 树干/目标标注另见 /automap/semantic_landmarks（圆柱 Marker）。
        onEventAsync<SemanticCloudEvent>([this](const SemanticCloudEvent& ev) {
            const auto recv_count = semantic_cloud_events_received_.fetch_add(1, std::memory_order_relaxed) + 1;
            const auto recv_count_snapshot = semantic_cloud_events_received_.load(std::memory_order_relaxed);
            const auto pub_count_snapshot = semantic_cloud_events_published_.load(std::memory_order_relaxed);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[V3][SEM_CLOUD][STATS] stage=recv recv_total=%lu pub_total=%lu frame_id=%s pts=%zu",
                static_cast<unsigned long>(recv_count_snapshot),
                static_cast<unsigned long>(pub_count_snapshot),
                ev.frame_id.c_str(),
                ev.labeled_cloud ? ev.labeled_cloud->size() : 0);
            try {
                auto snapshot = map_registry_->getPoseSnapshot();
                CloudXYZIPtr cloud_map(new CloudXYZI());

                Pose3d T_map_odom = Pose3d::Identity();
                if (snapshot->gps_aligned) {
                    T_map_odom.linear()      = snapshot->R_enu_to_map;
                    T_map_odom.translation() = snapshot->t_enu_to_map;
                }

                KeyFrame::Ptr anchor_kf = map_registry_->getLatestKeyFrameByTimestamp();
                Pose3d T_world_to_map = T_map_odom;
                if (anchor_kf) {
                    Pose3d T_k_opt = anchor_kf->T_map_b_optimized;
                    auto it = snapshot->keyframe_poses.find(anchor_kf->id);
                    if (it != snapshot->keyframe_poses.end()) {
                        T_k_opt = it->second;
                    }
                    if (anchor_kf->pose_frame == PoseFrame::ODOM && snapshot->gps_aligned) {
                        T_k_opt = T_map_odom * T_k_opt;
                    }
                    T_world_to_map = T_k_opt * anchor_kf->T_odom_b.inverse();
                }

                const double sem_ts_tol = ConfigManager::instance().semanticTimestampMatchToleranceS();
                if (ev.frame_id == "world") {
                    pcl::transformPointCloud(*ev.labeled_cloud, *cloud_map, T_world_to_map.matrix().cast<float>());
                } else {
                    // body：必须用该帧对应关键帧位姿；误用「最新锚点链」会把地面/语义整体拉偏
                    auto kf = map_registry_->getKeyFrameByTimestamp(ev.timestamp, sem_ts_tol);
                    if (!kf && std::isfinite(ev.timestamp) && ev.timestamp > 0.0) {
                        kf = map_registry_->getKeyFrameByTimestamp(ev.timestamp, std::min(0.5, sem_ts_tol * 2.0));
                    }
                    if (!kf) {
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                            "[V3][SEM_CLOUD] skip publish: no keyframe for ts=%.3f (body frame); "
                            "avoid wrong T_map_body fallback",
                            ev.timestamp);
                        return;
                    }
                    Pose3d T_kf_opt = kf->T_map_b_optimized;
                    auto it = snapshot->keyframe_poses.find(kf->id);
                    if (it != snapshot->keyframe_poses.end()) {
                        T_kf_opt = it->second;
                    }
                    if (kf->pose_frame == PoseFrame::ODOM && snapshot->gps_aligned) {
                        T_kf_opt = T_map_odom * T_kf_opt;
                    }
                    pcl::transformPointCloud(*ev.labeled_cloud, *cloud_map, T_kf_opt.matrix().cast<float>());
                }
                rviz_publisher_.publishSemanticCloud(cloud_map, "map");
                const auto pub_count = semantic_cloud_events_published_.fetch_add(1, std::memory_order_relaxed) + 1;
                requestRefresh(); // 语义地标等随 MapUpdate 刷新；此处再触发一次以免仅语义帧到达时滞后
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "[V3][SEM_CLOUD][STATS] stage=publish recv_total=%lu pub_total=%lu map_pts=%zu",
                    static_cast<unsigned long>(recv_count),
                    static_cast<unsigned long>(pub_count),
                    cloud_map->size());
            } catch (const std::exception& e) {
                const auto recv_now = semantic_cloud_events_received_.load(std::memory_order_relaxed);
                const auto pub_now = semantic_cloud_events_published_.load(std::memory_order_relaxed);
                RCLCPP_ERROR(node_->get_logger(), "[V3][DIAG] step=SemanticCloud EXCEPTION: %s", e.what());
                RCLCPP_ERROR(node_->get_logger(),
                    "[V3][SEM_CLOUD][STATS] stage=exception recv_total=%lu pub_total=%lu",
                    static_cast<unsigned long>(recv_now),
                    static_cast<unsigned long>(pub_now));
            }
        });

        // 树干：maskCloud(tree_label) 后聚类前 / Trellis 聚类后 → /automap/semantic_trunk_pre_cluster|post_cluster
        onEventAsync<SemanticTrunkVizEvent>([this](const SemanticTrunkVizEvent& ev) {
            try {
                auto snapshot = map_registry_->getPoseSnapshot();
                Pose3d T_map_odom = Pose3d::Identity();
                if (snapshot->gps_aligned) {
                    T_map_odom.linear()      = snapshot->R_enu_to_map;
                    T_map_odom.translation() = snapshot->t_enu_to_map;
                }
                KeyFrame::Ptr anchor_kf = map_registry_->getLatestKeyFrameByTimestamp();
                Pose3d T_world_to_map = T_map_odom;
                if (anchor_kf) {
                    Pose3d T_k_opt = anchor_kf->T_map_b_optimized;
                    auto it_a = snapshot->keyframe_poses.find(anchor_kf->id);
                    if (it_a != snapshot->keyframe_poses.end()) {
                        T_k_opt = it_a->second;
                    }
                    if (anchor_kf->pose_frame == PoseFrame::ODOM && snapshot->gps_aligned) {
                        T_k_opt = T_map_odom * T_k_opt;
                    }
                    T_world_to_map = T_k_opt * anchor_kf->T_odom_b.inverse();
                }

                const double trunk_ts_tol = ConfigManager::instance().semanticTimestampMatchToleranceS();
                const auto to_map = [&](const CloudXYZIPtr& body) -> CloudXYZIPtr {
                    if (!body || body->empty()) return nullptr;
                    CloudXYZIPtr out(new CloudXYZI());
                    if (ev.frame_id == "world") {
                        pcl::transformPointCloud(*body, *out, T_world_to_map.matrix().cast<float>());
                    } else {
                        auto kf = map_registry_->getKeyFrameByTimestamp(ev.timestamp, trunk_ts_tol);
                        if (!kf && std::isfinite(ev.timestamp) && ev.timestamp > 0.0) {
                            kf = map_registry_->getKeyFrameByTimestamp(ev.timestamp, std::min(0.5, trunk_ts_tol * 2.0));
                        }
                        if (!kf) {
                            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                "[V3][SEM_TRUNK_VIZ] skip cloud: no keyframe for ts=%.3f (body frame)",
                                ev.timestamp);
                            return nullptr;
                        }
                        Pose3d T_kf_opt = kf->T_map_b_optimized;
                        auto it_k = snapshot->keyframe_poses.find(kf->id);
                        if (it_k != snapshot->keyframe_poses.end()) {
                            T_kf_opt = it_k->second;
                        }
                        if (kf->pose_frame == PoseFrame::ODOM && snapshot->gps_aligned) {
                            T_kf_opt = T_map_odom * T_kf_opt;
                        }
                        pcl::transformPointCloud(*body, *out, T_kf_opt.matrix().cast<float>());
                    }
                    return out;
                };

                CloudXYZIPtr pre_map = to_map(ev.pre_cluster_body);
                CloudXYZIPtr post_map = to_map(ev.post_cluster_body);
                if (pre_map && !pre_map->empty()) {
                    rviz_publisher_.publishSemanticTrunkPreCluster(pre_map, "map");
                }
                if (post_map && !post_map->empty()) {
                    rviz_publisher_.publishSemanticTrunkPostCluster(post_map, "map");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[V3][DIAG] step=SemanticTrunkViz EXCEPTION: %s", e.what());
            }
        });

        // 订阅优化结果事件
        onEventAsync<OptimizationResultEvent>([this](const OptimizationResultEvent& /*ev*/) {
            requestRefresh();
        });
        RCLCPP_INFO(node_->get_logger(),
                    "[PIPELINE][VIZ] ctor OK RvizPublisher+SyncedFrame/GPSAligned/MapUpdate/SemanticTrunk/OptResult");
    }

protected:
    void run() override {
        RCLCPP_INFO(node_->get_logger(), "[V3][VisualizationModule] Started worker thread");
        
        while (running_) {
            updateHeartbeat();
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::seconds(1), [this] { 
                    return !running_ || refresh_pending_.load(); 
                });
                if (!running_) break;
                
                if (!refresh_pending_.load()) continue;
                refresh_pending_ = false;
            }

            // 执行刷新逻辑
            publishEverything();
        }
    }

private:
    void requestRefresh() {
        refresh_pending_ = true;
        cv_.notify_one();
    }

    void publishEverything() {
        auto snapshot = map_registry_->getPoseSnapshot();
        auto all_sm = map_registry_->getAllSubMaps();
        if (all_sm.empty()) return;
        auto all_kf = map_registry_->getAllKeyFrames();
        
        Eigen::Isometry3d T_odom_to_map;
        T_odom_to_map.linear() = snapshot->R_enu_to_map;
        T_odom_to_map.translation() = snapshot->t_enu_to_map;

        bool has_correction = snapshot->gps_aligned;
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=publishEverything version=%lu sm=%zu kf=%zu T_applied=%s (grep V3 DIAG)",
            snapshot->version, all_sm.size(), all_kf.size(), has_correction ? "odom_to_map" : "identity");
        
        // 发布优化后的轨迹（使用快照中的位姿）
        rviz_publisher_.publishOptimizedPath(all_sm, snapshot);
        // 发布所有关键帧位姿
        rviz_publisher_.publishKeyframePoses(all_kf, snapshot);

        // 发布语义地标（如树木圆柱体）
        rviz_publisher_.publishSemanticLandmarks(all_sm);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
            "[V3][POSE_DIAG] Visualization published Everything: version=%lu sm=%zu kf=%zu aligned=%d",
            snapshot->version, all_sm.size(), all_kf.size(), snapshot->gps_aligned ? 1 : 0);
    }

    rclcpp::Node::SharedPtr node_;
    RvizPublisher rviz_publisher_;
    
    std::atomic<bool> refresh_pending_{false};
    std::atomic<uint64_t> semantic_cloud_events_received_{0};
    std::atomic<uint64_t> semantic_cloud_events_published_{0};
};

} // namespace automap_pro::v3
