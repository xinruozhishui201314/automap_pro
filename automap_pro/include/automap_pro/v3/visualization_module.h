#pragma once

#include "automap_pro/v3/module_base.h"
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

        // 订阅同步帧（用于显示当前点云）
        onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
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

                KeyFrame::Ptr anchor_kf = map_registry_->getLatestKeyFrameByTimestamp();
                Pose3d T_world_to_map = Pose3d::Identity();
                if (anchor_kf) {
                    Pose3d T_k_opt = anchor_kf->T_map_b_optimized;
                    auto it = snapshot->keyframe_poses.find(anchor_kf->id);
                    if (it != snapshot->keyframe_poses.end()) {
                        T_k_opt = it->second;
                    }
                    // p_map ≈ T_k_opt * inv(T_k_odom) * p_lio_world（帧间相对仍用前端里程计）
                    T_world_to_map = T_k_opt * anchor_kf->T_odom_b.inverse();
                }

                if (ConfigManager::instance().frontendCloudFrame() == "world") {
                    if (anchor_kf) {
                        pcl::transformPointCloud(*ev.cloud, *cloud_map, T_world_to_map.matrix().cast<float>());
                    } else {
                        pcl::transformPointCloud(*ev.cloud, *cloud_map, T_map_odom.matrix().cast<float>());
                    }
                } else {
                    if (anchor_kf) {
                        Pose3d T_map_body = T_world_to_map * ev.T_odom_b;
                        pcl::transformPointCloud(*ev.cloud, *cloud_map, T_map_body.matrix().cast<float>());
                    } else {
                        Pose3d T_map_body = T_map_odom * ev.T_odom_b;
                        pcl::transformPointCloud(*ev.cloud, *cloud_map, T_map_body.matrix().cast<float>());
                    }
                }
                rviz_publisher_.publishCurrentCloud(cloud_map);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[V3][DIAG] step=SyncedFrame_cloud EXCEPTION: %s fallback=raw", e.what());
                rviz_publisher_.publishCurrentCloud(ev.cloud);
            }
        });

        // 🏛️ 已移除同步回调中的复杂逻辑，全部改由 publishEverything 使用快照处理
        onEvent<GPSAlignedEvent>([this](const GPSAlignedEvent& /*ev*/) {
            // 仅触发刷新，实际对齐逻辑已由 MapRegistry 快照化
            requestRefresh();
        });

        // 订阅地图变更事件
        onEvent<MapUpdateEvent>([this](const MapUpdateEvent& /*ev*/) {
            requestRefresh();
        });

        // 订阅优化结果事件
        onEvent<OptimizationResultEvent>([this](const OptimizationResultEvent& /*ev*/) {
            requestRefresh();
        });
        RCLCPP_INFO(node_->get_logger(),
                    "[PIPELINE][VIZ] ctor OK RvizPublisher+SyncedFrame/GPSAligned/MapUpdate/OptResult");
    }

protected:
    void run() override {
        RCLCPP_INFO(node_->get_logger(), "[V3][VisualizationModule] Started worker thread");
        
        while (running_) {
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

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
            "[V3][POSE_DIAG] Visualization published Everything: version=%lu sm=%zu kf=%zu aligned=%d",
            snapshot->version, all_sm.size(), all_kf.size(), snapshot->gps_aligned ? 1 : 0);
    }

    rclcpp::Node::SharedPtr node_;
    RvizPublisher rviz_publisher_;
    
    std::atomic<bool> refresh_pending_{false};
};

} // namespace automap_pro::v3
