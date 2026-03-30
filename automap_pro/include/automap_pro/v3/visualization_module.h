#pragma once
/**
 * @file v3/visualization_module.h
 * @brief V3 微内核：模块编排、事件总线、Registry、前端/语义/优化流水线。
 */


#include "automap_pro/v3/module_base.h"
#include "automap_pro/v3/pose_chain.hpp"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/visualization/rviz_publisher.h"
#include <algorithm>
#include <cmath>
#include <pcl/common/transforms.h>
#include <unordered_set>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string_view>
#include <thread>
#include <variant>

namespace automap_pro::v3 {

/**
 * @brief 可视化模块 (Visualization Micro-Kernel Module)
 * 
 * 职责：
 * 1. 负责向 RViz 发布地图、轨迹、回环等可视化信息
 * 2. 监听 MapUpdateEvent 和 OptimizationResultEvent 自动刷新显示
 * 3. SyncedFrame / 语义 viz 事件：同步订阅仅入队，由本模块 run() 单线程 FIFO 处理（禁止 EventBus::subscribeAsync
 *    每事件 detach 线程导致的乱序与 Registry 竞态）；SyncedFrame 另与 Mapping 对齐 Registry 屏障与过时 epoch 丢弃。
 * 4. V1：PoseSnapshot.session_alignments 按事件 meta.session_id 解析 T_map_odom，支持多会话对齐历史并行可视化。
 * 5. V2：链式 world→map 与 GPS 直接 T_map_odom 按平移漂移做 SE(3) 插值（blend_w），大漂移时平滑靠拢而非硬切。
 */
class VisualizationModule : public ModuleBase {
public:
    /** 合并队列上限：按 publish 到达顺序丢弃最旧项（与 subscribe 同步回调顺序一致）。 */
    static constexpr size_t kMaxVizWorkQueue = 512;

    VisualizationModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
        : ModuleBase("VisualizationModule", event_bus, map_registry), node_(node) {
        
        rviz_publisher_.init(node_);
        rviz_publisher_.setFrameId("map");

        // SyncedFrame：同步回调仅 push（O(1)），变换与 Registry 读在 run() 单线程顺序执行，避免 subscribeAsync 乱序。
        onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) { enqueueVizWork(ev); });

        // 刷新类事件：同步回调仅置位，避免 subscribeAsync 每事件起线程。
        onEvent<GPSAlignedEvent>([this](const GPSAlignedEvent& ev) {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "[V3][VIZ_REFRESH] GPSAlignedEvent success=%d epoch_obs=%lu reg_ver=%lu -> requestRefresh (grep VIZ_REFRESH)",
                ev.success ? 1 : 0,
                static_cast<unsigned long>(ev.alignment_epoch),
                static_cast<unsigned long>(map_registry_->getVersion()));
            requestRefresh();
        });

        onEvent<MapUpdateEvent>([this](const MapUpdateEvent& ev) {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "[V3][VIZ_REFRESH] MapUpdateEvent map_ver=%lu change_type=%d affected_ids=%zu -> requestRefresh (grep VIZ_REFRESH)",
                static_cast<unsigned long>(ev.version),
                static_cast<int>(ev.type),
                ev.affected_ids.size());
            requestRefresh();
        });

        // 订阅语义点云：与输入同点数（强度为类别 id，RViz 中映射为 RGB），用于「在完整几何上叠加语义」；
        // 树干/目标标注另见 /automap/semantic_landmarks（圆柱 Marker）。
        onEvent<SemanticCloudEvent>([this](const SemanticCloudEvent& ev) {
            if (!ev.isValid()) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[V3][CONTRACT] Visualization: drop invalid SemanticCloudEvent before queue ts=%.3f",
                    ev.timestamp);
                return;
            }
            semantic_cloud_events_received_.fetch_add(1, std::memory_order_relaxed);
            const auto recv_count_snapshot = semantic_cloud_events_received_.load(std::memory_order_relaxed);
            const auto pub_count_snapshot = semantic_cloud_events_published_.load(std::memory_order_relaxed);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[V3][SEM_CLOUD][STATS] stage=recv recv_total=%lu pub_total=%lu frame_id=%s pts=%zu",
                static_cast<unsigned long>(recv_count_snapshot),
                static_cast<unsigned long>(pub_count_snapshot),
                ev.frame_id.c_str(),
                ev.labeled_cloud ? ev.labeled_cloud->size() : 0);
            enqueueVizWork(ev);
        });

        // 树干：maskCloud(tree_label) 后聚类前 / Trellis 聚类后 → /automap/semantic_trunk_pre_cluster|post_cluster
        onEvent<SemanticTrunkVizEvent>([this](const SemanticTrunkVizEvent& ev) {
            if (!ev.isValid()) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[V3][CONTRACT] Visualization: drop invalid SemanticTrunkVizEvent before queue ts=%.3f",
                    ev.timestamp);
                return;
            }
            enqueueVizWork(ev);
        });

        // 订阅优化结果事件
        onEvent<OptimizationResultEvent>([this](const OptimizationResultEvent& ev) {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "[V3][VIZ_REFRESH] OptimizationResultEvent ver=%lu src=%s sm_up=%zu kf_up=%zu ep=%lu -> requestRefresh",
                static_cast<unsigned long>(ev.version),
                ev.source_module.c_str(),
                ev.submap_poses.size(),
                ev.keyframe_poses.size(),
                static_cast<unsigned long>(ev.alignment_epoch));
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
                    return !running_ || refresh_pending_.load() || !viz_work_queue_.empty();
                });
                if (!running_) {
                    break;
                }
            }

            for (;;) {
                VizWorkItem work;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (viz_work_queue_.empty()) {
                        break;
                    }
                    work = std::move(viz_work_queue_.front());
                    viz_work_queue_.pop_front();
                }
                if (auto* synced = std::get_if<SyncedFrameEvent>(&work)) {
                    const bool barrier_ok = waitRegistryBarrierForSyncedVisualization(*synced);
                    if (!barrier_ok) {
                        METRICS_INCREMENT(metrics::V3_VIZ_REGISTRY_BARRIER_TIMEOUT_TOTAL);
                        RCLCPP_WARN_THROTTLE(
                            node_->get_logger(), *node_->get_clock(), 2000,
                            "[V3][VIZ_CONTRACT] SyncedFrame barrier timeout ts=%.3f ev_v=%lu ev_ep=%lu reg_v=%lu reg_ep=%lu "
                            "(降级：仍发布当前云，由 shouldBypass / 会话 T_map_odom 处理；见 docs/V3_BARRIER_AND_META_CONTRACTS.md)",
                            synced->timestamp,
                            static_cast<unsigned long>(synced->ref_map_version),
                            static_cast<unsigned long>(synced->ref_alignment_epoch),
                            static_cast<unsigned long>(map_registry_->getVersion()),
                            static_cast<unsigned long>(map_registry_->getAlignmentEpoch()));
                    }
                    processSyncedFrameViz(*synced);
                } else if (auto* sem = std::get_if<SemanticCloudEvent>(&work)) {
                    processSemanticCloudViz(*sem);
                } else if (auto* trunk = std::get_if<SemanticTrunkVizEvent>(&work)) {
                    processSemanticTrunkViz(*trunk);
                }
            }

            bool do_refresh = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (refresh_pending_.load()) {
                    refresh_pending_ = false;
                    do_refresh = true;
                }
            }
            if (do_refresh) {
                publishEverything();
            }
        }
    }

private:
    using VizWorkItem = std::variant<SyncedFrameEvent, SemanticCloudEvent, SemanticTrunkVizEvent>;

    template <typename T>
    void enqueueVizWork(T&& payload) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (viz_work_queue_.size() >= kMaxVizWorkQueue) {
                viz_work_queue_.pop_front();
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "[V3][VIZ_BACKPRESSURE] viz_work_queue cap=%zu dropped oldest (FIFO)",
                    static_cast<unsigned long>(kMaxVizWorkQueue));
            }
            viz_work_queue_.emplace_back(std::forward<T>(payload));
        }
        cv_.notify_one();
    }

    /**
     * 与 Mapping 屏障一致：reg_ver 追上 ref_map_version，且 reg_ep >= ref_alignment_epoch（Registry 已不低于事件契约）。
     * 禁止要求 reg_ep == ref_ep：对齐升级后旧 ref_ep 帧仍应显示，由 shouldBypass 与当前 snapshot 处理坐标。
     */
    bool waitRegistryBarrierForSyncedVisualization(const SyncedFrameEvent& ev) const {
        constexpr double kWaitTimeoutS = 5.0;
        const double deadline = node_->now().seconds() + kWaitTimeoutS;
        while (running_.load()) {
            const uint64_t reg_ep = map_registry_->getAlignmentEpoch();
            const uint64_t reg_ver = map_registry_->getVersion();
            if (reg_ver >= ev.ref_map_version && reg_ep >= ev.ref_alignment_epoch) {
                return true;
            }
            if (node_->now().seconds() >= deadline) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }

    void processSyncedFrameViz(const SyncedFrameEvent& ev) {
        try {
            auto snapshot = map_registry_->getPoseSnapshot();
            // 🏛️ [对齐纪元] 过滤过时的在途帧：如果事件所属纪元落后于当前 Registry 快照纪元，
            // 说明坐标系已发生跳变（如 GPS 对齐成功），旧纪元的点云会在 map 系下产生重影，必须丢弃。
            if (ev.ref_alignment_epoch > 0 && ev.ref_alignment_epoch < snapshot->alignment_epoch) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "[V3][VIZ_EPOCH_FILTER] Dropping stale SyncedFrameEvent: ev_ep=%lu < snap_ep=%lu ts=%.3f",
                    static_cast<unsigned long>(ev.ref_alignment_epoch),
                    static_cast<unsigned long>(snapshot->alignment_epoch),
                    ev.timestamp);
                return;
            }

            // 🏛️ [架构加固] 注入 session_id 过滤，防止跨段回放时的位姿跳变
            KeyFrame::Ptr anchor_kf = map_registry_->getAnchorKeyFrameForEventTime(ev.timestamp, ev.meta.session_id);
            CloudXYZIPtr cloud_map(new CloudXYZI());
            
            const PoseSnapshot& snap_ref = *snapshot;
            const Pose3d T_map_odom_ev = pose_chain::resolveTMapOdomFromSnapshot(snap_ref, ev.meta.session_id);
            const bool gps_elev = pose_chain::sessionGpsActiveForViz(snap_ref, ev.meta.session_id);

            Pose3d T_base_to_map = Pose3d::Identity();
            if (ev.pose_frame == PoseFrame::ODOM) {
                T_base_to_map = T_map_odom_ev;
            }

            const bool bypass_anchor_chain = shouldBypassAnchorChainForVisualization(
                ev.timestamp, ev.ref_map_version, ev.ref_alignment_epoch, ev.pose_frame, anchor_kf,
                std::string_view(ev.cloud_frame));

            // GPS 对齐后：与 MappingModule 一致，不用锚点链 / blend（避免 T_chain 与 T_map_odom 双路径抖动）。
            // world 点云：p_map = T_map_odom * p_world；body 点云：p_map = T_map_odom * T_odom_b * p_body（见下方 * ev.T_odom_b）。
            const bool viz_mapping_gps_odom =
                (ev.pose_frame == PoseFrame::ODOM && gps_elev);

            Pose3d T_world_to_map = T_base_to_map;
            double blend_w = 1.0;
            double delta_norm = 0.0;
            if (!viz_mapping_gps_odom && anchor_kf && !bypass_anchor_chain) {
                const Pose3d T_k_opt = pose_chain::keyframeOptimizedInMapFrame(
                    T_map_odom_ev, gps_elev, anchor_kf->pose_frame, anchor_kf->T_map_b_optimized);
                const Pose3d T_chain =
                    pose_chain::worldToMapFromAnchorChain(T_k_opt, anchor_kf->T_odom_b);
                T_world_to_map = T_chain;
            }

            if (ConfigManager::instance().frontendCloudFrame() == "world") {
                pcl::transformPointCloud(*ev.cloud, *cloud_map, T_world_to_map.matrix().cast<float>());
            } else {
                Pose3d T_map_body = T_world_to_map * ev.T_odom_b;
                pcl::transformPointCloud(*ev.cloud, *cloud_map, T_map_body.matrix().cast<float>());
            }

            {
                const uint64_t reg_ver = map_registry_->getVersion();
                double tkx = 0, tky = 0, tkz = 0;
                uint64_t anchor_id = 0;
                double anchor_kf_ts = 0;
                int snap_has_kf = 0;
                if (anchor_kf) {
                    const Pose3d T_k_log = pose_chain::keyframeOptimizedInMapFrame(
                        T_map_odom_ev, gps_elev, anchor_kf->pose_frame, anchor_kf->T_map_b_optimized);
                    const auto& tk = T_k_log.translation();
                    tkx = tk.x();
                    tky = tk.y();
                    tkz = tk.z();
                    anchor_id = static_cast<uint64_t>(anchor_kf->id);
                    anchor_kf_ts = anchor_kf->timestamp;
                    snap_has_kf = snapshot->keyframe_poses.count(anchor_kf->id) > 0 ? 1 : 0;
                }
                const auto& tev = ev.T_odom_b.translation();
                const double dt_kf = anchor_kf ? (ev.timestamp - anchor_kf->timestamp) : 0.0;
                const double reg_max_kf_ts = map_registry_->getLatestKeyFrameTimestamp();
                const double ev_minus_reg_max =
                    std::isfinite(reg_max_kf_ts) ? (ev.timestamp - reg_max_kf_ts) : 0.0;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 3000,
                    "[V3][CURR_CLOUD_CHAIN] ev_ts=%.3f ev_ref_ver=%lu ev_ref_ep=%lu snap_ver=%lu snap_ep=%lu reg_ver=%lu "
                    "anchor_id=%lu session_id=%lu anchor_kf_ts=%.3f dt_ev_minus_anchor_kf=%.3f reg_max_kf_ts=%.3f ev_minus_reg_max_kf_ts=%.3f "
                    "bypass_chain=%d cloud_frame=%s pose_frame=%d delta_norm=%.3f blend_w=%.3f "
                    "mapping_gps_odom=%d "
                    "pts_in=%zu pts_out=%zu T_k_t=[%.3f,%.3f,%.3f] T_ev_odom_t=[%.3f,%.3f,%.3f] "
                    "snap_has_anchor_kf_pose=%d T_k_source=kf_object",
                    ev.timestamp,
                    static_cast<unsigned long>(ev.ref_map_version),
                    static_cast<unsigned long>(ev.ref_alignment_epoch),
                    static_cast<unsigned long>(snapshot->version),
                    static_cast<unsigned long>(snapshot->alignment_epoch),
                    static_cast<unsigned long>(reg_ver),
                    static_cast<unsigned long>(anchor_id),
                    static_cast<unsigned long>(ev.meta.session_id),
                    anchor_kf_ts,
                    dt_kf,
                    reg_max_kf_ts,
                    ev_minus_reg_max,
                    bypass_anchor_chain ? 1 : 0,
                    ev.cloud_frame.c_str(),
                    static_cast<int>(ev.pose_frame),
                    delta_norm,
                    blend_w,
                    viz_mapping_gps_odom ? 1 : 0,
                    ev.cloud ? ev.cloud->size() : 0,
                    cloud_map->size(),
                    tkx,
                    tky,
                    tkz,
                    tev.x(),
                    tev.y(),
                    tev.z(),
                    snap_has_kf);
            }

            METRICS_GAUGE_SET(metrics::V3_VIZ_LAST_CURRENT_CLOUD_BLEND_W, blend_w);
            METRICS_HISTOGRAM_OBSERVE(metrics::V3_VIZ_CURRENT_CLOUD_CHAIN_DRIFT_M, delta_norm);

            rviz_publisher_.publishCurrentCloud(cloud_map);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][DIAG] step=SyncedFrame_cloud EXCEPTION: %s fallback=raw", e.what());
            rviz_publisher_.publishCurrentCloud(ev.cloud);
        }
    }

    void processSemanticCloudViz(const SemanticCloudEvent& ev) {
        try {
            if (!ev.isValid()) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[V3][CONTRACT] SemanticCloudViz: skip invalid SemanticCloudEvent ts=%.3f", ev.timestamp);
                return;
            }

            auto snapshot = map_registry_->getPoseSnapshot();
            // 🏛️ [对齐纪元] 过滤过时的在途帧
            if (ev.meta.ref_epoch > 0 && ev.meta.ref_epoch < snapshot->alignment_epoch) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "[V3][VIZ_EPOCH_FILTER] Dropping stale SemanticCloudEvent: ev_ep=%lu < snap_ep=%lu ts=%.3f",
                    static_cast<unsigned long>(ev.meta.ref_epoch),
                    static_cast<unsigned long>(snapshot->alignment_epoch),
                    ev.timestamp);
                return;
            }

            const uint64_t sem_sess =
                ev.meta.session_id != 0 ? ev.meta.session_id : map_registry_->getSessionId();
            // 🏛️ [架构加固] 注入 session_id 过滤
            KeyFrame::Ptr anchor_kf = map_registry_->getAnchorKeyFrameForEventTime(ev.timestamp, sem_sess);
            CloudXYZIPtr cloud_map(new CloudXYZI());
            
            const PoseSnapshot& snap_ref = *snapshot;
            const Pose3d T_map_odom_ev = pose_chain::resolveTMapOdomFromSnapshot(snap_ref, sem_sess);
            const bool gps_elev = pose_chain::sessionGpsActiveForViz(snap_ref, sem_sess);

            Pose3d T_world_to_map = T_map_odom_ev;
            double blend_w_sem = 1.0;
            double delta_norm_sem = 0.0;
            const bool bypass_sem_world = shouldBypassAnchorChainForVisualization(
                ev.timestamp, 0, 0, PoseFrame::ODOM, anchor_kf, std::string_view(ev.frame_id));
            // world 系语义云：GPS 对齐后与 Mapping 一致，仅用 T_map_odom（不锚点链 / 不 blend）
            if (anchor_kf && !bypass_sem_world && !gps_elev) {
                const Pose3d T_k_opt = pose_chain::keyframeOptimizedInMapFrame(
                    T_map_odom_ev, gps_elev, anchor_kf->pose_frame, anchor_kf->T_map_b_optimized);
                const Pose3d T_chain =
                    pose_chain::worldToMapFromAnchorChain(T_k_opt, anchor_kf->T_odom_b);
                T_world_to_map = T_chain;
            }

            const double sem_ts_tol = ConfigManager::instance().semanticTimestampMatchToleranceS();
            if (ev.frame_id == "world") {
                pcl::transformPointCloud(*ev.labeled_cloud, *cloud_map, T_world_to_map.matrix().cast<float>());
                RCLCPP_DEBUG_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 3000,
                    "[V3][SEM_CLOUD_CHAIN] frame=world ts=%.3f snap_ver=%lu reg_ver=%lu anchor_id=%lu pts=%zu",
                    ev.timestamp,
                    static_cast<unsigned long>(snapshot->version),
                    static_cast<unsigned long>(map_registry_->getVersion()),
                    static_cast<unsigned long>(anchor_kf ? anchor_kf->id : 0),
                    ev.labeled_cloud ? ev.labeled_cloud->size() : 0);
            } else {
                auto kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp, sem_ts_tol, sem_sess);
                if (!kf && std::isfinite(ev.timestamp) && ev.timestamp > 0.0) {
                    kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp,
                                                                        std::min(0.5, sem_ts_tol * 2.0),
                                                                        sem_sess);
                }
                if (!kf) {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                         "[V3][SEM_CLOUD] skip publish: no keyframe for ts=%.3f (body frame); "
                                         "avoid wrong T_map_body fallback",
                                         ev.timestamp);
                    return;
                }
                const Pose3d T_kf_opt = pose_chain::keyframeOptimizedInMapFrame(
                    T_map_odom_ev, gps_elev, kf->pose_frame, kf->T_map_b_optimized);
                pcl::transformPointCloud(*ev.labeled_cloud, *cloud_map, T_kf_opt.matrix().cast<float>());
                RCLCPP_DEBUG_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 3000,
                    "[V3][SEM_CLOUD_CHAIN] frame=body ts=%.3f kf_id=%lu kf_ts=%.3f snap_ver=%lu reg_ver=%lu pts=%zu",
                    ev.timestamp,
                    static_cast<unsigned long>(kf->id),
                    kf->timestamp,
                    static_cast<unsigned long>(snapshot->version),
                    static_cast<unsigned long>(map_registry_->getVersion()),
                    ev.labeled_cloud ? ev.labeled_cloud->size() : 0);
            }
            METRICS_GAUGE_SET(metrics::V3_VIZ_LAST_CURRENT_CLOUD_BLEND_W, blend_w_sem);
            METRICS_HISTOGRAM_OBSERVE(metrics::V3_VIZ_CURRENT_CLOUD_CHAIN_DRIFT_M, delta_norm_sem);

            rviz_publisher_.publishSemanticCloud(cloud_map, "map");
            const auto pub_count = semantic_cloud_events_published_.fetch_add(1, std::memory_order_relaxed) + 1;
            requestRefresh();
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[V3][SEM_CLOUD][STATS] stage=publish recv_total=%lu pub_total=%lu map_pts=%zu",
                                 static_cast<unsigned long>(semantic_cloud_events_received_.load(std::memory_order_relaxed)),
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
    }

    void processSemanticTrunkViz(const SemanticTrunkVizEvent& ev) {
        try {
            if (!ev.isValid()) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[V3][CONTRACT] SemanticTrunkViz: skip invalid SemanticTrunkVizEvent ts=%.3f", ev.timestamp);
                return;
            }

            auto snapshot = map_registry_->getPoseSnapshot();
            // 🏛️ [对齐纪元] 过滤过时的在途帧
            if (ev.meta.ref_epoch > 0 && ev.meta.ref_epoch < snapshot->alignment_epoch) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "[V3][VIZ_EPOCH_FILTER] Dropping stale SemanticTrunkVizEvent: ev_ep=%lu < snap_ep=%lu ts=%.3f",
                    static_cast<unsigned long>(ev.meta.ref_epoch),
                    static_cast<unsigned long>(snapshot->alignment_epoch),
                    ev.timestamp);
                return;
            }

            const uint64_t sem_sess =
                ev.meta.session_id != 0 ? ev.meta.session_id : map_registry_->getSessionId();
            // 🏛️ [架构加固] 注入 session_id 过滤
            KeyFrame::Ptr anchor_kf = map_registry_->getAnchorKeyFrameForEventTime(ev.timestamp, sem_sess);
            const PoseSnapshot& snap_ref = *snapshot;
            const Pose3d T_map_odom_ev = pose_chain::resolveTMapOdomFromSnapshot(snap_ref, sem_sess);
            const bool gps_elev = pose_chain::sessionGpsActiveForViz(snap_ref, sem_sess);
            Pose3d T_world_to_map = T_map_odom_ev;
            double blend_w_trunk = 1.0;
            double delta_norm_trunk = 0.0;
            const bool bypass_trunk_world = shouldBypassAnchorChainForVisualization(
                ev.timestamp, 0, 0, PoseFrame::ODOM, anchor_kf, std::string_view(ev.frame_id));
            if (anchor_kf && !bypass_trunk_world && !gps_elev) {
                const Pose3d T_k_opt = pose_chain::keyframeOptimizedInMapFrame(
                    T_map_odom_ev, gps_elev, anchor_kf->pose_frame, anchor_kf->T_map_b_optimized);
                const Pose3d T_chain =
                    pose_chain::worldToMapFromAnchorChain(T_k_opt, anchor_kf->T_odom_b);
                T_world_to_map = T_chain;
            }

            const double trunk_ts_tol = ConfigManager::instance().semanticTimestampMatchToleranceS();
            const auto to_map = [&](const CloudXYZIPtr& body) -> CloudXYZIPtr {
                if (!body || body->empty()) {
                    return nullptr;
                }
                CloudXYZIPtr out(new CloudXYZI());
                if (ev.frame_id == "world") {
                    pcl::transformPointCloud(*body, *out, T_world_to_map.matrix().cast<float>());
                } else {
                    auto kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp, trunk_ts_tol, sem_sess);
                    if (!kf && std::isfinite(ev.timestamp) && ev.timestamp > 0.0) {
                        kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp,
                                                                            std::min(0.5, trunk_ts_tol * 2.0),
                                                                            sem_sess);
                    }
                    if (!kf) {
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                             "[V3][SEM_TRUNK_VIZ] skip cloud: no keyframe for ts=%.3f (body frame)",
                                             ev.timestamp);
                        return nullptr;
                    }
                    const Pose3d T_kf_opt = pose_chain::keyframeOptimizedInMapFrame(
                        T_map_odom_ev, gps_elev, kf->pose_frame, kf->T_map_b_optimized);
                    pcl::transformPointCloud(*body, *out, T_kf_opt.matrix().cast<float>());
                }
                return out;
            };

            CloudXYZIPtr pre_map = to_map(ev.pre_cluster_body);
            CloudXYZIPtr post_map = to_map(ev.post_cluster_body);
            METRICS_GAUGE_SET(metrics::V3_VIZ_LAST_CURRENT_CLOUD_BLEND_W, blend_w_trunk);
            METRICS_HISTOGRAM_OBSERVE(metrics::V3_VIZ_CURRENT_CLOUD_CHAIN_DRIFT_M, delta_norm_trunk);

            if (pre_map && !pre_map->empty()) {
                rviz_publisher_.publishSemanticTrunkPreCluster(pre_map, "map");
            }
            if (post_map && !post_map->empty()) {
                rviz_publisher_.publishSemanticTrunkPostCluster(post_map, "map");
            }
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 4000,
                "[V3][SEM_TRUNK_CHAIN] ts=%.3f frame_id=%s snap_ver=%lu pre_pts=%zu post_pts=%zu",
                ev.timestamp,
                ev.frame_id.c_str(),
                static_cast<unsigned long>(snapshot->version),
                pre_map ? pre_map->size() : 0,
                post_map ? post_map->size() : 0);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][DIAG] step=SemanticTrunkViz EXCEPTION: %s", e.what());
        }
    }

    /**
     * Registry 版本滞后：bypass。对齐世代：仅 reg_ep < ref_ep 时 bypass；reg_ep > ref_ep 且 world+anchor 仍走链（标签过时）。
     * 锚点 dt bypass：仅非 world；world 在两 KF 之间 dt 大，切 T_map_odom-only 会阶跃（run_20260328_233220 62181/62198）。
     * 链式变换与漂移混合实现见 pose_chain.hpp。
     */
    bool shouldBypassAnchorChainForVisualization(double event_ts,
                                                   uint64_t ref_map_version,
                                                   uint64_t ref_alignment_epoch,
                                                   PoseFrame pose_frame,
                                                   KeyFrame::Ptr anchor_kf,
                                                   std::string_view cloud_or_frame_id) const {
        if (pose_frame != PoseFrame::ODOM) {
            return false;
        }
        const double max_chain_age_s = std::max(0.5, 1.2 * ConfigManager::instance().kfMaxInterval());
        const bool viz_world = (cloud_or_frame_id == "world");
        if (ref_map_version > 0 && map_registry_->getVersion() < ref_map_version) {
            return true;
        }
        // 仅当 Registry 对齐世代严格落后于事件 ref 时 bypass；ref<reg（事件标签过时）在 world+anchor 下仍走链，
        // 避免 GPS 后邻帧间「链式 / 纯 T_map_odom」来回切（full.log 中 ep 已一致仍可能因旧逻辑误判）。
        if (ref_alignment_epoch > 0) {
            const uint64_t reg_ep = map_registry_->getAlignmentEpoch();
            if (reg_ep < ref_alignment_epoch) {
                return true;
            }
            if (reg_ep != ref_alignment_epoch && !(viz_world && anchor_kf)) {
                return true;
            }
        }
        if (!viz_world && anchor_kf && std::isfinite(event_ts) &&
            (event_ts - anchor_kf->timestamp > max_chain_age_s)) {
            return true;
        }
        return false;
    }

    void requestRefresh() {
        refresh_pending_ = true;
        cv_.notify_one();
    }

    void publishEverything() {
        auto snapshot = map_registry_->getPoseSnapshot();
        auto all_sm = map_registry_->getAllSubMaps();
        if (all_sm.empty()) return;
        auto all_kf = map_registry_->getAllKeyFrames();

        std::unordered_set<uint64_t> kf_sessions;
        for (const auto& kf : all_kf) {
            if (kf) {
                kf_sessions.insert(kf->session_id);
            }
        }
        if (kf_sessions.size() > 1u) {
            METRICS_INCREMENT(metrics::V3_VIZ_MULTI_SESSION_AGGREGATE_TOTAL);
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 10000,
                "[V3][VIZ_AGGREGATE_SESSION] distinct_kf_session_count=%zu：全图 optimized_path / keyframe_poses "
                "使用快照全局 T_map_odom，仅在「单会话 + 单一 GPS 对齐」下与数学真值一致；多会话请依赖 "
                "SyncedFrame/按会话可视化或分话题。详见 docs/V3_BARRIER_AND_META_CONTRACTS.md",
                kf_sessions.size());
        }

        Eigen::Isometry3d T_odom_to_map;
        T_odom_to_map.linear() = snapshot->R_enu_to_map;
        T_odom_to_map.translation() = snapshot->t_enu_to_map;

        bool has_correction = snapshot->gps_aligned;
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "[V3][VIZ_PUBLISH_ALL] snap_ver=%lu snap_ep=%lu reg_ver=%lu sm=%zu kf=%zu snap_kf_pose_entries=%zu "
            "gps_aligned=%d (grep VIZ_PUBLISH_ALL; path+kf_poses use snapshot)",
            static_cast<unsigned long>(snapshot->version),
            static_cast<unsigned long>(snapshot->alignment_epoch),
            static_cast<unsigned long>(map_registry_->getVersion()),
            all_sm.size(),
            all_kf.size(),
            snapshot->keyframe_poses.size(),
            snapshot->gps_aligned ? 1 : 0);
        
        // 发布优化后的轨迹（使用快照中的位姿）
        rviz_publisher_.publishOptimizedPath(all_sm, snapshot, snapshot->alignment_epoch);
        // 发布所有关键帧位姿
        rviz_publisher_.publishKeyframePoses(all_kf, snapshot, snapshot->alignment_epoch);

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

    std::deque<VizWorkItem> viz_work_queue_;
};

} // namespace automap_pro::v3
