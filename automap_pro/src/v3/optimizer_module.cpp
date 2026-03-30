/**
 * @file v3/optimizer_module.cpp
 * @brief V3 流水线模块实现。
 */
#include "automap_pro/v3/optimizer_module.h"
#include "automap_pro/backend/gps_constraint_policy.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/v3/map_registry.h"
#include <algorithm>
#include <atomic>
#include <limits>
#include <vector>

namespace {
std::atomic<uint64_t> g_gps_kf_candidates_total{0};
std::atomic<uint64_t> g_gps_kf_added_total{0};
std::atomic<uint64_t> g_gps_kf_reject_quality_total{0};

uint64_t computeBatchHash(const std::unordered_map<int, automap_pro::Pose3d>& sm_poses,
                          const std::unordered_map<uint64_t, automap_pro::Pose3d>& kf_poses) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    mix(static_cast<uint64_t>(sm_poses.size()));
    mix(static_cast<uint64_t>(kf_poses.size()));
    std::vector<int> sm_ids;
    sm_ids.reserve(sm_poses.size());
    for (const auto& [id, _] : sm_poses) sm_ids.push_back(id);
    std::sort(sm_ids.begin(), sm_ids.end());
    for (int id : sm_ids) {
        const auto& pose = sm_poses.at(id);
        mix(static_cast<uint64_t>(id));
        mix(static_cast<uint64_t>(pose.translation().x() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().y() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().z() * 1000.0));
        mix(static_cast<uint64_t>(pose.linear()(0, 0) * 1e6));
        mix(static_cast<uint64_t>(pose.linear()(1, 1) * 1e6));
        mix(static_cast<uint64_t>(pose.linear()(2, 2) * 1e6));
    }
    std::vector<uint64_t> kf_ids;
    kf_ids.reserve(kf_poses.size());
    for (const auto& [id, _] : kf_poses) kf_ids.push_back(id);
    std::sort(kf_ids.begin(), kf_ids.end());
    for (uint64_t id : kf_ids) {
        const auto& pose = kf_poses.at(id);
        mix(id);
        mix(static_cast<uint64_t>(pose.translation().x() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().y() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().z() * 1000.0));
        mix(static_cast<uint64_t>(pose.linear()(0, 0) * 1e6));
        mix(static_cast<uint64_t>(pose.linear()(1, 1) * 1e6));
        mix(static_cast<uint64_t>(pose.linear()(2, 2) * 1e6));
    }
    return h;
}

uint64_t semanticTraceId(uint64_t kf_id, double ts) {
    const uint64_t ts_us = std::isfinite(ts) ? static_cast<uint64_t>(std::max(0.0, ts) * 1e6) : 0ull;
    return (kf_id << 20) ^ ts_us;
}

/** `kf->gps.position_enu` 为 IMU 在 ENU（前端 onGPS 已与 GPSManager 同步做杆臂）；仅 ENU→map。 */
inline Eigen::Vector3d gpsImuPositionInMapFromImuEnu(const Eigen::Matrix3d& R_enu_to_map,
                                                    const Eigen::Vector3d& t_enu_to_map,
                                                    const Eigen::Vector3d& position_enu_imu) {
    return R_enu_to_map * position_enu_imu + t_enu_to_map;
}
} // namespace

namespace automap_pro::v3 {

OptimizerModule::OptimizerModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("OptimizerModule", event_bus, map_registry), node_(node) {

    optimizer_.registerPoseUpdateCallback([this](const OptimizationResult& res) {
        try {
        if (!running_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[V3][CONTRACT] Reject optimizer callback: module is stopping/stopped");
            return;
        }
        if (res.pose_frame == PoseFrame::UNKNOWN) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject optimization result: success-path pose_frame=UNKNOWN");
            METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
            return;
        }
        if (!res.isFinite() || !res.isReasonable()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject optimization result: non-finite or unreasonable pose");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
        if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" &&
            res.pose_frame != PoseFrame::MAP &&
            !map_registry_->isGPSAligned()) {
            RCLCPP_WARN(node_->get_logger(),
                "[V3][CONTRACT] strict_map_only bypassed before GPS alignment: accepting non-MAP optimizer result");
        } else if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" &&
                   res.pose_frame != PoseFrame::MAP) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] strict_map_only rejects non-MAP optimizer result");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=Optimizer_poseCallback_enter sm=%zu kf=%zu (grep V3 DIAG)",
            res.submap_poses.size(), res.keyframe_poses.size());
        const auto try_publish = [this, &res](uint64_t source_alignment_epoch) -> uint64_t {
            std::unordered_map<int, Pose3d> sm_updates = res.submap_poses;
            std::unordered_map<uint64_t, Pose3d> kf_updates = res.keyframe_poses;
            PoseFrame output_frame = res.pose_frame;
            uint32_t transform_flags = static_cast<uint32_t>(OptimizationTransformFlags::NONE);
            const double publish_ts = node_->now().seconds();

            // Keep Optimizer->MapRegistry contract consistent with Mapping gateway:
            // once GPS aligned, convert ODOM results to MAP before writing into SSoT.
            if (map_registry_->isGPSAligned() && output_frame != PoseFrame::MAP) {
                Eigen::Matrix3d R_enu_to_map = Eigen::Matrix3d::Identity();
                Eigen::Vector3d t_enu_to_map = Eigen::Vector3d::Zero();
                map_registry_->getGPSTransform(R_enu_to_map, t_enu_to_map);

                Pose3d T_map_odom = Pose3d::Identity();
                T_map_odom.linear() = R_enu_to_map;
                T_map_odom.translation() = t_enu_to_map;
                if (!T_map_odom.matrix().allFinite()) {
                    RCLCPP_ERROR(node_->get_logger(),
                        "[V3][CONTRACT] Reject optimizer result: non-finite GPS transform while converting ODOM->MAP");
                    METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
                    return map_registry_->getVersion();
                }

                for (auto& [id, pose] : sm_updates) pose = T_map_odom * pose;
                for (auto& [id, pose] : kf_updates) pose = T_map_odom * pose;
                output_frame = PoseFrame::MAP;
                transform_flags |= static_cast<uint32_t>(OptimizationTransformFlags::MAP_COMPENSATION_APPLIED);
            }
            OptimizationDeltaEvent delta_ev;
            delta_ev.alignment_epoch = source_alignment_epoch;
            delta_ev.pose_frame = output_frame;
            delta_ev.submap_delta = sm_updates;
            delta_ev.keyframe_delta = kf_updates;
            delta_ev.producer = "OptimizerModule";
            delta_ev.meta.event_id = map_registry_->getVersion() + 1;
            delta_ev.meta.idempotency_key = computeBatchHash(sm_updates, kf_updates);
            if (delta_ev.meta.idempotency_key == 0) delta_ev.meta.idempotency_key = delta_ev.meta.event_id;
            delta_ev.meta.producer_seq = delta_ev.meta.event_id;
            delta_ev.meta.ref_version = map_registry_->getVersion();
            delta_ev.meta.ref_epoch = source_alignment_epoch;
            delta_ev.meta.session_id = map_registry_->getSessionId();
            delta_ev.meta.source_ts = publish_ts;
            delta_ev.meta.publish_ts = publish_ts;
            delta_ev.meta.producer = "OptimizerModule";
            delta_ev.meta.route_tag = "legacy";
            if (delta_ev.isValid()) {
                event_bus_->publish(delta_ev);
            }
            uint64_t batch_hash = computeBatchHash(sm_updates, kf_updates);
            return map_registry_->updatePoses(
                sm_updates, kf_updates, output_frame, "OptimizerModule", source_alignment_epoch,
                transform_flags, batch_hash);
        };

        const uint64_t source_alignment_epoch = map_registry_->getAlignmentEpoch();
        const uint64_t version_before = map_registry_->getVersion();
        uint64_t version = try_publish(source_alignment_epoch);
        if (version == version_before &&
            (!res.submap_poses.empty() || !res.keyframe_poses.empty()) &&
            map_registry_->getAlignmentEpoch() != source_alignment_epoch) {
            const uint64_t retry_epoch = map_registry_->getAlignmentEpoch();
            RCLCPP_WARN(node_->get_logger(),
                "[V3][CONTRACT] Optimizer publish epoch-race detected, retry once: epoch=%lu -> %lu",
                static_cast<unsigned long>(source_alignment_epoch),
                static_cast<unsigned long>(retry_epoch));
            version = try_publish(retry_epoch);
        }
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=Optimizer_poseCallback_done version=%lu (grep V3 DIAG)",
            static_cast<unsigned long>(version));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][OptimizerModule][EXCEPTION] pose update callback failed: %s (drop this update, keep process alive)",
                e.what());
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][OptimizerModule][EXCEPTION] pose update callback unknown exception (drop this update, keep process alive)");
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        }
    });

    onEvent<LoopConstraintEvent>([this](const LoopConstraintEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(task_mutex_);
        OptTaskItem task;
        task.type = OptTaskItem::Type::LOOP_FACTOR;
        task.loop_constraint = ev.constraint;
        task_queue_.push_back(task);
        cv_.notify_one();
    });

    onEvent<GraphTaskEvent>([this](const GraphTaskEvent& ev) {
        if (!running_.load()) return;
        if (!ev.isValid()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][CONTRACT] OptimizerModule: drop GraphTaskEvent with invalid EventMeta");
            return;
        }
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_queue_.push_back(ev.task);
        cv_.notify_one();
    });

    onEvent<MapUpdateEvent>([](const MapUpdateEvent& /*ev*/) {
        // 预留：在 run() / MapRegistry 中处理新关键帧等
    });
    RCLCPP_INFO(node_->get_logger(),
                "[PIPELINE][OPT] ctor OK IncrementalOptimizer+Loop/Graph/MapUpdate tasks");
}

bool OptimizerModule::isIdle() const {
    std::lock_guard<std::mutex> lock(task_mutex_);
    return task_queue_.empty();
}

std::vector<std::pair<std::string, size_t>> OptimizerModule::queueDepths() const {
    std::lock_guard<std::mutex> lock(task_mutex_);
    return {{"task_queue", task_queue_.size()}};
}

std::string OptimizerModule::idleDetail() const {
    return "";
}

void OptimizerModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][OptimizerModule] Started worker thread");
    
    while (running_) {
        try {
            updateHeartbeat();
            std::deque<OptTaskItem> tasks;
            {
                std::unique_lock<std::mutex> lock(task_mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(100), 
                    [this] { return !running_ || !task_queue_.empty(); });
                
                if (!running_) break;
                if (task_queue_.empty()) continue;
                
                // 🏛️ [P0 性能优化] 任务合并 (Task Coalescing)
                // 一次性取出当前队列中所有任务进行批量处理，显著减少 iSAM2::update() 的调用频率
                // 对于高频关键帧流，这能极大降低 GTSAM 重线性化带来的 CPU 抖动
                tasks = std::move(task_queue_);
                task_queue_.clear();
            }

            // 处理一批优化任务
            bool needs_update = false;

            for (const auto& task : tasks) {
                if (task.type == OptTaskItem::Type::RESET) {
                    optimizer_.reset();
                    needs_update = false; // reset 后不需要立即 update
                    continue;
                }
                
                if (task.type == OptTaskItem::Type::FORCE_UPDATE) {
                    needs_update = true;
                }

                // 如果已经 reset，后续非关键任务在同一批次中可能不再有意义，但为了逻辑简单继续处理
                processTaskInternal(task);
                
                // 保证版本推进：任何非 RESET 批次都至少触发一次优化提交
                needs_update = true;
            }

            if (needs_update && running_) {
                optimizer_.forceUpdate();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][OptimizerModule][EXCEPTION] run loop caught: %s (isolate+continue)", e.what());
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][OptimizerModule][EXCEPTION] run loop unknown exception (isolate+continue)");
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        }
    }
}

void OptimizerModule::processTaskInternal(const OptTaskItem& task) {
    switch (task.type) {
        case OptTaskItem::Type::LOOP_FACTOR:
            optimizer_.addLoopFactor(task.loop_constraint);
            break;
        case OptTaskItem::Type::FORCE_UPDATE:
            // 仅标记需要 update，循环末尾会统一执行
            break;
        case OptTaskItem::Type::GPS_FACTOR:
            optimizer_.addGPSFactor(task.to_id, task.gps_pos, task.gps_cov);
            break;
        case OptTaskItem::Type::SUBMAP_NODE:
            optimizer_.addSubMapNode(task.to_id, task.rel_pose, task.fixed);
            break;
        case OptTaskItem::Type::ODOM_FACTOR:
            optimizer_.addOdomFactor(task.from_id, task.to_id, task.rel_pose, task.info_matrix);
            break;
        case OptTaskItem::Type::KEYFRAME_CREATE:
            processKeyframeCreate(task);
            break;
        case OptTaskItem::Type::GPS_BATCH_KF:
            processGPSBatchKF(task);
            break;
        case OptTaskItem::Type::CYLINDER_LANDMARK_FACTOR:
            processCylinderLandmarkFactor(task);
            break;
        case OptTaskItem::Type::PLANE_LANDMARK_FACTOR:
            processPlaneLandmarkFactor(task);
            break;
        default:
            break;
    }
}

void OptimizerModule::processCylinderLandmarkFactor(const OptTaskItem& task) {
    if (task.cylinder_factors.empty()) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Optimizer][processCylinderLandmarkFactor] step=skip reason=empty_factors kf_id=%d",
            task.to_id);
        return;
    }
    uint64_t kf_id = static_cast<uint64_t>(task.to_id);
    double approx_ts = 0.0;
    if (!task.cylinder_factors.empty()) {
        kf_id = task.cylinder_factors.front().kf_id;
        if (auto kf = map_registry_->getKeyFrame(static_cast<int>(kf_id))) {
            approx_ts = kf->timestamp;
        }
    }
    const uint64_t trace_id = semanticTraceId(kf_id, approx_ts);

    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Optimizer][processCylinderLandmarkFactor] step=entry trace=%lu kf_id=%d factors=%zu",
        static_cast<unsigned long>(trace_id), task.to_id, task.cylinder_factors.size());

    for (const auto& factor : task.cylinder_factors) {
        optimizer_.addCylinderFactorForKeyFrame(static_cast<int>(factor.kf_id), factor);
    }

    RCLCPP_INFO(node_->get_logger(),
        "[CHAIN][B5 SEM->OPT] action=factor_applied trace=%lu kf_id=%d factors=%zu",
        static_cast<unsigned long>(trace_id), task.to_id, task.cylinder_factors.size());
}

void OptimizerModule::processPlaneLandmarkFactor(const OptTaskItem& task) {
    if (task.plane_factors.empty()) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Optimizer][processPlaneLandmarkFactor] step=skip reason=empty_factors kf_id=%d",
            task.to_id);
        return;
    }
    uint64_t kf_id = static_cast<uint64_t>(task.to_id);
    double approx_ts = 0.0;
    if (!task.plane_factors.empty()) {
        kf_id = task.plane_factors.front().kf_id;
        if (auto kf = map_registry_->getKeyFrame(static_cast<int>(kf_id))) {
            approx_ts = kf->timestamp;
        }
    }
    const uint64_t trace_id = semanticTraceId(kf_id, approx_ts);

    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Optimizer][processPlaneLandmarkFactor] step=entry trace=%lu kf_id=%d factors=%zu",
        static_cast<unsigned long>(trace_id), task.to_id, task.plane_factors.size());

    for (const auto& factor : task.plane_factors) {
        optimizer_.addPlaneFactorForKeyFrame(static_cast<int>(factor.kf_id), factor);
    }

    RCLCPP_INFO(node_->get_logger(),
        "[CHAIN][B5 SEM->OPT] action=plane_factor_applied trace=%lu kf_id=%d factors=%zu",
        static_cast<unsigned long>(trace_id), task.to_id, task.plane_factors.size());
}

void OptimizerModule::processTask(const OptTaskItem& task) {
    // 兼容旧接口：立即处理并更新
    if (task.type == OptTaskItem::Type::RESET) {
        optimizer_.reset();
        return;
    }
    processTaskInternal(task);
    optimizer_.forceUpdate();
}

void OptimizerModule::processGPSBatchKF(const OptTaskItem& task) {
    const Eigen::Matrix3d& R = task.R_enu_to_map;
    const Eigen::Vector3d& t = task.t_enu_to_map;
    auto all_kfs = map_registry_->getAllKeyFrames();
    std::vector<IncrementalOptimizer::GPSFactorItemKF> factors;
    size_t reject_quality = 0;
    const auto& cfg = ConfigManager::instance();
    const double gps_max_dt = cfg.gpsKeyframeMatchWindowS();

    // 🏛️ [对齐逻辑] 若当前处于 ODOM 系，注入批量 GPS 因子前必须先进行坐标系转换与重建
    // [RC1 修复] 传入 suppress_pose_notify=true：REBUILD 产生的是仅含 T_map_odom 平移的中间态，
    // GPS 位置因子尚未注入，直接发布该状态会导致轨迹跳变 3-5m。
    // addGPSFactorsForKeyFramesBatch 之后会在收敛后统一调用 notifyPoseUpdate。
    if (optimizer_.getPoseFrame() == PoseFrame::ODOM) {
        Pose3d T_map_odom = Pose3d::Identity();
        T_map_odom.linear() = R;
        T_map_odom.translation() = t;
        optimizer_.transformHistoryAndRebuild(T_map_odom, /*suppress_pose_notify=*/true);
        RCLCPP_INFO(node_->get_logger(),
            "[V3][POSE_DIAG] step=processGPSBatchKF: Triggered IncrementalOptimizer REBUILD from ODOM to MAP "
            "(suppress_pose_notify=true, GPS factors will notify after convergence)");
    }

    // [P1 GPS批量去重] 对齐时对历史帧批量注入 GPS 因子：按时间顺序遍历，跳过距上条 GPS 因子
    // 位移不足 factor_interval_m 的重复坐标（GPS 1Hz、KF ~10Hz 时同一 GPS 坐标覆盖多帧）。
    // 这与 gps_manager.cpp 的实时路径去重逻辑互补，专门覆盖批量对齐路径。
    const double batch_gps_interval = cfg.gpsFactorIntervalM();
    Eigen::Vector3d batch_last_pos = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    size_t reject_duplicate = 0;
    for (const auto& kf : all_kfs) {
        if (!kf) continue;
        const double gps_dt = std::abs(kf->timestamp - kf->gps.timestamp);
        const auto decision = evaluateKeyframeGpsConstraint(
            kf->gps, kf->has_valid_gps, cfg.gpsMinAcceptedQualityLevel(), true, gps_dt, gps_max_dt);
        if (!decision.accepted) {
            if (decision.reason == GPSConstraintRejectReason::QUALITY_BELOW_POLICY) {
                reject_quality++;
            }
            continue;
        }
        Eigen::Vector3d pos_map = gpsImuPositionInMapFromImuEnu(R, t, kf->gps.position_enu);
        const bool first_batch = !batch_last_pos.allFinite();
        const double dist = first_batch ? std::numeric_limits<double>::max()
                                        : (pos_map - batch_last_pos).norm();
        if (dist < batch_gps_interval) {
            reject_duplicate++;
            continue;
        }
        batch_last_pos = pos_map;
        factors.push_back(IncrementalOptimizer::GPSFactorItemKF{
            static_cast<int>(kf->id), pos_map, kf->gps.covariance, kf->gps.hdop, kf->gps.num_satellites});
    }
    // 同步更新 processTaskInternal 的去重状态，使批量对齐后单帧路径延续去重边界
    if (!factors.empty()) {
        last_gps_kf_factor_pos_ = factors.back().pos;
    }
    if (factors.empty()) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=processGPSBatchKF factors=0 (no KF with valid GPS)");
        return;
    }
    g_gps_kf_candidates_total.fetch_add(all_kfs.size(), std::memory_order_relaxed);
    g_gps_kf_added_total.fetch_add(factors.size(), std::memory_order_relaxed);
    g_gps_kf_reject_quality_total.fetch_add(reject_quality, std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
        "[CONSTRAINT_KPI][GPS_KF] mode=batch candidates=%zu added=%zu reject_quality=%zu reject_duplicate=%zu "
        "interval_m=%.1f min_accepted_quality_level=%d total_added=%lu",
        all_kfs.size(), factors.size(), reject_quality, reject_duplicate,
        batch_gps_interval, cfg.gpsMinAcceptedQualityLevel(),
        static_cast<unsigned long>(g_gps_kf_added_total.load(std::memory_order_relaxed)));
    optimizer_.addGPSFactorsForKeyFramesBatch(factors);
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=processGPSBatchKF done count=%zu (grep V3 DIAG)", factors.size());
}

void OptimizerModule::processKeyframeCreate(const OptTaskItem& task) {
    if (!task.keyframe) return;
    auto kf = task.keyframe;
    if (kf->id > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] processKeyframeCreate reject: kf_id=%lu exceeds int max (GTSAM KF symbol); skip addKeyFrameNode",
            static_cast<unsigned long>(kf->id));
        return;
    }
    RCLCPP_INFO(node_->get_logger(),
        "[V3][CONTRACT] processKeyframeCreate kf_id=%lu pose_frame=%d initial_guess=%s gps_aligned=%d has_gps=%d",
        static_cast<unsigned long>(kf->id), static_cast<int>(kf->pose_frame),
        (kf->pose_frame == PoseFrame::MAP) ? "T_map_b_optimized" : "T_odom_b",
        task.gps_aligned ? 1 : 0, kf->has_valid_gps ? 1 : 0);
    
    // 1. 添加关键帧节点
    bool is_first_kf_of_submap = (kf->index_in_submap == 0);
    const int sm_anchor = kf->submap_id;
    if (is_first_kf_of_submap && sm_anchor < 0) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] first KF of submap but submap_id<0: kf_id=%lu — SM–KF anchor skipped "
            "(ensure sm_manager_.addKeyFrame runs before KEYFRAME_CREATE)",
            static_cast<unsigned long>(kf->id));
    }
    // 🏛️ [架构加固] 必须使用 T_map_b_optimized 作为初始值。
    // 如果已对齐，T_map_b_optimized 已在 MappingModule 中由 T_odom_b 升级到 MAP 系；
    // 如果未对齐，T_map_b_optimized 与 T_odom_b 等同。
    // 若错误地始终使用 T_odom_b，当因子图已切换到 MAP 系时，新节点会产生巨大的初始位姿误差，导致优化后出现跳变。
    optimizer_.addKeyFrameNode(static_cast<int>(kf->id), kf->T_map_b_optimized, kf->T_odom_b, false,
                               is_first_kf_of_submap, sm_anchor);
    
    // 2. 里程计 Between 因子由 IncrementalOptimizer::addKeyFrameNode 统一添加。
    // 这里不再重复 add，避免同一 (prev, curr) 键值对被双重约束。
    
    // 3. 添加 GPS 因子 (如果已对齐)：position_enu 已为 IMU/ENU，仅乘对齐 R,t 到 map
    // grep CONSTRAINT][GPS_KF 闭环：对齐状态 / 是否有观测 / 质量策略 / hdop
    const auto& gcfg = ConfigManager::instance();
    const double gps_dt = std::abs(kf->timestamp - kf->gps.timestamp);
    const auto gps_decision = evaluateKeyframeGpsConstraint(
        kf->gps, kf->has_valid_gps, gcfg.gpsMinAcceptedQualityLevel(), true, gps_dt, gcfg.gpsKeyframeMatchWindowS());
    if (task.gps_aligned && gps_decision.accepted) {
        Eigen::Vector3d pos_map = gpsImuPositionInMapFromImuEnu(
            task.gps_transform_R, task.gps_transform_t, kf->gps.position_enu);

        // [P1 GPS去重] 跳过与上次添加位置过近的重复 GPS 因子：
        // GPS 1Hz 而关键帧 ~10Hz，多帧共享同一 GPS 坐标。若对同位点重复添加多个 PriorFactor，
        // 优化器会将这几帧同时拉向同一位置，与里程计 Between 因子产生局部压缩冲突，
        // 造成因子图在该区域产生逐帧小幅跳变，累积后导致地图重影。
        const double gps_kf_interval = gcfg.gpsFactorIntervalM();
        const bool first_gps_factor = !last_gps_kf_factor_pos_.allFinite();
        const double dist_from_last = first_gps_factor ? std::numeric_limits<double>::max()
                                                       : (pos_map - last_gps_kf_factor_pos_).norm();
        if (dist_from_last < gps_kf_interval) {
            g_gps_kf_candidates_total.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_DEBUG(node_->get_logger(),
                "[CONSTRAINT][GPS_KF] result=skip reason=duplicate_position kf_id=%lu dist_from_last=%.3fm interval=%.1fm",
                static_cast<unsigned long>(kf->id), dist_from_last, gps_kf_interval);
        } else {
        optimizer_.addGPSFactorForKeyFrame(
            kf->id, pos_map, kf->gps.covariance, kf->gps.hdop, kf->gps.num_satellites);
        last_gps_kf_factor_pos_ = pos_map;
        g_gps_kf_candidates_total.fetch_add(1, std::memory_order_relaxed);
        g_gps_kf_added_total.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 30000,
            "[Z_DRIFT_DIAG] stage=gps_factor_kf pos_map=R_enu_to_map*position_enu_imu+t (杆臂仅在前端/GPSManager 入口)");
        RCLCPP_INFO(node_->get_logger(),
            "[CONSTRAINT][GPS_KF] result=ok kf_id=%lu quality=%d rank=%d min_rank=%d hdop=%.2f sats=%d dist_from_last=%.2fm "
            "(grep CONSTRAINT GPS_KF)",
            static_cast<unsigned long>(kf->id), static_cast<int>(kf->gps.quality),
            gpsQualityRank(kf->gps.quality), gcfg.gpsMinAcceptedQualityLevel(),
            kf->gps.hdop, kf->gps.num_satellites, dist_from_last);
        }
    } else if (task.gps_aligned && gps_decision.reason == GPSConstraintRejectReason::QUALITY_BELOW_POLICY) {
        g_gps_kf_candidates_total.fetch_add(1, std::memory_order_relaxed);
        g_gps_kf_reject_quality_total.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_INFO(node_->get_logger(),
            "[CONSTRAINT][GPS_KF] result=skip reason=quality_below_policy kf_id=%lu quality=%d rank=%d min_rank=%d "
            "hdop=%.2f sats=%d (grep CONSTRAINT GPS_KF)",
            static_cast<unsigned long>(kf->id), static_cast<int>(kf->gps.quality),
            gpsQualityRank(kf->gps.quality), gcfg.gpsMinAcceptedQualityLevel(),
            kf->gps.hdop, kf->gps.num_satellites);
    } else if (!task.gps_aligned) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[CONSTRAINT][GPS_KF] result=skip reason=not_aligned_yet kf_id=%lu has_valid_gps=%d",
            static_cast<unsigned long>(kf->id), kf->has_valid_gps ? 1 : 0);
    } else if (gps_decision.reason == GPSConstraintRejectReason::NO_GPS_ON_KEYFRAME ||
               gps_decision.reason == GPSConstraintRejectReason::ENU_NOT_FINITE ||
               gps_decision.reason == GPSConstraintRejectReason::COV_NOT_FINITE ||
               gps_decision.reason == GPSConstraintRejectReason::OUTSIDE_TIME_WINDOW) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[CONSTRAINT][GPS_KF] result=skip reason=invalid_or_missing_measurement kf_id=%lu aligned=%d reject_reason=%d "
            "(前端 GPS_CACHE 未命中/ENU未就绪/协方差异常；grep GPS_CACHE KPI)",
            static_cast<unsigned long>(kf->id), task.gps_aligned ? 1 : 0, static_cast<int>(gps_decision.reason));
    }
}

Mat66d OptimizerModule::computeOdomInfoMatrixForKeyframes(const KeyFrame::Ptr& prev_kf, const KeyFrame::Ptr& curr_kf, const Pose3d& rel) const {
    (void)prev_kf;
    (void)curr_kf;
    (void)rel;
    Mat66d info = Mat66d::Identity();
    const auto& cfg = ConfigManager::instance();
    const double trans_noise = cfg.kfOdomTransNoise();
    const double trans_noise_z = cfg.kfOdomTransNoiseZ();
    const double rot_noise = cfg.kfOdomRotNoise();
    info(0, 0) = 1.0 / (trans_noise * trans_noise);
    info(1, 1) = 1.0 / (trans_noise * trans_noise);
    info(2, 2) = 1.0 / (trans_noise_z * trans_noise_z);
    for (int i = 3; i < 6; ++i) info(i, i) = 1.0 / (rot_noise * rot_noise);
    return info;
}

} // namespace automap_pro::v3
