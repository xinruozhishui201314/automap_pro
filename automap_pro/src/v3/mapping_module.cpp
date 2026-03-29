#include "automap_pro/v3/mapping_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/opt_task_types.h"
#include "automap_pro/v3/map_registry.h"
#include "automap_pro/v3/pose_chain.hpp"
#include "automap_pro/v3/semantic_backend_gates.h"
#include "automap_pro/core/data_types.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <vector>
#include <unordered_set>
#include <filesystem>
#include <chrono>
#include <future>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>

namespace fs = std::filesystem;

namespace automap_pro::v3 {
namespace {
uint64_t computeBatchHash(const std::unordered_map<int, Pose3d>& sm_poses,
                          const std::unordered_map<uint64_t, Pose3d>& kf_poses) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
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

uint64_t makeEventId(double ts, uint64_t seq) {
    const uint64_t ts_us = std::isfinite(ts) ? static_cast<uint64_t>(std::max(0.0, ts) * 1e6) : 0ull;
    return (seq << 20) ^ ts_us;
}

/** 后端噪声与 DCS 缩放用：检测置信度 × 子图内地标可观测性（多帧几何一致） */
double semanticBackendFactorWeight(double detection_confidence, double submap_observability) {
    const double obs = std::clamp(submap_observability, 0.05, 1.0);
    return std::max(1e-3, detection_confidence * obs);
}
} // namespace

MappingModule::MappingModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("MappingModule", event_bus, map_registry), node_(node) {

    const auto& cfg = ConfigManager::instance();
    const auto mapping_cfg = cfg.mappingDomain();
    const auto semantic_cfg = cfg.semanticDomain();
    max_frame_queue_size_ = mapping_cfg.frame_queue_max_size;
    max_semantic_queue_size_ = semantic_cfg.mapping_queue_max_size;
    max_pending_semantic_events_ = semantic_cfg.pending_queue_max_size;
    semantic_timestamp_match_tolerance_s_ = mapping_cfg.semantic_timestamp_match_tolerance_s;

    current_session_id_ = static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
    map_registry_->setSessionId(current_session_id_);
    processed_alignment_epoch_.store(map_registry_->getAlignmentEpoch());

    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=SubMapManager::init");
    sm_manager_.init(node_);
    sm_manager_.startNewSession(current_session_id_);
    sm_manager_.registerSubmapFrozenCallback([this](const SubMap::Ptr& sm) { this->onSubmapFrozen(sm); });
    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=session_id=%lu SubMap frozen callback registered",
                static_cast<unsigned long>(current_session_id_));

    hba_optimizer_.init();
    hba_optimizer_.start();
    
    // 🏛️ [修复] 恢复 V3 架构中遗失的 HBA 结果处理逻辑
    hba_optimizer_.registerDoneCallback([this](const HBAResult& result) {
        // 🏛️ [产品化加固] 模块停止后拒绝一切异步回调
        if (!running_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[V3][Mapping] Rejecting HBA result: Module is STOPPED.");
            return;
        }

        if (!result.success || result.optimized_poses.empty()) return;
        if (result.pose_frame == PoseFrame::UNKNOWN) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject HBA result: success-path pose_frame=UNKNOWN");
            METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
            return;
        }
        if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" &&
            result.pose_frame != PoseFrame::MAP &&
            !map_registry_->isGPSAligned()) {
            RCLCPP_WARN(node_->get_logger(),
                "[V3][CONTRACT] strict_map_only bypassed before GPS alignment: accepting non-MAP HBA result");
        } else if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" &&
                   result.pose_frame != PoseFrame::MAP) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] strict_map_only rejects non-MAP HBA result");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }

        // 🏛️ [防错判断] 数据合法性校验，防止 NaN 污染 MapRegistry
        if (!result.isFinite() || !result.isReasonable()) {
            RCLCPP_ERROR(node_->get_logger(), 
                "[V3][CRASH_GUARD] Rejecting HBA result: NaN or unreasonable translation detected! (Check HBA logs)");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] HBA optimization finished, applying results via gateway...");

        // 🏛️ [架构加固] 转换为统一 Map 格式并通过网关应用，确保坐标系语义安全
        std::unordered_map<uint64_t, Pose3d> kf_updates;
        if (result.optimized_keyframe_ids.size() == result.optimized_poses.size()) {
            for (size_t i = 0; i < result.optimized_poses.size(); ++i) {
                kf_updates[result.optimized_keyframe_ids[i]] = result.optimized_poses[i];
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][CONTRACT] Reject HBA result: keyframe id list mismatch (%zu vs %zu)",
                result.optimized_keyframe_ids.size(), result.optimized_poses.size());
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }

        PoseFrame output_frame = result.pose_frame;
        uint32_t transform_flags = static_cast<uint32_t>(OptimizationTransformFlags::NONE);
        if (map_registry_->isGPSAligned() && output_frame != PoseFrame::MAP) {
            Eigen::Matrix3d R_enu_to_map = Eigen::Matrix3d::Identity();
            Eigen::Vector3d t_enu_to_map = Eigen::Vector3d::Zero();
            map_registry_->getGPSTransform(R_enu_to_map, t_enu_to_map);

            Pose3d T_map_odom = Pose3d::Identity();
            T_map_odom.linear() = R_enu_to_map;
            T_map_odom.translation() = t_enu_to_map;
            if (!T_map_odom.matrix().allFinite()) {
                RCLCPP_ERROR(node_->get_logger(),
                    "[V3][CONTRACT] Reject HBA result: non-finite GPS transform while converting ODOM->MAP");
                METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
                return;
            }
            for (auto& [id, pose] : kf_updates) {
                pose = T_map_odom * pose;
            }
            output_frame = PoseFrame::MAP;
            transform_flags |= static_cast<uint32_t>(OptimizationTransformFlags::MAP_COMPENSATION_APPLIED);
        }

        const uint64_t source_alignment_epoch = result.alignment_epoch_snapshot;
        const uint64_t current_alignment_epoch = map_registry_->getAlignmentEpoch();
        if (source_alignment_epoch == 0 || source_alignment_epoch != current_alignment_epoch) {
            RCLCPP_WARN(node_->get_logger(),
                "[V3][CONTRACT] Drop HBA result by alignment_epoch mismatch: result_epoch=%lu current_epoch=%lu",
                static_cast<unsigned long>(source_alignment_epoch),
                static_cast<unsigned long>(current_alignment_epoch));
            METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
            return;
        }
        const uint64_t batch_hash = computeBatchHash({}, kf_updates);
        const uint64_t version = map_registry_->updatePoses(
            {}, kf_updates, output_frame, "HBAOptimizer", source_alignment_epoch,
            transform_flags, batch_hash);
        RCLCPP_INFO(node_->get_logger(),
            "[V3][HBA->REGISTRY] Applied HBA poses via MapRegistry: version=%lu pose_frame=%d kf=%zu epoch=%lu",
            static_cast<unsigned long>(version),
            static_cast<int>(output_frame),
            kf_updates.size(),
            static_cast<unsigned long>(source_alignment_epoch));
    });

    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=HBAOptimizer init+start+callback OK");

    // 订阅动态过滤后的帧
    onEvent<FilteredFrameEventRequiredDs>([this](const FilteredFrameEventRequiredDs& ev) {
        if (!running_.load()) return;
        if (quiescing_.load()) {
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][Mapping] Drop frame: Module is QUIESCING");
            return;
        }
        if (!ev.isValid()) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][CONTRACT] Reject invalid FilteredFrameEventRequiredDs before queueing");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (frame_queue_.size() >= max_frame_queue_size_) {
                frame_queue_.pop_front();
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[V3][BACKPRESSURE] frame_queue overflow, dropping oldest (cap=%zu). SENDING WARNING.",
                    max_frame_queue_size_);
                
                BackpressureWarningEvent warn;
                warn.module_name = name_;
                warn.queue_usage_ratio = 1.0f;
                warn.critical = true;
                warn.meta.event_id = warn.meta.idempotency_key =
                    makeEventId(node_->now().seconds(), graph_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1);
                warn.meta.producer_seq = warn.meta.event_id;
                warn.meta.ref_version = map_registry_->getVersion();
                warn.meta.ref_epoch = map_registry_->getAlignmentEpoch();
                warn.meta.source_ts = ev.timestamp;
                warn.meta.publish_ts = node_->now().seconds();
                warn.meta.producer = "MappingModule";
                warn.meta.session_id =
                    ev.meta.session_id != 0 ? ev.meta.session_id : map_registry_->getSessionId();
                warn.meta.route_tag = "legacy";
                if (warn.isValid()) {
                    event_bus_->publish(warn);
                }
            } else if (frame_queue_.size() > max_frame_queue_size_ * 0.8) {
                BackpressureWarningEvent warn;
                warn.module_name = name_;
                warn.queue_usage_ratio = static_cast<float>(frame_queue_.size()) / max_frame_queue_size_;
                warn.critical = false;
                warn.meta.event_id = warn.meta.idempotency_key =
                    makeEventId(node_->now().seconds(), graph_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1);
                warn.meta.producer_seq = warn.meta.event_id;
                warn.meta.ref_version = map_registry_->getVersion();
                warn.meta.ref_epoch = map_registry_->getAlignmentEpoch();
                warn.meta.source_ts = ev.timestamp;
                warn.meta.publish_ts = node_->now().seconds();
                warn.meta.producer = "MappingModule";
                warn.meta.session_id =
                    ev.meta.session_id != 0 ? ev.meta.session_id : map_registry_->getSessionId();
                warn.meta.route_tag = "legacy";
                if (warn.isValid()) {
                    event_bus_->publish(warn);
                }
            }
            frame_queue_.push_back(ev);
        }
        cv_.notify_one();
    });

    // 订阅 GPS 对齐结果
    onEvent<GPSAlignedEvent>([this](const GPSAlignedEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(queue_mutex_);
        gps_event_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅优化结果（用于更新 SubMapManager 内部位姿）
    // 使用 queue_mutex_ 与 run 循环一致，避免 pose_opt_queue_ 的读写竞态
    onEvent<OptimizationResultEvent>([this](const OptimizationResultEvent& ev) {
        if (!running_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[V3][CONTRACT] Drop optimization event on stopping/stopped module");
            return;
        }
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pose_opt_queue_.push_back(ev);
        cv_.notify_one();
    });
    onEvent<OptimizationDeltaEvent>([this](const OptimizationDeltaEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pose_delta_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅回环（用于 HBA 缓存）
    onEvent<LoopConstraintEvent>([this](const LoopConstraintEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
        loop_constraints_.push_back(ev.constraint);
    });

    // 订阅地图保存与构建命令
    onEvent<SaveMapRequestEvent>([this](const SaveMapRequestEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        Command cmd;
        cmd.type = Command::Type::SAVE_MAP;
        cmd.output_dir = ev.output_dir;
        cmd.save_completion = ev.completion;
        command_queue_.push_back(cmd);
        cv_.notify_one();
    });

    onEvent<GlobalMapBuildRequestEvent>([this](const GlobalMapBuildRequestEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        Command cmd;
        cmd.type = Command::Type::BUILD_GLOBAL_MAP;
        cmd.voxel_size = ev.voxel_size;
        cmd.async = ev.async;
        command_queue_.push_back(cmd);
        cv_.notify_one();
    });

    // 订阅手动 HBA 触发
    onEvent<HBARequestEvent>([this](const HBARequestEvent& ev) {
        if (!running_.load()) return;
        auto all_frozen = sm_manager_.getFrozenSubmaps();
        std::vector<LoopConstraint::Ptr> loops;
        {
            std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
            loops = loop_constraints_;
        }
        hba_optimizer_.triggerAsync(all_frozen, loops, ev.wait_for_result, "ManualRequest",
                                    processed_alignment_epoch_.load());
    });

    // 订阅系统静默请求 (🏛️ [架构契约] 屏障同步)
    onEvent<SystemQuiesceRequestEvent>([this](const SystemQuiesceRequestEvent& ev) {
        this->quiesce(ev.enable);
    });

    // Orchestrator 对任意观测事件（含 Semantic*、OptimizationDelta、FilteredFrame 等）均可发 RouteAdvice；
    // takeover 语义统一由本 latch 驱动 GraphTask/SemanticInput 的 route_tag 与 legacy 优化应用路径。
    onEvent<RouteAdviceEvent>([this](const RouteAdviceEvent& ev) {
        if (!ev.meta.isValid()) return;
        route_advice_recv_total_.fetch_add(1, std::memory_order_relaxed);
        route_takeover_enabled_.store(ev.takeover_enabled, std::memory_order_relaxed);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[ORCH][ROUTE_ADVICE] event=%s takeover=%d fallback_legacy=%d reason=%s recv_total=%lu",
            ev.event_type.c_str(),
            ev.takeover_enabled ? 1 : 0,
            ev.fallback_to_legacy ? 1 : 0,
            ev.reason.c_str(),
            static_cast<unsigned long>(route_advice_recv_total_.load(std::memory_order_relaxed)));
    });

    // 🏛️ [架构演进] 订阅异步语义地标
    onEvent<SemanticLandmarkEvent>([this](const SemanticLandmarkEvent& ev) {
        if (!running_.load()) return;
        const auto sem_in = semantic_in_total_.fetch_add(1, std::memory_order_relaxed) + 1;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (semantic_landmark_queue_.size() >= max_semantic_queue_size_) {
            semantic_landmark_queue_.pop_front();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][Mapping] semantic_landmark_queue overflow, dropping oldest event (cap=%zu)",
                max_semantic_queue_size_);
        }
        semantic_landmark_queue_.push_back(ev);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B3 MAP_IN] sem_in=%lu queue_size=%zu ts=%.3f landmarks=%zu",
            static_cast<unsigned long>(sem_in),
            semantic_landmark_queue_.size(),
            ev.timestamp,
            ev.landmarks.size());
        cv_.notify_one();
    });

    // 语义点云（intensity=类别 id）挂到关键帧，供回环 ICP 加权（与 SemanticLandmarkEvent 独立到达）
    onEvent<SemanticCloudEvent>([this](const SemanticCloudEvent& ev) {
        if (!running_.load()) return;
        if (!ev.isValid()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][CONTRACT] Reject invalid SemanticCloudEvent ts=%.3f", ev.timestamp);
            return;
        }
        const auto& cfg = ConfigManager::instance();
        const double tol = cfg.semanticTimestampMatchToleranceS();
        const uint64_t sid =
            ev.meta.session_id != 0 ? ev.meta.session_id : map_registry_->getSessionId();
        KeyFrame::Ptr kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp, tol, sid);
        if (!kf && std::isfinite(ev.timestamp) && ev.timestamp > 0.0) {
            kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp, std::min(0.5, tol * 2.0), sid);
        }
        if (!kf) {
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][MAP] SemanticCloudEvent: no keyframe for ts=%.3f", ev.timestamp);
            return;
        }
        if (!kf->cloud_body || kf->cloud_body->empty()) return;
        if (ev.labeled_cloud->size() != kf->cloud_body->size()) {
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][MAP] labeled_cloud pts=%zu != cloud_body %zu kf_id=%lu skip",
                ev.labeled_cloud->size(), kf->cloud_body->size(),
                static_cast<unsigned long>(kf->id));
            return;
        }
        const double min_agree = cfg.semanticCloudAttachMinPointAgreement();
        if (min_agree > 0.0 && kf->cloud_semantic_labeled_body &&
            kf->cloud_semantic_labeled_body->size() == ev.labeled_cloud->size()) {
            size_t same = 0;
            const size_t n = ev.labeled_cloud->size();
            for (size_t pi = 0; pi < n; ++pi) {
                const int a = static_cast<int>(std::lround(kf->cloud_semantic_labeled_body->points[pi].intensity));
                const int b = static_cast<int>(std::lround(ev.labeled_cloud->points[pi].intensity));
                if (a == b) ++same;
            }
            const double ratio = static_cast<double>(same) / static_cast<double>(n > 0 ? n : 1);
            if (ratio < min_agree) {
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "[SEMANTIC][MAP] semantic flicker reject kf_id=%lu point_agreement=%.2f < min=%.2f (keep previous labels)",
                    static_cast<unsigned long>(kf->id), ratio, min_agree);
                return;
            }
        }
        kf->cloud_semantic_labeled_body = std::make_shared<CloudXYZI>();
        *kf->cloud_semantic_labeled_body = *ev.labeled_cloud;
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][MAP] attached semantic labels on kf_id=%lu pts=%zu",
            static_cast<unsigned long>(kf->id), kf->cloud_semantic_labeled_body->size());
    });

    RCLCPP_INFO(node_->get_logger(),
                "[PIPELINE][MAP] ctor OK events=FilteredFrame+GPSAligned+OptResult+Loop+SaveMap+GlobalMap+HBA+Semantic+SemanticCloud");
    RCLCPP_INFO(node_->get_logger(),
                "[V3][SELF_CHECK][CONTRACT] module=Mapping prequeue_valid_guard=on process_entry_guard=on safe_cloud_ds_fallback=on required_ds_subscription=on");
}

void MappingModule::start() {
    ModuleBase::start();
}

void MappingModule::stop() {
    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] stop step=HBAOptimizer::stop");
    hba_optimizer_.stop();

    // 🏛️ [架构优化] 在正式停止前，强制执行一次全量保存，确保即便析构函数没跑完，数据也已落盘
    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] stop step=forceSyncSave");
    SaveMapRequestEvent final_save;
    final_save.output_dir = ConfigManager::instance().outputDir();
    handleSaveMap(final_save);

    ModuleBase::stop();
}

bool MappingModule::isIdle() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return frame_queue_.empty() && pose_opt_queue_.empty() && pose_delta_queue_.empty() &&
           gps_event_queue_.empty() && command_queue_.empty() &&
           semantic_landmark_queue_.empty() &&
           sm_manager_.isIdle(); // 🏛️ [修复] 检查点云合并子管理器是否也空闲
}

std::vector<std::pair<std::string, size_t>> MappingModule::queueDepths() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    size_t pending_sem = 0;
    {
        std::lock_guard<std::mutex> sem_lock(pending_semantic_mutex_);
        pending_sem = pending_semantic_landmarks_.size();
    }
    return {
        {"frame_queue", frame_queue_.size()},
        {"pose_opt_queue", pose_opt_queue_.size()},
        {"pose_delta_queue", pose_delta_queue_.size()},
        {"gps_event_queue", gps_event_queue_.size()},
        {"command_queue", command_queue_.size()},
        {"semantic_landmark_queue", semantic_landmark_queue_.size()},
        {"pending_semantic_landmarks", pending_sem},
    };
}

std::string MappingModule::idleDetail() const {
    const bool sm_idle = sm_manager_.isIdle();
    if (!sm_idle) {
        return "SubMapManager busy(merge/freeze-post)";
    }
    return "";
}

void MappingModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Started worker thread");
    
    while (running_) {
        try {
            updateHeartbeat();
            FilteredFrameEventRequiredDs event;
            OptimizationResultEvent opt_ev;
            OptimizationDeltaEvent delta_ev;
            GPSAlignedEvent gps_ev;
            SemanticLandmarkEvent sem_ev;
            Command cmd;
            bool has_frame = false;
            bool has_opt = false;
            bool has_delta = false;
            bool has_gps_ev = false;
            bool has_cmd = false;
            bool has_sem = false;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                    return !running_ || !frame_queue_.empty() || !pose_opt_queue_.empty() || 
                           !pose_delta_queue_.empty() || !gps_event_queue_.empty() || !command_queue_.empty() || !semantic_landmark_queue_.empty(); 
                });
                if (!running_) break;
                
                // 优先处理状态变更类事件
                if (!gps_event_queue_.empty()) {
                    gps_ev = gps_event_queue_.front();
                    gps_event_queue_.pop_front();
                    has_gps_ev = true;
                } else if (!pose_delta_queue_.empty()) {
                    delta_ev = pose_delta_queue_.front();
                    pose_delta_queue_.pop_front();
                    has_delta = true;
                } else if (!pose_opt_queue_.empty()) {
                    opt_ev = pose_opt_queue_.front();
                    pose_opt_queue_.pop_front();
                    has_opt = true;
                } else if (!command_queue_.empty()) {
                    cmd = command_queue_.front();
                    command_queue_.pop_front();
                    has_cmd = true;
                } else if (!semantic_landmark_queue_.empty()) {
                    sem_ev = semantic_landmark_queue_.front();
                    semantic_landmark_queue_.pop_front();
                    has_sem = true;
                } else if (!frame_queue_.empty()) {
                // 🏛️ [修复] 确定性因果序屏障：版本须 reg 已追上；对齐世代须 target_epoch <= local_epoch。
                // 禁止 target_epoch == local_epoch 的严格相等：GPS 对齐后 processed_alignment_epoch_ 先升级，
                // 队列里仍大量 ref_alignment_epoch 为对齐前值的帧（见 full.log Barrier TIMEOUT target_epoch=1 current_epoch=2），
                // 会导致整段帧 5s 超时丢弃、Registry 长时间无新 KF、当前云 bypass 与 optimized_path 剧烈脱节。
                // target_epoch < current_epoch：事件标签过时，几何仍为 LIO odom，processFrame 按当前 gps_aligned_ 升 MAP 即可消费。
                // target_epoch > current_epoch：须先处理 GPS 等使 local_epoch 追上（队列优先 gps_event_queue_）。
                    const uint64_t target_version = frame_queue_.front().ref_map_version;
                    const uint64_t target_epoch = frame_queue_.front().ref_alignment_epoch;
                    const uint64_t registry_version = map_registry_->getVersion();
                    const uint64_t current_epoch = processed_alignment_epoch_.load();

                    if (target_version <= registry_version && target_epoch <= current_epoch) {
                        event = frame_queue_.front();
                        frame_queue_.pop_front();
                        has_frame = true;
                        last_barrier_wait_start_time_ = -1.0; // 重置计时
                    } else {
                        // 🏛️ [P1 稳定性修复] 引入屏障超时机制，防止后端（Optimizer）卡死导致前端无限阻塞内存撑爆
                        double now_s = node_->now().seconds();
                        if (last_barrier_wait_start_time_ < 0) {
                            last_barrier_wait_start_time_ = now_s;
                        }
                        
                        if (now_s - last_barrier_wait_start_time_ > 5.0) {
                            RCLCPP_ERROR(node_->get_logger(),
                                "[CRITICAL_V3][MappingModule] Barrier TIMEOUT (5s)! Drop blocked frame: target_ref=%lu registry_version=%lu target_epoch=%lu current_epoch=%lu.",
                                target_version, registry_version, target_epoch, current_epoch);
                            frame_queue_.pop_front();
                            METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
                            last_barrier_wait_start_time_ = -1.0;
                        } else {
                            // 地图版本尚未追上，本轮循环跳过处理新帧，等待优化结果入队并处理
                            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "[V3][MappingModule] Frame blocked: ref_v=%lu reg_v=%lu ref_ep=%lu local_ep=%lu "
                                "(dequeue when ref_v<=reg_v && ref_ep<=local_ep)",
                                target_version, registry_version, target_epoch, current_epoch);
                        }
                    }
                }
            }

            if (has_delta) {
                onPoseDelta(delta_ev);
            } else if (has_opt) {
                onPoseOptimized(opt_ev);
            } else if (has_gps_ev) {
                updateGPSAlignment(gps_ev);
            } else if (has_sem) {
                onSemanticLandmarks(sem_ev);
            } else if (has_cmd) {
                if (cmd.type == Command::Type::SAVE_MAP) {
                    SaveMapRequestEvent ev;
                    ev.output_dir = cmd.output_dir;
                    try {
                        handleSaveMap(ev);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(node_->get_logger(),
                            "[V3][MappingModule] SaveMap command failed: %s", e.what());
                    }
                    if (cmd.save_completion) {
                        try {
                            cmd.save_completion->set_value();
                        } catch (const std::future_error&) {
                            // already satisfied
                        }
                    }
                } else {
                    GlobalMapBuildRequestEvent ev;
                    ev.voxel_size = cmd.voxel_size;
                    ev.async = cmd.async;
                    handleGlobalMapBuild(ev);
                }
            } else if (has_frame) {
                processFrame(event);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][MappingModule][EXCEPTION] run loop caught: %s (isolate+continue)", e.what());
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][MappingModule][EXCEPTION] run loop unknown exception (isolate+continue)");
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        }
    }
}

void MappingModule::processFrame(const FilteredFrameEventRequiredDs& event) {
    // 最后防线：即使上游是 RequiredDs 契约，这里仍做保护，彻底避免裸解引用
    if (!event.cloud || event.cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject frame: cloud is null/empty");
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    CloudXYZIPtr safe_cloud_ds = (event.cloud_ds && !event.cloud_ds->empty()) ? event.cloud_ds : event.cloud;
    if (!safe_cloud_ds || safe_cloud_ds->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject frame: cloud_ds fallback failed");
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }

    if (!kf_manager_.shouldCreateKeyFrame(event.T_odom_b, event.timestamp)) {
        return;
    }

    // 🏛️ [架构契约] Motion Continuity Invariant (运动连续性守卫)
    // 防止前端退化导致的位姿巨大跳变进入后端
    KeyFrame::Ptr prev_kf = kf_manager_.getLastKeyFrame();
    if (prev_kf) {
        double dist = (event.T_odom_b.translation() - prev_kf->T_odom_b.translation()).norm();
        double dt = event.timestamp - prev_kf->timestamp;
        if (dt > 0.0) {
            double v = dist / dt;
            const double max_v = ConfigManager::instance().mappingMaxReasonableVelocityMps();
            if (v > max_v) {
                RCLCPP_ERROR(node_->get_logger(),
                    "[V3][CRASH_GUARD] Reject frame: unreasonable velocity detected! v=%.2f m/s > max=%.2f m/s (ts=%.3f)",
                    v, max_v, event.timestamp);
                METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
                return;
            }
        }
        // 瞬间跳变检测 (即使 dt 很大，单次 jump 也不应超过 10m)
        const double max_jump = ConfigManager::instance().mappingMaxReasonableJumpM();
        if (dist > max_jump && dt < 0.5) {
             RCLCPP_ERROR(node_->get_logger(),
                "[V3][CRASH_GUARD] Reject frame: unreasonable pose jump! dist=%.2f m > max=%.2f m (dt=%.3fs, ts=%.3f)",
                dist, max_jump, dt, event.timestamp);
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
    }

    RCLCPP_INFO(node_->get_logger(),
        "[V3][CONTRACT] processFrame ingress ts=%.3f pose_frame=%d cloud_frame=%s has_gps=%d ref_map_version=%lu ref_alignment_epoch=%lu",
        event.timestamp, static_cast<int>(event.pose_frame), event.cloud_frame.c_str(),
        event.has_gps ? 1 : 0, static_cast<unsigned long>(event.ref_map_version),
        static_cast<unsigned long>(event.ref_alignment_epoch));

    // 🏛️ [点云契约] 明确校验点云坐标系语义，禁止隐式混用
    if (event.cloud_frame != "body" && event.cloud_frame != "world") {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject frame: unsupported cloud_frame=%s (expected body/world)",
            event.cloud_frame.c_str());
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    // 当前仅允许 world 点云与 ODOM 语义位姿配对（通过 T_odom_b 反变换回 body）
    if (event.cloud_frame == "world" && event.pose_frame != PoseFrame::ODOM) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject frame: cloud_frame=world requires pose_frame=ODOM, got pose_frame=%d",
            static_cast<int>(event.pose_frame));
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    if (event.cloud_frame == "body" && event.pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject frame: cloud_frame=body with pose_frame=UNKNOWN");
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return;
    }

    
    // ── V3: 处理坐标系不匹配 (Double Transformation Fix) ──
    // 如果 cloud_frame 为 "world"，则云已经在世界系下。
    // Mapping 模块后续会再次应用 T_map_odom 变换到世界系，导致双重变换。
    // 因此这里先变换回 body 系。
    CloudXYZIPtr cloud = event.cloud;
    CloudXYZIPtr cloud_ds = safe_cloud_ds;
    if (event.cloud_frame == "world") {
        CloudXYZIPtr body_cloud(new CloudXYZI());
        pcl::transformPointCloud(*event.cloud, *body_cloud, event.T_odom_b.inverse().cast<float>());
        cloud = body_cloud;

        CloudXYZIPtr body_cloud_ds(new CloudXYZI());
        pcl::transformPointCloud(*safe_cloud_ds, *body_cloud_ds, event.T_odom_b.inverse().cast<float>());
        cloud_ds = body_cloud_ds;

        // grep GEO_PIPELINE：世界系关键帧入库前 span 对比，区分「未转换」与「转换后正常车体系尺度」
        static std::atomic<uint32_t> geo_pipeline_world_kf{0};
        const uint32_t wk = geo_pipeline_world_kf.fetch_add(1, std::memory_order_relaxed) + 1;
        const bool log_geo = (wk <= 25u || (wk % 200u) == 0u);
        if (log_geo && event.cloud && cloud) {
            auto bboxStats = [](const CloudXYZIPtr& pc) {
                struct {
                    double xy_span = -1.0;
                    double z_span = -1.0;
                    double x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;
                } out;
                if (!pc || pc->empty()) {
                    return out;
                }
                out.x_min = std::numeric_limits<double>::infinity();
                out.x_max = -std::numeric_limits<double>::infinity();
                out.y_min = std::numeric_limits<double>::infinity();
                out.y_max = -std::numeric_limits<double>::infinity();
                out.z_min = std::numeric_limits<double>::infinity();
                out.z_max = -std::numeric_limits<double>::infinity();
                for (const auto& p : pc->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                        continue;
                    }
                    out.x_min = std::min(out.x_min, static_cast<double>(p.x));
                    out.x_max = std::max(out.x_max, static_cast<double>(p.x));
                    out.y_min = std::min(out.y_min, static_cast<double>(p.y));
                    out.y_max = std::max(out.y_max, static_cast<double>(p.y));
                    out.z_min = std::min(out.z_min, static_cast<double>(p.z));
                    out.z_max = std::max(out.z_max, static_cast<double>(p.z));
                }
                if (std::isfinite(out.x_min) && std::isfinite(out.z_min)) {
                    out.xy_span = std::max(out.x_max - out.x_min, out.y_max - out.y_min);
                    out.z_span = out.z_max - out.z_min;
                }
                return out;
            };
            const auto sw = bboxStats(event.cloud);
            const auto sb = bboxStats(cloud);
            const Eigen::Vector3d tw = event.T_odom_b.translation();
            RCLCPP_INFO(
                node_->get_logger(),
                "[GEO_PIPELINE][MAP_COORD] world_kf_seq=%u ts=%.3f pts_w=%zu span_xy_w=%.2f dz_w=%.2f "
                "pts_b=%zu span_xy_b=%.2f dz_b=%.2f T_odom_b_t=[%.2f,%.2f,%.2f] | cloud_frame=world->body",
                wk,
                event.timestamp,
                event.cloud->size(),
                sw.xy_span,
                sw.z_span,
                cloud->size(),
                sb.xy_span,
                sb.z_span,
                tw.x(),
                tw.y(),
                tw.z());
            
            // 🏛️ [V3 诊断] 打印原始与转换后中心，验证 Body 云是否真正以 0,0,0 为中心（解决回环 p90>50m 问题）
            if (log_geo) {
                double cx_w = (sw.x_min + sw.x_max) * 0.5;
                double cy_w = (sw.y_min + sw.y_max) * 0.5;
                double cx_b = (sb.x_min + sb.x_max) * 0.5;
                double cy_b = (sb.y_min + sb.y_max) * 0.5;
                RCLCPP_INFO(node_->get_logger(),
                    "[GEO_PIPELINE][CENTER_DIAG] seq=%u world_center=[%.2f,%.2f] body_center=[%.2f,%.2f] offset_to_zero=%.2fm",
                    wk, cx_w, cy_w, cx_b, cy_b, std::sqrt(cx_b*cx_b + cy_b*cy_b));
            }
        }

        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][CONTRACT] cloud_frame=world converted to body (ts=%.3f pose_frame=%d)",
            event.timestamp, static_cast<int>(event.pose_frame));
    }
    
    // ── V3: 使用 FilteredFrameEventRequiredDs 中的 GPS 观测 ──
    GPSMeasurement gps = event.has_gps ? event.gps : GPSMeasurement();
    bool has_gps = event.has_gps;
    
    KeyFrame::Ptr kf = kf_manager_.createKeyFrame(
        event.T_odom_b, event.covariance, event.timestamp, cloud, 
        cloud_ds, gps, has_gps, current_session_id_
    );
    
    if (!kf) return;
    if (has_gps) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 30000,
            "[MAPPING][GPS_ATTACH] kf_id=%lu quality=%d is_valid=%d hdop=%.2f sats=%d "
            "(闭环: 与 CONSTRAINT GPS_KF / GPS_CACHE KPI 对照)",
            static_cast<unsigned long>(kf->id), static_cast<int>(kf->gps.quality),
            kf->gps.is_valid ? 1 : 0, kf->gps.hdop, kf->gps.num_satellites);
    }
    kf->livo_info = event.kf_info;
    kf->pose_frame = event.pose_frame; // 🏛️ [架构契约] 继承来源语义
    kf->alignment_epoch = event.ref_alignment_epoch; // 🏛️ [对齐纪元] 继承事件纪元

    // 应用 GPS 对齐变换 (🏛️ [架构加固] R/t 偏移与其对齐状态原子性快照，避免锁外读导致的脏读)
    Eigen::Matrix3d R_snapshot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_snapshot = Eigen::Vector3d::Zero();
    bool aligned = false;
    {
        std::lock_guard<std::mutex> lk(gps_transform_mutex_);
        aligned = gps_aligned_.load();
        if (aligned) {
            R_snapshot = gps_transform_R_;
            t_snapshot = gps_transform_t_;
        }
    }

    if (aligned) {
        // 🏛️ [契约核校] 仅当输入为 ODOM 时才需要进行 Map 补偿转换
        if (kf->pose_frame == PoseFrame::ODOM) {
            Pose3d T_mo = Pose3d::Identity();
            T_mo.linear() = R_snapshot;
            T_mo.translation() = t_snapshot;
            kf->T_map_b_optimized = pose_chain::mapBodyFromOdomBody(T_mo, kf->T_odom_b);
            kf->pose_frame = PoseFrame::MAP; // 🏛️ [架构加固] 标注语义已提升为 MAP

            {
                const double oz = kf->T_odom_b.translation().z();
                const double mz = kf->T_map_b_optimized.translation().z();
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                    "[Z_DRIFT_DIAG] stage=odom_to_map_upgrade kf_id=%lu odom_z=%.4f map_z=%.4f dz_map_minus_odom=%.4f "
                    "(对齐为 yaw+XY 平移、tz=0 时期望 dz≈0；grep Z_DRIFT_DIAG)",
                    static_cast<unsigned long>(kf->id), oz, mz, mz - oz);
            }

            // [RC2 修复] 该帧由 ODOM 升级到 MAP 坐标系，其 ref_alignment_epoch 可能早于当前
            // GPS 对齐纪元（帧在对齐前入队、对齐后才处理）。此时 publishOptimizedPath 的
            // alignment_epoch_limit == current_epoch 会过滤这些 KF，导致轨迹帧数骤减闪烁。
            // 修复：位姿已在当前 GPS 对齐变换下升级，将 alignment_epoch 同步到当前纪元。
            const uint64_t current_epoch = processed_alignment_epoch_.load(std::memory_order_relaxed);
            if (current_epoch > kf->alignment_epoch) {
                RCLCPP_DEBUG(node_->get_logger(),
                    "[V3][RC2_FIX] kf_id=%lu bump alignment_epoch %lu->%lu (ODOM->MAP upgrade, avoid epoch filter flicker)",
                    static_cast<unsigned long>(kf->id),
                    static_cast<unsigned long>(kf->alignment_epoch),
                    static_cast<unsigned long>(current_epoch));
                kf->alignment_epoch = current_epoch;
            }

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][POSE_DIAG] Keyframe upgraded to MAP: T_odom_b=[%.2f,%.2f,%.2f] -> T_map_b_optimized=[%.2f,%.2f,%.2f]",
                kf->T_odom_b.translation().x(), kf->T_odom_b.translation().y(), kf->T_odom_b.translation().z(),
                kf->T_map_b_optimized.translation().x(), kf->T_map_b_optimized.translation().y(), kf->T_map_b_optimized.translation().z());
        }
    }

    // 注册到 MapRegistry（先完成 pose 数据语义升级再发布，避免并发读到半更新状态）
    map_registry_->addKeyFrame(kf);

    // 必须先写入 SubMapManager，使 kf->submap_id / index_in_submap 就绪，再投递 KEYFRAME_CREATE；
    // 否则 IncrementalOptimizer 无法做 Between(SM(sm), KF(首帧)) 的正确锚定。
    sm_manager_.addKeyFrame(kf);

    bool has_prev_kf = (prev_kf != nullptr);
    int prev_kf_id = has_prev_kf ? static_cast<int>(prev_kf->id) : -1;

    // OptimizerModule 仅订阅 GraphTaskEvent：必须每帧发布 KEYFRAME_CREATE，不可因「独立语义输入」而关闭，
    // 否则 iSAM2 收不到 addKeyFrameNode（曾导致因子图与子图位姿脱节）。
    // 语义侧另走 SemanticInputEvent；若同时开启 semanticInputAcceptGraphTask 与 AcceptIndependentEvent，可能双路同一 KF，需配置只开其一入语义。
    {
        GraphTaskEvent task_ev;
        task_ev.task.type = OptTaskItem::Type::KEYFRAME_CREATE;
        task_ev.task.keyframe = kf;
        task_ev.task.has_prev_kf = has_prev_kf;
        task_ev.task.prev_kf_id = prev_kf_id;
        task_ev.task.prev_keyframe = prev_kf;
        task_ev.task.gps_aligned = aligned;
        task_ev.task.gps_transform_R = R_snapshot;
        task_ev.task.gps_transform_t = t_snapshot;
        publishGraphTaskEvent(task_ev, kf->timestamp);
    }

    const auto& cfg = ConfigManager::instance();
    const bool use_independent_semantic_input = cfg.semanticInputUseIndependentEvent();
    if (use_independent_semantic_input) {
        SemanticInputEvent sem_in_ev;
        sem_in_ev.timestamp = kf->timestamp;
        sem_in_ev.keyframe = kf;
        const uint64_t seq = graph_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
        const double now_ts = node_->now().seconds();
        sem_in_ev.meta.event_id = makeEventId(kf->timestamp, seq);
        sem_in_ev.meta.idempotency_key = sem_in_ev.meta.event_id;
        sem_in_ev.meta.producer_seq = seq;
        sem_in_ev.meta.ref_version = map_registry_->getVersion();
        sem_in_ev.meta.ref_epoch = map_registry_->getAlignmentEpoch();
        sem_in_ev.meta.session_id = map_registry_->getSessionId();
        sem_in_ev.meta.source_ts = kf->timestamp;
        sem_in_ev.meta.publish_ts = now_ts;
        sem_in_ev.meta.producer = "MappingModule";
        sem_in_ev.meta.route_tag = route_takeover_enabled_.load(std::memory_order_relaxed) ? "orchestrated" : "legacy";
        sem_in_ev.processing_state = quiescing_.load() ? ProcessingState::DEGRADED : ProcessingState::NORMAL;
        if (!sem_in_ev.isValid()) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][CONTRACT] Drop invalid SemanticInputEvent kf_id=%lu", static_cast<unsigned long>(kf->id));
        } else {
            event_bus_->publish(sem_in_ev);
        }
    }
    const uint64_t trace_id = semanticTraceId(kf->id, kf->timestamp);
    size_t frame_q_size = 0;
    size_t sem_q_size = 0;
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        frame_q_size = frame_queue_.size();
        sem_q_size = semantic_landmark_queue_.size();
    }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[CHAIN][B2 MAP->SEM] action=publish_kf trace=%lu kf_id=%lu ts=%.3f cloud_pts=%zu ds_pts=%zu q_frame=%zu q_sem=%zu",
        static_cast<unsigned long>(trace_id),
        static_cast<unsigned long>(kf->id),
        kf->timestamp,
        cloud ? cloud->size() : 0,
        cloud_ds ? cloud_ds->size() : 0,
        frame_q_size,
        sem_q_size);

    // 🏛️ [P1 稳定性修复] 检查是否有延迟到达的语义地标，现在 KeyFrame 已经分配了 submap_id
    SemanticLandmarkEvent deferred_ev;
    bool has_deferred = false;
    {
        std::lock_guard<std::mutex> lk(pending_semantic_mutex_);
        for (auto it = pending_semantic_landmarks_.begin(); it != pending_semantic_landmarks_.end(); ++it) {
            bool match = false;
            if (it->keyframe_id_hint != 0 && it->keyframe_id_hint == kf->id) {
                match = true;
            } else {
                if (std::isfinite(it->keyframe_timestamp_hint) && it->keyframe_timestamp_hint > 0.0) {
                    const double dt_hint = std::abs(it->keyframe_timestamp_hint - kf->timestamp);
                    if (dt_hint <= semantic_timestamp_match_tolerance_s_) {
                        match = true;
                    }
                }
                if (!match) {
                    const double dt = std::abs(it->timestamp - kf->timestamp);
                    match = (dt <= semantic_timestamp_match_tolerance_s_);
                }
            }
            if (match) {
                deferred_ev = *it;
                has_deferred = true;
                pending_semantic_landmarks_.erase(it);
                break;
            }
        }
    }

    if (has_deferred) {
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Mapping][processFrame] step=process_deferred ts=%.3f kf_id=%lu sm_id=%d landmarks=%zu",
            kf->timestamp, kf->id, kf->submap_id, deferred_ev.landmarks.size());
        onSemanticLandmarks(deferred_ev);
    }

    // 🏛️ [架构演进] 语义地标现在通过 onSemanticLandmarks 异步处理，不再阻塞关键帧流程

    // 🏛️ [P0 优化] 重新引入基于帧数的地图发布逻辑
    // V3 架构此前仅依赖 AutoMapSystem 的 WallTimer (10s)，导致无数据时也冗余构建
    // 这里根据配置的 backend.publish_global_map_every_n_processed 触发构建请求
    const int count = ++processed_frame_count_;
    const int interval = ConfigManager::instance().backendPublishGlobalMapEveryNProcessed();
    if (count % interval == 0) {
        RCLCPP_INFO(node_->get_logger(), 
            "[V3][MappingModule] Triggering periodic global map build (processed_frames=%d interval=%d)",
            count, interval);
        
        GlobalMapBuildRequestEvent build_ev;
        build_ev.voxel_size = ConfigManager::instance().mapVoxelSize();
        build_ev.async = ConfigManager::instance().globalMapBuildAsync();
        event_bus_->publish(build_ev);
    }
}

void MappingModule::onSubmapFrozen(const SubMap::Ptr& submap) {
    if (!submap) return;
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=onSubmapFrozen sm_id=%d (grep V3 DIAG)", submap->id);
    map_registry_->addSubMap(submap);
    frozen_submap_count_++;
    const int count = frozen_submap_count_.load();

    // 发布子图节点任务
    GraphTaskEvent node_ev;
    node_ev.task.type = OptTaskItem::Type::SUBMAP_NODE;
    node_ev.task.to_id = submap->id;
    node_ev.task.rel_pose = submap->pose_odom_anchor;
    node_ev.task.fixed = (count == 1);
    publishGraphTaskEvent(node_ev, submap->t_end);

    // 子图间里程计因子：用冻结顺序上的「上一块」子图，避免 getFrozenSubmaps()（持 SubMapManager::mutex_，与 merge 长临界区死锁）
    {
        std::lock_guard<std::mutex> lk(prev_frozen_for_odom_mutex_);
        if (prev_frozen_for_odom_) {
            auto prev = prev_frozen_for_odom_;
            Pose3d rel = prev->pose_odom_anchor.inverse() * submap->pose_odom_anchor;
            Mat66d info = computeOdomInfoMatrix(prev, submap, rel);

            GraphTaskEvent odom_ev;
            odom_ev.task.type = OptTaskItem::Type::ODOM_FACTOR;
            odom_ev.task.from_id = prev->id;
            odom_ev.task.to_id = submap->id;
            odom_ev.task.rel_pose = rel;
            odom_ev.task.info_matrix = info;
            publishGraphTaskEvent(odom_ev, submap->t_end);
        }
        prev_frozen_for_odom_ = submap;
    }

    // 强制更新
    GraphTaskEvent force_ev;
    force_ev.task.type = OptTaskItem::Type::FORCE_UPDATE;
    publishGraphTaskEvent(force_ev, submap->t_end);

    // HBA 触发（子图列表来自 MapRegistry，避免 SubMapManager::mutex_）
    if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnLoop()) {
        const int trigger_mod = ConfigManager::instance().hbaTriggerSubmaps();
        if (count % trigger_mod == 0) {
            std::vector<LoopConstraint::Ptr> loops;
            {
                std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
                loops = loop_constraints_;
            }
            std::vector<SubMap::Ptr> frozen_for_hba;
            for (const auto& sm : map_registry_->getAllSubMaps()) {
                if (sm && (sm->state == SubMapState::FROZEN || sm->state == SubMapState::OPTIMIZED))
                    frozen_for_hba.push_back(sm);
            }
            std::sort(frozen_for_hba.begin(), frozen_for_hba.end(),
                      [](const SubMap::Ptr& a, const SubMap::Ptr& b) {
                          return a->id < b->id;
                      });
            hba_optimizer_.triggerAsync(frozen_for_hba, loops, false, "onSubmapFrozen",
                                        processed_alignment_epoch_.load());
        }
    }
}

void MappingModule::updateGPSAlignment(const GPSAlignedEvent& ev) {
    const uint64_t last_seq = last_gps_event_seq_.load();
    if (ev.event_seq <= last_seq) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Drop stale/duplicate GPSAlignedEvent by event_seq: event_seq=%lu last_seq=%lu",
            static_cast<unsigned long>(ev.event_seq),
            static_cast<unsigned long>(last_seq));
        return;
    }
    last_gps_event_seq_.store(ev.event_seq);

    // GPSModule 携带的 alignment_epoch 是「发布瞬间」的 getAlignmentEpoch() 提示值（见 map_registry.h 注释），
    // 与 MapRegistry 初始值均为 1。若用 <= 会把「首次成功对齐」误判为重复（1<=1）而丢弃，导致永不 setGPSAligned。
    // 仅当事件的观测世代严格落后于本模块已处理的世代时才视为过期。
    const uint64_t local_epoch_before = processed_alignment_epoch_.load();
    if (ev.alignment_epoch != 0 && ev.alignment_epoch < local_epoch_before) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Drop stale GPSAlignedEvent: event_observed_epoch=%lu local_processed_epoch=%lu success=%d",
            static_cast<unsigned long>(ev.alignment_epoch),
            static_cast<unsigned long>(local_epoch_before),
            ev.success ? 1 : 0);
        return;
    }

    if (!ev.success) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][POSE_DIAG] GPS alignment event success=false: clearing Mapping/MapRegistry/HBA aligned state");
        {
            std::lock_guard<std::mutex> lk(gps_transform_mutex_);
            gps_transform_R_ = Eigen::Matrix3d::Identity();
            gps_transform_t_ = Eigen::Vector3d::Zero();
            gps_aligned_.store(false);
        }
        uint64_t version = map_registry_->setGPSAligned(
            false, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), 0.0,
            Eigen::Matrix3d::Identity());
        processed_map_version_.store(version);
        const uint64_t new_epoch = map_registry_->getAlignmentEpoch();
        processed_alignment_epoch_.store(new_epoch);
        GPSAlignResult hba_align{};
        hba_align.success = false;
        hba_optimizer_.setGPSAlignedState(hba_align);
        RCLCPP_INFO(node_->get_logger(),
            "[V3][POSE_DIAG] MapRegistry version updated to %lu after GPS alignment cleared (epoch=%lu, event_epoch=%lu)",
            static_cast<unsigned long>(version),
            static_cast<unsigned long>(new_epoch),
            static_cast<unsigned long>(ev.alignment_epoch));
        return;
    }

    if (!ev.isValid()) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject GPSAlignedEvent: success=true but payload invalid (NaN/Inf)");
        return;
    }

    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] GPS Alignment updated: rmse=%.3fm R_enu_map=[...], t_enu_map=[%.2f, %.2f, %.2f]",
        ev.rmse, ev.t_enu_to_map.x(), ev.t_enu_to_map.y(), ev.t_enu_to_map.z());
    {
        std::lock_guard<std::mutex> lk(gps_transform_mutex_);
        gps_transform_R_ = ev.R_enu_to_map;
        gps_transform_t_ = ev.t_enu_to_map;
        gps_aligned_.store(true);
    }

    // 🏛️ 单一写者：由本模块根据 GPSAlignedEvent 同步 MapRegistry（GPSModule 不再 setGPSAligned）
    uint64_t version = map_registry_->setGPSAligned(true, ev.R_enu_to_map, ev.t_enu_to_map, ev.rmse,
                                                    ev.R_enu_odom);
    processed_map_version_.store(version);
    const uint64_t new_epoch = map_registry_->getAlignmentEpoch();
    processed_alignment_epoch_.store(new_epoch);

    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] MapRegistry version updated to %lu after GPS alignment (epoch=%lu, event_epoch=%lu)",
        version, static_cast<unsigned long>(new_epoch), static_cast<unsigned long>(ev.alignment_epoch));
    {
        const double t_norm = ev.t_enu_to_map.norm();
        RCLCPP_INFO(node_->get_logger(),
            "[V3][GPS_ALIGN_APPLIED] map_ver=%lu epoch=%lu rmse=%.3fm t_norm=%.3fm event_seq=%lu "
            "t_enu_map=[%.3f,%.3f,%.3f] (grep 本行与 POSE_DRIFT 时间线对照可验证「对齐→可视化漂移」)",
            static_cast<unsigned long>(version),
            static_cast<unsigned long>(new_epoch),
            ev.rmse,
            t_norm,
            static_cast<unsigned long>(ev.event_seq),
            ev.t_enu_to_map.x(), ev.t_enu_to_map.y(), ev.t_enu_to_map.z());
    }

    GPSAlignResult hba_align;
    hba_align.success = true;
    hba_align.R_gps_lidar = ev.R_enu_odom;
    hba_align.R_enu_to_map = ev.R_enu_to_map;
    hba_align.t_enu_to_map = ev.t_enu_to_map;
    hba_align.rmse_m = ev.rmse;
    hba_optimizer_.setGPSAlignedState(hba_align);
    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] HBA GPS alignment state synchronized: rmse=%.3fm", ev.rmse);

    // 🏛️ [GPS 对齐屏障] 历史关键帧仍为 ODOM 语义时，SubMapManager 内 T_map_b_optimized 曾与 T_odom_b 等同；
    // 若不先升级并与 Optimizer transformHistory 一致，则 merged_cloud / buildGlobalMap 与新建 MAP 关键帧双轨并存。
    // 必须在 freeze 前完成，使冻结路径上的 merged_cloud 与锚点已是 map 系。
    sm_manager_.applyGpsMapOriginToOdomKeyframes(ev.R_enu_to_map, ev.t_enu_to_map, version, new_epoch);

    // 🏛️ [修复] 解决中途对齐导致的严重重影：对齐后立即冻结当前活跃子图
    // 确保对齐后的关键帧在新的坐标系下开启新子图，避免与对齐前（Odom系）的关键帧混在同一子图导致 T_submap_kf 剧变
    auto sm = sm_manager_.getActiveSubmap();
    bool did_freeze = (sm && sm->state == SubMapState::ACTIVE);
    if (did_freeze)
        sm_manager_.freezeSubmap(sm);
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=updateGPSAlignment freezeSubmap=%s (grep V3 DIAG)",
        did_freeze ? "yes" : "no");

    // V3 addBatchGPSFactors 等价逻辑：若配置允许，投递 GPS_BATCH_KF任务，批量为历史关键帧添加 GPS 因子
    if (ConfigManager::instance().gpsAddConstraintsOnAlign()) {
        OptTaskItem batch_task;
        batch_task.type = OptTaskItem::Type::GPS_BATCH_KF;
        batch_task.R_enu_to_map = ev.R_enu_to_map;
        batch_task.t_enu_to_map = ev.t_enu_to_map;
        GraphTaskEvent task_ev;
        task_ev.task = batch_task;
        publishGraphTaskEvent(task_ev, node_->now().seconds());
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=updateGPSAlignment enqueue GPS_BATCH_KF (grep V3 DIAG)");
    }
}

void MappingModule::onPoseOptimized(const OptimizationResultEvent& ev) {
    if (!running_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[V3][CONTRACT] Reject optimization event: module is stopping/stopped");
        return;
    }
    legacy_opt_observe_total_.fetch_add(1, std::memory_order_relaxed);
    if (!shouldAcceptOptimizationEvent(ev)) return;
    if (route_takeover_enabled_.load(std::memory_order_relaxed)) {
        legacy_opt_skip_takeover_total_.fetch_add(1, std::memory_order_relaxed);
        uint64_t delta_hash = 0;
        {
            std::lock_guard<std::mutex> lk(delta_consistency_mutex_);
            auto it = recent_delta_hash_by_version_.find(ev.version);
            if (it != recent_delta_hash_by_version_.end()) delta_hash = it->second;
        }
        const uint64_t opt_hash = (ev.batch_hash != 0) ? ev.batch_hash : computeBatchHash(ev.submap_poses, ev.keyframe_poses);
        if (delta_hash != 0 && opt_hash != 0 && delta_hash != opt_hash) {
            consistency_mismatch_total_.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[V3][CONSISTENCY] delta/hash mismatch version=%lu delta_hash=%lu opt_hash=%lu mismatch_total=%lu",
                static_cast<unsigned long>(ev.version),
                static_cast<unsigned long>(delta_hash),
                static_cast<unsigned long>(opt_hash),
                static_cast<unsigned long>(consistency_mismatch_total_.load(std::memory_order_relaxed)));
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[V3][TAKEOVER] legacy_opt_observe=%lu skipped_apply=%lu delta_apply=%lu delta_reject=%lu mismatch=%lu",
            static_cast<unsigned long>(legacy_opt_observe_total_.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(legacy_opt_skip_takeover_total_.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(delta_apply_total_.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(delta_reject_total_.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(consistency_mismatch_total_.load(std::memory_order_relaxed)));
        return;
    }
    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] Gateway entry: event_id=%lu version=%lu pose_frame=%d sm=%zu kf=%zu src=%s flags=0x%x hash=%lu",
        static_cast<unsigned long>(ev.event_id),
        static_cast<unsigned long>(ev.version), static_cast<int>(ev.pose_frame),
        ev.submap_poses.size(), ev.keyframe_poses.size(),
        ev.source_module.c_str(),
        ev.transform_applied_flags,
        static_cast<unsigned long>(ev.batch_hash));
    
    // 🏛️ [架构加固] 统一通过语义网关应用位姿
    const bool sync_viz = ConfigManager::instance().vizSyncGlobalMapBuild();
    const bool hba = (ev.source_module == "HBAOptimizer");
    const bool applied =
        applyOptimizedPoses(ev.submap_poses, ev.keyframe_poses, ev.pose_frame, ev.version,
                            ev.alignment_epoch,
                            sync_viz && hba);
    if (applied) {
        legacy_opt_apply_total_.fetch_add(1, std::memory_order_relaxed);
    }
    if (applied && hba) {
        sm_manager_.rebuildMergedCloudFromOptimizedPoses();
        if (sync_viz) {
            const int interval =
                std::max(1, ConfigManager::instance().backendPublishGlobalMapEveryNProcessed());
            const int cnt = optimized_apply_count_.load(std::memory_order_relaxed);
            if (cnt % interval == 0) {
                GlobalMapBuildRequestEvent ge;
                ge.voxel_size = ConfigManager::instance().mapVoxelSize();
                ge.async = false;
                handleGlobalMapBuild(ge);
                RCLCPP_INFO(node_->get_logger(),
                    "[V3][VIZ_SYNC] HBA: rebuildMergedCloud -> sync handleGlobalMapBuild "
                    "optimized_apply_count=%d interval=%d (grep VIZ_SYNC)",
                    cnt, interval);
            } else {
                RCLCPP_DEBUG(node_->get_logger(),
                    "[V3][VIZ_SYNC] HBA: skip sync global map this round (throttle cnt=%d interval=%d)",
                    cnt, interval);
            }
        }
    }
}

void MappingModule::onPoseDelta(const OptimizationDeltaEvent& ev) {
    if (!running_.load()) return;
    if (!shouldAcceptOptimizationDelta(ev)) {
        delta_reject_total_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint64_t target_version = std::max<uint64_t>(processed_map_version_.load() + 1, ev.meta.ref_version + 1);
    const bool applied = applyOptimizedPoses(ev.submap_delta, ev.keyframe_delta, ev.pose_frame, target_version);
    if (applied) {
        delta_apply_total_.fetch_add(1, std::memory_order_relaxed);
        const uint64_t delta_hash = computeBatchHash(ev.submap_delta, ev.keyframe_delta);
        {
            std::lock_guard<std::mutex> lk(delta_consistency_mutex_);
            recent_delta_hash_by_version_[target_version] = delta_hash;
            if (recent_delta_hash_by_version_.size() > 64) {
                auto it = recent_delta_hash_by_version_.begin();
                recent_delta_hash_by_version_.erase(it);
            }
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[V3][DELTA] applied=%lu rejected=%lu producer=%s ref_v=%lu ref_e=%lu",
            static_cast<unsigned long>(delta_apply_total_.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(delta_reject_total_.load(std::memory_order_relaxed)),
            ev.producer.c_str(),
            static_cast<unsigned long>(ev.meta.ref_version),
            static_cast<unsigned long>(ev.meta.ref_epoch));
    }
}

bool MappingModule::applyOptimizedPoses(const std::unordered_map<int, Pose3d>& sm_poses,
                                       const std::unordered_map<uint64_t, Pose3d>& kf_poses,
                                       PoseFrame frame, uint64_t version,
                                       uint64_t alignment_epoch,
                                       bool skip_optimize_driven_global_map_request) {
    const auto t0 = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][APPLY_POSE_CHAIN] enter version=%lu pose_frame=%d epoch=%lu sm_poses=%zu kf_poses=%zu proc_map_ver=%lu (grep APPLY_POSE_CHAIN)",
        static_cast<unsigned long>(version),
        static_cast<int>(frame),
        static_cast<unsigned long>(alignment_epoch),
        sm_poses.size(),
        kf_poses.size(),
        static_cast<unsigned long>(processed_map_version_.load()));
    // 🏛️ [产品化加固] 状态守卫
    if (!running_.load()) return false;

    // 🏛️ [版本隔离] 拒绝过时的优化结果覆盖新数据
    if (version < processed_map_version_.load()) {
        RCLCPP_WARN(node_->get_logger(), 
            "[V3][GATEWAY] Rejecting stale optimization result: version=%lu, current_version=%lu",
            version, processed_map_version_.load());
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        return false;
    }

    // 1. 语义校准：如果系统已 GPS 对齐，但输入是 ODOM，则自动补偿 T_map_odom
    std::unordered_map<int, Pose3d> sm_to_apply = sm_poses;
    std::unordered_map<uint64_t, Pose3d> kf_to_apply = kf_poses;
    PoseFrame effective_frame = frame;

    if (gps_aligned_.load() && frame != PoseFrame::MAP) {
        Pose3d T_map_odom = Pose3d::Identity();
        {
            std::lock_guard<std::mutex> lk(gps_transform_mutex_);
            if (gps_transform_R_.allFinite() && gps_transform_t_.allFinite()) {
                T_map_odom.linear() = gps_transform_R_;
                T_map_odom.translation() = gps_transform_t_;
            }
        }

        // 🏛️ [熔断检查] 校验 T_map_odom 是否存在数值异常
        if (!T_map_odom.matrix().allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CRASH_GUARD] Rejecting compensation: non-finite T_map_odom");
            return false;
        }

        // 🏛️ [熔断检查] 如果补偿量过大（平移 > 1000m），通常意味着 GPS 对齐异常
        if (T_map_odom.translation().norm() > 1000.0) {
            RCLCPP_ERROR(node_->get_logger(), 
                "[V3][CRASH_GUARD] ABORT apply: T_map_odom translation=%.1fm > 1000m. Refusing to destroy map structure!",
                T_map_odom.translation().norm());
            return false;
        }
        RCLCPP_WARN(node_->get_logger(),
            "[V3][POSE_DIAG] ODOM->MAP compensation active: T_map_odom_t_norm=%.3f input_sm=%zu input_kf=%zu",
            T_map_odom.translation().norm(), sm_to_apply.size(), kf_to_apply.size());

        for (auto& [id, pose] : sm_to_apply) {
            pose = T_map_odom * pose;
        }
        for (auto& [id, pose] : kf_to_apply) {
            pose = T_map_odom * pose;
        }
        effective_frame = PoseFrame::MAP;
        RCLCPP_WARN(node_->get_logger(),
            "[V3][POSE_DIAG] Applied T_map_odom compensation to %zu submaps and %zu keyframes",
            sm_to_apply.size(), kf_to_apply.size());
    }

    // 🏛️ [数值安全] 入库前最终扫描，确保不包含 NaN/Inf
    for (const auto& [id, pose] : sm_to_apply) {
        if (!pose.matrix().allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CRASH_GUARD] Found NaN in optimized submap pose %d! Rejecting batch.", id);
            return false;
        }
    }
    for (const auto& [id, pose] : kf_to_apply) {
        if (!pose.matrix().allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CRASH_GUARD] Found NaN in optimized keyframe pose %lu! Rejecting batch.", id);
            return false;
        }
    }

    // 2. 批量分发到位姿后端 (SubMapManager / MapRegistry)
    if (!sm_to_apply.empty()) {
        sm_manager_.batchUpdateSubmapPoses(sm_to_apply, version, effective_frame, alignment_epoch);
    }
    if (!kf_to_apply.empty()) {
        sm_manager_.batchUpdateKeyFramePoses(kf_to_apply, version, effective_frame, alignment_epoch);
    }

    // 3. 屏障同步与子图隔离 (🏛️ [架构契约] 消除重影的核心逻辑)
    // 每次应用优化结果后，必须强制冻结当前子图。
    // 理由：子图内部所有点云必须在同一坐标系语义下。如果子图处理一半时坐标系跳变，
    // 则该子图后续的点云将与前半部分产生重影。通过强制切分，将跳变完全限制在子图间。
    auto active_sm = sm_manager_.getActiveSubmap();
    if (active_sm && active_sm->state == SubMapState::ACTIVE) {
        RCLCPP_INFO(node_->get_logger(),
            "[V3][GHOST_GUARD] Freezing submap %d due to pose optimization (version %lu)",
            active_sm->id, version);
        sm_manager_.freezeSubmap(active_sm);
    }

    // 4. 计算位姿跳变增量并反馈前端 (Propagation)
    // 选取最后一个关键帧作为参考点计算坐标系偏移
    if (!kf_to_apply.empty()) {
        uint64_t latest_kf_id = 0;
        for (const auto& [id, _] : kf_to_apply) latest_kf_id = std::max(latest_kf_id, id);
        
        auto kf = map_registry_->getKeyFrame(static_cast<int>(latest_kf_id));
        if (kf) {
            Pose3d T_old = kf->T_map_b_optimized;
            Pose3d T_new = kf_to_apply.at(latest_kf_id);
            Pose3d delta = T_new * T_old.inverse();

            FrontendPoseAdjustEvent adjust_ev;
            adjust_ev.from_version = processed_map_version_.load();
            adjust_ev.to_version = version;
            adjust_ev.T_map_new_map_old = delta;
            adjust_ev.target_frame = effective_frame;
            event_bus_->publish(adjust_ev);
            
            RCLCPP_INFO(node_->get_logger(),
                "[V3][GHOST_GUARD] Propagating pose jump to Frontend: delta_t=[%.2f,%.2f,%.2f]",
                delta.translation().x(), delta.translation().y(), delta.translation().z());
            if (ConfigManager::instance().vizLogPoseJumpDetail()) {
                const double t_norm = delta.translation().norm();
                const double rot_deg =
                    Eigen::AngleAxisd(delta.linear()).angle() * (180.0 / M_PI);
                RCLCPP_INFO(node_->get_logger(),
                    "[V3][GHOST_GUARD][POSE_JUMP_DETAIL] delta_t_norm=%.4fm delta_rot_deg=%.2f "
                    "from_ver=%lu to_ver=%lu kf_id=%lu",
                    t_norm, rot_deg,
                    static_cast<unsigned long>(adjust_ev.from_version),
                    static_cast<unsigned long>(adjust_ev.to_version),
                    static_cast<unsigned long>(latest_kf_id));
            }
        }
    }

    processed_map_version_.store(version);
    last_applied_version_ = version;

    // 5. 可视化同步（节流：避免每次优化都触发一次全局构图造成后端抖动）
    // 计数始终前进，供 HBA 后 sync 路径与 suppress_optimize_driven 组合时节流仍正确。
    const int optimize_interval =
        std::max(1, ConfigManager::instance().backendPublishGlobalMapEveryNProcessed());
    const int optimized_count = ++optimized_apply_count_;
    if (!ConfigManager::instance().vizSuppressOptimizeDrivenGlobalMap()) {
        if (skip_optimize_driven_global_map_request) {
            RCLCPP_DEBUG(node_->get_logger(),
                "[V3][PERF] Skip optimize-driven global map publish (HBA sync viz builds after rebuild) "
                "optimized_count=%d interval=%d",
                optimized_count, optimize_interval);
        } else if (optimized_count % optimize_interval == 0) {
            GlobalMapBuildRequestEvent build_ev;
            build_ev.voxel_size = ConfigManager::instance().mapVoxelSize();
            build_ev.async = ConfigManager::instance().globalMapBuildAsync();
            event_bus_->publish(build_ev);
            RCLCPP_INFO(node_->get_logger(),
                "[V3][PERF] Trigger optimize-driven global map build: optimized_count=%d interval=%d version=%lu async=%d",
                optimized_count, optimize_interval, static_cast<unsigned long>(version),
                build_ev.async ? 1 : 0);
        } else {
            RCLCPP_DEBUG(node_->get_logger(),
                "[V3][PERF] Skip optimize-triggered global map build (optimized_count=%d interval=%d)",
                optimized_count, optimize_interval);
        }
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(node_->get_logger(),
        "[V3][PERF] applyOptimizedPoses done: version=%lu frame=%d sm=%zu kf=%zu elapsed=%ldms",
        static_cast<unsigned long>(version), static_cast<int>(effective_frame),
        sm_to_apply.size(), kf_to_apply.size(), static_cast<long>(elapsed_ms));
    return true;
}

bool MappingModule::shouldAcceptOptimizationEvent(const OptimizationResultEvent& ev) {
    if (!ev.isValid()) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject invalid OptimizationResultEvent: event_id=%lu version=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            ev.source_module.c_str());
        return false;
    }
    if (ev.pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject event: pose_frame=UNKNOWN");
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return false;
    }
    if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" &&
        ev.pose_frame != PoseFrame::MAP &&
        !map_registry_->isGPSAligned()) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] strict_map_only bypassed before GPS alignment: accepting event pose_frame=%d",
            static_cast<int>(ev.pose_frame));
    } else if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" &&
               ev.pose_frame != PoseFrame::MAP) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject event by frame_policy=strict_map_only: pose_frame=%d",
            static_cast<int>(ev.pose_frame));
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return false;
    }
    if (ev.version < processed_map_version_.load()) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject stale event: event_id=%lu version=%lu current=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            static_cast<unsigned long>(processed_map_version_.load()),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        return false;
    }
    const uint64_t local_epoch = processed_alignment_epoch_.load();
    if (ev.alignment_epoch != local_epoch) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject event by alignment_epoch mismatch: event_id=%lu version=%lu event_epoch=%lu local_epoch=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            static_cast<unsigned long>(ev.alignment_epoch),
            static_cast<unsigned long>(local_epoch),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        return false;
    }
    const bool has_map_compensation_flag =
        (ev.transform_applied_flags & static_cast<uint32_t>(OptimizationTransformFlags::MAP_COMPENSATION_APPLIED)) != 0u;
    if (ev.pose_frame == PoseFrame::ODOM && has_map_compensation_flag) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject inconsistent event: ODOM frame cannot carry MAP compensation flag");
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return false;
    }
    if (ev.event_id == last_applied_event_id_) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject duplicate event_id: event_id=%lu version=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::DUPLICATE_OPTIMIZATION_EVENT_TOTAL);
        return false;
    }
    if (ev.version == last_applied_version_ && ev.batch_hash != 0 && ev.batch_hash == last_applied_batch_hash_) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject duplicate batch_hash: version=%lu hash=%lu src=%s",
            static_cast<unsigned long>(ev.version),
            static_cast<unsigned long>(ev.batch_hash),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::DUPLICATE_OPTIMIZATION_EVENT_TOTAL);
        return false;
    }
    last_applied_event_id_ = ev.event_id;
    last_applied_batch_hash_ = ev.batch_hash;
    return true;
}

bool MappingModule::shouldAcceptOptimizationDelta(const OptimizationDeltaEvent& ev) const {
    if (!ev.isValid()) return false;
    const uint64_t local_epoch = processed_alignment_epoch_.load(std::memory_order_relaxed);
    if (ev.alignment_epoch != local_epoch) return false;
    if (ev.meta.ref_epoch != local_epoch) return false;
    return ev.meta.ref_version >= processed_map_version_.load(std::memory_order_relaxed);
}

void MappingModule::handleSaveMap(const SaveMapRequestEvent& ev) {
    auto all_submaps = sm_manager_.getAllSubmaps();
    RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Saving %zu submaps to %s", 
                all_submaps.size(), ev.output_dir.c_str());

    fs::create_directories(ev.output_dir);
    for (const auto& sm : all_submaps) {
        if (sm) sm_manager_.archiveSubmap(sm, ev.output_dir);
    }

    // ========== [架构增强] 强制生成并保存一份全量全局地图 ==========
    try {
        RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Building final global map for saving...");
        float save_voxel_size = ConfigManager::instance().mapVoxelSize();
        uint64_t current_epoch = map_registry_->getAlignmentEpoch();
        
        // 🏛️ [架构加固] 保存前强制触发一次增量重建，确保 fallback 路径的 merged_cloud 也是基于最新位姿的
        sm_manager_.rebuildMergedCloudFromOptimizedPoses(); 
        
        sm_manager_.invalidateGlobalMapCache();
        CloudXYZIPtr global_map = sm_manager_.buildGlobalMap(save_voxel_size, current_epoch);
        if (global_map && !global_map->empty()) {
            // [RC-4 修复] 将最终 GPS 对齐地图保存在 optimized/ 子目录，与 fast-livo 保存的
            // all_raw_points.pcd（odom 系）物理隔离，防止用户误将两个不同坐标系的点云同时加载导致重影。
            const fs::path opt_dir = fs::path(ev.output_dir) / "optimized";
            std::error_code ec;
            fs::create_directories(opt_dir, ec);
            if (ec) {
                RCLCPP_WARN(node_->get_logger(),
                    "[V3][MappingModule] Failed to create optimized/ subdir (%s), saving to output_dir",
                    ec.message().c_str());
            }
            const std::string global_path = (!ec && fs::is_directory(opt_dir))
                ? (opt_dir / "global_map_final.pcd").string()
                : ev.output_dir + "/global_map_final.pcd";
            pcl::io::savePCDFileBinary(global_path, *global_map);
            RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Global map saved to %s (%zu pts)", 
                        global_path.c_str(), global_map->size());
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "[V3][MappingModule] Final global map empty or null; skip global_map_final.pcd (output_dir=%s)",
                ev.output_dir.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][MappingModule] Failed to save final global map: %s", e.what());
    }
    // ========================================================
}

void MappingModule::publishGlobalMapResultAndTriggerVizSync(const CloudXYZIPtr& global) {
    if (!global || global->empty()) {
        return;
    }
    // 先发布点云结果，再发 MapUpdate：EventBus 按类型分派，订阅 GlobalMapBuildResult 的节点（AutoMapSystem）
    // 先于 MapUpdate 的处理器跑完，RViz 收到云后再由 VisualizationModule 因 GLOBAL_MAP_REBUILT 刷新轨迹，避免「云已换系/位姿而 path 仍旧」的窗口。
    GlobalMapBuildResultEvent res;
    res.global_map = global;
    event_bus_->publish(res);
    MapUpdateEvent mue;
    mue.version = map_registry_->getVersion();
    mue.type = MapUpdateEvent::ChangeType::GLOBAL_MAP_REBUILT;
    event_bus_->publish(mue);
}

void MappingModule::handleGlobalMapBuild(const GlobalMapBuildRequestEvent& ev) {
    try {
        uint64_t limit = ev.alignment_epoch_limit;
        if (limit == 0) {
            limit = map_registry_->getAlignmentEpoch();
        }

        if (ev.async) {
            // 🏛️ [稳定性修复] 使用异步构建且不再通过 .get() 阻塞 MappingModule 线程 (解决 HUNG/ZOMBIE 问题)
            // 获取 future 后通过单独的辅助线程处理结果并发布事件，确保 MappingModule 心跳不中断
            auto future = sm_manager_.buildGlobalMapAsync(ev.voxel_size, limit);
            std::thread([this, f = std::move(future)]() mutable {
                try {
                    auto global = f.get();
                    publishGlobalMapResultAndTriggerVizSync(global);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "[V3][MappingModule][ASYNC_BUILD] Build failed: %s", e.what());
                }
            }).detach();
        } else {
            CloudXYZIPtr global = sm_manager_.buildGlobalMap(ev.voxel_size, limit);
            publishGlobalMapResultAndTriggerVizSync(global);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][MappingModule] Global map build failed: %s", e.what());
    }
}

Mat66d MappingModule::computeOdomInfoMatrix(const SubMap::Ptr& prev, const SubMap::Ptr& curr, const Pose3d& rel) const {
    Mat66d info = Mat66d::Identity();
    const auto& cfg = ConfigManager::instance();
    double trans_noise = cfg.smOdomTransNoise();
    double rot_noise = cfg.smOdomRotNoise();
    for (int i = 0; i < 3; ++i) info(i, i) = 1.0 / (trans_noise * trans_noise);
    for (int i = 3; i < 6; ++i) info(i, i) = 1.0 / (rot_noise * rot_noise);
    return info;
}

void MappingModule::publishGraphTaskEvent(GraphTaskEvent& ev, double source_ts) {
    const uint64_t seq = graph_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
    const double now_ts = node_->now().seconds();
    ev.meta.event_id = makeEventId(source_ts, seq);
    ev.meta.idempotency_key = ev.meta.event_id;
    ev.meta.producer_seq = seq;
    ev.meta.ref_version = map_registry_->getVersion();
    ev.meta.ref_epoch = map_registry_->getAlignmentEpoch();
    ev.meta.session_id = map_registry_->getSessionId();
    ev.meta.source_ts = std::isfinite(source_ts) ? source_ts : now_ts;
    ev.meta.publish_ts = now_ts;
    ev.meta.producer = "MappingModule";
    ev.meta.route_tag = route_takeover_enabled_.load(std::memory_order_relaxed) ? "orchestrated" : "legacy";
    ev.processing_state = quiescing_.load() ? ProcessingState::DEGRADED : ProcessingState::NORMAL;
    if (!ev.isValid()) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[V3][CONTRACT] Drop invalid GraphTaskEvent (meta contract)");
        return;
    }
    event_bus_->publish(ev);
}

void MappingModule::onSemanticLandmarks(const SemanticLandmarkEvent& ev) {
    if (!ev.isValid()) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Mapping][onSemanticLandmarks] step=reject reason=invalid_event ts=%.3f landmarks=%zu",
            ev.timestamp, ev.landmarks.size());
        return;
    }

    const uint64_t trace_id = semanticTraceId(
        ev.keyframe_id_hint != 0 ? ev.keyframe_id_hint : static_cast<uint64_t>(std::max(0.0, ev.timestamp) * 1000.0),
        ev.timestamp);
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2500,
        "[SEMANTIC][Mapping][onSemanticLandmarks] step=entry trace=%lu ts=%.3f trees_in=%zu planes_in=%zu kf_hint_id=%lu kf_hint_ts=%.3f "
        "(trees/planes 入图前仍受关联/support/gating；地面标签走 SemanticCloudEvent 不在此事件)",
        static_cast<unsigned long>(trace_id),
        ev.timestamp,
        ev.landmarks.size(),
        ev.plane_landmarks.size(),
        static_cast<unsigned long>(ev.keyframe_id_hint),
        ev.keyframe_timestamp_hint);

    auto defer_semantic_event = [this, trace_id](const SemanticLandmarkEvent& pending_ev, const char* reason, uint64_t kf_id, int sm_id) {
        std::lock_guard<std::mutex> lk(pending_semantic_mutex_);
        semantic_defer_total_.fetch_add(1, std::memory_order_relaxed);
        if (pending_semantic_landmarks_.size() >= max_pending_semantic_events_) {
            pending_semantic_landmarks_.pop_front();
            
            // 🏛️ [架构契约] 待处理队列满，发布警告
            BackpressureWarningEvent warn;
            warn.module_name = name_ + "_pending_semantic";
            warn.queue_usage_ratio = 1.0f;
            warn.critical = true;
            {
                const uint64_t seq = graph_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
                const double pts = node_->now().seconds();
                warn.meta.event_id = warn.meta.idempotency_key = makeEventId(pts, seq);
                warn.meta.producer_seq = warn.meta.event_id;
                warn.meta.session_id = pending_ev.meta.session_id != 0 ? pending_ev.meta.session_id
                                                                      : map_registry_->getSessionId();
                warn.meta.ref_version = map_registry_->getVersion();
                warn.meta.ref_epoch = map_registry_->getAlignmentEpoch();
                warn.meta.source_ts = pending_ev.timestamp;
                warn.meta.publish_ts = pts;
                warn.meta.producer = "MappingModule";
                warn.meta.route_tag = "legacy";
            }
            if (warn.isValid()) {
                event_bus_->publish(warn);
            }
        } else if (pending_semantic_landmarks_.size() > max_pending_semantic_events_ * 0.8) {
            BackpressureWarningEvent warn;
            warn.module_name = name_ + "_pending_semantic";
            warn.queue_usage_ratio = static_cast<float>(pending_semantic_landmarks_.size()) / max_pending_semantic_events_;
            warn.critical = false;
            {
                const uint64_t seq = graph_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
                const double pts = node_->now().seconds();
                warn.meta.event_id = warn.meta.idempotency_key = makeEventId(pts, seq);
                warn.meta.producer_seq = warn.meta.event_id;
                warn.meta.session_id = pending_ev.meta.session_id != 0 ? pending_ev.meta.session_id
                                                                       : map_registry_->getSessionId();
                warn.meta.ref_version = map_registry_->getVersion();
                warn.meta.ref_epoch = map_registry_->getAlignmentEpoch();
                warn.meta.source_ts = pending_ev.timestamp;
                warn.meta.publish_ts = pts;
                warn.meta.producer = "MappingModule";
                warn.meta.route_tag = "legacy";
            }
            if (warn.isValid()) {
                event_bus_->publish(warn);
            }
        }
        pending_semantic_landmarks_.push_back(pending_ev);
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Mapping][onSemanticLandmarks] step=defer reason=%s ts=%.3f kf_id=%lu sm_id=%d pending=%zu",
            reason, pending_ev.timestamp, static_cast<unsigned long>(kf_id), sm_id, pending_semantic_landmarks_.size());
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B3 MAP_IN] action=defer trace=%lu reason=%s sem_defer=%lu pending=%zu ts=%.3f kf_id=%lu sm_id=%d trees=%zu planes=%zu",
            static_cast<unsigned long>(trace_id),
            reason,
            static_cast<unsigned long>(semantic_defer_total_.load(std::memory_order_relaxed)),
            pending_semantic_landmarks_.size(),
            pending_ev.timestamp,
            static_cast<unsigned long>(kf_id),
            sm_id,
            pending_ev.landmarks.size(),
            pending_ev.plane_landmarks.size());
    };

    const uint64_t ev_session =
        ev.meta.session_id != 0 ? ev.meta.session_id : map_registry_->getSessionId();

    // 1. 查找对应的关键帧（优先 keyframe_id_hint，其次 keyframe timestamp hint，再兜底语义事件时间戳）
    KeyFrame::Ptr kf = nullptr;
    if (ev.keyframe_id_hint != 0) {
        kf = map_registry_->getKeyFrame(static_cast<int>(ev.keyframe_id_hint));
        if (kf && ev_session != 0 && kf->session_id != ev_session) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][Mapping][onSemanticLandmarks] kf_hint_id=%lu session mismatch (kf_sess=%lu ev_sess=%lu), "
                "ignore id hint",
                static_cast<unsigned long>(ev.keyframe_id_hint),
                static_cast<unsigned long>(kf->session_id),
                static_cast<unsigned long>(ev_session));
            kf = nullptr;
        }
    }
    if (std::isfinite(ev.keyframe_timestamp_hint) && ev.keyframe_timestamp_hint > 0.0) {
        if (!kf || std::abs(kf->timestamp - ev.keyframe_timestamp_hint) > semantic_timestamp_match_tolerance_s_) {
            kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.keyframe_timestamp_hint,
                                                                   semantic_timestamp_match_tolerance_s_,
                                                                   ev_session);
            if (!kf) {
                kf = map_registry_->getKeyFrameByTimestampPreferPrior(
                    ev.keyframe_timestamp_hint, std::min(0.5, semantic_timestamp_match_tolerance_s_ * 2.0),
                    ev_session);
            }
        }
    }
    if (!kf) {
        kf = map_registry_->getKeyFrameByTimestampPreferPrior(ev.timestamp, semantic_timestamp_match_tolerance_s_,
                                                               ev_session);
        if (!kf) {
            kf = map_registry_->getKeyFrameByTimestampPreferPrior(
                ev.timestamp, std::min(0.5, semantic_timestamp_match_tolerance_s_ * 2.0), ev_session);
        }
    }
    if (!kf) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B4 SEM_RESOLVE] action=reject trace=%lu reason=kf_not_found ts=%.3f reason_code=E_KF_NOT_FOUND",
            static_cast<unsigned long>(trace_id), ev.timestamp);
        defer_semantic_event(ev, "kf_not_found", 0, -1);
        return;
    }
    if (kf->submap_id < 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B4 SEM_RESOLVE] action=reject trace=%lu reason=submap_unassigned ts=%.3f kf_id=%lu reason_code=E_SUBMAP_UNASSIGNED",
            static_cast<unsigned long>(trace_id), ev.timestamp, static_cast<unsigned long>(kf->id));
        defer_semantic_event(ev, "submap_id_unassigned", kf->id, kf->submap_id);
        return;
    }

    // 2. 更新关键帧的地标
    // 🏛️ [架构契约] 数据完整性校验
    std::vector<CylinderLandmark::Ptr> valid_trees;
    for (const auto& l : ev.landmarks) {
        if (l && l->isValid()) {
            valid_trees.push_back(l);
        }
    }
    std::vector<PlaneLandmark::Ptr> valid_planes;
    for (const auto& l : ev.plane_landmarks) {
        if (l && l->isValid()) {
            valid_planes.push_back(l);
        }
    }

    if (valid_trees.empty() && valid_planes.empty()) return;
    kf->landmarks = valid_trees;
    kf->plane_landmarks = valid_planes;

    // 3. 关联到子图（由 SubMapManager 内部完成互斥，避免外层重复加锁导致自锁）
    auto sm = sm_manager_.getSubmap(static_cast<int>(kf->submap_id));
    if (!sm) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Mapping][onSemanticLandmarks] step=sm_not_found ts=%.3f kf_id=%lu sm_id=%d → skip",
            ev.timestamp, kf->id, kf->submap_id);
    } else {
        // 🏛️ [架构增强] 检查地标是否重复关联，避免同一物理地标在同一 KF 下产生多重约束
        sm_manager_.associateLandmarks(sm, kf);
        
        // 4. 发布优化任务
        OptTaskItem landmark_task;
        landmark_task.type = OptTaskItem::Type::CYLINDER_LANDMARK_FACTOR;
        landmark_task.to_id = static_cast<int>(kf->id);
        OptTaskItem plane_task;
        plane_task.type = OptTaskItem::Type::PLANE_LANDMARK_FACTOR;
        plane_task.to_id = static_cast<int>(kf->id);
        
        const auto& cfg = ConfigManager::instance();
        const int min_confirmations = cfg.semanticAssocCylinderMinConfirmations();
        const double min_observability = cfg.semanticAssocCylinderMinObservability();
        const int plane_min_confirmations = cfg.semanticAssocPlaneMinConfirmations();
        const double plane_min_observability = cfg.semanticAssocPlaneMinObservability();
        const bool protection_mode_active = sm_manager_.isSemanticProtectionModeActive();
        const bool weak_semantic_context = !gps_aligned_.load(std::memory_order_relaxed) && [&]() {
            std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
            return loop_constraints_.empty();
        }();
        const int sample_stride = protection_mode_active
            ? cfg.semanticAssocFactorSampleStrideProtected()
            : cfg.semanticAssocFactorSampleStrideNormal();
        const int weak_min_support = cfg.semanticAssocFactorWeakContextMinSupport();
        const double weak_min_observability = cfg.semanticAssocFactorWeakContextMinObservability();
        const uint64_t tree_input_this_frame = static_cast<uint64_t>(kf->landmarks.size());
        uint64_t tree_emitted_this_frame = 0;
        uint64_t tree_suppressed_this_frame = 0;
        uint64_t tree_suppressed_sampling_this_frame = 0;
        std::unordered_set<int> used_cylinder_idx;
        size_t candidate_idx = 0;
        for (const auto& l_kf : kf->landmarks) {
            if (l_kf->associated_idx >= 0 && l_kf->associated_idx < static_cast<int>(sm->landmarks.size())) {
                if (used_cylinder_idx.find(l_kf->associated_idx) != used_cylinder_idx.end()) {
                    ++tree_suppressed_this_frame;
                    continue;
                }
                const auto& l_sm = sm->landmarks[l_kf->associated_idx];
                if (!l_sm || !l_sm->isValid()) {
                    ++tree_suppressed_this_frame;
                    continue;
                }
                if (static_cast<int>(l_sm->support_count) < min_confirmations ||
                    l_sm->observability_score < min_observability) {
                    ++tree_suppressed_this_frame;
                    continue;
                }
                if (weak_semantic_context &&
                    (static_cast<int>(l_sm->support_count) < weak_min_support ||
                     l_sm->observability_score < weak_min_observability)) {
                    ++tree_suppressed_this_frame;
                    continue;
                }
                // 手动防呆：若树干 root 贴近强支撑墙面，则视为墙边伪树干，直接拒绝本帧因子。
                if (semanticBackendCylinderTooCloseToPlanes(
                        cfg, kf->T_submap_kf, l_kf, sm->plane_landmarks)) {
                    ++tree_suppressed_this_frame;
                    continue;
                }
                if (!semanticBackendCylinderPassesGating(cfg, kf->T_submap_kf, l_kf, l_sm)) {
                    ++tree_suppressed_this_frame;
                    continue;
                }
                if (sample_stride > 1 && ((candidate_idx++ % static_cast<size_t>(sample_stride)) != 0)) {
                    ++tree_suppressed_sampling_this_frame;
                    ++tree_suppressed_this_frame;
                    continue;
                }
                
                CylinderFactorItemKF factor;
                factor.kf_id = kf->id;
                factor.sm_id = static_cast<uint64_t>(sm->id);
                factor.point_body = semanticBackendCylinderSampleBody(l_kf);
                factor.root_submap = l_sm->root; 
                factor.ray_submap = l_sm->ray;
                factor.radius = l_sm->radius;
                factor.weight = semanticBackendFactorWeight(l_kf->confidence, l_sm->observability_score);
                
                landmark_task.cylinder_factors.push_back(factor);
                used_cylinder_idx.insert(l_kf->associated_idx);
                ++tree_emitted_this_frame;
            }
        }

        const uint64_t plane_input_this_frame = static_cast<uint64_t>(kf->plane_landmarks.size());
        uint64_t plane_emitted_this_frame = 0;
        uint64_t plane_suppressed_this_frame = 0;
        std::unordered_set<int> used_plane_idx;
        for (const auto& p_kf : kf->plane_landmarks) {
            if (!p_kf || !p_kf->isValid()) {
                continue;
            }

            if (p_kf->associated_idx < 0 || p_kf->associated_idx >= static_cast<int>(sm->plane_landmarks.size())) {
                continue;
            }
            if (used_plane_idx.find(p_kf->associated_idx) != used_plane_idx.end()) {
                ++plane_suppressed_this_frame;
                continue;
            }
            const auto& p_sm = sm->plane_landmarks[p_kf->associated_idx];
            if (!p_sm || !p_sm->isValid()) continue;
            if (static_cast<int>(p_sm->support_count) < plane_min_confirmations ||
                p_sm->observability_score < plane_min_observability) {
                ++plane_suppressed_this_frame;
                continue;
            }
            if (weak_semantic_context &&
                (static_cast<int>(p_sm->support_count) < weak_min_support ||
                 p_sm->observability_score < weak_min_observability)) {
                ++plane_suppressed_this_frame;
                continue;
            }
            if (!semanticBackendPlanePassesGating(cfg, p_kf)) {
                ++plane_suppressed_this_frame;
                continue;
            }

            Eigen::Vector3d centroid_b = Eigen::Vector3d::Zero();
            size_t cnt = 0;
            if (p_kf->points && !p_kf->points->empty()) {
                for (const auto& pt : p_kf->points->points) {
                    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                    centroid_b += Eigen::Vector3d(pt.x, pt.y, pt.z);
                    ++cnt;
                }
            }
            if (cnt == 0) {
                Eigen::Vector3d n_b = p_kf->normal;
                if (!n_b.allFinite() || n_b.norm() < 1e-6) continue;
                n_b.normalize();
                centroid_b = -p_kf->distance * n_b;
            } else {
                centroid_b /= static_cast<double>(cnt);
            }

            PlaneFactorItemKF pf;
            pf.kf_id = kf->id;
            pf.sm_id = static_cast<uint64_t>(sm->id);
            pf.point_body = centroid_b;
            pf.normal_submap = p_sm->normal.normalized();
            pf.distance_submap = p_sm->distance;
            pf.weight = semanticBackendFactorWeight(std::max(1e-3, p_kf->confidence), p_sm->observability_score);
            plane_task.plane_factors.push_back(pf);
            used_plane_idx.insert(p_kf->associated_idx);
            ++plane_emitted_this_frame;
        }

        semantic_assoc_tree_input_total_.fetch_add(tree_input_this_frame, std::memory_order_relaxed);
        semantic_assoc_tree_emitted_total_.fetch_add(tree_emitted_this_frame, std::memory_order_relaxed);
        semantic_assoc_tree_suppressed_total_.fetch_add(tree_suppressed_this_frame, std::memory_order_relaxed);
        semantic_assoc_plane_input_total_.fetch_add(plane_input_this_frame, std::memory_order_relaxed);
        semantic_assoc_plane_emitted_total_.fetch_add(plane_emitted_this_frame, std::memory_order_relaxed);
        semantic_assoc_plane_suppressed_total_.fetch_add(plane_suppressed_this_frame, std::memory_order_relaxed);
        
        if (!landmark_task.cylinder_factors.empty()) {
            GraphTaskEvent landmark_ev;
            landmark_ev.task = landmark_task;
            publishGraphTaskEvent(landmark_ev, ev.timestamp);
            const auto dispatched = semantic_dispatch_total_.fetch_add(1, std::memory_order_relaxed) + 1;

            RCLCPP_INFO(node_->get_logger(),
                "[SEMANTIC][Mapping][onSemanticLandmarks] step=dispatch ts=%.3f kf_id=%lu sm_id=%d factors=%zu → GraphTaskEvent",
                ev.timestamp, kf->id, sm->id, landmark_task.cylinder_factors.size());
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[CHAIN][B3 MAP_IN] action=dispatch trace=%lu sem_dispatch=%lu ts=%.3f kf_id=%lu sm_id=%d factors=%zu",
                static_cast<unsigned long>(trace_id),
                static_cast<unsigned long>(dispatched),
                ev.timestamp,
                static_cast<unsigned long>(kf->id),
                sm->id,
                landmark_task.cylinder_factors.size());
        } else {
            RCLCPP_DEBUG(node_->get_logger(),
                "[SEMANTIC][Mapping][onSemanticLandmarks] step=no_factors ts=%.3f kf_id=%lu sm_id=%d landmarks=%zu (none associated)",
                ev.timestamp, kf->id, sm->id, kf->landmarks.size());
        }
        if (tree_suppressed_sampling_this_frame > 0 || protection_mode_active || weak_semantic_context) {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[SEMANTIC][Mapping][factor_sampling] ts=%.3f kf_id=%lu sm_id=%d protection=%d weak_ctx=%d stride=%d "
                "tree(input/emitted/suppressed/sampling_suppressed)=(%lu/%lu/%lu/%lu)",
                ev.timestamp,
                static_cast<unsigned long>(kf->id),
                sm->id,
                protection_mode_active ? 1 : 0,
                weak_semantic_context ? 1 : 0,
                sample_stride,
                static_cast<unsigned long>(tree_input_this_frame),
                static_cast<unsigned long>(tree_emitted_this_frame),
                static_cast<unsigned long>(tree_suppressed_this_frame),
                static_cast<unsigned long>(tree_suppressed_sampling_this_frame));
        }

        if (!plane_task.plane_factors.empty()) {
            GraphTaskEvent plane_ev;
            plane_ev.task = plane_task;
            publishGraphTaskEvent(plane_ev, ev.timestamp);
            RCLCPP_INFO(node_->get_logger(),
                "[SEMANTIC][Mapping][onSemanticLandmarks] step=dispatch_plane ts=%.3f kf_id=%lu sm_id=%d factors=%zu → GraphTaskEvent",
                ev.timestamp, kf->id, sm->id, plane_task.plane_factors.size());
        }

        const uint64_t tree_input_total = semantic_assoc_tree_input_total_.load(std::memory_order_relaxed);
        const uint64_t tree_emitted_total = semantic_assoc_tree_emitted_total_.load(std::memory_order_relaxed);
        const uint64_t tree_suppressed_total = semantic_assoc_tree_suppressed_total_.load(std::memory_order_relaxed);
        const uint64_t plane_input_total = semantic_assoc_plane_input_total_.load(std::memory_order_relaxed);
        const uint64_t plane_emitted_total = semantic_assoc_plane_emitted_total_.load(std::memory_order_relaxed);
        const uint64_t plane_suppressed_total = semantic_assoc_plane_suppressed_total_.load(std::memory_order_relaxed);
        if (((tree_input_total + plane_input_total) % 50) == 0) {
            const double tree_supp_ratio = tree_input_total > 0
                ? (100.0 * static_cast<double>(tree_suppressed_total) / static_cast<double>(tree_input_total))
                : 0.0;
            const double plane_supp_ratio = plane_input_total > 0
                ? (100.0 * static_cast<double>(plane_suppressed_total) / static_cast<double>(plane_input_total))
                : 0.0;
            const double tree_emit_ratio = tree_input_total > 0
                ? (100.0 * static_cast<double>(tree_emitted_total) / static_cast<double>(tree_input_total))
                : 0.0;
            const double plane_emit_ratio = plane_input_total > 0
                ? (100.0 * static_cast<double>(plane_emitted_total) / static_cast<double>(plane_input_total))
                : 0.0;
            RCLCPP_INFO(node_->get_logger(),
                "[SEMANTIC][Mapping][assoc_factor_stats] tree(input/emitted/suppressed)=(%lu/%lu/%lu) tree_rates(emitted/suppressed)=(%.1f%%/%.1f%%) "
                "plane(input/emitted/suppressed)=(%lu/%lu/%lu) plane_rates(emitted/suppressed)=(%.1f%%/%.1f%%)",
                static_cast<unsigned long>(tree_input_total),
                static_cast<unsigned long>(tree_emitted_total),
                static_cast<unsigned long>(tree_suppressed_total),
                tree_emit_ratio, tree_supp_ratio,
                static_cast<unsigned long>(plane_input_total),
                static_cast<unsigned long>(plane_emitted_total),
                static_cast<unsigned long>(plane_suppressed_total),
                plane_emit_ratio, plane_supp_ratio);
        }
    }
}

} // namespace automap_pro::v3
