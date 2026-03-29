#pragma once

#include "automap_pro/core/data_types.h"

#include "automap_pro/core/opt_task_types.h"
#include "automap_pro/v3/event_bus.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/submap/submap_manager.h"

#include <unordered_map>
#include <mutex>
#include <memory>
#include <future>
#include <vector>
#include <atomic>
#include <string>
#include <cmath>
#include <cstdint>

namespace automap_pro::v3 {

enum class ProcessingState : uint8_t {
    NORMAL = 0,
    DEGRADED = 1,
    RECOVERING = 2
};

struct EventMeta {
    uint64_t event_id = 0;
    uint64_t session_id = 0;    // 🏛️ [架构契约] 显式标注所属会话，用于离线回放多段数据时的隔离
    uint64_t idempotency_key = 0;
    uint64_t producer_seq = 0;
    uint64_t ref_version = 0;
    uint64_t ref_epoch = 0;
    double source_ts = 0.0;
    double publish_ts = 0.0;
    std::string producer = "unknown";
    std::string route_tag = "legacy";

    bool isValid() const {
        if (event_id == 0 || idempotency_key == 0 || producer_seq == 0) return false;
        // 🏛️ [P0 稳定性修复] 必须显式校验 session_id，防止跨段回放时逻辑混乱
        if (session_id == 0) return false;
        if (ref_epoch == 0) return false;
        if (!std::isfinite(source_ts) || !std::isfinite(publish_ts)) return false;
        if (producer.empty()) return false;
        return true;
    }
};

struct RouteAdviceEvent {
    EventMeta meta;
    std::string event_type;
    std::string suggested_owner;
    bool takeover_enabled = false;
    bool fallback_to_legacy = true;
    std::string reason;
};

enum class OptimizationTransformFlags : uint32_t {
    NONE = 0u,
    MAP_COMPENSATION_APPLIED = 1u << 0u
};

/**
 * @brief 核心地图注册中心 (Central Map Registry)
 * 
 * 职责：
 * 1. 作为整个后端地图数据的单一可信源 (Single Source of Truth)
 * 2. 管理 KeyFrame 和 SubMap 的增删查改
 * 3. 维护位姿的版本信息 (Pose Versioning)
 * 4. 触发地图变更事件 (Map Change Events)
 */
class MapRegistry {
public:
    using Ptr = std::shared_ptr<MapRegistry>;

    /**
     * @brief 构造函数
     * @param event_bus 依赖的事件总线
     */
    explicit MapRegistry(EventBus::Ptr event_bus) 
        : event_bus_(event_bus), next_version_(1) {
    }

    // --- KeyFrame 管理 ---
    
    void addKeyFrame(KeyFrame::Ptr kf);
    KeyFrame::Ptr getKeyFrame(int id) const;
    /** 根据时间戳近邻查找关键帧；在容差内返回 |Δt| 最小项，无时返回 nullptr */
    KeyFrame::Ptr getKeyFrameByTimestamp(double timestamp, double tolerance_s = 1e-4) const;
    /**
     * 语义/数据挂接推荐：优先取 kf.timestamp ≤ timestamp 且 (timestamp − kf.timestamp) ≤ tolerance 的**最近**关键帧，
     * 避免晚到事件在宽容差下挂到「下一帧」KF。若无满足条件的 prior（事件早于首帧或超前过多），则退化为
     * getKeyFrameByTimestamp 的 |Δt| 最小策略。
     * @param session_id 非 0 时仅匹配该会话关键帧，禁止跨 session 挂接（多段回放/多会话地图）。
     */
    KeyFrame::Ptr getKeyFrameByTimestampPreferPrior(double timestamp, double tolerance_s = 1e-4,
                                                    uint64_t session_id = 0) const;
    /** 时间戳最大的关键帧；无时返回 nullptr */
    KeyFrame::Ptr getLatestKeyFrameByTimestamp() const;
    /**
     * 可视化/当前帧链式校正用的锚点：优先取 ts<=event_timestamp 的最新关键帧（与 ev.T_odom_b 同属「已过去」时间线）；
     * 🏛️ [架构加固] 必须匹配 session_id，防止离线回放多段数据时跨段寻锚导致的瞬移跳变。
     */
    KeyFrame::Ptr getAnchorKeyFrameForEventTime(double event_timestamp, uint64_t session_id) const;
    /** 已注册关键帧中的最大 timestamp；无关键帧返回 -inf（用于诊断 VIZ 与 Mapping 时序滞后） */
    double getLatestKeyFrameTimestamp() const;
    std::vector<KeyFrame::Ptr> getAllKeyFrames() const;
    size_t keyframeCount() const { return keyframes_count_.load(); }

    // --- 会话管理 ---
    uint64_t getSessionId() const { return session_id_.load(); }
    /** 切换会话前将当前 GPS 对齐写入 history，并刷新快照中的 session_alignments（V1 多会话）。 */
    void setSessionId(uint64_t id);

    // --- SubMap 管理 ---

    void addSubMap(SubMap::Ptr sm);
    SubMap::Ptr getSubMap(int id) const;
    std::vector<SubMap::Ptr> getAllSubMaps() const;
    size_t submapCount() const { return submaps_count_.load(); }

    // --- 位姿更新与版本控制 ---

    /**
     * @brief 批量更新位姿
     * @param sm_updates submap_id -> pose
     * @param kf_updates keyframe_id -> pose
     * @return 新的版本号
     */
    uint64_t updatePoses(const std::unordered_map<int, Pose3d>& sm_updates,
                         const std::unordered_map<uint64_t, Pose3d>& kf_updates,
                         PoseFrame pose_frame,
                         const std::string& source_module,
                         uint64_t source_alignment_epoch,
                         uint32_t transform_applied_flags = static_cast<uint32_t>(OptimizationTransformFlags::NONE),
                         uint64_t batch_hash = 0);

    /**
     * @brief 获取当前版本号
     */
    uint64_t getVersion() const { return current_version_.load(); }
    uint64_t getAlignmentEpoch() const { return alignment_epoch_.load(); }

    // --- 拓扑结构 ---

    void addConstraint(const LoopConstraint::Ptr& lc);
    std::vector<LoopConstraint::Ptr> getConstraints() const;

    // --- 会话加载 ---
    void loadSession(const std::string& session_dir, uint64_t session_id);

    // --- GPS 状态 ---
    uint64_t setGPSAligned(bool aligned, const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
                           const Eigen::Vector3d& t = Eigen::Vector3d::Zero(), double rmse = 0.0);
    bool isGPSAligned() const { return gps_aligned_.load(); }
    void getGPSTransform(Eigen::Matrix3d& R, Eigen::Vector3d& t) const {
        std::lock_guard<std::mutex> lk(gps_state_mutex_);
        R = R_enu_to_map_;
        t = t_enu_to_map_;
    }
    double getGPSRMSE() const {
        std::lock_guard<std::mutex> lk(gps_state_mutex_);
        return gps_rmse_;
    }

    // --- GPS 原点 ---
    void setGPSOrigin(double lat, double lon, double alt) {
        if (!gps_origin_set_.load()) {
            origin_lat_ = lat;
            origin_lon_ = lon;
            origin_alt_ = alt;
            gps_origin_set_.store(true);
        }
    }
    bool getGPSOrigin(double& lat, double& lon, double& alt) const {
        if (!gps_origin_set_.load()) return false;
        lat = origin_lat_;
        lon = origin_lon_;
        alt = origin_alt_;
        return true;
    }

    /**
     * @brief 获取当前位姿快照 (线程安全，读操作无锁竞争)
     */
    PoseSnapshot::Ptr getPoseSnapshot() const {
        std::lock_guard<std::mutex> lk(snapshot_mutex_);
        return current_snapshot_;
    }

private:
    EventBus::Ptr event_bus_;
    
    mutable std::mutex snapshot_mutex_;
    PoseSnapshot::Ptr current_snapshot_ = std::make_shared<PoseSnapshot>();
    
    mutable std::mutex kf_mutex_;
    std::unordered_map<int, KeyFrame::Ptr> keyframes_;
    std::atomic<size_t> keyframes_count_{0};

    mutable std::mutex sm_mutex_;
    std::unordered_map<int, SubMap::Ptr> submaps_;
    std::atomic<size_t> submaps_count_{0};

    mutable std::mutex constraint_mutex_;
    std::vector<LoopConstraint::Ptr> constraints_;

    std::atomic<uint64_t> current_version_{0};
    std::atomic<uint64_t> next_version_{1};
    uint64_t padding_for_atomic_version_ = 0; // Not really needed, just replacing the line

    // GPS 状态缓存（需同步读写，避免竞态）
    mutable std::mutex gps_state_mutex_;
    std::atomic<bool> gps_aligned_{false};
    std::atomic<uint64_t> alignment_epoch_{1};
    Eigen::Matrix3d R_enu_to_map_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_enu_to_map_ = Eigen::Vector3d::Zero();
    double gps_rmse_ = 0.0;

    // GPS 原点
    std::atomic<bool> gps_origin_set_{false};
    double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_alt_ = 0.0;

    std::atomic<uint64_t> session_id_{0};
    /** 🏛️ [V1] 持久化所有会话的对齐信息，跨快照共享 */
    std::unordered_map<uint64_t, PoseSnapshot::SessionAlignment> session_alignments_history_;
};

// --- 事件定义 (Events) ---

struct MapUpdateEvent {
    uint64_t version;
    enum class ChangeType {
        KEYFRAME_ADDED,
        SUBMAP_ADDED,
        POSES_OPTIMIZED,
        CONSTRAINT_ADDED,
        /** SubMapManager::buildGlobalMap* 成功且已发布 GlobalMapBuildResultEvent 之后触发，强制 RViz 轨迹/关键帧与刚发布的全局点云同一 Registry 快照对齐 */
        GLOBAL_MAP_REBUILT
    } type;
    std::vector<int> affected_ids;
};

// --- GPS 事件 (GPS Events) ---

struct RawGPSEvent {
    double timestamp;
    double lat, lon, alt, hdop;
    int sats;
};

struct GPSAlignedEvent {
    /** 与 GPSAlignResult::success 一致；MapRegistry 对齐态仅由 MappingModule 根据本字段写入 */
    bool success = false;
    /** GPS 对齐事件序号（单调递增），用于去重与乱序保护。 */
    uint64_t event_seq = 0;
    /** 发布时观测到的当前 alignment_epoch（提示值，非目标写入值）。 */
    uint64_t alignment_epoch = 0;
    Eigen::Matrix3d R_enu_to_map;
    Eigen::Vector3d t_enu_to_map;
    double rmse = 0.0;
    EventMeta meta;

    bool isValid() const {
        if (event_seq == 0) return false;
        if (!R_enu_to_map.allFinite() || !t_enu_to_map.allFinite()) return false;
        if (!meta.isValid()) return false;
        if (success) return std::isfinite(rmse);
        return true;
    }
};

struct GPSFactorEvent {
    int submap_id;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};

// --- 回环事件 (Loop Events) ---

struct LoopConstraintEvent {
    LoopConstraint::Ptr constraint;
    EventMeta meta;

    bool isValid() const {
        return constraint != nullptr && meta.isValid();
    }
};

struct IntraLoopTaskEvent {
    SubMap::Ptr submap;
    int query_idx;
};

// --- 优化事件 (Optimization Events) ---

struct OptimizationResultEvent {
    uint64_t version;
    uint64_t event_id = 0;
    uint64_t alignment_epoch = 0;
    std::unordered_map<int, Pose3d> submap_poses;
    std::unordered_map<uint64_t, Pose3d> keyframe_poses;
    std::string source_module = "unknown";
    uint32_t transform_applied_flags = 0;
    uint64_t batch_hash = 0;
    PoseFrame pose_frame = PoseFrame::MAP; // 🏛️ [架构加固] 显式标注位姿坐标系
    EventMeta meta;

    bool isValid() const {
        if (event_id == 0 || version == 0 || alignment_epoch == 0) return false;
        if (pose_frame == PoseFrame::UNKNOWN) return false;
        if (source_module.empty()) return false;
        if (!meta.isValid()) return false;
        for (const auto& [id, pose] : submap_poses) if (!pose.matrix().allFinite()) return false;
        for (const auto& [id, pose] : keyframe_poses) if (!pose.matrix().allFinite()) return false;
        return true;
    }
};

struct OptimizationDeltaEvent {
    EventMeta meta;
    uint64_t alignment_epoch = 0;
    PoseFrame pose_frame = PoseFrame::UNKNOWN;
    std::unordered_map<int, Pose3d> submap_delta;
    std::unordered_map<uint64_t, Pose3d> keyframe_delta;
    std::string producer = "unknown";

    bool isValid() const {
        if (!meta.isValid()) return false;
        if (alignment_epoch == 0) return false;
        if (pose_frame == PoseFrame::UNKNOWN) return false;
        return true;
    }
};

// --- 传感器与前端事件 (Sensor & FrontEnd Events) ---

struct RawOdometryEvent {
    double timestamp;
    Pose3d pose;
    Mat66d covariance;
};

struct RawCloudEvent {
    double timestamp;
    CloudXYZIPtr cloud;
};

struct RawKFInfoEvent {
    LivoKeyFrameInfo info;
};

/**
 * @brief 已同步并经过预处理的帧事件
 * 由 FrontEndModule 发布，MappingModule 订阅
 */
struct SyncedFrameEvent {
    double timestamp;
    CloudXYZIConstPtr cloud;
    CloudXYZIConstPtr cloud_ds; // Downsampled
    Pose3d T_odom_b;
    Mat66d covariance;
    PoseFrame pose_frame = PoseFrame::ODOM; // 🏛️ [架构契约] 显式标注 T_odom_b 的坐标系语义
    std::string cloud_frame = "body";       // 🏛️ [架构契约] 显式标注 cloud/cloud_ds 的坐标系语义：body/world
    LivoKeyFrameInfo kf_info;
    
    // GPS 观测 (可选)
    bool has_gps = false;
    GPSMeasurement gps;

    // 语义地标 (可选)
    std::vector<CylinderLandmark::Ptr> landmarks;

    // 🏛️ 生产级确定性：本帧依赖的地图版本
    // MappingModule 必须等待 MapRegistry 达到此版本后才处理本帧，确保坐标系一致
    uint64_t ref_map_version = 0;
    uint64_t ref_alignment_epoch = 0;
    EventMeta meta;
    ProcessingState processing_state = ProcessingState::NORMAL;

    bool isValid() const {
        if (!std::isfinite(timestamp)) return false;
        if (!cloud || cloud->empty()) return false;
        if (!T_odom_b.matrix().allFinite()) return false;
        if (!covariance.allFinite()) return false;
        if (ref_alignment_epoch == 0) return false;
        if (!meta.isValid()) return false;
        if (pose_frame == PoseFrame::UNKNOWN) return false;
        if (cloud_frame != "body" && cloud_frame != "world") return false;
        if (has_gps) {
            if (!gps.position_enu.allFinite()) return false;
            if (!gps.covariance.allFinite()) return false;
        }
        return true;
    }
};

/**
 * @brief 语义地标事件 (Semantic Landmark Event)
 * 🏛️ [架构演进] 异步语义处理结果，解耦前端与深度学习推理。
 * 由 SemanticModule 发布，MappingModule 订阅。
 */
struct SemanticLandmarkEvent {
    double timestamp = 0.0;
    double keyframe_timestamp_hint = 0.0; // 关键帧模式下由前端 KFInfo 时间透传，优先用于绑定 KeyFrame
    uint64_t keyframe_id_hint = 0;        // 关键帧 ID 提示（优先于时间戳匹配）
    std::vector<CylinderLandmark::Ptr> landmarks; // 树干/杆状物
    std::vector<PlaneLandmark::Ptr> plane_landmarks; // 墙面/平面
    EventMeta meta;
    ProcessingState processing_state = ProcessingState::NORMAL;

    bool isValid() const {
        return std::isfinite(timestamp) && (!landmarks.empty() || !plane_landmarks.empty()) && meta.isValid();
    }
};

/**
 * @brief 语义点云事件 (Semantic Cloud Event)
 * 用于可视化带标签的点云。
 */
struct SemanticCloudEvent {
    double timestamp = 0.0;
    CloudXYZIConstPtr labeled_cloud; // intensity 字段存储 label
    std::string frame_id = "body";
    EventMeta meta;

    bool isValid() const {
        if (!std::isfinite(timestamp)) return false;
        if (!labeled_cloud || labeled_cloud->empty()) return false;
        if (frame_id != "body" && frame_id != "world") return false;
        return meta.isValid();
    }
};

/**
 * @brief 树干（tree_label / maskCloud 结果）调试可视化：聚类前为 mask 后有效点；聚类后 intensity 为簇编号(1..N)。
 * frame_id 与 SyncedFrameEvent / SemanticCloudEvent 一致（通常为 body）；VisualizationModule 将其变到 map 再发 RViz。
 */
struct SemanticTrunkVizEvent {
    double timestamp = 0.0;
    std::string frame_id = "body";
    CloudXYZIPtr pre_cluster_body;
    CloudXYZIPtr post_cluster_body;
    EventMeta meta;

    bool isValid() const {
        if (!std::isfinite(timestamp)) return false;
        if (frame_id != "body" && frame_id != "world") return false;
        const bool has_pts = (pre_cluster_body && !pre_cluster_body->empty()) ||
                             (post_cluster_body && !post_cluster_body->empty());
        if (!has_pts) return false;
        return meta.isValid();
    }
};

enum class FilterFallbackReason : uint8_t {
    NONE = 0,
    EMPTY_STATIC,
    INPUT_INVALID,
    CLOUD_FRAME_UNSUPPORTED,
    FILTER_DISABLED,
    SHADOW_MODE,
    INTERNAL_ERROR
};

inline const char* toString(FilterFallbackReason r) {
    switch (r) {
        case FilterFallbackReason::NONE: return "none";
        case FilterFallbackReason::EMPTY_STATIC: return "empty_static";
        case FilterFallbackReason::INPUT_INVALID: return "input_invalid";
        case FilterFallbackReason::CLOUD_FRAME_UNSUPPORTED: return "cloud_frame_unsupported";
        case FilterFallbackReason::FILTER_DISABLED: return "filter_disabled";
        case FilterFallbackReason::SHADOW_MODE: return "shadow_mode";
        case FilterFallbackReason::INTERNAL_ERROR: return "internal_error";
        default: return "unknown";
    }
}

/**
 * @brief 动态过滤后的帧事件（可选 cloud_ds）
 * 用于模块内部与兼容链路，消费者必须显式处理 cloud_ds 为空的降级逻辑。
 */
struct FilteredFrameEventOptionalDs {
    double timestamp;
    CloudXYZIPtr cloud;
    CloudXYZIPtr cloud_ds;
    Pose3d T_odom_b;
    Mat66d covariance;
    PoseFrame pose_frame = PoseFrame::ODOM;
    std::string cloud_frame = "body";
    LivoKeyFrameInfo kf_info;
    bool has_gps = false;
    GPSMeasurement gps;
    uint64_t ref_map_version = 0;
    uint64_t ref_alignment_epoch = 0;
    EventMeta meta;
    ProcessingState processing_state = ProcessingState::NORMAL;

    // filtering diagnostics
    bool filter_executed = false;
    bool filtered_output_used = false;
    FilterFallbackReason fallback_reason = FilterFallbackReason::NONE;
    double dynamic_ratio = 0.0;
    double filter_latency_ms = 0.0;
    size_t input_points = 0;
    size_t output_points = 0;

    bool isValid() const {
        if (!std::isfinite(timestamp)) return false;
        if (!cloud || cloud->empty()) return false;
        if (!T_odom_b.matrix().allFinite()) return false;
        if (!covariance.allFinite()) return false;
        if (ref_alignment_epoch == 0) return false;
        if (!meta.isValid()) return false;
        if (pose_frame == PoseFrame::UNKNOWN) return false;
        if (cloud_frame != "body" && cloud_frame != "world") return false;
        if (filtered_output_used && !filter_executed) return false;
        if (!std::isfinite(dynamic_ratio) || dynamic_ratio < 0.0 || dynamic_ratio > 1.0) return false;
        if (!std::isfinite(filter_latency_ms) || filter_latency_ms < 0.0) return false;
        if (has_gps) {
            if (!gps.position_enu.allFinite()) return false;
            if (!gps.covariance.allFinite()) return false;
        }
        return true;
    }
};

/**
 * @brief 动态过滤后的帧事件（强契约：cloud_ds 必填）
 * MappingModule 仅订阅此类型，禁止默认可空+隐式假设非空。
 */
struct FilteredFrameEventRequiredDs : public FilteredFrameEventOptionalDs {
    bool isValid() const {
        if (!FilteredFrameEventOptionalDs::isValid()) return false;
        if (!cloud_ds || cloud_ds->empty()) return false;
        return true;
    }
};

// --- 图维护事件 (Graph Maintenance Events) ---

struct GraphTaskEvent {
    OptTaskItem task;
    EventMeta meta;
    ProcessingState processing_state = ProcessingState::NORMAL;

    bool isValid() const { return meta.isValid(); }
};

/**
 * @brief Semantic 独立输入事件
 * 与优化任务事件解耦，仅用于驱动语义链路。
 */
struct SemanticInputEvent {
    double timestamp = 0.0;
    KeyFrame::Ptr keyframe;
    EventMeta meta;
    ProcessingState processing_state = ProcessingState::NORMAL;

    bool isValid() const {
        return std::isfinite(timestamp) && keyframe != nullptr && meta.isValid();
    }
};

struct GPSAlignRequestEvent {
    bool force = false;
};

struct HBARequestEvent {
    bool wait_for_result = false;
};

struct LoadSessionRequestEvent {
    std::string session_dir;
    uint64_t session_id;
};

struct SaveMapRequestEvent {
    std::string output_dir;
    /** If non-null, set_value() after archive + final global PCD attempt (success or failure). */
    std::shared_ptr<std::promise<void>> completion;
};

struct GlobalMapBuildRequestEvent {
    float voxel_size;
    bool async = true;
    uint64_t alignment_epoch_limit = 0; // 0 means no filter
};

struct GlobalMapBuildResultEvent {
    CloudXYZIPtr global_map;
};

// --- 系统健康与状态事件 (System Health & Status Events) ---

/**
 * @brief 系统静默请求 (Quiescence Request)
 * 🏛️ [架构契约] 强制所有模块排空队列，进入只读/待机模式。
 */
struct SystemQuiesceRequestEvent {
    bool enable = true;
    std::string reason = "manual";
    EventMeta meta;

    /** 静默请求须带完备 meta，供 Orchestrator 审计；无 Registry 时不要发布裸事件。 */
    bool isValid() const { return meta.isValid(); }
};

/**
 * @brief 背压警告事件 (Backpressure Warning)
 * 🏛️ [架构契约] 当消费者队列过载时，向上游发出减速信号。
 */
struct BackpressureWarningEvent {
    std::string module_name;
    float queue_usage_ratio; // 0.0 ~ 1.0
    bool critical = false;
    ProcessingState processing_state = ProcessingState::DEGRADED;
    EventMeta meta;

    bool isValid() const {
        if (module_name.empty()) return false;
        if (!std::isfinite(queue_usage_ratio) || queue_usage_ratio < 0.0f || queue_usage_ratio > 1.0f) {
            return false;
        }
        return meta.isValid();
    }
};

struct ModuleStatus {
    std::string name;
    bool ok;
    double last_heartbeat;
    double age_s;
};

struct SystemStatusEvent {
    double timestamp;
    std::vector<ModuleStatus> modules;
    bool overall_ok;
};

/**
 * @brief 前端位姿调整事件 (Frontend Pose Adjustment)
 * 🏛️ [架构契约] 当后端发生回环或 GPS 对齐导致地图坐标系跳变时，通知前端同步调整。
 */
struct FrontendPoseAdjustEvent {
    uint64_t from_version;
    uint64_t to_version;
    Pose3d T_map_new_map_old; // 坐标系跳变增量
    PoseFrame target_frame;
};

} // namespace automap_pro::v3
