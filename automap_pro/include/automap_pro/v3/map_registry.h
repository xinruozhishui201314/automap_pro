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

namespace automap_pro::v3 {

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
    /** 时间戳最大的关键帧（用于与当前帧点云对齐到最新优化位姿链）；无时返回 nullptr */
    KeyFrame::Ptr getLatestKeyFrameByTimestamp() const;
    std::vector<KeyFrame::Ptr> getAllKeyFrames() const;
    size_t keyframeCount() const { return keyframes_count_.load(); }

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
                         uint32_t transform_applied_flags = static_cast<uint32_t>(OptimizationTransformFlags::NONE),
                         uint64_t batch_hash = 0);

    /**
     * @brief 获取当前版本号
     */
    uint64_t getVersion() const { return current_version_.load(); }

    // --- 拓扑结构 ---

    void addConstraint(const LoopConstraint::Ptr& lc);
    std::vector<LoopConstraint::Ptr> getConstraints() const;

    // --- 会话加载 ---
    void loadSession(const std::string& session_dir, uint64_t session_id);

    // --- GPS 状态 ---
    uint64_t setGPSAligned(bool aligned, const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(), 
                       const Eigen::Vector3d& t = Eigen::Vector3d::Zero(), double rmse = 0.0) {
        std::lock_guard<std::mutex> lk(gps_state_mutex_);
        gps_aligned_.store(aligned);
        R_enu_to_map_ = R;
        t_enu_to_map_ = t;
        gps_rmse_ = rmse;

        // 🏛️ 同步更新快照
        std::lock_guard<std::mutex> snap_lk(snapshot_mutex_);
        auto new_snap = std::make_shared<PoseSnapshot>(*current_snapshot_);
        new_snap->gps_aligned = aligned;
        new_snap->R_enu_to_map = R;
        new_snap->t_enu_to_map = t;
        new_snap->gps_rmse = rmse;
        
        uint64_t version = ++next_version_;
        new_snap->version = version;
        current_snapshot_ = new_snap;
        current_version_.store(version);
        return version;
    }
    bool isGPSAligned() const { return gps_aligned_.load(); }
    void getGPSTransform(Eigen::Matrix3d& R, Eigen::Vector3d& t) const {
        std::lock_guard<std::mutex> lk(gps_state_mutex_);
        R = R_enu_to_map_;
        t = t_enu_to_map_;
    }
    double getGPSRMSE() const { return gps_rmse_; }

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
    Eigen::Matrix3d R_enu_to_map_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_enu_to_map_ = Eigen::Vector3d::Zero();
    double gps_rmse_ = 0.0;

    // GPS 原点
    std::atomic<bool> gps_origin_set_{false};
    double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_alt_ = 0.0;
};

// --- 事件定义 (Events) ---

struct MapUpdateEvent {
    uint64_t version;
    enum class ChangeType {
        KEYFRAME_ADDED,
        SUBMAP_ADDED,
        POSES_OPTIMIZED,
        CONSTRAINT_ADDED
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
    Eigen::Matrix3d R_enu_to_map;
    Eigen::Vector3d t_enu_to_map;
    double rmse;

    bool isValid() const {
        return R_enu_to_map.allFinite() && t_enu_to_map.allFinite() && std::isfinite(rmse);
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
};

struct IntraLoopTaskEvent {
    SubMap::Ptr submap;
    int query_idx;
};

// --- 优化事件 (Optimization Events) ---

struct OptimizationResultEvent {
    uint64_t version;
    uint64_t event_id = 0;
    std::unordered_map<int, Pose3d> submap_poses;
    std::unordered_map<uint64_t, Pose3d> keyframe_poses;
    std::string source_module = "unknown";
    uint32_t transform_applied_flags = 0;
    uint64_t batch_hash = 0;
    PoseFrame pose_frame = PoseFrame::MAP; // 🏛️ [架构加固] 显式标注位姿坐标系

    bool isValid() const {
        if (event_id == 0 || version == 0) return false;
        if (pose_frame == PoseFrame::UNKNOWN) return false;
        if (source_module.empty()) return false;
        for (const auto& [id, pose] : submap_poses) if (!pose.matrix().allFinite()) return false;
        for (const auto& [id, pose] : keyframe_poses) if (!pose.matrix().allFinite()) return false;
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
    CloudXYZIPtr cloud;
    CloudXYZIPtr cloud_ds; // Downsampled
    Pose3d T_odom_b;
    Mat66d covariance;
    PoseFrame pose_frame = PoseFrame::ODOM; // 🏛️ [架构契约] 显式标注 T_odom_b 的坐标系语义
    std::string cloud_frame = "body";       // 🏛️ [架构契约] 显式标注 cloud/cloud_ds 的坐标系语义：body/world
    LivoKeyFrameInfo kf_info;
    
    // GPS 观测 (可选)
    bool has_gps = false;
    GPSMeasurement gps;

    // 🏛️ 生产级确定性：本帧依赖的地图版本
    // MappingModule 必须等待 MapRegistry 达到此版本后才处理本帧，确保坐标系一致
    uint64_t ref_map_version = 0;

    bool isValid() const {
        if (!std::isfinite(timestamp)) return false;
        if (!cloud || cloud->empty()) return false;
        if (!T_odom_b.matrix().allFinite()) return false;
        if (!covariance.allFinite()) return false;
        if (pose_frame == PoseFrame::UNKNOWN) return false;
        if (cloud_frame != "body" && cloud_frame != "world") return false;
        if (has_gps) {
            if (!gps.position_enu.allFinite()) return false;
            if (!gps.covariance.allFinite()) return false;
        }
        return true;
    }
};

// --- 图维护事件 (Graph Maintenance Events) ---

struct GraphTaskEvent {
    OptTaskItem task;
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
};

struct GlobalMapBuildResultEvent {
    CloudXYZIPtr global_map;
};

// --- 系统健康与状态事件 (System Health & Status Events) ---

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

} // namespace automap_pro::v3
