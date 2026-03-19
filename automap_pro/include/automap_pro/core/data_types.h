#pragma once

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <functional>
#include <optional>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

constexpr double kMaxReasonableTranslationNorm = 1e6;
constexpr int MAX_KF_PER_SUBMAP = 100000;

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 基础类型别名
// ─────────────────────────────────────────────────────────────────────────────
using PointType    = pcl::PointXYZINormal;
using CloudXYZI    = pcl::PointCloud<pcl::PointXYZI>;
using CloudXYZIN   = pcl::PointCloud<PointType>;
using CloudXYZIPtr = CloudXYZI::Ptr;
using CloudXYZINPtr = CloudXYZIN::Ptr;
using Pose3d       = Eigen::Isometry3d;
using Mat66d       = Eigen::Matrix<double, 6, 6>;
using Vec3d        = Eigen::Vector3d;
using Vec6d        = Eigen::Matrix<double, 6, 1>;
using Quat         = Eigen::Quaterniond;

// ─────────────────────────────────────────────────────────────────────────────
// GPS 质量枚举
// ─────────────────────────────────────────────────────────────────────────────
enum class GPSQuality : int {
    INVALID   = 0,
    LOW       = 1,   // HDOP > 5.0
    MEDIUM    = 2,   // 2.0 < HDOP ≤ 5.0
    HIGH      = 3,   // 1.0 < HDOP ≤ 2.0
    EXCELLENT = 4    // HDOP ≤ 1.0
};

// GPS 对齐状态机
enum class GPSAlignState : int {
    WAITING_GPS = -1,    // 等待首帧GPS数据（GPS延迟到达场景）
    NOT_ALIGNED = 0,     // 尚未对齐（GPS信号不足）
    ALIGNING    = 1,     // 正在计算对齐（SVD求解中）
    ALIGNED     = 2,     // 已对齐，可添加GPS约束
    DEGRADED    = 3,     // 对齐后GPS信号再次变差，暂停约束
};

// ─────────────────────────────────────────────────────────────────────────────
// IMU 数据（与 ImuProcessor/TimeSync 共用）
// ─────────────────────────────────────────────────────────────────────────────
struct ImuData {
    double timestamp = 0.0;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

// ─────────────────────────────────────────────────────────────────────────────
// 姿态估计来源（用于日志与因子权重）
// ─────────────────────────────────────────────────────────────────────────────
enum class AttitudeSource : int {
    NONE = 0,              // 无有效姿态
    IMU_ONLY = 1,          // 仅 IMU（pitch/roll）
    GPS_TRAJECTORY = 2,    // GPS 航迹角 yaw
    GPS_DUAL_ANTENNA = 3,  // GPS 双天线完整姿态
    ODOMETRY = 4,          // 里程计 yaw
    FUSED = 5              // 融合估计（IMU + GPS yaw / odom yaw）
};

// ─────────────────────────────────────────────────────────────────────────────
// 姿态估计结果（弧度，ENU）
// ─────────────────────────────────────────────────────────────────────────────
struct AttitudeEstimate {
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
    Eigen::Vector3d variance = Eigen::Vector3d::Ones() * 0.1;  // [pitch, roll, yaw] 方差 rad²
    AttitudeSource source = AttitudeSource::NONE;
    bool is_valid = false;
    double velocity_horizontal = 0.0;  // 水平速度 m/s，用于 yaw 质量评估
};

// ─────────────────────────────────────────────────────────────────────────────
// GPS 测量值
// ─────────────────────────────────────────────────────────────────────────────
struct GPSMeasurement {
    double timestamp          = 0.0;
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero();  // ENU坐标（米）
    GPSQuality quality        = GPSQuality::INVALID;
    double hdop               = 99.0;
    int num_satellites        = 0;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e6;
    bool is_valid             = false;
    bool is_outlier            = false;  // 跳变/一致性检测异常
    // 原始GNSS
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;

    // 姿态估计（无 GPS 姿态时由 IMU + 航迹角/里程计 估计）
    AttitudeEstimate attitude;
    // 速度估计（由位置差分，用于航迹角与质量评估）
    Eigen::Vector3d velocity_enu = Eigen::Vector3d::Zero();
    double speed_horizontal = 0.0;
    bool has_velocity = false;

    using Ptr = std::shared_ptr<GPSMeasurement>;
};

// ─────────────────────────────────────────────────────────────────────────────
// FAST-LIVO2 扩展关键帧信息（Composable Node 额外发布）
// ─────────────────────────────────────────────────────────────────────────────
struct LivoKeyFrameInfo {
    double timestamp            = 0.0;
    double esikf_cov_norm       = 1.0;   // ESIKF协方差矩阵Frobenius范数（质量指标）
    bool   is_degenerate        = false; // ESIKF退化标志（平坦场景/走廊）
    Eigen::Vector3d gyro_bias   = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias  = Eigen::Vector3d::Zero();
    float  map_update_rate      = 0.0f;
    uint32_t cloud_point_count  = 0;
    bool   cloud_valid          = false; // 点云是否有效（与 KeyFrameInfoMsg 一致）
    using Ptr = std::shared_ptr<LivoKeyFrameInfo>;
};

// ─────────────────────────────────────────────────────────────────────────────
// 关键帧
// ─────────────────────────────────────────────────────────────────────────────
struct KeyFrame {
    uint64_t id         = 0;
    uint64_t session_id = 0;
    int      submap_id  = -1;
    int      index_in_submap = -1; // ✅ V2 修复：存储在子图中的索引，用于 iSAM2 节点编码
    double   timestamp  = 0.0;

    // 位姿（世界系下 body 位姿）
    Pose3d T_w_b            = Pose3d::Identity();
    Pose3d T_w_b_optimized  = Pose3d::Identity(); // 优化后位姿
    Mat66d covariance       = Mat66d::Identity() * 1e-4;

    // 点云（body系下）
    CloudXYZIPtr cloud_body;      // 原始体素化点云（body frame）
    CloudXYZIPtr cloud_ds_body;   // 下采样点云（用于匹配）

    // GPS
    GPSMeasurement gps;
    bool has_valid_gps = false;

    // ESIKF质量信息
    LivoKeyFrameInfo livo_info;

    // 图像（可选）
    cv::Mat image;

    // 标记
    bool is_anchor = false;  // 是否为子图锚定帧

    using Ptr = std::shared_ptr<KeyFrame>;
};

// ─────────────────────────────────────────────────────────────────────────────
// 回环约束
// ─────────────────────────────────────────────────────────────────────────────
enum class LoopStatus { PENDING, ACCEPTED, REJECTED };

struct LoopConstraint {
    int      submap_i = -1, submap_j = -1;   // i:较旧(目标), j:较新(查询)
    int      keyframe_i = -1, keyframe_j = -1;
    /** 关键帧全局 id（图中 KF 节点 id），用于子图间 keyframe 级回环入图；-1 表示未设置 */
    int      keyframe_global_id_i = -1, keyframe_global_id_j = -1;
    uint64_t session_i = 0,  session_j = 0;

    /** T_i_j：子图 j 在子图 i 系下的位姿（target=i, source=j），用于 BetweenFactor(i, j, delta_T) */
    Pose3d   delta_T = Pose3d::Identity();
    Mat66d   information = Mat66d::Identity();

    float    overlap_score = 0.0f;
    float    inlier_ratio  = 0.0f;
    float    rmse          = 1e6f;
    bool     is_inter_session = false;
    LoopStatus status = LoopStatus::PENDING;

    using Ptr = std::shared_ptr<LoopConstraint>;
};

// ─────────────────────────────────────────────────────────────────────────────
// 子图状态机
// ─────────────────────────────────────────────────────────────────────────────
enum class SubMapState {
    ACTIVE,   // 正在接收关键帧
    FROZEN,   // 不再接收新帧，等待优化
    OPTIMIZED,// 已被位姿图优化更新
    ARCHIVED  // 已持久化到磁盘
};

// ─────────────────────────────────────────────────────────────────────────────
// 子图
// ─────────────────────────────────────────────────────────────────────────────
struct SubMap {
    int      id          = -1;
    uint64_t session_id  = 0;
    SubMapState state    = SubMapState::ACTIVE;

    std::vector<KeyFrame::Ptr> keyframes;

    // ========== 子图内回环检测：关键帧描述子数据库 ==========
    // 每个关键帧的 256 维 OverlapTransformer 描述子（仅当未启用 ScanContext 时使用）
    std::vector<Eigen::VectorXf> keyframe_descriptors;
    // 每个关键帧的 ScanContext 描述子（当 loop_closure.scancontext.enabled 时用于子图内回环，替代 fallback）
    std::vector<Eigen::MatrixXd> keyframe_scancontexts_;
    // 关键帧下采样点云（用于子图内配准）
    std::vector<CloudXYZIPtr> keyframe_clouds_ds;

    // 锚定位姿（第一帧的世界系位姿）
    Pose3d pose_w_anchor           = Pose3d::Identity();
    Pose3d pose_w_anchor_optimized = Pose3d::Identity();

    // 点云
    CloudXYZIPtr merged_cloud;       // 合并点云（世界系）
    CloudXYZIPtr downsampled_cloud;  // 降采样点云（用于描述子/匹配）

    // OverlapTransformer 256维描述子
    Eigen::VectorXf overlap_descriptor = Eigen::VectorXf::Zero(256);
    bool has_descriptor = false;
    float overlap_descriptor_norm = 1.0f;  // ✅ 缓存 L2 norm，避免重复计算

    // GPS 中心（ENU，用于GPS范围过滤）
    Eigen::Vector3d gps_center = Eigen::Vector3d::Zero();
    Pose3d gps_enu_pose = Pose3d::Identity();
    Eigen::Matrix3d gps_cov = Eigen::Matrix3d::Identity() * 1e6;
    bool has_valid_gps = false;

    // 时间范围
    double t_start = 0.0, t_end = 0.0;

    // 统计
    double spatial_extent_m = 0.0;

    using Ptr = std::shared_ptr<SubMap>;
};

// ─────────────────────────────────────────────────────────────────────────────
// 会话信息（MS-Mapping 多会话支持）
// ─────────────────────────────────────────────────────────────────────────────
struct SessionInfo {
    uint64_t    id = 0;
    std::string data_dir;
    double      t_start = 0.0, t_end = 0.0;
    int         submap_count = 0;
    int         keyframe_count = 0;
    bool        has_descriptor_db = false;
    Pose3d      gps_origin_pose = Pose3d::Identity();  // GPS对齐后的原点位姿
    using Ptr = std::shared_ptr<SessionInfo>;
};

// ─────────────────────────────────────────────────────────────────────────────
// 优化结果
// ─────────────────────────────────────────────────────────────────────────────
struct OptimizationResult {
    bool    success = false;
    int     nodes_updated = 0;
    double  elapsed_ms = 0.0;
    double  final_rmse = 0.0;
    std::unordered_map<int, Pose3d> submap_poses;
    std::unordered_map<uint64_t, Pose3d> keyframe_poses;
};

// ─────────────────────────────────────────────────────────────────────────────
// HBA 结果
// ─────────────────────────────────────────────────────────────────────────────
struct HBAResult {
    bool    success = false;
    double  final_mme = 0.0;
    int     iterations_per_layer = 0;
    double  elapsed_ms = 0.0;
    std::vector<Pose3d> optimized_poses; // 与输入keyframe顺序一致
};

// ─────────────────────────────────────────────────────────────────────────────
// GPS 对齐结果（SVD 轨迹匹配）
// ─────────────────────────────────────────────────────────────────────────────
struct GPSAlignResult {
    bool   success = false;
    Eigen::Matrix3d R_gps_lidar = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_gps_lidar = Eigen::Vector3d::Zero();
    // ENU到Map坐标系的变换（SVD对齐成功后设置）
    Eigen::Matrix3d R_enu_to_map = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_enu_to_map = Eigen::Vector3d::Zero();
    double rmse_m   = 1e6;
    int    matched_points = 0;
    size_t used_measurements = 0;  // 参与对齐的GPS测量数量
    std::string message;  // 对齐结果消息
};

// ─────────────────────────────────────────────────────────────────────────────
// 系统状态
// ─────────────────────────────────────────────────────────────────────────────
enum class SystemState {
    IDLE, INITIALIZING, MAPPING, LOOP_CLOSING, OPTIMIZING, SAVING, ERROR
};

// ─────────────────────────────────────────────────────────────────────────────
// 回调类型定义
// ─────────────────────────────────────────────────────────────────────────────
using KeyFrameCallback    = std::function<void(const KeyFrame::Ptr&)>;
using SubMapFrozenCallback = std::function<void(const SubMap::Ptr&)>;
using LoopConstraintCallback = std::function<void(const LoopConstraint::Ptr&)>;
using PoseUpdateCallback  = std::function<void(int submap_id, const Pose3d&)>;
using HBADoneCallback     = std::function<void(const HBAResult&)>;
using GPSAlignCallback    = std::function<void(const GPSAlignResult&)>;

} // namespace automap_pro
