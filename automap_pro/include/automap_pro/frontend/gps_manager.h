#pragma once

#include "automap_pro/core/data_types.h"
#include <deque>
#include <mutex>
#include <vector>
#include <functional>

namespace automap_pro {

/**
 * GPS 延迟对齐管理器
 *
 * 设计原则（来自 MapSystem 架构文档）：
 *   "先建图后对齐" —— GPS信号差时先做局部LiDAR建图，
 *    当连续 N 帧 GPS 信号质量超过阈值时，用 SVD 轨迹匹配
 *    计算 GPS-LiDAR 外参，再将 GPS 约束添加到 GTSAM 因子图。
 *
 * 状态机：
 *   NOT_ALIGNED → (收到足够高质量GPS) → ALIGNING → (SVD成功) → ALIGNED
 *                                                             → (GPS变差) → DEGRADED
 *   DEGRADED → (GPS恢复) → ALIGNED
 */
class GPSManager {
public:
    explicit GPSManager();

    // ── 输入接口 ──────────────────────────────────────────────────────────

    /** 注入 NavSatFix 数据，计算 ENU 坐标并评估质量 */
    void addGPSMeasurement(
        double timestamp,
        double latitude, double longitude, double altitude,
        double hdop, int num_sats);

    /** 注入关键帧位姿（LiDAR轨迹），用于 SVD 匹配 */
    void addKeyFramePose(double timestamp, const Pose3d& T_w_b);

    // ── 状态查询 ──────────────────────────────────────────────────────────

    GPSAlignState state()    const;
    bool          isAligned()const { return state_ == GPSAlignState::ALIGNED; }
    const GPSAlignResult& alignResult() const { return align_result_; }

    /** 获取最近的有效 GPS ENU 坐标（如已对齐，则已转换到 LiDAR 坐标系） */
    std::optional<Eigen::Vector3d> getLatestPositionENU() const;

    /** 查询给定时间戳的GPS测量（最近邻插值，时间差<0.1s） */
    std::optional<GPSMeasurement> queryByTimestamp(double ts) const;

    /** 当前 GPS 质量 */
    GPSQuality currentQuality() const;

    // ── GPS-LiDAR 外参 ────────────────────────────────────────────────────

    /** 获取旋转矩阵 R_lidar_gps（将GPS ENU坐标变换到LiDAR轨迹坐标系） */
    const Eigen::Matrix3d& R_lidar_gps() const { return align_result_.R_gps_lidar; }
    const Eigen::Vector3d& t_lidar_gps() const { return align_result_.t_gps_lidar; }

    /** 将 GPS ENU 坐标转换为 LiDAR 地图坐标系下的位置 */
    Eigen::Vector3d enu_to_map(const Eigen::Vector3d& enu) const;

    // ── 回调注册 ──────────────────────────────────────────────────────────

    /** 对齐成功回调（触发一次 GPS 约束批量添加） */
    using AlignCallback = std::function<void(const GPSAlignResult&)>;
    void registerAlignCallback(AlignCallback cb) { align_cbs_.push_back(std::move(cb)); }

    /** 每条高质量GPS位置（对齐后）可用于实时因子添加 */
    using GpsFactorCallback = std::function<void(double ts, const Eigen::Vector3d& pos_map,
                                                  const Eigen::Matrix3d& cov)>;
    void registerGpsFactorCallback(GpsFactorCallback cb) { gps_factor_cbs_.push_back(std::move(cb)); }

    // ── 强制重新对齐（用于 GPS 信号恢复后） ──────────────────────────────
    void triggerRealign();

private:
    // ENU 原点（第一个有效GPS点确定）
    double enu_origin_lat_ = 0.0, enu_origin_lon_ = 0.0, enu_origin_alt_ = 0.0;
    bool   enu_origin_set_ = false;

    // 滑动窗口缓存
    struct GPSRecord {
        double timestamp;
        Eigen::Vector3d pos_enu;
        GPSQuality quality;
        double hdop;
    };
    std::deque<GPSRecord>          gps_window_;   // 最近 N 个GPS
    std::deque<std::pair<double, Pose3d>> kf_window_;  // 最近 N 个KF位姿

    // 参数
    int    min_align_points_    = 50;
    double min_align_dist_m_    = 30.0;
    double quality_hdop_thresh_ = 2.0;
    double rmse_accept_thresh_  = 1.5;
    int    good_samples_needed_ = 30;
    int    good_sample_count_   = 0;

    // 状态
    std::atomic<GPSAlignState> state_{GPSAlignState::NOT_ALIGNED};
    GPSAlignResult align_result_;

    // 对齐后的GPS缓存（供因子图使用）
    struct AlignedGPS { double ts; Eigen::Vector3d pos_map; Eigen::Matrix3d cov; };
    std::vector<AlignedGPS> aligned_gps_buffer_;
    double last_gps_factor_dist_ = -1e9;  // 上次添加GPS因子时的距离

    mutable std::mutex mutex_;

    std::vector<AlignCallback>      align_cbs_;
    std::vector<GpsFactorCallback>  gps_factor_cbs_;

    // ── 私有方法 ──────────────────────────────────────────────────────────
    Eigen::Vector3d wgs84_to_enu(double lat, double lon, double alt) const;
    GPSAlignResult  compute_svd_alignment();
    void            try_align();
    void            on_aligned(const GPSAlignResult& result);
    GPSQuality      hdop_to_quality(double hdop) const;
};

} // namespace automap_pro
