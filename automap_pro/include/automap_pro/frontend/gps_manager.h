#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/sensor/attitude_estimator.h"
#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace automap_pro {

/**
 * GPS 延迟对齐管理器
 *
 * 设计原则：
 *  - GPS 为可选：有质量好的 GPS 就使用，没有或质量差则不用；建图不依赖 GPS，时有时无均正常。
 *  - "先建图后对齐"：GPS 信号差时先做局部 LiDAR 建图；当连续 N 帧高质量 GPS 时做 SVD 对齐并添加约束。
*  - 质量门控：仅 HIGH/EXCELLENT 用于对齐与因子；轨迹日志中仅 MEDIUM 及以上记为有效。
*
 * 状态机：
*   NOT_ALIGNED → (足够高质量GPS) → ALIGNING → (SVD成功) → ALIGNED
*                         -> (GPS变差) → DEGRADED
*   DEGRADED -> (GPS恢复) -> ALIGNED
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
    void addKeyFramePose(double timestamp, const Pose3d& T_odom_b);

    // ── 状态查询 ──────────────────────────────────────────────────────────

    GPSAlignState state() const;
    bool          isAligned() const { return state_ == GPSAlignState::ALIGNED; }
    const GPSAlignResult& alignResult() const { return align_result_; }

    /** 获取最近的有效 GPS ENU 坐标（如已对齐则已转换到 LiDAR 坐标系） */
    std::optional<Eigen::Vector3d> getLatestPositionENU() const;

    /**
     * 查询给定时间戳的 GPS 测量，用于关键帧约束。
     * - 位置：在夹住 ts 的两条 GPS 之间做线性插值（时间窗内）；否则在 keyframe_match_window_s 内取最近邻。
     * - 姿态：由里程计/前端提供，GPS 仅约束位置；若未来接入双天线朝向，可对四元数做球面插值(slerp)。
     */
    std::optional<GPSMeasurement> queryByTimestamp(double ts) const;

    /**
     * 按位置查找最近 GPS，再用该 GPS 时间戳做插值（用于关键帧–GPS 绑定）。
     * - 在 map 系下找 gps_window_ 中与 position_map 距离最近的 GPS 记录，取其时间戳 t*，
     *   返回 queryByTimestamp(t*)（即在该时间戳处的插值结果）。
     * - 仅当已对齐（isAligned）时使用，保证 keyframe 与 GPS 同在 map 系；未对齐时返回 null，调用方应回退到 queryByTimestampEnhanced(ts)。
     */
    std::optional<GPSMeasurement> queryByNearestPosition(const Eigen::Vector3d& position_map) const;

    /** 轨迹日志用：放宽时间窗口的查询（1Hz GPS + 时间偏差时仍可匹配），max_dt_s 建议 0.5~1.0 */
    std::optional<GPSMeasurement> queryByTimestampForLog(double ts, double max_dt_s = 0.5) const;

    /**
     * 增强版时间戳查询：在 queryByTimestamp 基础上支持边界外推与协方差缩放。
     * - 先尝试线性插值 / 最近邻；若失败且 ts 在窗口边缘 ± extrapolation_margin_s 内，则用速度模型外推。
     * - 外推时协方差乘以 extrapolation_uncertainty_scale。
     */
    std::optional<GPSMeasurement> queryByTimestampEnhanced(double ts) const;

    /** 在给定时间戳附近估计 GPS 速度（用于外推），需窗口内至少 2 个点。内部持锁。 */
    std::optional<Eigen::Vector3d> estimateGpsVelocity(double ts) const;

    /** 假定已持 mutex_，在 ts 附近估计速度（供 queryByTimestampEnhanced 使用）。 */
    std::optional<Eigen::Vector3d> estimateGpsVelocityLocked(double ts) const;

    /** 当前 GPS 质量 */
    GPSQuality currentQuality() const;

    /** 诊断用：当前 GPS 滑动窗口条数（用于排查轨迹 CSV 无 GPS 时是"无数据"还是"时间不匹配"） */
    size_t getGpsWindowSize() const;

    /** 诊断用：当前 GPS 窗口时间范围 [min_ts, max_ts]，便于判断 odom 时间是否落在范围内。返回 true 表示有数据并已写入 out_min_ts、 out_max_ts */
    bool getGpsWindowTimeRange(double* out_min_ts, double* out_max_ts) const;

    /** 诊断用：获取首个 GPS 时间戳（用于计算 GPS 到达延迟） */
    double getFirstGpsTimestamp() const;

    /** 诊断用：获取最后一个 GPS 时间戳 */
    double getLastGpsTimestamp() const;

    /** 
     * @brief 为对齐后的历史关键帧补充 GPS 绑定
     * @param kf_timestamps 关键帧时间戳和子图ID列表 [(timestamp, submap_id), ...]
     * @param max_dt 最大时间差（秒），默认 0.5s
     * @return 成功绑定的 [(submap_id, GPSMeasurement), ...]
     * 
     * 使用场景：GPS 数据延迟到达时，对齐前已创建的关键帧无法绑定 GPS。
     * 对齐成功后调用此方法，为历史关键帧补充 GPS 约束。
     */
    std::vector<std::pair<int, GPSMeasurement>> getHistoricalGPSBindings(
        const std::vector<std::pair<double, int>>& kf_timestamps,
        double max_dt = 0.5) const;

    // ── GPS-LiDAR 外参 ────────────────────────────────────────────────────

    /** 获取旋转矩阵 R_lidar_gps（将GPS ENU坐标变换到LiDAR轨迹坐标系） */
    const Eigen::Matrix3d& R_lidar_gps() const { return align_result_.R_gps_lidar; }
    const Eigen::Vector3d& t_lidar_gps() const { return align_result_.t_gps_lidar; }

    /** 将 GPS ENU 坐标转换为 LiDAR 地图坐标系下的位置 */
    Eigen::Vector3d enu_to_map(const Eigen::Vector3d& enu) const;

    /**
     * 将 GPS ENU 坐标转换为地图系位置，并返回实际坐标系名称（"map" 或 "enu"）。
     * 未对齐时返回 enu 原样且 frame="enu"，便于调用方正确填写 gps_frame 列。
     */
    std::pair<Eigen::Vector3d, std::string> enu_to_map_with_frame(const Eigen::Vector3d& enu) const;

    /** 诊断用：当前连续高质量 GPS 样本数（用于判断是否接近触发对齐） */
    int getGoodSampleCount() const;
    /** 诊断用：当前关键帧窗口内轨迹累积距离（米），用于判断是否满足 min_align_dist_m */
    double getAccumulatedDistM() const;
    /** 诊断用：关键帧匹配 GPS 的最大时间差(s)（来自配置 keyframe_match_window_s） */
    double getKeyframeMatchWindowS() const { return keyframe_match_window_s_; }
    /** 诊断用：对齐所需最小点数、最小累积距离(m)、所需高质量样本数 */
    int getMinAlignPoints() const { return min_align_points_; }
    double getMinAlignDistM() const { return min_align_dist_m_; }
    int getGoodSamplesNeeded() const { return good_samples_needed_; }

    /**
     * 获取当前滑动窗口内所有 GPS 点在地图坐标系下的位置（建图结束时导出 PCD 用）。
     * 仅当已对齐（ALIGNED/DEGRADED）时返回非空；未对齐时返回空向量。
     * 返回 [(timestamp, pos_map), ...]，按时间顺序。
     */
    std::vector<std::pair<double, Eigen::Vector3d>> getGpsPositionsInMapFrame() const;

    /** 窗口快照类型（供 .cpp 实现使用；getSnapshot() 为 private） */
    struct GpsWindowSnapshot {
        struct Record {
            double timestamp = 0.0;
            Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();
            GPSQuality quality = GPSQuality::INVALID;
            double hdop = 0.0;
        };
        std::vector<Record> window;
        bool is_aligned = false;
        Eigen::Matrix3d R_gps_lidar = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_gps_lidar = Eigen::Vector3d::Zero();
        double keyframe_match_window_s = 1.0;
        double max_interp_gap_s = 2.0;
        double extrapolation_margin_s = 0.5;
        double extrapolation_uncertainty_scale = 2.0;
        double velocity_estimation_window_s = 2.0;
        double keyframe_max_hdop = 15.0;
        int good_sample_count = 0;
    };

    // ── 回调注册 ──────────────────────────────────────────────────────────

    /** 对齐成功回调（触发一次 GPS 约束批量添加） */
    using AlignCallback = std::function<void(const GPSAlignResult&)>;
    void registerAlignCallback(AlignCallback cb) { align_cbs_.push_back(std::move(cb)); }

    /** 每条高质量GPS位置（对齐后）可用于实时因子添加 */
    using GpsFactorCallback = std::function<void(double ts, const Eigen::Vector3d& pos_map,
                                                  const Eigen::Matrix3d& cov)>;
    void registerGpsFactorCallback(GpsFactorCallback cb) { gps_factor_cbs_.push_back(std::move(cb)); }

    /** 每条 GPS 测量写入轨迹对比文件（ts, pos_enu），由 AutoMapSystem 转 map 系后落盘 */
    using MeasurementLogCallback = std::function<void(double ts, const Eigen::Vector3d& pos_enu)>;
    void registerMeasurementLogCallback(MeasurementLogCallback cb) { measurement_log_cbs_.push_back(std::move(cb)); }

    /** 手动请求执行一次 GPS 对齐（通常在满足最小子图数后调用） */
    void requestAlignment() { try_align(); }

    /** 重置对齐矩阵为单位阵（用于 AutoMapSystem 完成全局变换后，同步坐标系） */
    void resetAlignmentToIdentity();

    // ── 强制重新对齐（用于 GPS 信号恢复后） ──────────────────────────────
    void triggerRealign();

    /**
     * 设置对齐调度器（可选）。若设置，达到对齐条件时不在此线程执行 try_align()，而是调用调度器；
     * 由调用方在后台线程中执行 runScheduledAlignment()，避免阻塞 ROS 回调。
     * 不设置则保持同步行为（当前逻辑）。
     */
    using AlignScheduler = std::function<void()>;
    void setAlignScheduler(AlignScheduler f) { align_scheduler_ = std::move(f); }

    /**
     * 执行已调度的对齐。由后台线程（如 backend worker）在适当时机调用；
     * 若此前 addGPSMeasurement 已触发调度（pending_align_），则执行 try_align() 并清除标志。
     */
    void runScheduledAlignment();

    /** 设置姿态估计器（可选）。设置后 queryByTimestamp/queryByTimestampForLog 会填充 attitude/velocity */
    void setAttitudeEstimator(std::shared_ptr<AttitudeEstimator> estimator) { attitude_estimator_ = std::move(estimator); }

    /**
     * 从 ConfigManager 重新加载 GPS 相关参数并写回成员。
     * 用于修复「GPSManager 在 config 加载前构造导致 YAML 配置未生效」的问题；
     * 应在 config 已加载后调用（如 AutoMapSystem::deferredSetupModules 中、注册 GPS 回调前）。
     */
    void applyConfig();

private:
    /** 地图系 → ENU（保证 query 返回的 position_enu 恒为 ENU，如外推路径） */
    Eigen::Vector3d map_to_enu(const Eigen::Vector3d& map_pos) const;

    // ENU 原点（第一个有效GPS点确定，call_once 避免多线程竞态）
    double enu_origin_lat_ = 0.0, enu_origin_lon_ = 0.0, enu_origin_alt_ = 0.0;
    std::atomic<bool> enu_origin_set_{false};
    std::once_flag    enu_origin_once_;

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
    int    min_align_points_        = 50;
    double min_align_dist_m_        = 30.0;
    double keyframe_match_window_s_ = 1.0;   // 关键帧匹配 GPS 的最大时间差(s)，1Hz GPS 建议 0.5~1.0
    double keyframe_max_hdop_       = 15.0;  // 关键帧绑定 GPS 的 HDOP 上界，放宽弱 GPS 场景（M2DGR 等）
    double max_interp_gap_s_        = 2.0;   // 双样本线性插值最大间隔(s)，超过则退化为最近邻
    double extrapolation_margin_s_  = 0.5;   // 外推边界(s)，窗口边缘 ± margin 内启用速度外推
    double extrapolation_uncertainty_scale_ = 2.0;  // 外推协方差放大倍数
    double velocity_estimation_window_s_   = 2.0;  // 速度估计窗口(s)
    double quality_hdop_thresh_ = 12.0;  // 【修复】放宽：2.0→12.0 适配弱GPS
    double rmse_accept_thresh_  = 5.0;   // 【修复】放宽：1.5→5.0 弱GPS噪声大
    int    good_samples_needed_ = 20;    // 【修复】降低：30→20 弱GPS高质量帧少
    int    good_sample_count_   = 0;

    // 状态
    std::atomic<GPSAlignState> state_{GPSAlignState::NOT_ALIGNED};
    GPSAlignResult align_result_;

    // 对齐后的GPS缓存（供因子图使用）
    struct AlignedGPS { double ts; Eigen::Vector3d pos_map; Eigen::Matrix3d cov; };
    std::vector<AlignedGPS> aligned_gps_buffer_;
    double last_gps_factor_dist_ = -1e9;  // 上次添加GPS因子时的距离

    mutable std::recursive_mutex mutex_;

    std::vector<AlignCallback>         align_cbs_;
    std::vector<GpsFactorCallback>     gps_factor_cbs_;
    std::vector<MeasurementLogCallback> measurement_log_cbs_;

    /** 异步对齐：达到条件时调用此调度器，由后台线程执行 runScheduledAlignment() */
    AlignScheduler align_scheduler_;
    std::atomic<bool> pending_align_{false};

    std::shared_ptr<AttitudeEstimator> attitude_estimator_;

    /** 持 mutex_ 仅做最小拷贝后返回，供只读 query 在锁外使用 */
    GpsWindowSnapshot getSnapshot() const;

    // ── 私有方法 ──────────────────────────────────────────────────────────
    Eigen::Vector3d wgs84_to_enu(double lat, double lon, double alt) const;
    GPSAlignResult  compute_svd_alignment();
    void            try_align();
    void            on_aligned(const GPSAlignResult& result);
    GPSQuality      hdop_to_quality(double hdop) const;

    // ── 精准对齐增强方法 ───────────────────────────────────────────────────────

    /**
     * @brief 使用里程计积分估计GPS初始位置
     * 当GPS数据延迟到达时，使用里程计积分来估计GPS在里程计起始时刻的位置
     * 这样可以消除GPS延迟导致的系统初始偏差
     */
    Eigen::Vector3d estimateGpsPositionByOdom(double gps_ts, double odom_start_ts,
                                               const Pose3d& odom_start_pose);

    /**
     * @brief 在线校准：持续校准GPS-里程计偏差
     * 用于处理GPS延迟、时钟漂移等问题
     */
    void onlineCalibrate(double gps_ts, const Eigen::Vector3d& gps_enu,
                        double odom_ts, const Pose3d& odom_pose);

    /**
     * @brief 查找最近邻里程计位姿
     */
    std::optional<std::pair<double, Pose3d>> findNearestOdomPose(double ts) const;

    // ── 精准对齐相关成员 ────────────────────────────────────────────────────
    std::deque<std::pair<double, Pose3d>> all_odom_poses_;  // 缓存所有里程计位姿用于插值
    Eigen::Vector3d accumulated_offset_{Eigen::Vector3d::Zero()};  // 累积的GPS-里程计偏差
    std::atomic<int> calib_count_{0};  // 在线校准次数
    double online_calib_min_dist_    = 5.0;   // 最小累积距离触发在线校正（米）
    double online_calib_max_rmse_    = 2.0;   // 在线校正RMSE阈值（米）
    int    online_calib_min_samples_ = 5;     // 在线校正最小样本数

    // GPS 杆臂补偿（V3 修复）：GPS 天线相对于 IMU 原点的偏移（IMU 系）
    Eigen::Vector3d lever_arm_imu_ = Eigen::Vector3d::Zero();
};

} // namespace automap_pro
