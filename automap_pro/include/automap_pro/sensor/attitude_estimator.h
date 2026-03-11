#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

/**
 * 多源姿态估计器（MVP）
 *
 * - Pitch/Roll：由 IMU 加速度计在静态/准静态下估计（重力方向），低通滤波。
 * - Yaw：优先 GPS 航迹角（运动时位置差分），低速时退化为里程计 yaw。
 * - 兼容 GPS 自带姿态（双天线等）：通过 addGPSAttitude 注入时优先使用。
 */
class AttitudeEstimator {
public:
    struct Config {
        double imu_lowpass_cutoff_hz = 0.5;
        double gravity_norm = 9.81;
        double gravity_threshold = 0.3;       // |acc| 与 g 偏差超过此值认为非静态
        double max_linear_accel = 0.5;       // 最大线加速度假设 m/s²
        double min_velocity_for_yaw = 1.0;   // 计算 GPS 航迹角的最小速度 m/s
        int yaw_smoothing_window = 5;
        double yaw_max_change_rad = 0.5;     // 单步 yaw 最大变化，超过则丢弃
        double pitch_roll_base_var = 0.01;
        double yaw_var_velocity_scale = 0.1; // yaw_var = scale / velocity²
        size_t imu_buffer_max = 2000;
        size_t gps_pos_buffer_max = 500;
        size_t odom_yaw_buffer_max = 500;
        double max_imu_age_s = 10.0;
        double max_gps_age_s = 30.0;
        /** 为 true 时：若已通过 addGPSAttitude 注入接收机姿态（双天线/INS），则优先使用；为 false 时：始终使用估计姿态 */
        bool use_gps_attitude_when_available = false;
    };

    /** 默认配置在 .cpp 中构造，避免类内默认实参与嵌套结构体默认成员初始化器的解析顺序问题 */
    explicit AttitudeEstimator(const Config& cfg = getDefaultConfig());
    ~AttitudeEstimator() = default;

    static Config getDefaultConfig();

    /** 注入 IMU 数据（用于 pitch/roll 与静态检测） */
    void addIMU(double ts, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);

    /** 注入 GPS 位置（ENU），用于航迹角 yaw */
    void addGPSPos(double ts, const Eigen::Vector3d& pos_enu);

    /** 注入里程计 yaw（弧度），用于低速或无 GPS 时 */
    void addOdometryYaw(double ts, double yaw);

    /** 注入 GPS 自带姿态（双天线等），优先使用 */
    void addGPSAttitude(double ts, double roll, double pitch, double yaw, double confidence = 1.0);

    /**
     * 查询给定时间戳的姿态估计。
     * 会插值/最近邻 IMU、用最近 GPS 位置序列算航迹角、或用里程计 yaw。
     */
    AttitudeEstimate estimateAt(double ts) const;

    bool hasValidYaw() const;
    double currentVelocity() const { return state_.velocity_horizontal; }

private:
    struct IMUSample {
        double ts = 0.0;
        Eigen::Vector3d accel_raw = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_filtered = Eigen::Vector3d::Zero();
    };
    struct GPSPosSample {
        double ts = 0.0;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    };
    struct OdomYawSample {
        double ts = 0.0;
        double yaw = 0.0;
    };

    std::pair<double, double> estimatePitchRollFromAccel(const Eigen::Vector3d& accel) const;
    void lowpassAccel(IMUSample& s) const;
    std::pair<double, bool> estimateYawFromGPS(double ts, double& out_velocity) const;
    double getOdomYawAt(double ts) const;
    void pruneBuffers(double now_ts);

    Config cfg_;
    mutable std::mutex mutex_;

    std::deque<IMUSample> imu_buffer_;
    std::deque<GPSPosSample> gps_pos_buffer_;
    std::deque<OdomYawSample> odom_yaw_buffer_;

    // 低通状态（一阶）
    mutable Eigen::Vector3d accel_lowpass_ = Eigen::Vector3d::Zero();
    mutable bool accel_lowpass_initialized_ = false;

    // 最近一次 GPS 姿态（双天线）
    std::optional<std::pair<double, std::pair<Eigen::Vector3d, double>>> last_gps_attitude_;
    static constexpr double kGpsAttitudeTimeoutSec = 2.0;

    // 缓存当前状态（供 hasValidYaw / currentVelocity）
    struct State {
        double pitch = 0.0, roll = 0.0, yaw = 0.0;
        Eigen::Vector3d variance = Eigen::Vector3d::Ones() * 0.1;
        AttitudeSource source = AttitudeSource::NONE;
        bool yaw_valid = false;
        double velocity_horizontal = 0.0;
    };
    mutable State state_;
};

}  // namespace automap_pro
