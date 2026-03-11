#include "automap_pro/sensor/attitude_estimator.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace automap_pro {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// 一阶低通系数：alpha = dt / (dt + 1/(2*pi*fc))，离散近似
double lowpassAlpha(double dt, double cutoff_hz) {
    if (cutoff_hz <= 0.0 || !std::isfinite(dt) || dt <= 0.0) return 1.0;
    double tau = 1.0 / (2.0 * kPi * cutoff_hz);
    return dt / (dt + tau);
}

// 角度归一化到 [-pi, pi]
double normalizeAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

// 最短角度差
double angleDiff(double a, double b) {
    return normalizeAngle(a - b);
}
}  // namespace

AttitudeEstimator::AttitudeEstimator(const Config& cfg) : cfg_(cfg) {}

AttitudeEstimator::Config AttitudeEstimator::getDefaultConfig() {
    return Config{};
}

void AttitudeEstimator::addIMU(double ts, const Eigen::Vector3d& accel, const Eigen::Vector3d& /*gyro*/) {
    if (!accel.allFinite() || !std::isfinite(ts)) return;
    std::lock_guard<std::mutex> lk(mutex_);
    IMUSample s;
    s.ts = ts;
    s.accel_raw = accel;
    s.accel_filtered = accel;
    if (!imu_buffer_.empty()) {
        double dt = ts - imu_buffer_.back().ts;
        if (dt > 0.0 && dt < 1.0) {
            double alpha = lowpassAlpha(dt, cfg_.imu_lowpass_cutoff_hz);
            if (!accel_lowpass_initialized_) {
                accel_lowpass_ = imu_buffer_.back().accel_filtered;
                accel_lowpass_initialized_ = true;
            }
            accel_lowpass_ = (1.0 - alpha) * accel_lowpass_ + alpha * accel;
            s.accel_filtered = accel_lowpass_;
        }
    }
    imu_buffer_.push_back(s);
    while (imu_buffer_.size() > cfg_.imu_buffer_max) imu_buffer_.pop_front();
    pruneBuffers(ts);
}

void AttitudeEstimator::addGPSPos(double ts, const Eigen::Vector3d& pos_enu) {
    if (!pos_enu.allFinite() || !std::isfinite(ts)) return;
    std::lock_guard<std::mutex> lk(mutex_);
    gps_pos_buffer_.push_back({ts, pos_enu});
    while (gps_pos_buffer_.size() > cfg_.gps_pos_buffer_max) gps_pos_buffer_.pop_front();
    pruneBuffers(ts);
}

void AttitudeEstimator::addOdometryYaw(double ts, double yaw) {
    if (!std::isfinite(ts) || !std::isfinite(yaw)) return;
    std::lock_guard<std::mutex> lk(mutex_);
    odom_yaw_buffer_.push_back({ts, normalizeAngle(yaw)});
    while (odom_yaw_buffer_.size() > cfg_.odom_yaw_buffer_max) odom_yaw_buffer_.pop_front();
}

void AttitudeEstimator::addGPSAttitude(double ts, double roll, double pitch, double yaw, double confidence) {
    if (!std::isfinite(ts)) return;
    std::lock_guard<std::mutex> lk(mutex_);
    last_gps_attitude_ = {{ts, {{roll, pitch, yaw}, confidence}}};
}

std::pair<double, double> AttitudeEstimator::estimatePitchRollFromAccel(const Eigen::Vector3d& accel) const {
    double g = cfg_.gravity_norm;
    double n = accel.norm();
    if (n < 1e-6) return {0.0, 0.0};
    double ax = accel.x(), ay = accel.y(), az = accel.z();
    // pitch = atan2(-ax, sqrt(ay^2+az^2))
    double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    // roll = atan2(ay, az)
    double roll = std::atan2(ay, az);
    return {pitch, roll};
}

void AttitudeEstimator::lowpassAccel(IMUSample& s) const {
    (void)s;
    // 已在 addIMU 中更新
}

void AttitudeEstimator::pruneBuffers(double now_ts) {
    while (!imu_buffer_.empty() && (now_ts - imu_buffer_.front().ts) > cfg_.max_imu_age_s) {
        imu_buffer_.pop_front();
    }
    while (!gps_pos_buffer_.empty() && (now_ts - gps_pos_buffer_.front().ts) > cfg_.max_gps_age_s) {
        gps_pos_buffer_.pop_front();
    }
}

std::pair<double, bool> AttitudeEstimator::estimateYawFromGPS(double ts, double& out_velocity) const {
    out_velocity = 0.0;
    if (gps_pos_buffer_.size() < 2) return {0.0, false};

    // 找 ts 前后最近的两点做差分，或 ts 之前最近的两点
    const GPSPosSample* p0 = nullptr;
    const GPSPosSample* p1 = nullptr;
    double best_dt = 1e9;
    for (size_t i = 0; i + 1 < gps_pos_buffer_.size(); ++i) {
        double t_mid = 0.5 * (gps_pos_buffer_[i].ts + gps_pos_buffer_[i + 1].ts);
        double dt = std::abs(t_mid - ts);
        if (dt < best_dt && gps_pos_buffer_[i + 1].ts > gps_pos_buffer_[i].ts) {
            best_dt = dt;
            p0 = &gps_pos_buffer_[i];
            p1 = &gps_pos_buffer_[i + 1];
        }
    }
    if (!p0 || !p1) return {0.0, false};

    double dt = p1->ts - p0->ts;
    if (dt <= 0.0) return {0.0, false};
    double dx = p1->pos.x() - p0->pos.x();
    double dy = p1->pos.y() - p0->pos.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    out_velocity = dist / dt;
    if (out_velocity < cfg_.min_velocity_for_yaw) return {0.0, false};

    double yaw = std::atan2(dy, dx);
    return {yaw, true};
}

double AttitudeEstimator::getOdomYawAt(double ts) const {
    if (odom_yaw_buffer_.empty()) return 0.0;
    double best_dt = 1e9;
    double yaw = 0.0;
    for (const auto& s : odom_yaw_buffer_) {
        double dt = std::abs(s.ts - ts);
        if (dt < best_dt) {
            best_dt = dt;
            yaw = s.yaw;
        }
    }
    return best_dt <= 1.0 ? yaw : 0.0;
}

AttitudeEstimate AttitudeEstimator::estimateAt(double ts) const {
    std::lock_guard<std::mutex> lk(mutex_);
    AttitudeEstimate out;
    out.is_valid = false;

    // 1) GPS 自带姿态（双天线/INS）：仅当配置 gps.has_attitude=true 且已注入时使用
    if (cfg_.use_gps_attitude_when_available && last_gps_attitude_) {
        double age = ts - last_gps_attitude_->first;
        if (age >= 0.0 && age <= kGpsAttitudeTimeoutSec) {
            const auto& rpy = last_gps_attitude_->second.first;
            out.roll = rpy(0);
            out.pitch = rpy(1);
            out.yaw = rpy(2);
            out.source = AttitudeSource::GPS_DUAL_ANTENNA;
            double c = last_gps_attitude_->second.second;
            out.variance = Eigen::Vector3d(0.01, 0.01, 0.01) / std::max(0.1, c);
            out.is_valid = true;
            state_ = State{out.pitch, out.roll, out.yaw, out.variance, out.source, true, 0.0};
            return out;
        }
    }

    // 2) 最近邻 IMU 用于 pitch/roll
    const IMUSample* imu_best = nullptr;
    double best_dt_imu = 1e9;
    for (const auto& s : imu_buffer_) {
        double dt = std::abs(s.ts - ts);
        if (dt < best_dt_imu) {
            best_dt_imu = dt;
            imu_best = &s;
        }
    }
    if (imu_best && best_dt_imu <= 0.5) {
        double g_err = std::abs(imu_best->accel_filtered.norm() - cfg_.gravity_norm);
        bool quasi_static = g_err <= cfg_.gravity_threshold;
        auto pr = estimatePitchRollFromAccel(imu_best->accel_filtered);
        out.pitch = pr.first;
        out.roll = pr.second;
        out.variance(0) = out.variance(1) = cfg_.pitch_roll_base_var + (quasi_static ? 0.0 : 0.05);
    } else {
        out.pitch = state_.pitch;
        out.roll = state_.roll;
        out.variance(0) = out.variance(1) = 0.1;
    }

    // 3) Yaw: GPS 航迹角 或 里程计
    double vel_h = 0.0;
    auto [yaw_gps, yaw_gps_ok] = estimateYawFromGPS(ts, vel_h);
    out.velocity_horizontal = vel_h;

    if (yaw_gps_ok) {
        double prev_yaw = state_.yaw;
        double out_yaw;
        if (!state_.yaw_valid) {
            out_yaw = yaw_gps;
        } else {
            double dyaw = angleDiff(yaw_gps, prev_yaw);
            if (std::abs(dyaw) > cfg_.yaw_max_change_rad) {
                dyaw = (dyaw > 0 ? 1.0 : -1.0) * cfg_.yaw_max_change_rad;
            }
            out_yaw = normalizeAngle(prev_yaw + dyaw);
        }
        out.yaw = out_yaw;
        out.source = AttitudeSource::FUSED;
        out.variance(2) = std::max(1e-4, cfg_.yaw_var_velocity_scale / std::max(vel_h * vel_h, 0.01));
        out.is_valid = true;
        state_.yaw_valid = true;
    } else {
        double yaw_odom = getOdomYawAt(ts);
        out.yaw = yaw_odom;
        out.source = AttitudeSource::FUSED;
        out.variance(2) = 0.1;
        out.is_valid = (imu_best != nullptr && best_dt_imu <= 0.5);
        state_.yaw_valid = false;
    }

    state_.pitch = out.pitch;
    state_.roll = out.roll;
    state_.yaw = out.yaw;
    state_.variance = out.variance;
    state_.source = out.source;
    state_.velocity_horizontal = out.velocity_horizontal;
    return out;
}

bool AttitudeEstimator::hasValidYaw() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_.yaw_valid;
}

}  // namespace automap_pro
