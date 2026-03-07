#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/map_frame_config.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/SVD>
#include <numeric>
#include <cmath>

#define MOD "GPSManager"

namespace automap_pro {

GPSManager::GPSManager() {
    const auto& cfg = ConfigManager::instance();
    min_align_points_    = cfg.gpsAlignMinPoints();
    min_align_dist_m_    = cfg.gpsAlignMinDist();
    quality_hdop_thresh_ = cfg.gpsQualityThreshold();
    rmse_accept_thresh_  = cfg.gpsAlignRmseThresh();
    good_samples_needed_ = cfg.gpsGoodSamplesNeeded();
}

void GPSManager::addGPSMeasurement(
    double timestamp,
    double latitude, double longitude, double altitude,
    double hdop, int num_sats)
{
    // ENU 原点仅设置一次，避免多线程竞态；并写入 map_frame.cfg 供后端统一使用
    std::call_once(enu_origin_once_, [this, latitude, longitude, altitude]() {
        enu_origin_lat_ = latitude;
        enu_origin_lon_ = longitude;
        enu_origin_alt_ = altitude;
        enu_origin_set_.store(true);
        ALOG_INFO(MOD, "ENU origin set: lat={:.6f} lon={:.6f} alt={:.2f}",
                  latitude, longitude, altitude);
        std::string cfg_path = ConfigManager::instance().mapFrameConfigPath();
        MapFrameConfig::write(cfg_path, latitude, longitude, altitude);
    });

    std::lock_guard<std::mutex> lk(mutex_);

    Eigen::Vector3d pos_enu = wgs84_to_enu(latitude, longitude, altitude);
    GPSQuality quality = hdop_to_quality(hdop);

    // 填入滑动窗口
    GPSRecord rec;
    rec.timestamp = timestamp;
    rec.pos_enu   = pos_enu;
    rec.quality   = quality;
    rec.hdop      = hdop;
    gps_window_.push_back(rec);
    if (gps_window_.size() > 2000) gps_window_.pop_front();

    // 统计连续高质量帧
    if (quality >= GPSQuality::HIGH) {
        good_sample_count_++;
        ALOG_DEBUG(MOD, "GPS good sample #{} hdop={:.2f} enu=({:.2f},{:.2f},{:.2f})",
                   good_sample_count_, hdop, pos_enu.x(), pos_enu.y(), pos_enu.z());
    } else {
        ALOG_DEBUG(MOD, "GPS low quality: hdop={:.2f} quality={} sats={}",
                   hdop, static_cast<int>(quality), num_sats);
        good_sample_count_ = std::max(0, good_sample_count_ - 1);
        if (state_ == GPSAlignState::ALIGNED) {
            ALOG_WARN(MOD, "GPS signal degraded (hdop={:.2f}), pausing GPS constraints", hdop);
            state_ = GPSAlignState::DEGRADED;
        }
    }

    // 恢复：DEGRADED + GPS好转
    if (state_ == GPSAlignState::DEGRADED && good_sample_count_ >= good_samples_needed_ / 2) {
        ALOG_INFO(MOD, "GPS signal recovered, re-enabling constraints");
        state_ = GPSAlignState::ALIGNED;
    }

    // 触发对齐判断
    if (state_ == GPSAlignState::NOT_ALIGNED &&
        good_sample_count_ >= good_samples_needed_) {
        ALOG_INFO(MOD, "Sufficient good GPS samples ({}), triggering alignment...",
                  good_sample_count_);
        try_align();
    }

    // 已对齐：发布 GPS 因子
    if (state_ == GPSAlignState::ALIGNED && quality >= GPSQuality::HIGH) {
        Eigen::Vector3d pos_map = enu_to_map(pos_enu);
        double hdop_safe = std::max(hdop, 0.1);
        double sigma_h = hdop_safe * 0.3;  // 水平精度估算
        double sigma_v = hdop_safe * 0.5;
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        cov(0,0) = sigma_h * sigma_h;
        cov(1,1) = sigma_h * sigma_h;
        cov(2,2) = sigma_v * sigma_v;

        // 距离控制（每 factor_interval_m 添加一个GPS因子）
        double dist = pos_map.norm();
        double interval = ConfigManager::instance().gpsFactorIntervalM();
        if (dist - last_gps_factor_dist_ >= interval) {
            last_gps_factor_dist_ = dist;
            aligned_gps_buffer_.push_back({timestamp, pos_map, cov});
            for (auto& cb : gps_factor_cbs_) cb(timestamp, pos_map, cov);
        }
    }
}

void GPSManager::addKeyFramePose(double timestamp, const Pose3d& T_w_b) {
    std::lock_guard<std::mutex> lk(mutex_);
    kf_window_.push_back({timestamp, T_w_b});
    if (kf_window_.size() > 2000) kf_window_.pop_front();
}

void GPSManager::try_align() {
    // 检查轨迹长度
    if (kf_window_.empty() || gps_window_.empty()) return;

    // 计算轨迹总长度
    double total_dist = 0.0;
    for (size_t i = 1; i < kf_window_.size(); ++i) {
        total_dist += (kf_window_[i].second.translation() -
                       kf_window_[i-1].second.translation()).norm();
    }
    if (total_dist < min_align_dist_m_) return;

    // 检查GPS点数量
    size_t good_gps_count = 0;
    for (const auto& g : gps_window_) {
        if (g.quality >= GPSQuality::HIGH) good_gps_count++;
    }
    if ((int)good_gps_count < min_align_points_) return;

    state_ = GPSAlignState::ALIGNING;
    GPSAlignResult result = compute_svd_alignment();

    if (result.success && result.rmse_m < rmse_accept_thresh_) {
        ALOG_INFO(MOD, "GPS alignment SUCCESS: rmse={:.3f}m matched={} pts",
                  result.rmse_m, result.matched_points);
        align_result_ = result;
        on_aligned(result);
    } else {
        ALOG_WARN(MOD, "GPS alignment FAILED: success={} rmse={:.3f}m > thresh={:.3f}m matched={}",
                  result.success, result.rmse_m, rmse_accept_thresh_, result.matched_points);
        state_ = GPSAlignState::NOT_ALIGNED;
        good_sample_count_ = 0;
    }
}

// 最大允许的“最近邻”时间差（秒），用于无前后双关键帧时的回退
static constexpr double kMaxNearestDtSec = 0.1;
// 最大允许的插值区间（秒），超过则退化为最近邻
static constexpr double kMaxInterpIntervalSec = 0.5;

/**
 * SVD 轨迹匹配算法（参考 HBA gps_factor.hpp 的 path_match 实现）
 *
 * 方法：对 GPS ENU 轨迹和 LiDAR 轨迹进行点集配准
 *   1. 对每个高质量 GPS 时间戳，找「前、后」关键帧，在二者间线性插值得到 LiDAR 位置；
 *      若无双侧关键帧或插值区间过大，则退化为 100ms 内最近邻。
 *   2. 中心化 → 协方差矩阵 → SVD → R, t
 */
GPSAlignResult GPSManager::compute_svd_alignment() {
    GPSAlignResult result;

    // 找时间对齐的 (GPS_pos, LiDAR_pos) 对：前后关键帧 + 线性插值
    struct MatchPair { Eigen::Vector3d gps_enu; Eigen::Vector3d lidar_pos; };
    std::vector<MatchPair> pairs;

    if (kf_window_.empty()) return result;

    for (const auto& gr : gps_window_) {
        if (gr.quality < GPSQuality::HIGH) continue;

        // 找前关键帧：kt <= gr.timestamp 中最大的
        const std::pair<double, Pose3d>* kf_prev = nullptr;
        const std::pair<double, Pose3d>* kf_next = nullptr;
        for (const auto& kv : kf_window_) {
            if (kv.first <= gr.timestamp) kf_prev = &kv;
            if (kv.first >= gr.timestamp) { kf_next = &kv; break; }
        }

        Eigen::Vector3d lidar_pos;
        bool found = false;

        if (kf_prev && kf_next) {
            double t_prev = kf_prev->first;
            double t_next = kf_next->first;
            double interval = t_next - t_prev;
            if (interval <= 1e-9) {
                lidar_pos = kf_prev->second.translation();
                found = true;
            } else if (interval <= kMaxInterpIntervalSec) {
                // 线性插值：alpha in [0,1], pos = (1-alpha)*prev + alpha*next
                double alpha = (gr.timestamp - t_prev) / interval;
                alpha = std::max(0.0, std::min(1.0, alpha));
                lidar_pos = (1.0 - alpha) * kf_prev->second.translation()
                            + alpha * kf_next->second.translation();
                found = true;
            } else {
                // 区间过大，退化为 100ms 内最近邻
                double dt_prev = gr.timestamp - t_prev;
                double dt_next = t_next - gr.timestamp;
                if (dt_prev <= dt_next && dt_prev <= kMaxNearestDtSec) {
                    lidar_pos = kf_prev->second.translation();
                    found = true;
                } else if (dt_next <= kMaxNearestDtSec) {
                    lidar_pos = kf_next->second.translation();
                    found = true;
                }
            }
        } else if (kf_prev) {
            if (gr.timestamp - kf_prev->first <= kMaxNearestDtSec) {
                lidar_pos = kf_prev->second.translation();
                found = true;
            }
        } else if (kf_next) {
            if (kf_next->first - gr.timestamp <= kMaxNearestDtSec) {
                lidar_pos = kf_next->second.translation();
                found = true;
            }
        }

        if (found) {
            pairs.push_back({gr.pos_enu, lidar_pos});
        }
    }

    if ((int)pairs.size() < min_align_points_) return result;

    // 中心化
    Eigen::Vector3d gps_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d lio_centroid = Eigen::Vector3d::Zero();
    for (const auto& p : pairs) {
        gps_centroid += p.gps_enu;
        lio_centroid += p.lidar_pos;
    }
    gps_centroid /= pairs.size();
    lio_centroid /= pairs.size();

    // 协方差矩阵（仅XY平面，GPS高度不可靠）
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pairs) {
        Eigen::Vector3d gd = p.gps_enu   - gps_centroid;
        Eigen::Vector3d ld = p.lidar_pos - lio_centroid;
        gd.z() = 0.0;  // 忽略高度
        ld.z() = 0.0;
        cov += ld * gd.transpose();  // LiDAR←GPS
    }

    // SVD 求旋转（仅XY平面旋转，绕Z轴）
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // ✅ 退化场景检查
    Eigen::Vector3d singular_vals = svd.singularValues();
    const double min_sv = singular_vals.minCoeff();
    const double max_sv = singular_vals.maxCoeff();
    const double cond = (max_sv > 1e-12) ? (max_sv / min_sv) : 1e12;
    if (min_sv < 1e-6 || cond > 1e8) {
        ALOG_WARN(MOD, "GPS alignment: degenerate covariance (cond={:.2e}), rejecting", cond);
        return result;  // 返回失败
    }

    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    // 反射修正
    if (R.determinant() < 0) {
        Eigen::Matrix3d diag = Eigen::Matrix3d::Identity();
        diag(2,2) = -1;
        R = svd.matrixU() * diag * svd.matrixV().transpose();
    }

    Eigen::Vector3d t = lio_centroid - R * gps_centroid;

    // 计算 RMSE
    double rmse = 0.0;
    for (const auto& p : pairs) {
        Eigen::Vector3d pred = R * p.gps_enu + t;
        pred.z() = p.lidar_pos.z();  // 高度用LiDAR的
        rmse += (pred - p.lidar_pos).squaredNorm();
    }
    rmse = std::sqrt(rmse / pairs.size());

    result.success       = true;
    result.R_gps_lidar   = R;
    result.t_gps_lidar   = t;
    result.rmse_m        = rmse;
    result.matched_points = (int)pairs.size();
    return result;
}

void GPSManager::on_aligned(const GPSAlignResult& result) {
    state_ = GPSAlignState::ALIGNED;
    align_result_ = result;
    for (auto& cb : align_cbs_) cb(result);
}

Eigen::Vector3d GPSManager::wgs84_to_enu(double lat, double lon, double alt) const {
    GeographicLib::LocalCartesian proj(enu_origin_lat_, enu_origin_lon_, enu_origin_alt_);
    double e, n, u;
    proj.Forward(lat, lon, alt, e, n, u);
    return Eigen::Vector3d(e, n, u);
}

Eigen::Vector3d GPSManager::enu_to_map(const Eigen::Vector3d& enu) const {
    if (state_ != GPSAlignState::ALIGNED && state_ != GPSAlignState::DEGRADED)
        return enu;
    return align_result_.R_gps_lidar * enu + align_result_.t_gps_lidar;
}

GPSQuality GPSManager::hdop_to_quality(double hdop) const {
    if (hdop <= 1.0) return GPSQuality::EXCELLENT;
    if (hdop <= 2.0) return GPSQuality::HIGH;
    if (hdop <= 5.0) return GPSQuality::MEDIUM;
    if (hdop <= 20.0) return GPSQuality::LOW;
    return GPSQuality::INVALID;
}

GPSAlignState GPSManager::state() const {
    return state_.load();
}

GPSQuality GPSManager::currentQuality() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (gps_window_.empty()) return GPSQuality::INVALID;
    return gps_window_.back().quality;
}

std::optional<Eigen::Vector3d> GPSManager::getLatestPositionENU() const {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto it = gps_window_.rbegin(); it != gps_window_.rend(); ++it) {
        if (it->quality >= GPSQuality::MEDIUM) {
            if (isAligned()) return enu_to_map(it->pos_enu);
            return it->pos_enu;
        }
    }
    return std::nullopt;
}

std::optional<GPSMeasurement> GPSManager::queryByTimestamp(double ts) const {
    std::lock_guard<std::mutex> lk(mutex_);
    double best_dt = 0.1;
    const GPSRecord* best = nullptr;
    for (const auto& r : gps_window_) {
        double dt = std::abs(r.timestamp - ts);
        if (dt < best_dt) { best_dt = dt; best = &r; }
    }
    if (!best) return std::nullopt;
    GPSMeasurement m;
    m.timestamp    = best->timestamp;
    m.position_enu = isAligned() ? enu_to_map(best->pos_enu) : best->pos_enu;
    m.quality      = best->quality;
    m.hdop         = best->hdop;
    m.is_valid     = (best->quality >= GPSQuality::MEDIUM);
    return m;
}

void GPSManager::triggerRealign() {
    std::lock_guard<std::mutex> lk(mutex_);
    state_ = GPSAlignState::NOT_ALIGNED;
    good_sample_count_ = 0;
}

} // namespace automap_pro
