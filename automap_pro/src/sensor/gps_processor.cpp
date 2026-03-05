#include "automap_pro/sensor/gps_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/core/logger.h"

#include <chrono>

#define MOD "GPSProcessor"

namespace automap_pro {

GPSProcessor::GPSProcessor() {
    try {
        const auto& cfg = ConfigManager::instance();
        excellent_hdop_              = cfg.gpsExcellentHDOP();
        high_hdop_                   = cfg.gpsHighHDOP();
        medium_hdop_                 = cfg.gpsMediumHDOP();
        max_jump_                    = cfg.gpsMaxJump();
        max_velocity_                = cfg.gpsMaxVelocity();
        chi2_threshold_              = cfg.gpsChi2Threshold();
        consecutive_valid_required_  = cfg.gpsConsecutiveValid();
        jump_detection_enabled_      = cfg.gpsJumpDetection();
        consistency_check_enabled_   = cfg.gpsConsistencyCheck();
        
        ALOG_INFO(MOD, "GPSProcessor initialized: excellent_hdop={:.1f} max_velocity={:.1f}m/s", 
                  excellent_hdop_, max_velocity_);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "GPSProcessor config load failed: {}, using defaults", e.what());
        excellent_hdop_ = 0.8;
        high_hdop_ = 1.5;
        medium_hdop_ = 3.0;
        max_jump_ = 10.0;
        max_velocity_ = 30.0;
        chi2_threshold_ = 7.815;
        consecutive_valid_required_ = 3;
        jump_detection_enabled_ = true;
        consistency_check_enabled_ = true;
    }
}

void GPSProcessor::init(rclcpp::Node::SharedPtr node) {
    try {
        if (!node) {
            throw std::invalid_argument("GPSProcessor::init: node is null");
        }
        
        logger_ = node->get_logger();
        topic_name_ = ConfigManager::instance().gpsTopic();
        sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
            topic_name_, 100, std::bind(&GPSProcessor::gpsCallback, this, std::placeholders::_1));
        RCLCPP_INFO(logger_, "[GPSProcessor][TOPIC] subscribe: %s", topic_name_.c_str());
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "GPSProcessor::init failed: {}", e.what());
        throw;
    }
}

void GPSProcessor::setENUOrigin(double lat, double lon, double alt) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 验证坐标有效性
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)) {
        ALOG_ERROR(MOD, "setENUOrigin: invalid coordinates (lat={}, lon={}, alt={})", lat, lon, alt);
        return;
    }
    
    if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0) {
        ALOG_ERROR(MOD, "setENUOrigin: coordinates out of valid range (lat={}, lon={})", lat, lon);
        return;
    }
    
    enu_origin_.latitude  = lat;
    enu_origin_.longitude = lon;
    enu_origin_.altitude  = alt;
    enu_origin_.initialized = true;
    RCLCPP_INFO(logger_, "[GPSProcessor] ENU origin set: lat=%.7f lon=%.7f alt=%.2f", lat, lon, alt);
}

void GPSProcessor::registerCallback(MeasurementCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void GPSProcessor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    try {
        if (!msg) {
            ALOG_WARN(MOD, "GPSProcessor: received null message");
            return;
        }
        
        process(msg);
        double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        last_msg_ts_ = rclcpp::Time(msg->header.stamp).seconds();
        msg_count_++;
        if (now - last_log_time_ >= kDataFlowLogInterval) {
            RCLCPP_INFO(logger_, "[DataFlow] GPS | topic=%s | count=%lu | last_ts=%.3f | valid=%d",
                        topic_name_.c_str(), static_cast<unsigned long>(msg_count_), last_msg_ts_,
                        msg->status.status >= 0 ? 1 : 0);
            msg_count_ = 0;
            last_log_time_ = now;
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "gpsCallback exception: {}", e.what());
    }
}

void GPSProcessor::process(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);

    try {
        // 验证基本数据有效性
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || 
            !std::isfinite(msg->altitude)) {
            ALOG_WARN(MOD, "GPSProcessor: invalid lat/lon/alt values, skipping");
            return;
        }
        
        if (!enu_origin_.initialized) {
            if (msg->status.status < 0) {
                ALOG_DEBUG(MOD, "GPSProcessor: waiting for valid fix to set ENU origin");
                return;
            }
            enu_origin_.latitude  = msg->latitude;
            enu_origin_.longitude = msg->longitude;
            enu_origin_.altitude  = msg->altitude;
            enu_origin_.initialized = true;
            RCLCPP_INFO(logger_, "[GPSProcessor] Auto-init ENU origin: lat=%.7f lon=%.7f",
                     msg->latitude, msg->longitude);
        }

        GPSMeasurement meas;
        meas.timestamp  = rclcpp::Time(msg->header.stamp).seconds();
        meas.latitude   = msg->latitude;
        meas.longitude  = msg->longitude;
        meas.altitude   = msg->altitude;
        
        // 验证时间戳有效性
        if (meas.timestamp <= 0) {
            ALOG_WARN(MOD, "GPSProcessor: invalid timestamp {:.3f}", meas.timestamp);
            return;
        }

        double e, n, u;
        try {
            utils::wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
                              enu_origin_.latitude, enu_origin_.longitude, enu_origin_.altitude,
                              e, n, u);
        } catch (const std::exception& ex) {
            ALOG_ERROR(MOD, "wgs84ToENU conversion failed: {}", ex.what());
            return;
        }
        
        // 验证 ENU 坐标有效性
        if (!std::isfinite(e) || !std::isfinite(n) || !std::isfinite(u)) {
            ALOG_WARN(MOD, "GPSProcessor: ENU conversion produced invalid values");
            return;
        }
        
        meas.position_enu = Eigen::Vector3d(e, n, u);
        meas.quality = assessQuality(*msg);
        meas.hdop = 99.0;

        if (msg->position_covariance_type > 0) {
            meas.covariance = Eigen::Matrix3d::Identity();
            double cov0 = msg->position_covariance[0];
            double cov4 = msg->position_covariance[4];
            double cov8 = msg->position_covariance[8];
            
            // 验证协方差有效性
            if (std::isfinite(cov0) && cov0 > 0) meas.covariance(0,0) = cov0;
            if (std::isfinite(cov4) && cov4 > 0) meas.covariance(1,1) = cov4;
            if (std::isfinite(cov8) && cov8 > 0) meas.covariance(2,2) = cov8;
        } else {
            meas.covariance = qualityToCovariance(meas.quality, meas.hdop);
        }

        meas.is_valid = (meas.quality != GPSQuality::INVALID);

        if (jump_detection_enabled_ && !gps_history_.empty()) {
            double dt = meas.timestamp - gps_history_.back().first;
            if (dt > 0 && !jumpDetection(meas.position_enu, dt)) {
                meas.is_outlier = true;
                meas.is_valid   = false;
                consecutive_outliers_++;
                consecutive_valid_ = 0;
                ALOG_WARN(MOD, "GPS jump detected: dt={:.3f}s pos=({:.2f},{:.2f},{:.2f})", 
                          dt, meas.position_enu.x(), meas.position_enu.y(), meas.position_enu.z());
            } else {
                consecutive_outliers_ = 0;
                consecutive_valid_++;
            }
        }

        if (consistency_check_enabled_ && !meas.is_outlier && !odom_history_.empty()) {
            if (!consistencyCheck(meas)) {
                meas.is_outlier = true;
                meas.is_valid   = false;
                consecutive_outliers_++;
                ALOG_WARN(MOD, "GPS-odometry consistency check failed");
            }
        }

        gps_history_.push_back({meas.timestamp, meas.position_enu});
        if (gps_history_.size() > 200) gps_history_.pop_front();
        
        updateState(meas.is_valid && !meas.is_outlier);
        meas.is_valid = (state_ == GPSState::TRACKING || state_ == GPSState::DEGRADED) && meas.is_valid;

        for (auto& cb : callbacks_) {
            try {
                cb(meas);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "GPS callback exception: {}", e.what());
            }
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "GPSProcessor::process exception: {}", e.what());
    }
}

GPSQuality GPSProcessor::assessQuality(const sensor_msgs::msg::NavSatFix& msg) const {
    try {
        if (msg.status.status < 0) return GPSQuality::INVALID;
        
        double cov_xy = 1.0;
        if (msg.position_covariance_type > 0 && 
            std::isfinite(msg.position_covariance[0]) && msg.position_covariance[0] >= 0) {
            cov_xy = std::sqrt(msg.position_covariance[0]);
        }
        
        if (msg.status.status >= 2) {
            if (cov_xy <= excellent_hdop_) return GPSQuality::EXCELLENT;
            if (cov_xy <= high_hdop_)     return GPSQuality::HIGH;
        }
        if (msg.status.status >= 1) {
            if (cov_xy <= medium_hdop_) return GPSQuality::MEDIUM;
            return GPSQuality::LOW;
        }
        if (msg.status.status == 0) return GPSQuality::LOW;
        
        return GPSQuality::INVALID;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "assessQuality exception: {}", e.what());
        return GPSQuality::INVALID;
    }
}

Eigen::Matrix3d GPSProcessor::qualityToCovariance(GPSQuality q, double /*hdop*/) const {
    try {
        const auto& cfg = ConfigManager::instance();
        Eigen::Vector3d sigmas;
        switch (q) {
            case GPSQuality::EXCELLENT: sigmas = cfg.gpsCovExcellent(); break;
            case GPSQuality::HIGH:      sigmas = cfg.gpsCovHigh();      break;
            case GPSQuality::MEDIUM:    sigmas = cfg.gpsCovMedium();    break;
            case GPSQuality::LOW:       sigmas = cfg.gpsCovLow();       break;
            default:
                return Eigen::Matrix3d::Identity() * 1e6;
        }
        
        // 验证 sigmas 有效性
        if (!sigmas.allFinite() || (sigmas.array() < 0).any()) {
            ALOG_WARN(MOD, "qualityToCovariance: invalid sigmas, using defaults");
            sigmas = Eigen::Vector3d(1.0, 1.0, 3.0);
        }
        
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        cov(0,0) = sigmas(0) * sigmas(0);
        cov(1,1) = sigmas(1) * sigmas(1);
        cov(2,2) = sigmas(2) * sigmas(2);
        return cov;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "qualityToCovariance exception: {}", e.what());
        return Eigen::Matrix3d::Identity() * 1e6;
    }
}

bool GPSProcessor::jumpDetection(const Eigen::Vector3d& pos, double dt) {
    try {
        if (gps_history_.empty()) return true;
        
        const auto& prev = gps_history_.back().second;
        double dist = (pos - prev).norm();
        
        // 验证距离计算有效性
        if (!std::isfinite(dist)) {
            ALOG_WARN(MOD, "jumpDetection: invalid distance calculation");
            return true;
        }
        
        double max_allowed = max_velocity_ * std::max(dt, 0.1) + max_jump_;
        bool valid = dist <= max_allowed;
        
        if (!valid) {
            ALOG_DEBUG(MOD, "jumpDetection: rejected (dist={:.2f}m > max={:.2f}m)", dist, max_allowed);
        }
        
        return valid;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "jumpDetection exception: {}", e.what());
        return true;
    }
}

bool GPSProcessor::consistencyCheck(const GPSMeasurement& meas) {
    try {
        if (odom_history_.empty()) return true;
        
        Eigen::Vector3d odom_pos;
        double best_dt = 1e9;
        for (const auto& [ts, pos] : odom_history_) {
            double dt = std::abs(ts - meas.timestamp);
            if (dt < best_dt) { 
                best_dt = dt; 
                odom_pos = pos; 
            }
        }
        if (best_dt > 1.0) return true;
        
        Eigen::Vector3d delta = meas.position_enu - odom_pos;
        Eigen::Matrix3d cov = meas.covariance + Eigen::Matrix3d::Identity() * 0.25;
        
        double d_m = utils::mahalanobisDistance3d(delta, cov);
        
        // 验证 Mahalanobis 距离有效性
        if (!std::isfinite(d_m)) {
            ALOG_WARN(MOD, "consistencyCheck: invalid Mahalanobis distance");
            return true;
        }
        
        return d_m <= std::sqrt(chi2_threshold_);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "consistencyCheck exception: {}", e.what());
        return true;
    }
}

void GPSProcessor::updateState(bool valid) {
    try {
        if (!valid) {
            consecutive_outliers_++;
            consecutive_valid_ = 0;
            if (consecutive_outliers_ >= 3) {
                state_ = (state_ == GPSState::TRACKING) ? GPSState::DEGRADED : GPSState::LOST;
                ALOG_WARN(MOD, "GPS state degraded: state={} consecutive_outliers={}", 
                          static_cast<int>(state_), consecutive_outliers_);
            }
        } else {
            consecutive_valid_++;
            if (consecutive_valid_ >= consecutive_valid_required_) {
                state_ = GPSState::TRACKING;
                consecutive_outliers_ = 0;
                ALOG_INFO(MOD, "GPS state recovered: TRACKING");
            }
        }
        if (state_ == GPSState::INIT && valid) state_ = GPSState::TRACKING;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "updateState exception: {}", e.what());
    }
}

void GPSProcessor::updateOdometryPose(double timestamp, const Eigen::Vector3d& pos) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 验证输入有效性
    if (!std::isfinite(timestamp) || !pos.allFinite()) {
        ALOG_WARN(MOD, "updateOdometryPose: invalid timestamp or position");
        return;
    }
    
    odom_history_.push_back({timestamp, pos});
    if (odom_history_.size() > 500) odom_history_.pop_front();
}

GPSState  GPSProcessor::currentState() const { return state_; }
ENUOrigin GPSProcessor::enuOrigin()    const { return enu_origin_; }
bool      GPSProcessor::hasOrigin()    const { return enu_origin_.initialized; }

}  // namespace automap_pro
