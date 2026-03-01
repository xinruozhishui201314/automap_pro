#pragma once

#include <deque>
#include <mutex>
#include <functional>
#include <optional>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// Generic timestamp-based buffer with nearest/interpolated lookup
// ──────────────────────────────────────────────────────────
template<typename T>
class TimedBuffer {
public:
    explicit TimedBuffer(size_t max_size = 10000, double max_age_s = 5.0)
        : max_size_(max_size), max_age_s_(max_age_s) {}

    void push(double timestamp, const T& data) {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.push_back({timestamp, data});
        if (buffer_.size() > max_size_) buffer_.pop_front();
        evictOld(timestamp);
    }

    // Nearest-neighbor lookup
    std::optional<T> nearest(double t) const {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.empty()) return std::nullopt;
        double best_dt = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < buffer_.size(); ++i) {
            double dt = std::abs(buffer_[i].first - t);
            if (dt < best_dt) { best_dt = dt; best_idx = i; }
        }
        return buffer_[best_idx].second;
    }

    // Get all items in [t0, t1]
    std::vector<T> range(double t0, double t1) const {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<T> out;
        for (const auto& [ts, d] : buffer_) {
            if (ts >= t0 && ts <= t1) out.push_back(d);
        }
        return out;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return buffer_.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return buffer_.size();
    }

    double latestTimestamp() const {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.empty()) return -1.0;
        return buffer_.back().first;
    }

private:
    void evictOld(double now) {
        while (!buffer_.empty() && (now - buffer_.front().first) > max_age_s_) {
            buffer_.pop_front();
        }
    }

    mutable std::mutex mutex_;
    std::deque<std::pair<double, T>> buffer_;
    size_t max_size_;
    double max_age_s_;
};

// ──────────────────────────────────────────────────────────
// TimeSync: manages multi-sensor time alignment
// ──────────────────────────────────────────────────────────
class TimeSync {
public:
    TimeSync();

    void setLidarTimeOffset(double offset_s);
    void setImuTimeOffset(double offset_s);
    void setGpsTimeOffset(double offset_s);
    void setCameraTimeOffset(double offset_s);

    // Correct raw ROS stamp → corrected system timestamp
    double correctLidar (double ros_stamp) const;
    double correctImu   (double ros_stamp) const;
    double correctGps   (double ros_stamp) const;
    double correctCamera(double ros_stamp) const;

    // IMU interpolation at target time (for motion compensation)
    ImuData interpolateImu(
        const ImuData& imu0, const ImuData& imu1, double t) const;

private:
    double lidar_offset_  = 0.0;
    double imu_offset_    = 0.0;
    double gps_offset_    = 0.0;
    double camera_offset_ = 0.0;
};

}  // namespace automap_pro
