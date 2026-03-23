#include "automap_pro/v3/frontend_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

namespace automap_pro::v3 {

FrontEndModule::FrontEndModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("FrontEndModule", event_bus, map_registry), node_(node) {
    
    const auto& cfg = ConfigManager::instance();
    bool gps_enabled = cfg.gpsEnabled();
    std::string gps_topic = cfg.gpsTopic();
    
    livo_bridge_.init(node, gps_enabled, gps_topic);
    
    livo_bridge_.registerOdomCallback([this](double ts, const Pose3d& pose, const Mat66d& cov) {
        this->onOdometry(ts, pose, cov);
    });
    livo_bridge_.registerCloudCallback([this](double ts, const CloudXYZIPtr& cloud) {
        this->onCloud(ts, cloud);
    });
    livo_bridge_.registerKFInfoCallback([this](const LivoKeyFrameInfo& info) {
        this->onKFInfo(info);
    });
    livo_bridge_.registerGPSCallback([this](double ts, double lat, double lon, double alt, double hdop, int sats) {
        this->onGPS(ts, lat, lon, alt, hdop, sats);
    });

    const size_t max_ingress_q = cfg.ingressQueueMaxSize();
    const size_t max_frame_q = cfg.frameQueueMaxSize();
    frame_processor_.init(max_ingress_q, max_frame_q);
    RCLCPP_INFO(node_->get_logger(),
                "[PIPELINE][FE] ctor OK LivoBridge+FrameProcessor ingress_max=%zu frame_max=%zu gps=%s topic=%s",
                max_ingress_q, max_frame_q, gps_enabled ? "on" : "off", gps_topic.c_str());
}

void FrontEndModule::start() {
    if (running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::start (override) begin");
    running_ = true;
    
    frame_processor_.start();
    thread_ = std::thread(&FrontEndModule::run, this);
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::start sync thread spawned");
}

void FrontEndModule::stop() {
    if (!running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::stop begin");
    running_ = false;
    shutdown_requested_.store(true);
    frame_processor_.stop();
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::stop joined");
}

void FrontEndModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][FrontEndModule] Started sync thread");
    
    while (running_) {
        FrameToProcess f;
        bool woke_by_data = frame_processor_.tryPopFrame(100, f);
        if (!running_) break;
        if (!woke_by_data) continue;

        Pose3d pose = Pose3d::Identity();
        Mat66d cov = Mat66d::Identity() * 1e-4;

        if (!odomCacheGet(f.ts, pose, cov)) {
            // 插值失败时（多线程调度 / 短时滞后 / 历史边界）用最近一次里程计，与 LivoBridge「cloud 配 last odom」一致；过大时间差仍丢弃，避免错配
            std::lock_guard<std::mutex> lk(data_mutex_);
            constexpr double kMaxCloudOdomSkewS = 1.5;
            if (last_odom_ts_ >= 0.0 && std::abs(f.ts - last_odom_ts_) <= kMaxCloudOdomSkewS) {
                pose = last_odom_pose_;
                cov = last_cov_;
            } else {
                continue;
            }
        }

        LivoKeyFrameInfo kfinfo_copy;
        if (!kfinfoCacheGet(f.ts, kfinfo_copy)) {
            std::lock_guard<std::mutex> lk(data_mutex_);
            kfinfo_copy = last_livo_info_;
        }

        GPSMeasurement matched_gps;
        bool has_gps = gpsCacheGet(f.ts, matched_gps);

        SyncedFrameEvent event;
        event.timestamp = f.ts;
        event.cloud = f.cloud;
        event.cloud_ds = f.cloud_ds;
        event.T_odom_b = pose;
        event.covariance = cov;
        event.kf_info = kfinfo_copy;
        event.has_gps = has_gps;
        event.gps = matched_gps;
        event.ref_map_version = map_registry_->getVersion();

        event_bus_->publish(event);
    }
}

void FrontEndModule::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    RawOdometryEvent ev;
    ev.timestamp = ts;
    ev.pose = pose;
    ev.covariance = cov;
    event_bus_->publish(ev);

    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_odom_pose_ = pose;
        last_odom_ts_ = ts;
        last_cov_ = cov;
    }
    odomCacheAdd(ts, pose, cov);
}

void FrontEndModule::onCloud(double ts, const CloudXYZIPtr& cloud) {
    RawCloudEvent ev;
    ev.timestamp = ts;
    ev.cloud = cloud;
    event_bus_->publish(ev);

    frame_processor_.pushFrame(ts, cloud);
}

void FrontEndModule::onKFInfo(const LivoKeyFrameInfo& info) {
    RawKFInfoEvent ev;
    ev.info = info;
    event_bus_->publish(ev);

    { std::lock_guard<std::mutex> lk(data_mutex_); last_livo_info_ = info; }
    kfinfoCacheAdd(info.timestamp, info);
}

void FrontEndModule::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats) {
    RawGPSEvent ev;
    ev.timestamp = ts;
    ev.lat = lat; ev.lon = lon; ev.alt = alt; ev.hdop = hdop; ev.sats = sats;
    event_bus_->publish(ev);

    GPSMeasurement m;
    m.timestamp = ts;
    m.latitude = lat;
    m.longitude = lon;
    m.altitude = alt;
    m.hdop = hdop;
    m.num_satellites = sats;
    m.is_valid = (sats >= 4 && hdop < 10.0); // 这里的阈值可以根据需要调整

    if (m.is_valid) {
        map_registry_->setGPSOrigin(lat, lon, alt);
        
        double olat, olon, oalt;
        if (map_registry_->getGPSOrigin(olat, olon, oalt)) {
            m.position_enu = utils::wgs84ToEnu(lat, lon, alt, olat, olon, oalt);
            
            // 简单的协方差估算
            double sigma = std::max(0.1, hdop * 0.5);
            m.covariance = Eigen::Matrix3d::Identity() * (sigma * sigma);
        }
    }

    gpsCacheAdd(ts, m);
}

void FrontEndModule::gpsCacheAdd(double ts, const GPSMeasurement& m) {
    std::lock_guard<std::mutex> lk(gps_cache_mutex_);
    gps_cache_.push_back({ts, m});
    while (gps_cache_.size() > kMaxGPSCacheSize) gps_cache_.pop_front();
}

bool FrontEndModule::gpsCacheGet(double ts, GPSMeasurement& out_m) {
    std::lock_guard<std::mutex> lk(gps_cache_mutex_);
    if (gps_cache_.empty()) return false;
    
    // 查找最近的 GPS 观测，允许最大 0.5s 的偏差 (宽松模式，适应延迟)
    const double max_dt = 0.5;
    double min_dt = 1e9;
    const GPSCacheEntry* best = nullptr;
    
    for (auto it = gps_cache_.rbegin(); it != gps_cache_.rend(); ++it) {
        double dt = std::abs(it->ts - ts);
        if (dt < min_dt) {
            min_dt = dt;
            best = &(*it);
        }
        if (it->ts < ts - max_dt) break;
    }
    
    if (best && min_dt <= max_dt) {
        out_m = best->m;
        return true;
    }
    return false;
}

void FrontEndModule::odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    odom_cache_.push_back({ts, pose, cov});
    while (odom_cache_.size() > kMaxOdomCacheSize) odom_cache_.pop_front();
}

bool FrontEndModule::odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    if (odom_cache_.empty()) return false;
    
    // 如果只有一个样本，直接返回
    if (odom_cache_.size() == 1) {
        if (std::abs(odom_cache_.front().ts - ts) < 0.5) {
            out_pose = odom_cache_.front().pose;
            out_cov  = odom_cache_.front().cov;
            return true;
        }
        return false;
    }

    // 寻找插值区间 [it_prev, it_next]
    auto it_next = odom_cache_.begin();
    while (it_next != odom_cache_.end() && it_next->ts < ts) {
        ++it_next;
    }

    if (it_next == odom_cache_.begin()) {
        // 请求时间在最旧数据之前 (通常是系统刚启动)
        if (std::abs(it_next->ts - ts) < 0.2) {
            out_pose = it_next->pose;
            out_cov = it_next->cov;
            return true;
        }
        return false;
    }

    if (it_next == odom_cache_.end()) {
        // 请求时间在最新数据之后 (后端处理慢，里程计还没跟上)
        auto it_last = std::prev(it_next);
        if (std::abs(it_last->ts - ts) < 0.5) {
            out_pose = it_last->pose;
            out_cov = it_last->cov;
            return true;
        }
        // 如果差得太多，可能需要等待数据，这里暂时返回 false，外部 run() 会 continue
        return false;
    }

    // 线性插值
    auto it_prev = std::prev(it_next);
    double ratio = (ts - it_prev->ts) / (it_next->ts - it_prev->ts);
    
    // 位姿线性插值（平移线性，旋转 SLERP）
    Eigen::Vector3d t = (1.0 - ratio) * it_prev->pose.translation() + ratio * it_next->pose.translation();
    Eigen::Quaterniond q_prev(it_prev->pose.rotation());
    Eigen::Quaterniond q_next(it_next->pose.rotation());
    Eigen::Quaterniond q = q_prev.slerp(ratio, q_next);
    
    out_pose = Pose3d::Identity();
    out_pose.linear() = q.toRotationMatrix();
    out_pose.translation() = t;
    
    // 协方差线性插值
    out_cov = (1.0 - ratio) * it_prev->cov + ratio * it_next->cov;
    
    return true;
}

void FrontEndModule::kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    kfinfo_cache_.push_back({ts, info});
    while (kfinfo_cache_.size() > kMaxKFinfoCacheSize) kfinfo_cache_.pop_front();
}

bool FrontEndModule::kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    if (kfinfo_cache_.empty()) return false;
    const KFinfoCacheEntry* best = nullptr;
    for (auto it = kfinfo_cache_.rbegin(); it != kfinfo_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) return false;
    out_info = best->info;
    return true;
}

} // namespace automap_pro::v3
