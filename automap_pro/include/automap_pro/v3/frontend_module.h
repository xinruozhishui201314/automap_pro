#pragma once

#include "automap_pro/v3/module_base.h"
#include "automap_pro/v3/semantic_processor.h"
#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/system/frame_processor.h"
#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace automap_pro::v3 {

/**
 * @brief 前端数据处理模块 (FrontEndModule)
 * 
 * 职责：
 * 1. 管理 LivoBridge，对接 FAST-LIVO2 的 ROS 话题
 * 2. 维护里程计和关键帧信息的缓存 (odom_cache, kfinfo_cache)
 * 3. 管理 FrameProcessor，执行点云下采样
 * 4. 执行时间戳对齐与同步，发布 SyncedFrameEvent（可选按 frontend.sweep_accumulation_frames 叠加扫描，与 FastLIVO2Adapter 语义一致，供 LSK / 建图增密）
 * 5. 发布原始传感器事件 (RawOdometryEvent, RawCloudEvent, etc.)
 */
class FrontEndModule : public ModuleBase {
public:
    using Ptr = std::shared_ptr<FrontEndModule>;

    FrontEndModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node);

    void start() override;
    void stop() override;

protected:
    void run() override;

private:
    // LivoBridge 回调
    void onOdometry(double ts, const Pose3d& pose, const Mat66d& cov);
    void onCloud(double ts, const CloudXYZIPtr& cloud);
    void onKFInfo(const LivoKeyFrameInfo& info);
    void onGPS(double ts, double lat, double lon, double alt, double hdop, int sats);

    // GPS 缓存
    struct GPSCacheEntry {
        double ts = 0.0;
        GPSMeasurement m;
    };
    void gpsCacheAdd(double ts, const GPSMeasurement& m);
    bool gpsCacheGet(double ts, GPSMeasurement& out_m);

    // 缓存管理
    struct OdomCacheEntry {
        double ts = 0.0;
        Pose3d pose = Pose3d::Identity();
        Mat66d cov  = Mat66d::Identity() * 1e-4;
    };
    struct KFinfoCacheEntry {
        double ts = 0.0;
        LivoKeyFrameInfo info;
    };

    void odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov);
    bool odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov);
    void kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info);
    bool kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info);

    /// 与 FastLIVO2Adapter::accumulateSweeps 对齐：缓冲 world/body 扫描 + 对应 T_odom_b，发布前合并到当前帧坐标约定
    struct SweepBufferEntry {
        double ts = 0.0;
        CloudXYZIPtr cloud;
        Pose3d T_odom_b = Pose3d::Identity();
    };
    void sweepBufferAdd(double ts, const CloudXYZIPtr& cloud);
    static CloudXYZIPtr mergeSweepsToEventCloud(
        const Pose3d& T_curr_odom_b,
        const std::vector<SweepBufferEntry>& snaps,
        const std::string& cloud_frame);

    // 成员变量
    LivoBridge livo_bridge_;
    FrameProcessor frame_processor_;
    
    std::deque<OdomCacheEntry> odom_cache_;
    mutable std::mutex odom_cache_mutex_;
    static constexpr size_t kMaxOdomCacheSize = 5000;

    std::deque<KFinfoCacheEntry> kfinfo_cache_;
    mutable std::mutex kfinfo_cache_mutex_;
    static constexpr size_t kMaxKFinfoCacheSize = 2000;

    std::deque<GPSCacheEntry> gps_cache_;
    mutable std::mutex gps_cache_mutex_;
    static constexpr size_t kMaxGPSCacheSize = 2000;

    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> throttle_active_{false};
    std::atomic<double> throttle_until_{0.0};
    
    // 同步用数据（从 AutoMapSystem 移来）
    mutable std::mutex data_mutex_;
    LivoKeyFrameInfo last_livo_info_;
    std::atomic<uint64_t> kf_info_recv_total_{0};
    std::atomic<uint64_t> kf_info_invalid_ts_total_{0};
    std::atomic<double> kf_info_last_valid_ts_{-1.0};
    std::atomic<uint64_t> kf_info_cache_miss_total_{0};
    std::atomic<uint64_t> kf_info_cache_empty_total_{0};
    std::atomic<uint64_t> kf_info_cache_no_leq_total_{0};
    std::atomic<uint64_t> kf_info_fallback_invalid_total_{0};
    std::atomic<uint64_t> kf_info_map_hint_recovered_total_{0};
    std::atomic<uint64_t> cloud_recv_total_{0};
    std::atomic<uint64_t> synced_publish_total_{0};
    std::atomic<uint64_t> sweep_merge_diag_total_{0};
    std::atomic<uint64_t> event_seq_{0};
    Pose3d last_odom_pose_ = Pose3d::Identity();
    double last_odom_ts_ = -1.0;
    Mat66d last_cov_ = Mat66d::Identity() * 1e-4;

    std::deque<SweepBufferEntry> sweep_buffer_;
    mutable std::mutex sweep_buffer_mutex_;
    int max_sweep_buffer_size_{20};
};

} // namespace automap_pro::v3
