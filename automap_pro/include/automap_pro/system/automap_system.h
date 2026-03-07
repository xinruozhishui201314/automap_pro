#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/backend/hba_optimizer.h"
#include "automap_pro/visualization/rviz_publisher.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 本包消息类型
#include <automap_pro/msg/mapping_status_msg.hpp>
// 服务/Action 类型
#include <automap_pro/srv/save_map.hpp>
#include <automap_pro/srv/get_status.hpp>
#include <automap_pro/srv/trigger_hba.hpp>
#include <automap_pro/srv/trigger_optimize.hpp>
#include <automap_pro/srv/trigger_gps_align.hpp>
#include <automap_pro/srv/load_session.hpp>

#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <deque>
#include <condition_variable>
#include <chrono>
#include <fstream>

namespace automap_pro {

/** 按时间戳对齐：帧队列只存 (ts, cloud)，pose/cov/kfinfo 由 worker 从缓存按 ts 查询 */
struct FrameToProcess {
    double ts = 0.0;
    CloudXYZIPtr cloud;
};

/** 里程计按时间戳缓存，worker 用 get(cloud_ts) 对齐 */
struct OdomCacheEntry {
    double ts = 0.0;
    Pose3d pose = Pose3d::Identity();
    Mat66d cov  = Mat66d::Identity() * 1e-4;
};

/** KFinfo 按时间戳缓存，worker 用 get(cloud_ts) 对齐 */
struct KFinfoCacheEntry {
    double ts = 0.0;
    LivoKeyFrameInfo info;
};

/**
 * AutoMapSystem —— 主控制层（Composable Node）
 *
 * 职责：
 *   1. 编排所有子模块（LivoBridge, KeyFrameManager, GPSManager,
 *      SubMapManager, LoopDetector, IncrementalOptimizer, HBAOptimizer）
 *   2. 实现主数据流：传感器 → KF → SubMap → Loop → 优化 → 地图
 *   3. 管理系统状态机
 *   4. 暴露 ROS2 服务接口
 *   5. 支持多会话增量建图（MS-Mapping）
 *
 * Composable Node 模式：
 *   在同一 Component Container 中与 fast_livo 运行时，
 *   rclcpp 自动启用 Intra-Process Communication，实现零拷贝数据传递。
 *
 * GPS 策略（延迟对齐）：
 *   - 建图初期 GPS 信号不足：仅 LiDAR+IMU 建图，GPS 约束暂停
 *   - GPS 信号好转后：GPSManager 触发 SVD 对齐
 *   - 对齐成功后：批量补充 GPS 因子到 IncrementalOptimizer + HBA
 */
class AutoMapSystem : public rclcpp::Node {
public:
    explicit AutoMapSystem(const rclcpp::NodeOptions& options);
    ~AutoMapSystem();

private:
    // ── 子模块 ────────────────────────────────────────────────────────────
    LivoBridge            livo_bridge_;
    KeyFrameManager       kf_manager_;
    GPSManager            gps_manager_;
    SubMapManager         submap_manager_;
    LoopDetector          loop_detector_;
    IncrementalOptimizer  isam2_optimizer_;
    HBAOptimizer          hba_optimizer_;
    RvizPublisher         rviz_publisher_;

    // 回环约束缓存（用于可视化）
    std::vector<LoopConstraint::Ptr> loop_constraints_;
    mutable std::mutex               loop_constraints_mutex_;

    // ── 状态 ─────────────────────────────────────────────────────────────
    std::atomic<SystemState> state_{SystemState::IDLE};
    uint64_t current_session_id_ = 0;
    std::mutex state_mutex_;
    
    // 异步任务关闭请求标志
    std::atomic<bool> shutdown_requested_{false};

    // 后端帧队列：只存 (ts, cloud)，worker 按 ts 从 odom/kfinfo 缓存对齐，不阻塞回调
    static constexpr size_t kMaxFrameQueueSize = 500;
    std::queue<FrameToProcess> frame_queue_;
    std::mutex                frame_queue_mutex_;
    std::condition_variable   frame_queue_cv_;
    std::thread               backend_worker_;
    std::atomic<int>          frame_queue_dropped_{0};
    void backendWorkerLoop();

    // 按时间戳缓存的 odom / kfinfo，有界、非阻塞写，worker 按帧 ts 对齐读取
    static constexpr size_t kMaxOdomCacheSize   = 1000;
    static constexpr size_t kMaxKFinfoCacheSize = 1000;
    std::deque<OdomCacheEntry>   odom_cache_;
    std::mutex                   odom_cache_mutex_;
    std::deque<KFinfoCacheEntry> kfinfo_cache_;
    std::mutex                   kfinfo_cache_mutex_;
    void odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov);
    bool odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov);
    void kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info);
    bool kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info);

    // 当前里程计/点云状态缓存（pose 与 cloud 同帧：由 onCloud 触发 KF，保证用到的 last_odom_pose_ 已由本帧 odom 更新）
    Pose3d    last_odom_pose_    = Pose3d::Identity();
    double    last_odom_ts_      = -1.0;
    CloudXYZIPtr last_cloud_;
    double    last_cloud_ts_    = -1.0;  // 点云时间戳，用于日志与一致性检查
    Mat66d    last_cov_          = Mat66d::Identity() * 1e-4;
    mutable std::mutex data_mutex_;

    // ESIKF 质量
    LivoKeyFrameInfo last_livo_info_;

    // GPS 对齐状态
    bool gps_aligned_        = false;
    bool gps_batch_added_    = false;  // 对齐后是否已批量添加GPS因子

    // 子图计数（用于 HBA 周期触发）
    int  frozen_submap_count_ = 0;

    // 地图体素大小（init 时从 ConfigManager 缓存，避免 publishGlobalMap 回调中访问单例导致析构顺序 SIGSEGV）
    float map_voxel_size_ = 0.2f;
    // launch 传入的 output_dir，非空时优先于 system.output_dir，使前后端保存到同一目录
    std::string output_dir_override_;

    // 首次数据到达日志（各打一次，便于确认数据流）
    std::atomic<bool> first_odom_logged_{false};
    std::atomic<bool> first_cloud_logged_{false};
    // 传感器空闲结束建图：上次收到点云的墙钟时间；超时后触发最终处理并退出
    std::chrono::steady_clock::time_point last_sensor_data_wall_time_{std::chrono::steady_clock::now()};
    std::atomic<bool> sensor_idle_finish_triggered_{false};
    // 后端已处理的点云帧计数（用于每帧日志）
    std::atomic<int> backend_cloud_frames_processed_{0};
    // 周期性状态汇总（每 10 次 status 打一条，约 10s）
    int status_publish_count_ = 0;
    // 低频数据流日志：各模块收发/发布计数（每 15s 打一条）
    std::atomic<int> pub_odom_path_count_{0};
    std::atomic<int> pub_opt_path_count_{0};
    std::atomic<int> pub_map_count_{0};
    std::atomic<int> pub_status_count_{0};

    // ── 发布者 ────────────────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                opt_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      global_map_pub_;
    rclcpp::Publisher<automap_pro::msg::MappingStatusMsg>::SharedPtr status_pub_;
    nav_msgs::msg::Path odom_path_, opt_path_;

    // ── 服务 ─────────────────────────────────────────────────────────────
    rclcpp::Service<automap_pro::srv::SaveMap>::SharedPtr        save_map_srv_;
    rclcpp::Service<automap_pro::srv::GetStatus>::SharedPtr      get_status_srv_;
    rclcpp::Service<automap_pro::srv::TriggerHBA>::SharedPtr     trigger_hba_srv_;
    rclcpp::Service<automap_pro::srv::TriggerOptimize>::SharedPtr trigger_opt_srv_;
    rclcpp::Service<automap_pro::srv::TriggerGpsAlign>::SharedPtr trigger_gps_srv_;
    rclcpp::Service<automap_pro::srv::LoadSession>::SharedPtr     load_session_srv_;

    // ── 定时器（仅保留一次性 deferred init）────────────────────────────────
    // status/map/data_flow 已改为数据触发：在 backendWorkerLoop 中按处理帧数触发，见 setupTimers 注释
    rclcpp::TimerBase::SharedPtr deferred_init_timer_;

    // ── 初始化 ────────────────────────────────────────────────────────────
    void setupModules();
    /** 延后执行：在首次 spin 后调用，内部使用 shared_from_this()，避免 Composable 构造时 bad_weak_ptr */
    void deferredSetupModules();
    void setupPublishers();
    void setupServices();
    void setupTimers();
    void loadConfigAndInit();

    // ── 数据流回调 ────────────────────────────────────────────────────────
    void onOdometry(double ts, const Pose3d& pose, const Mat66d& cov);
    void onCloud(double ts, const CloudXYZIPtr& cloud);
    void onKFInfo(const LivoKeyFrameInfo& info);
    void onGPS(double ts, double lat, double lon, double alt, double hdop, int sats);

    // ── 内部处理 ──────────────────────────────────────────────────────────
    /** 从当前 last_* 读取并创建 KF（兼容旧调用） */
    void tryCreateKeyFrame(double ts);
    /** 使用入参创建 KF；optional_livo_info 非空时用于日志（时间戳对齐），否则读 last_livo_info_ */
    void tryCreateKeyFrame(double ts, const Pose3d& pose, const Mat66d& cov, const CloudXYZIPtr& cloud,
                           const LivoKeyFrameInfo* optional_livo_info = nullptr);
    /** 将世界系点云转为 body 系（T_b_w * cloud），用于 frontend.cloud_frame=world 时避免全局图双重变换 */
    CloudXYZIPtr transformWorldToBody(const CloudXYZIPtr& world_cloud, const Pose3d& T_w_b) const;
    void onSubmapFrozen(const SubMap::Ptr& submap);
    void onLoopDetected(const LoopConstraint::Ptr& lc);
    void onPoseUpdated(const std::unordered_map<int, Pose3d>& poses);
    void onHBADone(const HBAResult& result);
    void onGPSAligned(const GPSAlignResult& result);
    void addBatchGPSFactors();

    // ── 服务处理 ──────────────────────────────────────────────────────────
    void handleSaveMap(
        const std::shared_ptr<automap_pro::srv::SaveMap::Request>,
        std::shared_ptr<automap_pro::srv::SaveMap::Response>);
    void handleGetStatus(
        const std::shared_ptr<automap_pro::srv::GetStatus::Request>,
        std::shared_ptr<automap_pro::srv::GetStatus::Response>);
    void handleTriggerHBA(
        const std::shared_ptr<automap_pro::srv::TriggerHBA::Request>,
        std::shared_ptr<automap_pro::srv::TriggerHBA::Response>);
    void handleTriggerOptimize(
        const std::shared_ptr<automap_pro::srv::TriggerOptimize::Request>,
        std::shared_ptr<automap_pro::srv::TriggerOptimize::Response>);
    void handleTriggerGpsAlign(
        const std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Request>,
        std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Response>);
    void handleLoadSession(
        const std::shared_ptr<automap_pro::srv::LoadSession::Request>,
        std::shared_ptr<automap_pro::srv::LoadSession::Response>);

    // ── 定时任务 ──────────────────────────────────────────────────────────
    void publishStatus();
    void publishGlobalMap();
    void publishDataFlowSummary();  // 低频：各模块收发/发布汇总

    // ── 轨迹对比记录（每帧位姿 + GPS，便于脚本绘图分析建图精度）────────────────
    bool trajectory_log_enabled_ = true;
    std::string trajectory_log_dir_;
    std::ofstream trajectory_odom_file_;
    std::ofstream trajectory_gps_file_;
    std::mutex trajectory_log_mutex_;
    std::string trajectory_session_id_;  // 本次会话文件名后缀
    void ensureTrajectoryLogDir();
    void writeTrajectoryOdom(double ts, const Pose3d& pose, const Mat66d& cov);
    void onGPSMeasurementForLog(double ts, const Eigen::Vector3d& pos_enu);

    // ── 工具 ─────────────────────────────────────────────────────────────
    /** 返回实际输出目录：launch 传入的 output_dir 优先，否则用 system.output_dir */
    std::string getOutputDir() const;
    void saveMapToFiles(const std::string& output_dir);
    std::string stateToString(SystemState s) const;
    Mat66d computeOdomInfoMatrix(const SubMap::Ptr& prev,
                                 const SubMap::Ptr& curr,
                                 const Pose3d& rel) const;
};

} // namespace automap_pro

RCLCPP_COMPONENTS_REGISTER_NODE(automap_pro::AutoMapSystem)
