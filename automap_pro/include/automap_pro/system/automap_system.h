#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/sensor/attitude_estimator.h"
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/backend/delayed_gps_compensator.h"
#include "automap_pro/backend/hba_optimizer.h"
#include "automap_pro/visualization/rviz_publisher.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

// 本包消息类型
#include <automap_pro/msg/mapping_status_msg.hpp>
// 服务/Action 类型
#include <automap_pro/srv/save_map.hpp>
#include <automap_pro/srv/get_status.hpp>
#include <automap_pro/srv/trigger_hba.hpp>
#include <automap_pro/srv/trigger_optimize.hpp>
#include <automap_pro/srv/trigger_gps_align.hpp>
#include <automap_pro/srv/load_session.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <deque>
#include <condition_variable>
#include <chrono>
#include <fstream>

namespace automap_pro {

/** 按时间戳对齐：帧队列只存 (ts, cloud)，pose/cov/kfinfo 由 worker 从缓存按 ts 查询；可选 cloud_ds 由 feeder 预计算以省去 worker 内体素 */
struct FrameToProcess {
    double ts = 0.0;
    CloudXYZIPtr cloud;
    CloudXYZIPtr cloud_ds;  // 可选：feeder 预计算体素降采样，worker 有则直接用
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
    std::unique_ptr<DelayedGPSCompensator> gps_compensator_;
    HBAOptimizer          hba_optimizer_;
    RvizPublisher         rviz_publisher_;

    // GPS 姿态估计（IMU pitch/roll + GPS 航迹角 yaw）
    std::shared_ptr<AttitudeEstimator> attitude_estimator_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_for_attitude_;

    // 回环约束缓存（用于可视化）
    std::vector<LoopConstraint::Ptr> loop_constraints_;
    mutable std::mutex               loop_constraints_mutex_;

    // ── 状态 ─────────────────────────────────────────────────────────────
    std::atomic<SystemState> state_{SystemState::IDLE};
    uint64_t current_session_id_ = 0;
    std::mutex state_mutex_;
    
    // 异步任务关闭请求标志
    std::atomic<bool> shutdown_requested_{false};

    // 入口缓冲：订阅回调只写入 ingress，快速返回；feeder 线程将 ingress → frame_queue_，背压在 feeder 内，不阻塞 Executor
    size_t max_ingress_queue_size_ = 16;
    std::queue<FrameToProcess> ingress_queue_;
    std::mutex                 ingress_mutex_;
    std::condition_variable    ingress_not_full_cv_;   // feeder 取走一帧时唤醒回调
    std::condition_variable    ingress_not_empty_cv_;  // 回调放入一帧时唤醒 feeder
    std::thread                feeder_thread_;
    void feederLoop();

    // 后端帧队列：只存 (ts, cloud)，worker 按 ts 从 odom/kfinfo 缓存对齐；长度由配置 frame_queue_max_size 决定
    // 队列满时背压在 feeder 线程内等待，不丢帧
    size_t max_frame_queue_size_ = 500;
    std::queue<FrameToProcess> frame_queue_;
    std::mutex                frame_queue_mutex_;
    std::condition_variable   frame_queue_cv_;           // 有数据时唤醒 worker
    std::condition_variable   frame_queue_not_full_cv_; // 有空间时唤醒 feeder（背压）
    std::thread               backend_worker_;
    std::atomic<int>          frame_queue_dropped_{0};  // 背压模式下恒为 0
    std::atomic<int>          backpressure_force_drop_count_{0};  // 背压超限强制丢帧次数（可观测）
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

    // 地图发布异步化：专用线程执行 buildGlobalMap + publish。拆锁避免背压死锁：关键帧创建与地图发布分离。
    // keyframe_mutex_：保护 tryCreateKeyFrame 内对 submap_manager_/kf_manager_/isam2 的写操作（backend 持锁）。
    // map_publish_mutex_：仅保护 map_publish 线程的 cv 等待与 publishGlobalMap 调用（只读 submap 由 SubMapManager 自身锁保护）。
    std::thread               map_publish_thread_;
    std::mutex                keyframe_mutex_;
    std::mutex                map_publish_mutex_;
    std::condition_variable   map_publish_cv_;
    std::atomic<bool>         map_publish_pending_{false};
    /** 从 finish_mapping 进入到 HBA 回调结束期间为 true，避免 map_publish 与 HBA 写回并发导致重影（见 docs/GHOSTING_ROOT_CAUSE_HBA_VS_BACKEND_20260317.md） */
    std::atomic<bool>         finish_mapping_in_progress_{false};
    void mapPublishLoop();

    // 回环 iSAM2 更新异步：match_worker 只入队，本线程取任务执行 addLoopFactor，避免阻塞回环检测（有界队列防堆积/死锁）
    static constexpr size_t   kMaxLoopFactorQueueSize = 64;
    std::queue<LoopConstraint::Ptr> loop_factor_queue_;
    std::mutex                loop_opt_mutex_;
    std::condition_variable   loop_opt_cv_;
    std::thread               loop_opt_thread_;
    void loopOptThreadLoop();

    // 可视化与状态发布迁出后端：仅投递不阻塞
    static constexpr size_t   kVizQueueMaxSize = 2;
    std::queue<CloudXYZIPtr>  viz_cloud_queue_;
    std::mutex                viz_mutex_;
    std::condition_variable   viz_cv_;
    std::thread               viz_thread_;
    void vizThreadLoop();

    std::atomic<bool>         status_publish_pending_{false};
    std::atomic<bool>         data_flow_publish_pending_{false};
    std::mutex                status_pub_mutex_;
    std::condition_variable   status_pub_cv_;
    std::thread               status_publisher_thread_;
    void statusPublisherLoop();

    // 地图体素大小（init 时从 ConfigManager 缓存，避免 publishGlobalMap 回调中访问单例导致析构顺序 SIGSEGV）
    float map_voxel_size_ = 0.2f;
    /** HBA 完成后构建一次的全局图缓存，供发布与保存共用，避免 save 与 map_publish 两路 build 导致 PCD 重影（见 docs/PCD_GHOSTING_VS_RVIZ_ANALYSIS_20260317_2137.md） */
    CloudXYZIPtr last_hba_global_map_;
    std::mutex    last_hba_global_map_mutex_;
    // launch 传入的 output_dir，非空时优先于 system.output_dir，使前后端保存到同一目录
    std::string output_dir_override_;
    // GPS 配置：在 loadConfigAndInit 中一次性读取并传入 LivoBridge，避免 LivoBridge 再读 ConfigManager 时不一致
    bool        gps_enabled_from_config_ = false;
    std::string gps_topic_from_config_   = "/gps/fix";

    // 健康检查降级：>0 时 backend 使用此值替代 config 的 process_every_n_frames，减轻负载
    std::atomic<int> process_every_n_override_{0};

    // 首次数据到达日志（各打一次，便于确认数据流）
    std::atomic<bool> first_odom_logged_{false};
    std::atomic<bool> first_cloud_logged_{false};
    // 传感器空闲结束建图：上次收到点云的墙钟时间；超时后触发最终处理并退出
    std::chrono::steady_clock::time_point last_sensor_data_wall_time_{std::chrono::steady_clock::now()};
    std::atomic<bool> sensor_idle_finish_triggered_{false};
    /** 是否已因「10s 无前端数据且后端处理完」触发过 HBA（仅触发一次，避免重复） */
    std::atomic<bool> hba_triggered_by_frontend_idle_{false};
    // 后端已从队列弹出的点云帧计数（用于 DATA_FLOW 与帧率控制）
    std::atomic<int> backend_cloud_frames_processed_{0};
    // 后端实际参与 tryCreateKeyFrame 的帧计数（process_every_n_frames 跳帧时与上面不同，用于发布周期）
    std::atomic<int> backend_frames_actually_processed_{0};
    // 周期性状态汇总（每 10 次 status 打一条，约 10s）
    int status_publish_count_ = 0;
    // 低频数据流日志：各模块收发/发布计数（每 15s 打一条）
    std::atomic<int> pub_odom_path_count_{0};
    std::atomic<int> pub_opt_path_count_{0};
    std::atomic<int> pub_map_count_{0};
    std::atomic<int> pub_status_count_{0};
    /** 是否已打过一次「odom_path 与 global_map 不同系、同屏重影」的 WARN（仅打一次，见 HBA_GHOSTING_ANALYSIS_RUN_20260317_173943） */
    bool odom_path_ghosting_warned_{false};
    /** HBA 完成后置 true：不再向 odom_path 追加、并已发布空 Path 清空 RViz 显示，避免与 global_map 同屏重影 */
    std::atomic<bool> odom_path_stopped_after_hba_{false};

    // ── V2: 线程心跳监控 ─────────────────────────────────────────────────────
    // 各关键线程上次心跳时间戳（墙钟），用于检测线程是否卡住
    std::atomic<int64_t> feeder_heartbeat_ts_ms_{0};       // feeder 线程心跳
    std::atomic<int64_t> backend_heartbeat_ts_ms_{0};      // backend worker 线程心跳
    std::atomic<int64_t> map_pub_heartbeat_ts_ms_{0};      // map publish 线程心跳
    std::atomic<int64_t> loop_opt_heartbeat_ts_ms_{0};     // loop optimization 线程心跳
    std::atomic<int64_t> viz_heartbeat_ts_ms_{0};          // visualization 线程心跳
    std::atomic<int64_t> status_pub_heartbeat_ts_ms_{0};   // status publisher 线程心跳
    
    // 心跳阈值（毫秒）：超过此时间未更新视为异常
    static constexpr int64_t kHeartbeatWarnThresholdMs = 10000;   // 10秒未心跳警告
    static constexpr int64_t kHeartbeatErrorThresholdMs = 30000;  // 30秒未心跳报错

    /** 后端当前步骤 ID（卡住时 HEARTBEAT 会打印 last_backend_step，便于精准定位） */
    enum BackendStepId : int {
        BACKEND_STEP_IDLE = 0,
        BACKEND_STEP_TRY_CREATE_KF_ENTER,
        BACKEND_STEP_ADD_KEYFRAME_ENTER,
        BACKEND_STEP_INTRA_LOOP_ENTER,
        BACKEND_STEP_GPS_FACTOR_ENTER,
        BACKEND_STEP_FORCE_UPDATE,
        BACKEND_STEP_COUNT
    };
    std::atomic<int> last_backend_step_id_{BACKEND_STEP_IDLE};
    /** 将 BackendStepId 转为可读字符串，供 STUCK_DIAG/心跳日志使用 */
    const char* backendStepName(int id) const;

    // 心跳监控定时器
    rclcpp::TimerBase::SharedPtr heartbeat_monitor_timer_;
    void checkThreadHeartbeats();  // 检查各线程心跳状态

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
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr           finish_mapping_srv_;

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
    /** 使用入参创建 KF；optional_livo_info 非空时用于日志；optional_cloud_ds 非空且非空点云时跳过体素降采样（feeder 预计算） */
    void tryCreateKeyFrame(double ts, const Pose3d& pose, const Mat66d& cov, const CloudXYZIPtr& cloud,
                           const LivoKeyFrameInfo* optional_livo_info = nullptr,
                           const CloudXYZIPtr* optional_cloud_ds = nullptr);
    /** 将世界系点云转为 body 系（T_b_w * cloud），用于 frontend.cloud_frame=world 时避免全局图双重变换 */
    CloudXYZIPtr transformWorldToBody(const CloudXYZIPtr& world_cloud, const Pose3d& T_w_b) const;
    void onSubmapFrozen(const SubMap::Ptr& submap);
    void onLoopDetected(const LoopConstraint::Ptr& lc);
    void onPoseUpdated(const std::unordered_map<int, Pose3d>& poses);
    void onHBADone(const HBAResult& result);
    void onGPSAligned(const GPSAlignResult& result);
    void addBatchGPSFactors();

    /** 后端优化完成后再触发 HBA：等待 ISAM2 队列空并 flush pending，释放 GTSAM 相关变量，避免与 HBA 的 GTSAM 竞争崩溃。调用方在 triggerAsync 前必须调用。 */
    void ensureBackendCompletedAndFlushBeforeHBA();

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
    void handleFinishMapping(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response>);

    // ── 定时任务 ──────────────────────────────────────────────────────────
    void publishStatus();
    void publishGlobalMap();
    void publishDataFlowSummary();  // 低频：各模块收发/发布汇总

    // ── 轨迹对比记录（每帧位姿 + GPS，便于脚本绘图分析建图精度）────────────────
    bool trajectory_log_enabled_ = true;
    /** true=仅建图完成后写 trajectory_odom CSV（关键帧+最终GPS）；false=边建图边写（调试用，轨迹与GPS可能不重合） */
    bool trajectory_log_after_mapping_only_ = true;
    /** true=HBA 优化完成后自动启动 VTK 轨迹查看器并显示两条曲线与偏差曲线 */
    bool vtk_viewer_after_hba_ = true;
    std::string trajectory_log_dir_;
    std::ofstream trajectory_odom_file_;
    std::ofstream trajectory_gps_file_;
    std::mutex trajectory_log_mutex_;
    std::string trajectory_session_id_;  // 本次会话文件名后缀
    void ensureTrajectoryLogDir();
    void writeTrajectoryOdom(double ts, const Pose3d& pose, const Mat66d& cov);
    /** 建图完成后写：关键帧位姿 + 最终地图系下 GPS，保证轨迹与 GPS 在同一坐标系下可对比 */
    void writeTrajectoryOdomAfterMapping(const std::string& output_dir);
    /** HBA 完成后写：HBA 优化后的关键帧位姿 + GPS（map/ENU），便于建图精度分析（trajectory_hba_poses_*.csv） */
    void writeHbaPosesAndGpsForAccuracy();
    void onGPSMeasurementForLog(double ts, const Eigen::Vector3d& pos_enu);

    // ── 工具 ─────────────────────────────────────────────────────────────
    /** 从子图列表收集所有关键帧（按时间戳排序），用于 RViz 关键帧位姿实时刷新 */
    static std::vector<KeyFrame::Ptr> collectKeyframesFromSubmaps(const std::vector<SubMap::Ptr>& submaps);
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
