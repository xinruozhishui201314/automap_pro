#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/backend/hba_optimizer.h"

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

namespace automap_pro {

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

    // ── 状态 ─────────────────────────────────────────────────────────────
    std::atomic<SystemState> state_{SystemState::IDLE};
    uint64_t current_session_id_ = 0;
    std::mutex state_mutex_;

    // 当前里程计状态缓存
    Pose3d    last_odom_pose_    = Pose3d::Identity();
    double    last_odom_ts_      = -1.0;
    CloudXYZIPtr last_cloud_;
    Mat66d    last_cov_          = Mat66d::Identity() * 1e-4;
    mutable std::mutex data_mutex_;

    // ESIKF 质量
    LivoKeyFrameInfo last_livo_info_;

    // GPS 对齐状态
    bool gps_aligned_        = false;
    bool gps_batch_added_    = false;  // 对齐后是否已批量添加GPS因子

    // 子图计数（用于 HBA 周期触发）
    int  frozen_submap_count_ = 0;

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

    // ── 定时器 ────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

    // ── 初始化 ────────────────────────────────────────────────────────────
    void setupModules();
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
    void tryCreateKeyFrame(double ts);
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

    // ── 工具 ─────────────────────────────────────────────────────────────
    void saveMapToFiles(const std::string& output_dir);
    std::string stateToString(SystemState s) const;
};

} // namespace automap_pro

RCLCPP_COMPONENTS_REGISTER_NODE(automap_pro::AutoMapSystem)
