#pragma once
/**
 * @file system/automap_system.h
 * @brief 系统节点：Composable 主控、帧处理、位姿事务。
 */


#include "automap_pro/core/frame_types.h"
#include "automap_pro/core/data_types.h"
#include "automap_pro/sensor/attitude_estimator.h"
#include "automap_pro/v3/v3_context.h"
#include "automap_pro/v3/optimizer_module.h"
#include "automap_pro/v3/gps_module.h"
#include "automap_pro/v3/loop_module.h"
#include "automap_pro/v3/visualization_module.h"
#include "automap_pro/v3/frontend_module.h"
#include "automap_pro/v3/semantic_module.h"
#include "automap_pro/v3/dynamic_filter_module.h"
#include "automap_pro/v3/mapping_module.h"
#include "automap_pro/v3/map_orchestrator.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

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
#include <chrono>
#include <fstream>

namespace automap_pro {

/**
 * AutoMapSystem —— 主控制层（Composable Node）
 * 
 * 职责：
 * 1. 作为微内核的启动器 (Kernel Launcher)
 * 2. 负责 ROS 话题到 EventBus 的“硬翻译” (Hard Translation)
 * 3. 维护全局服务接口 (SaveMap, GetStatus, etc.)
 */
class AutoMapSystem : public rclcpp::Node {
public:
    explicit AutoMapSystem(const rclcpp::NodeOptions& options);
    ~AutoMapSystem();

private:
    // ── V3: 核心架构 ────────────────────────────────────────────────────────
    v3::V3Context::Ptr v3_context_;
    std::shared_ptr<v3::OptimizerModule> optimizer_module_;
    std::shared_ptr<v3::GPSModule> gps_module_;
    std::shared_ptr<v3::LoopModule> loop_module_;
    std::shared_ptr<v3::VisualizationModule> viz_module_;
    std::shared_ptr<v3::FrontEndModule> frontend_module_;
    std::shared_ptr<v3::SemanticModule> semantic_module_;
    std::shared_ptr<v3::DynamicFilterModule> dynamic_filter_module_;
    std::shared_ptr<v3::MapOrchestrator> map_orchestrator_;
    std::shared_ptr<v3::MappingModule> mapping_module_;

    // ── 状态与生命周期 ──────────────────────────────────────────────────────
    std::atomic<SystemState> state_{SystemState::IDLE};
    uint64_t current_session_id_ = 0;
    std::mutex state_mutex_;
    std::atomic<bool> shutdown_requested_{false};
    float map_voxel_size_ = 0.2f;
    std::string output_dir_override_;

    // ── 发布者 (面向外部系统) ───────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      global_map_pub_;
    rclcpp::Publisher<automap_pro::msg::MappingStatusMsg>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                ready_pub_;

    // ── 服务接口 ────────────────────────────────────────────────────────────
    rclcpp::Service<automap_pro::srv::SaveMap>::SharedPtr        save_map_srv_;
    rclcpp::Service<automap_pro::srv::GetStatus>::SharedPtr      get_status_srv_;
    rclcpp::Service<automap_pro::srv::TriggerHBA>::SharedPtr     trigger_hba_srv_;
    rclcpp::Service<automap_pro::srv::TriggerOptimize>::SharedPtr trigger_opt_srv_;
    rclcpp::Service<automap_pro::srv::TriggerGpsAlign>::SharedPtr trigger_gps_srv_;
    rclcpp::Service<automap_pro::srv::LoadSession>::SharedPtr     load_session_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr           finish_mapping_srv_;

    rclcpp::TimerBase::SharedPtr deferred_init_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    // ── 初始化 ────────────────────────────────────────────────────────────
    void setupModules();
    void deferredSetupModules();
    void setupPublishers();
    void setupServices();
    void setupTimers();
    void loadConfigAndInit();

    // ── 数据流回调 (Bridge to EventBus) ─────────────────────────────────────
    void onOdometry(double ts, const Pose3d& pose, const Mat66d& cov);
    void onCloud(double ts, const CloudXYZIPtr& cloud);
    void onKFInfo(const LivoKeyFrameInfo& info);
    void onGPS(double ts, double lat, double lon, double alt, double hdop, int sats);

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

    // ── 轨迹对比记录 ────────────────
    bool trajectory_log_enabled_ = true;
    std::string trajectory_log_dir_;
    std::mutex trajectory_log_mutex_;
    void writeTrajectoryOdomAfterMapping(const std::string& output_dir);
    void writeMappingAccuracyGpsVsHba(const std::string& output_dir);

    // ── 工具 ─────────────────────────────────────────────────────────────
    std::string getOutputDir() const;
    void saveMapToFiles(const std::string& output_dir);
    std::string stateToString(SystemState s) const;
    
    // 统计用
    std::atomic<int> pub_status_count_{0};
    std::atomic<int> pub_map_count_{0};
    int status_publish_count_ = 0;
    std::atomic<bool> finish_mapping_in_progress_{false};
    std::atomic<bool> finish_mapping_requested_{false};
};

} // namespace automap_pro
