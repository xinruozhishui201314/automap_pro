#include <rclcpp/rclcpp.hpp>

#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/data_types.h"
#include "automap_pro/core/utils.h"

#include "automap_pro/sensor/sensor_manager.h"
#include "automap_pro/frontend/fast_livo2_wrapper.h"
#include "automap_pro/frontend/fast_livo2_adapter.h"
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/submap/session_manager.h"
#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/backend/hba_wrapper.h"
#include "automap_pro/map/map_builder.h"
#include "automap_pro/map/map_filter.h"
#include "automap_pro/map/map_exporter.h"
#include "automap_pro/visualization/rviz_publisher.h"

#include <automap_pro/srv/save_map.hpp>
#include <automap_pro/srv/trigger_optimize.hpp>
#include <automap_pro/srv/trigger_hba.hpp>
#include <automap_pro/srv/get_status.hpp>
#include <automap_pro/srv/load_session.hpp>
#include "automap_pro/backend/hba_bridge.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

namespace automap_pro {

class AutoMapSystem {
public:
    AutoMapSystem() = default;
    ~AutoMapSystem() { shutdown(); }

    bool init(rclcpp::Node::SharedPtr node) {
        node_ = node;

        // 1. Load config
        std::string config_path;
        node->declare_parameter<std::string>("config", "");
        node->get_parameter("config", config_path);
        if (config_path.empty()) {
            try {
                config_path = ament_index_cpp::get_package_share_directory("automap_pro") + "/config/system_config.yaml";
            } catch (const std::exception&) {
                config_path = "config/system_config.yaml";
            }
        }
        if (!utils::fileExists(config_path)) {
            RCLCPP_WARN(node->get_logger(), "[AutoMapSystem] Config not found at %s, using defaults.", config_path.c_str());
        } else {
            ConfigManager::instance().loadFromFile(config_path);
        }

        const std::string ll = ConfigManager::instance().logLevel();
        if      (ll == "debug") utils::setLogLevel(utils::LogLevel::DEBUG);
        else if (ll == "warn")  utils::setLogLevel(utils::LogLevel::WARN);
        else if (ll == "error") utils::setLogLevel(utils::LogLevel::ERROR);
        else                    utils::setLogLevel(utils::LogLevel::INFO);

        const auto& cfg = ConfigManager::instance();
        std::string output_dir = cfg.outputDir();
        utils::createDirectories(output_dir);

        int session_id = session_mgr_.startNewSession(output_dir);

        sensor_mgr_.init(node);
        submap_mgr_.init(node, session_id);
        loop_detector_.init(node);
        hba_.init(node);
        rviz_.init(node);

        const std::string frontend_mode = cfg.frontendMode();
        if (frontend_mode == "external_fast_livo") {
            adapter_ = std::make_unique<FastLIVO2Adapter>();
            adapter_->init(node);
            adapter_->setSessionId(static_cast<uint64_t>(session_id));
            adapter_->registerKeyFrameCallback([this](const KeyFrame::Ptr& kf) { onNewKeyFrame(kf); });
            adapter_->registerPoseCallback([this](double ts, const Pose3d& pose, const Mat66d&) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                last_pose_ = pose;
                last_pose_time_ = ts;
            });
            RCLCPP_INFO(node->get_logger(), "[AutoMapSystem] Using external frontend: fast-livo2-humble");
        } else {
            frontend_.init(node, sensor_mgr_.imuBuffer(), sensor_mgr_.gpsBuffer());
            sensor_mgr_.imu().registerCallback([this](const ImuData& imu) { frontend_.feedImu(imu); });
            sensor_mgr_.gps().registerCallback([this](const GPSMeasurement& gps) {
                frontend_.feedGPS(gps);
                std::lock_guard<std::mutex> lk(pose_mutex_);
                sensor_mgr_.gps().updateOdometryPose(gps.timestamp, last_pose_.translation());
            });
            sensor_mgr_.lidar().registerCallback([this](const LidarFrame::Ptr& frame) { frontend_.feedLidar(frame); });
            frontend_.registerKeyFrameCallback([this](const KeyFrame::Ptr& kf) { onNewKeyFrame(kf); });
            frontend_.registerPoseCallback([this](double ts, const Pose3d& pose, const Mat66d&) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                last_pose_ = pose;
                last_pose_time_ = ts;
            });
        }

        submap_mgr_.registerFrozenCallback([this](const SubMap::Ptr& sm) {
            onSubmapFrozen(sm);
        });

        loop_detector_.registerCallback([this](const LoopConstraint::Ptr& lc) {
            onLoopConstraint(lc);
        });

        hba_.registerPoseUpdateCallback([this](int sm_id, const Pose3d& pose) {
            onPoseUpdate(sm_id, pose);
        });

        hba_.registerDoneCallback([this]() {
            onOptimizationDone();
        });

        save_map_srv_ = node->create_service<automap_pro::srv::SaveMap>(
            "/automap/save_map",
            std::bind(&AutoMapSystem::onSaveMap, this, std::placeholders::_1, std::placeholders::_2));
        trigger_opt_srv_ = node->create_service<automap_pro::srv::TriggerOptimize>(
            "/automap/trigger_optimize",
            std::bind(&AutoMapSystem::onTriggerOptimize, this, std::placeholders::_1, std::placeholders::_2));
        get_status_srv_ = node->create_service<automap_pro::srv::GetStatus>(
            "/automap/get_status",
            std::bind(&AutoMapSystem::onGetStatus, this, std::placeholders::_1, std::placeholders::_2));
        load_session_srv_ = node->create_service<automap_pro::srv::LoadSession>(
            "/automap/load_session",
            std::bind(&AutoMapSystem::onLoadSession, this, std::placeholders::_1, std::placeholders::_2));
        trigger_hba_srv_ = node->create_service<automap_pro::srv::TriggerHBA>(
            "/automap/trigger_hba",
            std::bind(&AutoMapSystem::onTriggerHBA, this, std::placeholders::_1, std::placeholders::_2));

        loop_detector_.start();
        hba_.start();

        double vis_rate = cfg.visPublishRate();
        if (vis_rate > 0.0) {
            vis_timer_ = node->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / vis_rate)),
                std::bind(&AutoMapSystem::onVisTimer, this));
        }

        state_ = SystemState::MAPPING;
        RCLCPP_INFO(node->get_logger(), "[AutoMapSystem] *** AutoMap-Pro started (mode=%s) ***", cfg.mode().c_str());
        return true;
    }

    void shutdown() {
        if (state_ == SystemState::IDLE) return;
        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[AutoMapSystem] Shutting down...");
        state_ = SystemState::SAVING;

        submap_mgr_.checkAndSplitSubmap();

        if (ConfigManager::instance().hbaTriggerOnFinish()) {
            hba_.triggerOptimization(true);
            auto start = std::chrono::steady_clock::now();
            while (hba_.isOptimizing()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > 60.0) break;
            }
        }

        auto global_cloud = map_builder_.buildGlobalMap(submap_mgr_.allSubmaps());
        auto filtered     = map_filter_.applyAll(global_cloud);

        MapExporter::ExportOptions opts;
        opts.output_dir  = ConfigManager::instance().outputDir();
        opts.save_pcd    = ConfigManager::instance().saveFormatPCD();
        opts.save_ply    = ConfigManager::instance().saveFormatPLY();
        opts.save_las    = ConfigManager::instance().saveFormatLAS();
        opts.tiling      = ConfigManager::instance().mapTilingEnabled();
        opts.tile_size   = ConfigManager::instance().mapTileSize();
        opts.save_traj_tum = opts.save_traj_kitti = opts.save_poses_json = true;

        for (const auto& sm : submap_mgr_.allSubmaps()) {
            submap_mgr_.archiveSubmap(sm->id, opts.output_dir + "/submaps");
        }

        PoseGraph shutdown_pg;
        map_exporter_.exportAll(filtered, submap_mgr_.allSubmaps(),
                                 loop_constraints_, shutdown_pg, opts);

        session_mgr_.saveDescriptorDB(
            opts.output_dir + "/descriptor_db.json", submap_mgr_.allSubmaps());
        session_mgr_.saveSession(
            session_mgr_.currentSessionId(), opts.output_dir, submap_mgr_.allSubmaps());

        state_ = SystemState::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[AutoMapSystem] Shutdown complete.");
    }

private:
    void onNewKeyFrame(const KeyFrame::Ptr& kf) {
        submap_mgr_.addKeyFrame(kf);
        {
            std::lock_guard<std::mutex> lk(kf_mutex_);
            if (!last_kf_) {
                hba_.addKeyFrameNode(kf->id, kf->T_w_b, true);
            } else {
                hba_.addKeyFrameNode(kf->id, kf->T_w_b);
                Pose3d rel = last_kf_->T_w_b.inverse() * kf->T_w_b;
                Mat66d info = kf->covariance.inverse();
                hba_.addOdomFactor(last_kf_->id, kf->id, rel, info);
            }
            last_kf_ = kf;
        }
        if (kf->has_valid_gps) {
            Eigen::Matrix3d gps_cov = kf->gps.covariance;
            hba_.addGPSFactor(kf->id, kf->gps.position_enu, gps_cov);
        }
        submap_mgr_.checkAndSplitSubmap();
    }

    void onSubmapFrozen(const SubMap::Ptr& sm) {
        loop_detector_.addSubmap(sm);
        hba_.addSubmapNode(sm->id, sm->pose_w_anchor);
        auto all = submap_mgr_.allSubmaps();
        for (const auto& prev : all) {
            if (prev->id == sm->id - 1 && prev->session_id == sm->session_id) {
                Pose3d rel = prev->pose_w_anchor.inverse() * sm->pose_w_anchor;
                hba_.addSubmapOdomFactor(prev->id, sm->id, rel, utils::poseCovariance6d(0.1, 0.01).inverse());
                break;
            }
        }
        if (sm->id > 0 && sm->id % ConfigManager::instance().hbaTriggerPeriodicSubmaps() == 0) {
            hba_.triggerOptimization(false);
        }
    }

    void onLoopConstraint(const LoopConstraint::Ptr& lc) {
        std::lock_guard<std::mutex> lk(loop_mutex_);
        loop_constraints_.push_back(lc);
        hba_.addLoopFactor(lc);
    }

    void onPoseUpdate(int sm_id, const Pose3d& pose) {
        submap_mgr_.updateSubmapPose(sm_id, pose);
    }

    void onOptimizationDone() {
        auto all_submaps = submap_mgr_.allSubmaps();
        auto global = map_builder_.buildGlobalMap(all_submaps);
        rviz_.publishGlobalMap(global);
        rviz_.publishOptimizedPath(all_submaps);
        rviz_.publishLoopMarkers(loop_constraints_, all_submaps);
    }

    void onVisTimer() {
        auto all_submaps = submap_mgr_.allSubmaps();
        if (ConfigManager::instance().visShowSubmapBoundaries()) {
            rviz_.publishSubmapBoundaries(all_submaps);
        }
        if (ConfigManager::instance().visShowGPSTrajectory()) {
            rviz_.publishGPSMarkers(all_submaps);
        }
    }

    void onSaveMap(
        const std::shared_ptr<automap_pro::srv::SaveMap::Request> req,
        std::shared_ptr<automap_pro::srv::SaveMap::Response> res) {
        RCLCPP_INFO(node_->get_logger(), "[AutoMapSystem] SaveMap requested to %s", req->output_dir.c_str());
        state_ = SystemState::SAVING;

        auto global = map_builder_.buildGlobalMap(submap_mgr_.allSubmaps());
        auto filtered = map_filter_.applyAll(global);

        MapExporter::ExportOptions opts;
        opts.output_dir = req->output_dir.empty() ? ConfigManager::instance().outputDir() : req->output_dir;
        opts.save_pcd   = req->save_pcd;
        opts.save_ply   = req->save_ply;
        opts.save_las   = req->save_las;
        opts.save_traj_tum = opts.save_traj_kitti = opts.save_poses_json = req->save_trajectory;

        PoseGraph empty_graph;
        bool ok = map_exporter_.exportAll(filtered, submap_mgr_.allSubmaps(),
                                           loop_constraints_, empty_graph, opts);
        res->success      = ok;
        res->output_path  = opts.output_dir;
        res->message      = ok ? "Map saved." : "Failed to save map.";
        state_ = SystemState::MAPPING;
    }

    void onTriggerOptimize(
        const std::shared_ptr<automap_pro::srv::TriggerOptimize::Request> req,
        std::shared_ptr<automap_pro::srv::TriggerOptimize::Response> res) {
        RCLCPP_INFO(node_->get_logger(), "[AutoMapSystem] TriggerOptimize (full=%d)", req->full_optimization);
        hba_.triggerOptimization(req->full_optimization);
        res->success             = true;
        res->message             = "Optimization triggered.";
        res->optimization_time_ms = 0.0;
        res->num_nodes            = 0;
        res->num_edges            = 0;
    }

    void onGetStatus(
        const std::shared_ptr<automap_pro::srv::GetStatus::Request>,
        std::shared_ptr<automap_pro::srv::GetStatus::Response> res) {
        res->num_keyframes        = submap_mgr_.numKeyFrames();
        res->num_submaps          = submap_mgr_.numSubmaps();
        res->num_loop_constraints = static_cast<int>(loop_constraints_.size());
        res->is_mapping           = (state_ == SystemState::MAPPING);
        res->gps_state            = gpsStateToString(sensor_mgr_.gps().currentState());
        res->frontend_rate_hz     = 0.0;  // optional: expose from frontend if available
        res->memory_usage_mb      = 0.0;  // optional: measure process memory if needed

        switch (state_) {
            case SystemState::MAPPING:    res->system_state = "MAPPING";    break;
            case SystemState::OPTIMIZING: res->system_state = "OPTIMIZING"; break;
            case SystemState::SAVING:     res->system_state = "SAVING";     break;
            case SystemState::IDLE:       res->system_state = "IDLE";       break;
            default:                      res->system_state = "UNKNOWN";
        }
    }

    void onLoadSession(
        const std::shared_ptr<automap_pro::srv::LoadSession::Request> req,
        std::shared_ptr<automap_pro::srv::LoadSession::Response> res) {
        RCLCPP_INFO(node_->get_logger(), "[AutoMapSystem] LoadSession %s (id=%d)", req->session_dir.c_str(), req->session_id);

        std::vector<SubMap::Ptr> hist_submaps;
        bool ok = session_mgr_.loadDescriptorDB(
            req->session_dir + "/descriptor_db.json", hist_submaps);

        if (ok) {
            for (const auto& sm : hist_submaps) {
                loop_detector_.addToDatabase(sm);
            }
            res->success = true;
            res->message = "Session loaded.";
            res->num_submaps_loaded = static_cast<int>(hist_submaps.size());
        } else {
            res->success = false;
            res->message = "Failed to load session.";
        }
    }

    void onTriggerHBA(
        const std::shared_ptr<automap_pro::srv::TriggerHBA::Request> req,
        std::shared_ptr<automap_pro::srv::TriggerHBA::Response> res) {
        const auto& cfg = ConfigManager::instance();
        std::string data_path = req->data_path.empty() ? cfg.hbaBridgeExportPath() : req->data_path;
        if (data_path.back() != '/') data_path += "/";

        auto all = submap_mgr_.allSubmaps();
        std::vector<KeyFrame::Ptr> keyframes = HBABridge::collectKeyframesInOrder(all);
        if (keyframes.empty()) {
            res->success = false;
            res->message = "No keyframes to export.";
            return;
        }
        if (!HBABridge::exportToHBAFormat(data_path, keyframes, 4)) {
            res->success = false;
            res->message = "HBA export failed.";
            return;
        }
        if (cfg.hbaBridgeRunAfterExport()) {
            if (!HBABridge::runHBAProcess(data_path, cfg.hbaBridgeRunCommandTemplate())) {
                res->success = false;
                res->message = "HBA process failed (run_after_export=true).";
                return;
            }
        }
        if (!HBABridge::loadHBAResultAndApply(data_path, keyframes)) {
            res->success = !cfg.hbaBridgeRunAfterExport();
            res->message = cfg.hbaBridgeRunAfterExport()
                ? "HBA ran but load pose_trans.json failed (run HBA manually and retry)."
                : "Exported. Run HBA manually, then call TriggerHBA again with same data_path to load.";
            return;
        }
        std::map<int, Pose3d> optimized_poses;
        for (const auto& sm : all) {
            if (sm->keyframes.empty()) continue;
            int anchor_idx = (sm->anchor_keyframe_id >= 0 && sm->anchor_keyframe_id < static_cast<int>(sm->keyframes.size()))
                ? sm->anchor_keyframe_id : 0;
            optimized_poses[sm->id] = sm->keyframes[anchor_idx]->T_w_b_optimized;
        }
        map_builder_.reprojectAllSubmaps(all, optimized_poses);
        res->success = true;
        res->message = "HBA export and apply done.";
    }

    rclcpp::Node::SharedPtr node_;
    SensorManager   sensor_mgr_;
    FastLIVO2Wrapper frontend_;
    std::unique_ptr<FastLIVO2Adapter> adapter_;
    SubMapManager   submap_mgr_;
    SessionManager  session_mgr_;
    LoopDetector    loop_detector_;
    HBAWrapper      hba_;
    MapBuilder      map_builder_;
    MapFilter       map_filter_;
    MapExporter     map_exporter_;
    RvizPublisher   rviz_;

    SystemState     state_ = SystemState::IDLE;
    Pose3d   last_pose_;
    double   last_pose_time_ = 0.0;
    mutable std::mutex pose_mutex_;
    KeyFrame::Ptr last_kf_;
    mutable std::mutex kf_mutex_;
    std::vector<LoopConstraint::Ptr> loop_constraints_;
    mutable std::mutex loop_mutex_;

    rclcpp::Service<automap_pro::srv::SaveMap>::SharedPtr save_map_srv_;
    rclcpp::Service<automap_pro::srv::TriggerOptimize>::SharedPtr trigger_opt_srv_;
    rclcpp::Service<automap_pro::srv::GetStatus>::SharedPtr get_status_srv_;
    rclcpp::Service<automap_pro::srv::LoadSession>::SharedPtr load_session_srv_;
    rclcpp::Service<automap_pro::srv::TriggerHBA>::SharedPtr trigger_hba_srv_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
};

}  // namespace automap_pro

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("automap_system_node");

    automap_pro::AutoMapSystem system;
    if (!system.init(node)) {
        RCLCPP_FATAL(node->get_logger(), "[automap_system_node] Initialization failed!");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
