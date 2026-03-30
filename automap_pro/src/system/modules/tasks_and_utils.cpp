/**
 * @file system/modules/tasks_and_utils.cpp
 * @brief 系统节点与 ROS 服务实现。
 */
#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <future>
#include <iomanip>

namespace automap_pro {

namespace fs = std::filesystem;

void AutoMapSystem::publishStatus() {
    try {
        if (!v3_context_ || !v3_context_->mapRegistry()) return;
        
        auto& registry = *v3_context_->mapRegistry();
        const int kf_count = static_cast<int>(registry.keyframeCount());
        const int sm_count = static_cast<int>(registry.submapCount());

        automap_pro::msg::MappingStatusMsg msg;
        msg.header.stamp  = now();
        msg.state         = stateToString(state_.load());
        msg.session_id    = current_session_id_;
        msg.keyframe_count = kf_count;
        msg.submap_count  = sm_count;
        
        msg.gps_aligned   = registry.isGPSAligned();
        msg.gps_alignment_score = static_cast<float>(registry.getGPSRMSE());

        if (status_pub_) {
            status_pub_->publish(msg);
            pub_status_count_++;
        }

        if (++status_publish_count_ >= 5) {
            status_publish_count_ = 0;
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][BACKEND] state=%s kf=%d sm=%d gps_aligned=%d",
                msg.state.c_str(), kf_count, sm_count, msg.gps_aligned ? 1 : 0);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][STATUS] publishStatus exception: %s", e.what());
    }
}

void AutoMapSystem::publishGlobalMap() {
    if (!v3_context_ || !v3_context_->mapRegistry()) return;
    
    v3::GlobalMapBuildRequestEvent ev;
    ev.voxel_size = map_voxel_size_;
    ev.async = ConfigManager::instance().globalMapBuildAsync();
    v3_context_->eventBus()->publish(ev);
    
    // 注意：结果将通过 GlobalMapBuildResultEvent 返回，AutoMapSystem 需要订阅它
}

void AutoMapSystem::saveMapToFiles(const std::string& output_dir) {
    if (!v3_context_ || !v3_context_->mapRegistry()) return;

    auto completion = std::make_shared<std::promise<void>>();
    std::future<void> done = completion->get_future();

    v3::SaveMapRequestEvent ev;
    ev.output_dir = output_dir;
    ev.completion = completion;
    v3_context_->eventBus()->publish(ev);

    constexpr int k_save_wait_hours = 4;
    const auto wt = done.wait_for(std::chrono::hours(k_save_wait_hours));
    if (wt == std::future_status::timeout) {
        RCLCPP_ERROR(get_logger(),
            "[AutoMapSystem] SaveMap timed out after %d h; trajectory CSV may precede incomplete map export",
            k_save_wait_hours);
    }

    writeTrajectoryOdomAfterMapping(output_dir);
    writeMappingAccuracyGpsVsHba(output_dir);
}

std::string AutoMapSystem::getOutputDir() const {
    const char* session = std::getenv("AUTOMAP_SESSION_OUTPUT_DIR");
    if (session && session[0] != '\0') {
        return std::string(session);
    }
    if (!output_dir_override_.empty()) return output_dir_override_;
    return ConfigManager::instance().outputDir();
}

std::string AutoMapSystem::stateToString(SystemState s) const {
    switch (s) {
        case SystemState::IDLE: return "IDLE";
        case SystemState::MAPPING: return "MAPPING";
        case SystemState::LOOP_CLOSING: return "LOOP_CLOSING";
        case SystemState::OPTIMIZING: return "OPTIMIZING";
        case SystemState::SAVING: return "SAVING";
        default: return "UNKNOWN";
    }
}

void AutoMapSystem::writeTrajectoryOdomAfterMapping(const std::string& output_dir) {
    if (!v3_context_ || !v3_context_->mapRegistry()) return;
    
    auto all_kfs = v3_context_->mapRegistry()->getAllKeyFrames();
    if (all_kfs.empty()) return;

    const fs::path opt_dir = fs::path(output_dir) / "optimized";
    std::error_code ec;
    fs::create_directories(opt_dir, ec);
    const std::string opt_base = (!ec && fs::is_directory(opt_dir)) ? opt_dir.string() : output_dir;
    std::string path = opt_base + "/trajectory_odom.csv";
    std::ofstream f(path);
    if (!f.is_open()) return;

    f << "timestamp,x,y,z,qx,qy,qz,qw,gps_lat,gps_lon,gps_alt,has_gps\n";
    for (const auto& kf : all_kfs) {
        const Pose3d T_exp = kf->mapPoseForExportPreferLastHba();
        const auto& t = T_exp.translation();
        const auto q = Eigen::Quaterniond(T_exp.rotation());
        f << std::fixed << std::setprecision(6) << kf->timestamp << ","
          << t.x() << "," << t.y() << "," << t.z() << ","
          << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ",";
        if (kf->has_valid_gps) {
            f << kf->gps.latitude << "," << kf->gps.longitude << "," << kf->gps.altitude << ",1\n";
        } else {
            f << "0,0,0,0\n";
        }
    }
    f.close();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Trajectory saved to %s", path.c_str());
}

void AutoMapSystem::writeMappingAccuracyGpsVsHba(const std::string& output_dir) {
    if (!v3_context_ || !v3_context_->mapRegistry()) return;
    
    auto& registry = *v3_context_->mapRegistry();
    auto all_kfs = registry.getAllKeyFrames();
    if (all_kfs.empty()) return;

    std::string path = output_dir + "/mapping_accuracy.csv";
    std::ofstream f(path);
    if (!f.is_open()) return;

    f << "timestamp,kf_id,err_x,err_y,err_z,err_norm\n";
    double total_err_norm = 0.0;
    int gps_count = 0;

    Eigen::Matrix3d R_map_enu = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_map_enu = Eigen::Vector3d::Zero();
    bool aligned = registry.isGPSAligned();
    if (aligned) {
        registry.getGPSTransform(R_map_enu, t_map_enu);
    }

    for (const auto& kf : all_kfs) {
        if (!kf || !kf->has_valid_gps) continue;
        
        Eigen::Vector3d pos_opt = kf->mapPoseForExportPreferLastHba().translation();
        
        // 🏛️ [V3] 坐标系：kf->T_map_b_optimized 已经在 map 系，kf->gps.position_enu 在 ENU 系
        // 注意：T_odom_b 始终在 odom 系，不受 GPS 对齐影响。
        // 我们需要把 position_enu 转到 map 系进行对比
        Eigen::Vector3d pos_gps_map;
        if (aligned) {
            pos_gps_map = R_map_enu * kf->gps.position_enu + t_map_enu;
        } else {
            // 未对齐时，对比没有意义，或者假设 T_map_odom 是单位阵
            continue; 
        }
        
        Eigen::Vector3d err = pos_opt - pos_gps_map;
        double norm = err.norm();
        
        f << std::fixed << std::setprecision(6) << kf->timestamp << ","
          << kf->id << "," << err.x() << "," << err.y() << "," << err.z() << "," << norm << "\n";
          
        total_err_norm += norm;
        gps_count++;
    }
    f.close();
    
    if (gps_count > 0) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Accuracy report saved to %s. Avg error: %.3f m", 
                    path.c_str(), total_err_norm / gps_count);
    }
}

} // namespace automap_pro
