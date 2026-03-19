// 模块9: 轨迹记录功能
// 包含: ensureTrajectoryLogDir, writeTrajectoryOdom, writeTrajectoryOdomAfterMapping, onGPSMeasurementForLog

#include "automap_pro/system/automap_system.h"
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 轨迹记录：确保目录存在
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::ensureTrajectoryLogDir() {
    if (!trajectory_session_id_.empty()) return;
    try {
        fs::create_directories(trajectory_log_dir_);
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm buf;
#if defined(_WIN32) || defined(_WIN64)
        std::tm* ptm = std::localtime(&t);
#else
        std::tm* ptm = ::localtime_r(&t, &buf);
#endif
        if (ptm) {
            std::ostringstream oss;
            oss << std::put_time(ptm, "%Y%m%d_%H%M%S");
            trajectory_session_id_ = oss.str();
        } else {
            trajectory_session_id_ = std::to_string(current_session_id_);
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] trajectory log dir=%s session_id=%s",
                    trajectory_log_dir_.c_str(), trajectory_session_id_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] create dir failed: %s", e.what());
        trajectory_session_id_ = "default";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 轨迹记录：写入里程计轨迹（实时）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::writeTrajectoryOdom(double ts, const Pose3d& pose, const Mat66d& cov) {
    std::lock_guard<std::mutex> lk(trajectory_log_mutex_);
    ensureTrajectoryLogDir();
    if (!trajectory_odom_file_.is_open()) {
        std::string path = trajectory_log_dir_ + "/trajectory_odom_" + trajectory_session_id_ + ".csv";
        trajectory_odom_file_.open(path, std::ios::out);
        if (trajectory_odom_file_.is_open()) {
            trajectory_odom_file_ << "timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid,gps_hdop,gps_quality\n";
            trajectory_odom_file_.flush();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] opened %s (with GPS columns). If CSV has no GPS: grep -E 'LivoBridge\\[GPS\\]|GPS_DIAG|TRAJ_LOG no GPS' in logs.",
                path.c_str());
        }
    }
    if (!trajectory_odom_file_.is_open()) return;

    constexpr double kTrajectoryLogGpsMaxDt = 1.0;

    static std::atomic<uint32_t> traj_row_count{0};
    uint32_t row = traj_row_count++;
    if (row == 0) {
        size_t gps_sz = gps_manager_.getGpsWindowSize();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] First trajectory row: odom_ts=%.3f (GPS match window=%.1fs; odom ts from LivoBridge /aft_mapped_to_init) gps_window_size=%zu",
            ts, kTrajectoryLogGpsMaxDt, gps_sz);
        if (gps_sz == 0) {
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] GPS window empty at first row. If bag has GPS: grep 'LivoBridge\\[GPS\\]' and 'GPS_DIAG' in logs; after ~45s see '[LivoBridge][GPS_DIAG] Still 0 NavSatFix' if topic not received.");
        }
        if (!gps_manager_.isAligned()) {
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] GPS not aligned! gps_frame will be ENU, not map. Check: good_samples=%d (need=%d) accumulated_dist_m=%.1f (min=%.1f) state=%d",
                gps_manager_.getGoodSampleCount(), gps_manager_.getGoodSamplesNeeded(),
                gps_manager_.getAccumulatedDistM(), gps_manager_.getMinAlignDistM(), static_cast<int>(gps_manager_.state()));
        }
    }
    if (!gps_manager_.isAligned() && row > 0 && (row % 200 == 0)) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] still not aligned at row=%u: good_samples=%d need=%d dist_m=%.1f min_dist=%.1f state=%d",
            row, gps_manager_.getGoodSampleCount(), gps_manager_.getGoodSamplesNeeded(),
            gps_manager_.getAccumulatedDistM(), gps_manager_.getMinAlignDistM(), static_cast<int>(gps_manager_.state()));
    }

    double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
    double gps_hdop = 0.0;
    int gps_quality = 0;
    bool gps_valid = false;
    std::string gps_frame_str = "none";

    auto gps_opt = gps_manager_.queryByTimestampForLog(ts, kTrajectoryLogGpsMaxDt);
    if (gps_opt) {
        auto [pos, frame] = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
        gps_x = pos.x();
        gps_y = pos.y();
        gps_z = pos.z();
        gps_frame_str = frame;
        gps_hdop = gps_opt->hdop;
        gps_quality = static_cast<int>(gps_opt->quality);
        gps_valid = gps_opt->is_valid;
        if (row == 0 && frame == "enu") {
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][TRAJ_LOG] CSV: trajectory (x,y,z)=map frame; gps_x/gps_y/gps_z=enu (not aligned). Use gps_frame column; after align both in map.");
        }
    } else {
        static std::atomic<uint32_t> traj_no_gps_count{0};
        uint32_t no_gps = traj_no_gps_count++;
        size_t gps_window_size = gps_manager_.getGpsWindowSize();
        const char* reason = gps_window_size == 0 ? "no GPS data received (LivoBridge onGPS never called)" : "no match within 1.0s";
        if (no_gps < 5 || (no_gps > 0 && no_gps % 500 == 0)) {
            double gps_min = 0.0, gps_max = 0.0;
            bool has_range = gps_manager_.getGpsWindowTimeRange(&gps_min, &gps_max);
            if (gps_window_size == 0 && no_gps < 5) {
                if (has_range) {
                    RCLCPP_WARN(get_logger(),
                        "[AutoMapSystem][TRAJ_LOG] no GPS for odom ts=%.3f gps_window_size=%zu gps_ts_range=[%.3f, %.3f] (reason: %s).",
                        ts, gps_window_size, gps_min, gps_max, reason);
                } else {
                    RCLCPP_WARN(get_logger(),
                        "[AutoMapSystem][TRAJ_LOG] no GPS for odom ts=%.3f gps_window_size=%zu (reason: %s). Check bag has topic and NavSatFix.",
                        ts, gps_window_size, reason);
                }
            }
        }
    }

    Eigen::Quaterniond q(pose.rotation());
    double px = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
    double py = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
    double pz = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
    trajectory_odom_file_ << std::fixed << std::setprecision(6)
        << ts << ","
        << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
        << px << "," << py << "," << pz << ","
        << gps_x << "," << gps_y << "," << gps_z << ","
        << gps_frame_str << "," << (gps_valid ? "1" : "0") << ","
        << gps_hdop << "," << gps_quality << "\n";
    trajectory_odom_file_.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// 轨迹记录：建图完成后写入（关键帧+GPS）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::writeTrajectoryOdomAfterMapping(const std::string& output_dir) {
    if (output_dir.empty()) return;
    auto all_sm = submap_manager_.getAllSubmaps();
    std::vector<std::pair<double, Pose3d>> kf_poses;
    std::vector<Mat66d> kf_covs;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            const Pose3d& T = kf->T_w_b_optimized;
            kf_poses.emplace_back(kf->timestamp, T);
            kf_covs.push_back(kf->covariance);
        }
    }
    if (kf_poses.empty()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] writeTrajectoryOdomAfterMapping: no keyframes, skip");
        return;
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][TRAJ_LOG] writing trajectory_odom at save (keyframe+GPS, map frame); same frame as keyframe_poses.pcd / gps_positions_map.pcd — use this file in save dir for trajectory-GPS comparison.");
    if (!gps_manager_.isAligned()) {
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] GPS not aligned: gps_x/gps_y/gps_z will be in ENU frame; trajectory vs GPS may not coincide in plot. Consider triggering GPS align before save if needed.");
    }
    const std::string filename = "trajectory_odom_" + trajectory_session_id_ + ".csv";
    const std::string path_primary = output_dir + "/" + filename;
    const bool also_to_log_dir = !trajectory_log_dir_.empty() && trajectory_log_dir_ != output_dir;

    std::ofstream out(path_primary);
    if (!out.is_open()) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] failed to open %s", path_primary.c_str());
        return;
    }
    std::ofstream out_log;
    if (also_to_log_dir) {
        fs::create_directories(trajectory_log_dir_);
        const std::string path_log = trajectory_log_dir_ + "/" + filename;
        out_log.open(path_log);
        if (!out_log.is_open()) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] failed to open log dir copy %s (will only write to output_dir)", path_log.c_str());
        }
    }

    const std::string header = "timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid,gps_hdop,gps_quality\n";
    out << header;
    if (out_log.is_open()) out_log << header;

    constexpr double kGpsMaxDt = 1.0;
    for (size_t i = 0; i < kf_poses.size(); ++i) {
        const double ts = kf_poses[i].first;
        const Pose3d& pose = kf_poses[i].second;
        const Mat66d& cov = kf_covs[i];
        double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0, gps_hdop = 0.0;
        int gps_quality = 0;
        bool gps_valid = false;
        std::string gps_frame_str = "none";
        auto gps_opt = gps_manager_.queryByTimestampForLog(ts, kGpsMaxDt);
        if (gps_opt) {
            auto [pos, frame] = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
            gps_x = pos.x(); gps_y = pos.y(); gps_z = pos.z();
            gps_frame_str = frame;
            gps_hdop = gps_opt->hdop;
            gps_quality = static_cast<int>(gps_opt->quality);
            gps_valid = gps_opt->is_valid;
        }
        Eigen::Quaterniond q(pose.rotation());
        double px = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
        double py = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
        double pz = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
        std::ostringstream line;
        line << std::fixed << std::setprecision(6)
            << ts << ","
            << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
            << px << "," << py << "," << pz << ","
            << gps_x << "," << gps_y << "," << gps_z << ","
            << gps_frame_str << "," << (gps_valid ? "1" : "0") << ","
            << gps_hdop << "," << gps_quality << "\n";
        const std::string line_str = line.str();
        out << line_str;
        if (out_log.is_open()) out_log << line_str;
    }
    out.close();
    if (out_log.is_open()) {
        out_log.close();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] wrote trajectory_odom (keyframe+GPS, map frame) to %s and %s (%zu rows). For trajectory-GPS comparison use the file in save dir (same as keyframe_poses.pcd).",
            path_primary.c_str(), (trajectory_log_dir_ + "/" + filename).c_str(), kf_poses.size());
    } else {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][TRAJ_LOG] wrote trajectory_odom (keyframe+GPS, map frame) to %s (%zu rows). Use this file for trajectory-GPS comparison (same frame as keyframe_poses.pcd).",
            path_primary.c_str(), kf_poses.size());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成后写入：HBA 优化关键帧位姿 + GPS（便于建图精度分析）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::writeHbaPosesAndGpsForAccuracy() {
    if (!trajectory_log_enabled_) return;
    std::lock_guard<std::mutex> lk(trajectory_log_mutex_);
    ensureTrajectoryLogDir();
    auto all_sm = submap_manager_.getFrozenSubmaps();
    size_t kf_count = 0;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        kf_count += sm->keyframes.size();
    }
    if (kf_count == 0) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] writeHbaPosesAndGpsForAccuracy: no keyframes, skip");
        return;
    }
    const std::string filename = "trajectory_hba_poses_" + trajectory_session_id_ + ".csv";
    const std::string path = trajectory_log_dir_ + "/" + filename;
    std::ofstream out(path);
    if (!out.is_open()) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][TRAJ_LOG] failed to open %s for HBA poses", path.c_str());
        return;
    }
    out << "timestamp,kf_id,submap_id,hba_x,hba_y,hba_z,hba_qx,hba_qy,hba_qz,hba_qw,"
        << "gps_valid,gps_x_map,gps_y_map,gps_z_map,gps_enu_x,gps_enu_y,gps_enu_z,gps_hdop,gps_quality\n";
    constexpr double kGpsMaxDt = 1.0;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            const Pose3d& T = kf->T_w_b_optimized;
            Eigen::Quaterniond q(T.rotation());
            double gps_x_map = 0.0, gps_y_map = 0.0, gps_z_map = 0.0;
            double gps_enu_x = 0.0, gps_enu_y = 0.0, gps_enu_z = 0.0;
            double gps_hdop = 0.0;
            int gps_quality = 0;
            bool gps_valid = false;
            if (kf->has_valid_gps) {
                gps_enu_x = kf->gps.position_enu.x();
                gps_enu_y = kf->gps.position_enu.y();
                gps_enu_z = kf->gps.position_enu.z();
                gps_hdop = kf->gps.hdop;
                gps_quality = static_cast<int>(kf->gps.quality);
                gps_valid = kf->gps.is_valid;
                auto [pos_map, frame] = gps_manager_.enu_to_map_with_frame(kf->gps.position_enu);
                gps_x_map = pos_map.x();
                gps_y_map = pos_map.y();
                gps_z_map = pos_map.z();
            } else {
                auto gps_opt = gps_manager_.queryByTimestampForLog(kf->timestamp, kGpsMaxDt);
                if (gps_opt) {
                    gps_enu_x = gps_opt->position_enu.x();
                    gps_enu_y = gps_opt->position_enu.y();
                    gps_enu_z = gps_opt->position_enu.z();
                    gps_hdop = gps_opt->hdop;
                    gps_quality = static_cast<int>(gps_opt->quality);
                    gps_valid = gps_opt->is_valid;
                    auto [pos_map, frame] = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
                    gps_x_map = pos_map.x();
                    gps_y_map = pos_map.y();
                    gps_z_map = pos_map.z();
                }
            }
            out << std::fixed << std::setprecision(6)
                << kf->timestamp << "," << static_cast<uint64_t>(kf->id) << "," << kf->submap_id << ","
                << T.translation().x() << "," << T.translation().y() << "," << T.translation().z() << ","
                << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
                << (gps_valid ? "1" : "0") << ","
                << gps_x_map << "," << gps_y_map << "," << gps_z_map << ","
                << gps_enu_x << "," << gps_enu_y << "," << gps_enu_z << ","
                << gps_hdop << "," << gps_quality << "\n";
        }
    }
    out.close();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][TRAJ_LOG] wrote trajectory_hba_poses (HBA keyframe + GPS) to %s (%zu rows). Use for mapping accuracy analysis.",
        path.c_str(), kf_count);
}

// ─────────────────────────────────────────────────────────────────────────────
// 轨迹记录：GPS测量日志
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onGPSMeasurementForLog(double ts, const Eigen::Vector3d& pos_enu) {
    if (!trajectory_log_enabled_) return;
    std::lock_guard<std::mutex> lk(trajectory_log_mutex_);
    ensureTrajectoryLogDir();
    if (!trajectory_gps_file_.is_open()) {
        std::string path = trajectory_log_dir_ + "/trajectory_gps_" + trajectory_session_id_ + ".csv";
        trajectory_gps_file_.open(path, std::ios::out);
        if (trajectory_gps_file_.is_open()) {
            trajectory_gps_file_ << "timestamp,x,y,z,frame,pitch,roll,yaw,attitude_source,velocity,attitude_valid\n";
            trajectory_gps_file_.flush();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] opened %s (with attitude columns)", path.c_str());
        }
    }
    if (!trajectory_gps_file_.is_open()) return;
    auto m_opt = gps_manager_.queryByTimestampForLog(ts, 0.1);
    if (!m_opt) return;
    const GPSMeasurement& m = *m_opt;
    auto [pos, frame_str] = gps_manager_.enu_to_map_with_frame(m.position_enu);
    const AttitudeEstimate& att = m.attitude;
    trajectory_gps_file_ << std::fixed << std::setprecision(6)
        << m.timestamp << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << frame_str << ","
        << att.pitch << "," << att.roll << "," << att.yaw << ","
        << static_cast<int>(att.source) << "," << att.velocity_horizontal << ","
        << (att.is_valid ? "1" : "0") << "\n";
    trajectory_gps_file_.flush();
}

}  // namespace automap_pro
