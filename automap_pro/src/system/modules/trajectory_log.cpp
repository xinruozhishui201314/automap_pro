// 模块9: 轨迹记录功能
// 包含: ensureTrajectoryLogDir, writeTrajectoryOdom, writeTrajectoryOdomAfterMapping, onGPSMeasurementForLog

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
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

        // 🔧 自动创建带时间戳的输出子目录，满足用户需求：@automap_output/run_timestamp
        if (output_dir_override_.empty()) {
            std::string base_dir = ConfigManager::instance().outputDir();
            output_dir_override_ = base_dir + "/run_" + trajectory_session_id_;
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][TRAJ_LOG] initialized timestamped output_dir: %s", output_dir_override_.c_str());
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
                if (gps_aligned_.load()) {
                    gps_x_map = kf->gps.position_enu.x();
                    gps_y_map = kf->gps.position_enu.y();
                    gps_z_map = kf->gps.position_enu.z();
                } else {
                    auto [pos_map, frame] = gps_manager_.enu_to_map_with_frame(kf->gps.position_enu);
                    gps_x_map = pos_map.x();
                    gps_y_map = pos_map.y();
                    gps_z_map = pos_map.z();
                }
            } else {
                auto gps_opt = gps_manager_.queryByTimestampForLog(kf->timestamp, kGpsMaxDt);
                if (gps_opt) {
                    gps_enu_x = gps_opt->position_enu.x();
                    gps_enu_y = gps_opt->position_enu.y();
                    gps_enu_z = gps_opt->position_enu.z();
                    gps_hdop = gps_opt->hdop;
                    gps_quality = static_cast<int>(gps_opt->quality);
                    gps_valid = gps_opt->is_valid;
                    if (gps_aligned_.load()) {
                        gps_x_map = gps_opt->position_enu.x();
                        gps_y_map = gps_opt->position_enu.y();
                        gps_z_map = gps_opt->position_enu.z();
                    } else {
                        auto [pos_map, frame] = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
                        gps_x_map = pos_map.x();
                        gps_y_map = pos_map.y();
                        gps_z_map = pos_map.z();
                    }
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
    
    // 获取位置和坐标系名称
    Eigen::Vector3d pos;
    std::string frame_str;
    if (gps_aligned_.load()) {
        // 如果系统已对齐并全球化，则当前 "map" 系就是全球 ENU 系
        // GPS 的原始 ENU 坐标即为全球系坐标，无需经过 GPSManager 的 enu_to_map 变换
        pos = m.position_enu;
        frame_str = "map";
    } else {
        // 未对齐时，使用 GPSManager 传出的坐标（此时 enu_to_map 为单位阵，即原始 ENU）
        auto res = gps_manager_.enu_to_map_with_frame(m.position_enu);
        pos = res.first;
        frame_str = res.second;
    }

    const AttitudeEstimate& att = m.attitude;
    trajectory_gps_file_ << std::fixed << std::setprecision(6)
        << m.timestamp << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << frame_str << ","
        << att.pitch << "," << att.roll << "," << att.yaw << ","
        << static_cast<int>(att.source) << "," << att.velocity_horizontal << ","
        << (att.is_valid ? "1" : "0") << "\n";
    trajectory_gps_file_.flush();
}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

double wrapToPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

/** R = Rz(yaw)*Ry(pitch)*Rx(roll) 与 Eigen::eulerAngles(2,1,0) 一致 → [yaw,pitch,roll] 弧度 */
Eigen::Vector3d yawPitchRollFromR(const Eigen::Matrix3d& R) {
    Eigen::Vector3d e = R.eulerAngles(2, 1, 0);
    return Eigen::Vector3d(e[0], e[1], e[2]);
}

void drawBand(cv::Mat& img, const cv::Rect& band, const std::vector<double>& xv, const std::vector<double>& yv,
              const cv::Scalar& color, const std::string& title) {
    if (xv.size() < 2 || xv.size() != yv.size()) return;
    double ymin = *std::min_element(yv.begin(), yv.end());
    double ymax = *std::max_element(yv.begin(), yv.end());
    if (!std::isfinite(ymin) || !std::isfinite(ymax)) return;
    if (std::abs(ymax - ymin) < 1e-12) {
        ymin -= 1.0;
        ymax += 1.0;
    }
    const double margin = (ymax - ymin) * 0.08 + 1e-6;
    ymin -= margin;
    ymax += margin;
    double xmin = xv.front(), xmax = xv.back();
    if (std::abs(xmax - xmin) < 1e-9) {
        xmin -= 1.0;
        xmax += 1.0;
    }
    const int x0 = band.x + 50, y0 = band.y + 10;
    const int bw = band.width - 60, bh = band.height - 30;
    cv::rectangle(img, band, cv::Scalar(230, 230, 230), 1);
    cv::putText(img, title, cv::Point(band.x + 5, band.y + 18), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
    auto tx = [&](double x) { return x0 + static_cast<int>((x - xmin) / (xmax - xmin) * bw); };
    auto ty = [&](double y) { return y0 + bh - static_cast<int>((y - ymin) / (ymax - ymin) * bh); };
    for (size_t i = 1; i < xv.size(); ++i) {
        cv::line(img, cv::Point(tx(xv[i - 1]), ty(yv[i - 1])), cv::Point(tx(xv[i]), ty(yv[i])), color, 1,
                 cv::LINE_AA);
    }
    cv::line(img, cv::Point(x0, y0 + bh), cv::Point(x0 + bw, y0 + bh), cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    cv::line(img, cv::Point(x0, y0), cv::Point(x0, y0 + bh), cv::Scalar(80, 80, 80), 1, cv::LINE_AA);

    // 坐标刻度与数值（横轴为 xv 物理量，纵轴为 yv 物理量）
    constexpr int kDiv = 5;
    constexpr double kFont = 0.35;
    const cv::Scalar tick_color(80, 80, 80);
    auto fmt_axis = [](double v, int prec) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(prec) << v;
        return o.str();
    };
    for (int i = 0; i <= kDiv; ++i) {
        const double xv_tick = xmin + (xmax - xmin) * static_cast<double>(i) / static_cast<double>(kDiv);
        const int px = tx(xv_tick);
        cv::line(img, cv::Point(px, y0 + bh), cv::Point(px, y0 + bh + 6), tick_color, 1, cv::LINE_AA);
        const std::string xs = fmt_axis(xv_tick, (xmax - xmin) >= 100.0 ? 0 : 1);
        int bl = 0;
        const cv::Size xsz = cv::getTextSize(xs, cv::FONT_HERSHEY_SIMPLEX, kFont, 1, &bl);
        int txp = px - xsz.width / 2;
        txp = std::max(band.x + 2, std::min(txp, band.x + band.width - xsz.width - 2));
        int x_baseline = y0 + bh + 12;
        x_baseline = std::min(x_baseline, band.y + band.height - 2);
        cv::putText(img, xs, cv::Point(txp, x_baseline), cv::FONT_HERSHEY_SIMPLEX, kFont, tick_color, 1, cv::LINE_AA);

        const double yv_tick = ymin + (ymax - ymin) * static_cast<double>(i) / static_cast<double>(kDiv);
        const int py = ty(yv_tick);
        cv::line(img, cv::Point(x0 - 6, py), cv::Point(x0, py), tick_color, 1, cv::LINE_AA);
        const std::string ys = fmt_axis(yv_tick, 3);
        cv::putText(img, ys, cv::Point(band.x + 4, std::min(py + 4, y0 + bh)), cv::FONT_HERSHEY_SIMPLEX, kFont,
                    tick_color, 1, cv::LINE_AA);
    }
}

bool saveTriplePlot(const std::string& path, const std::vector<double>& xv,
                    const std::vector<double>& y1, const std::vector<double>& y2, const std::vector<double>& y3,
                    const char* l1, const char* l2, const char* l3) {
    if (xv.size() < 2 || y1.size() != xv.size() || y2.size() != xv.size() || y3.size() != xv.size()) return false;
    const int W = 1100, H = 720, bandH = H / 3;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
    drawBand(img, cv::Rect(0, 0, W, bandH), xv, y1, cv::Scalar(200, 60, 60), std::string(l1));
    drawBand(img, cv::Rect(0, bandH, W, bandH), xv, y2, cv::Scalar(60, 140, 60), std::string(l2));
    drawBand(img, cv::Rect(0, 2 * bandH, W, bandH), xv, y3, cv::Scalar(60, 80, 200), std::string(l3));
    try {
        return cv::imwrite(path, img);
    } catch (...) {
        return false;
    }
}

}  // namespace

void AutoMapSystem::writeMappingAccuracyGpsVsHba(const std::string& output_dir) {
    if (output_dir.empty()) return;
    if (!trajectory_log_enabled_) return;

    const std::string sid = trajectory_session_id_.empty() ? "nosession" : trajectory_session_id_;
    const std::string base = "mapping_accuracy_gps_vs_hba_" + sid;
    const std::string csv_path = output_dir + "/" + base + ".csv";
    const std::string sum_path = output_dir + "/" + base + "_summary.txt";
    const std::string png_xyz = output_dir + "/" + base + "_err_xyz.png";
    const std::string png_rpy = output_dir + "/" + base + "_err_rpy_deg.png";

    const bool also_log = !trajectory_log_dir_.empty() && trajectory_log_dir_ != output_dir;

    auto all_sm = submap_manager_.getAllSubmaps();
    std::vector<KeyFrame::Ptr> kfs;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (kf) kfs.push_back(kf);
        }
    }
    std::sort(kfs.begin(), kfs.end(), [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
        return a->timestamp < b->timestamp;
    });

    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][ACCURACY] failed to open %s", csv_path.c_str());
        return;
    }

    csv << "timestamp,kf_id,submap_id,gps_pos_valid,gps_att_valid,pos_frame,"
        << "hba_x_m,hba_y_m,hba_z_m,hba_roll_deg,hba_pitch_deg,hba_yaw_deg,"
        << "gps_x_map_m,gps_y_map_m,gps_z_map_m,gps_roll_deg,gps_pitch_deg,gps_yaw_deg,"
        << "err_x_m,err_y_m,err_z_m,horiz_err_m,err_roll_deg,err_pitch_deg,err_yaw_deg,"
        << "gps_hdop,gps_quality\n";

    constexpr double kGpsMaxDt = 1.0;
    std::vector<double> plot_t;
    std::vector<double> ex, ey, ez;
    std::vector<double> plot_t_rpy;
    std::vector<double> er, ep, eya;

    double sum_h2 = 0.0, sum_z2 = 0.0;
    double sum_x2 = 0.0, sum_y2 = 0.0;
    int n_pos = 0;
    double sum_rr2 = 0.0, sum_pp2 = 0.0, sum_yy2 = 0.0;
    int n_att = 0;

    const double t0 = kfs.empty() ? 0.0 : kfs.front()->timestamp;

    for (const auto& kf : kfs) {
        const Pose3d& T = kf->T_w_b_optimized;
        const Eigen::Vector3d t_h = T.translation();
        Eigen::Vector3d ypr_h = yawPitchRollFromR(T.linear());

        double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
        double gps_hdop = 0.0;
        int gps_quality = 0;
        bool gps_pos_valid = false;
        std::string frame_str = "none";

        AttitudeEstimate gatt;
        if (kf->has_valid_gps) {
            gps_hdop = kf->gps.hdop;
            gps_quality = static_cast<int>(kf->gps.quality);
            gps_pos_valid = kf->gps.is_valid;
            
            // 🔧 修复：若系统已全球化（gps_aligned_=true），则 "map" 系就是 "enu" 系，无需再次变换
            if (gps_aligned_.load()) {
                gps_x = kf->gps.position_enu.x();
                gps_y = kf->gps.position_enu.y();
                gps_z = kf->gps.position_enu.z();
                frame_str = "map";
            } else {
                auto pr = gps_manager_.enu_to_map_with_frame(kf->gps.position_enu);
                gps_x = pr.first.x();
                gps_y = pr.first.y();
                gps_z = pr.first.z();
                frame_str = pr.second;
            }
            gatt = kf->gps.attitude;
        } else if (auto gps_opt = gps_manager_.queryByTimestampForLog(kf->timestamp, kGpsMaxDt)) {
            gps_hdop = gps_opt->hdop;
            gps_quality = static_cast<int>(gps_opt->quality);
            gps_pos_valid = gps_opt->is_valid;
            
            if (gps_aligned_.load()) {
                gps_x = gps_opt->position_enu.x();
                gps_y = gps_opt->position_enu.y();
                gps_z = gps_opt->position_enu.z();
                frame_str = "map";
            } else {
                auto pr = gps_manager_.enu_to_map_with_frame(gps_opt->position_enu);
                gps_x = pr.first.x();
                gps_y = pr.first.y();
                gps_z = pr.first.z();
                frame_str = pr.second;
            }
            gatt = gps_opt->attitude;
        }
        const bool gps_att_valid = gatt.is_valid;

        double gr = 0.0, gp = 0.0, gy = 0.0;
        if (gps_att_valid) {
            gr = gatt.roll * kRadToDeg;
            gp = gatt.pitch * kRadToDeg;
            gy = gatt.yaw * kRadToDeg;
        }

        const double hr = ypr_h[2] * kRadToDeg;
        const double hp = ypr_h[1] * kRadToDeg;
        const double hy = ypr_h[0] * kRadToDeg;

        double dx = 0.0, dy = 0.0, dz = 0.0, dh = 0.0;
        double dr = 0.0, dp = 0.0, dya = 0.0;
        if (gps_pos_valid) {
            dx = t_h.x() - gps_x;
            dy = t_h.y() - gps_y;
            dz = t_h.z() - gps_z;
            dh = std::sqrt(dx * dx + dy * dy);
            sum_h2 += dh * dh;
            sum_z2 += dz * dz;
            sum_x2 += dx * dx;
            sum_y2 += dy * dy;
            ++n_pos;
        }
        if (gps_att_valid) {
            dr = wrapToPi(ypr_h[2] - gatt.roll) * kRadToDeg;
            dp = wrapToPi(ypr_h[1] - gatt.pitch) * kRadToDeg;
            dya = wrapToPi(ypr_h[0] - gatt.yaw) * kRadToDeg;
            sum_rr2 += dr * dr;
            sum_pp2 += dp * dp;
            sum_yy2 += dya * dya;
            ++n_att;
        }
        if (gps_pos_valid) {
            plot_t.push_back(kf->timestamp - t0);
            ex.push_back(dx);
            ey.push_back(dy);
            ez.push_back(dz);
            if (gps_att_valid) {
                plot_t_rpy.push_back(kf->timestamp - t0);
                er.push_back(dr);
                ep.push_back(dp);
                eya.push_back(dya);
            }
        }

        csv << std::fixed << std::setprecision(6)
            << kf->timestamp << "," << static_cast<uint64_t>(kf->id) << "," << kf->submap_id << ","
            << (gps_pos_valid ? "1" : "0") << "," << (gps_att_valid ? "1" : "0") << "," << frame_str << ","
            << t_h.x() << "," << t_h.y() << "," << t_h.z() << "," << hr << "," << hp << "," << hy << ","
            << gps_x << "," << gps_y << "," << gps_z << "," << gr << "," << gp << "," << gy << ","
            << dx << "," << dy << "," << dz << "," << dh << "," << dr << "," << dp << "," << dya << ","
            << gps_hdop << "," << gps_quality << "\n";
    }
    csv.close();

    auto rmse = [](double sumsq, int n) { return n > 0 ? std::sqrt(sumsq / static_cast<double>(n)) : 0.0; };
    const GPSAlignResult& gar = gps_manager_.alignResult();

    std::ofstream sum(sum_path);
    if (sum.is_open()) {
        sum << "mapping_accuracy_gps_vs_hba summary (map frame, HBA/ISAM optimized T_w_b vs GPS)\n";
        sum << "note: position err is map-frame; RPY compares lidar-body euler(ZYX) vs GPS/IMU attitude (ENU), "
               "meaningful mainly after gps_aligned.\n";
        sum << "output_dir=" << output_dir << "\n";
        sum << "keyframes_total=" << kfs.size() << "\n";
        sum << "gps_aligned=" << (gps_aligned_.load() ? "true" : "false") << "\n";
        sum << "gps_align_success=" << (gar.success ? "true" : "false") << " rmse_m=" << gar.rmse_m
            << " matched_points=" << gar.matched_points << " used_measurements=" << gar.used_measurements << "\n";
        sum << "position_valid_samples=" << n_pos << "\n";
        sum << "rmse_horiz_m=" << rmse(sum_h2, n_pos) << " rmse_z_m=" << rmse(sum_z2, n_pos) << "\n";
        sum << "rmse_x_m=" << rmse(sum_x2, n_pos) << " rmse_y_m=" << rmse(sum_y2, n_pos) << "\n";
        sum << "attitude_valid_samples=" << n_att << "\n";
        sum << "rmse_roll_deg=" << rmse(sum_rr2, n_att) << " rmse_pitch_deg=" << rmse(sum_pp2, n_att)
            << " rmse_yaw_deg=" << rmse(sum_yy2, n_att) << "\n";
        sum << "loop_constraints_accepted=" << loop_detector_.loopDetectedCount() << "\n";
        sum.close();
    }

    bool ok_xyz = false, ok_rpy = false;
    if (plot_t.size() >= 2 && ex.size() == plot_t.size()) {
        ok_xyz = saveTriplePlot(png_xyz, plot_t, ex, ey, ez, "err_x (m)", "err_y (m)", "err_z (m)");
    }
    if (plot_t_rpy.size() >= 2 && er.size() == plot_t_rpy.size()) {
        ok_rpy = saveTriplePlot(png_rpy, plot_t_rpy, er, ep, eya, "err_roll (deg)", "err_pitch (deg)", "err_yaw (deg)");
    }

    RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][ACCURACY] wrote %s, %s, plots xyz=%d rpy=%d (pos_n=%d att_n=%d)",
                csv_path.c_str(), sum_path.c_str(), ok_xyz ? 1 : 0, ok_rpy ? 1 : 0, n_pos, n_att);

    if (also_log) {
        try {
            fs::create_directories(trajectory_log_dir_);
            fs::copy_file(csv_path, trajectory_log_dir_ + "/" + base + ".csv", fs::copy_options::overwrite_existing);
            fs::copy_file(sum_path, trajectory_log_dir_ + "/" + base + "_summary.txt",
                          fs::copy_options::overwrite_existing);
            if (ok_xyz)
                fs::copy_file(png_xyz, trajectory_log_dir_ + "/" + base + "_err_xyz.png",
                              fs::copy_options::overwrite_existing);
            if (ok_rpy)
                fs::copy_file(png_rpy, trajectory_log_dir_ + "/" + base + "_err_rpy_deg.png",
                              fs::copy_options::overwrite_existing);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][ACCURACY] copy to trajectory_log_dir failed: %s", e.what());
        }
    }
}

}  // namespace automap_pro
