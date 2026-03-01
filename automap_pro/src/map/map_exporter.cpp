#include "automap_pro/map/map_exporter.h"
#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace automap_pro {

MapExporter::MapExporter() = default;

bool MapExporter::exportAll(
        const CloudXYZIPtr& global_cloud,
        const std::vector<SubMap::Ptr>& submaps,
        const std::vector<LoopConstraint::Ptr>& loop_constraints,
        const PoseGraph& pose_graph,
        const ExportOptions& opts) {

    utils::createDirectories(opts.output_dir);
    utils::createDirectories(opts.output_dir + "/map");
    utils::createDirectories(opts.output_dir + "/trajectory");
    utils::createDirectories(opts.output_dir + "/loop_closures");
    utils::createDirectories(opts.output_dir + "/pose_graph");
    utils::createDirectories(opts.output_dir + "/submaps");

    bool ok = true;

    // 1. Global point cloud
    if (global_cloud && !global_cloud->empty()) {
        ok &= exportCloud(global_cloud,
                          opts.output_dir + "/map/global_map",
                          opts.save_pcd, opts.save_ply, opts.save_las);
        if (opts.tiling) {
            ok &= exportTiles(global_cloud,
                              opts.output_dir + "/map/tiles",
                              opts.tile_size);
        }
    }

    // 2. Trajectory
    ok &= exportTrajectory(submaps,
                            opts.output_dir + "/trajectory",
                            opts.save_traj_tum,
                            opts.save_traj_kitti,
                            opts.save_poses_json);

    // 3. Loop closure report
    if (opts.save_loop_report) {
        ok &= exportLoopReport(loop_constraints,
                               opts.output_dir + "/loop_closures/loop_report.json");
    }

    // 4. Pose graph
    if (opts.save_pose_graph) {
        ok &= pose_graph.saveG2O(opts.output_dir + "/pose_graph/pose_graph.g2o");
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Export complete to %s", opts.output_dir.c_str());
    return ok;
}

bool MapExporter::exportCloud(const CloudXYZIPtr& cloud,
                               const std::string& base_path,
                               bool pcd, bool ply, bool las) const {
    bool ok = true;
    if (pcd) {
        int ret = pcl::io::savePCDFileBinary(base_path + ".pcd", *cloud);
        if (ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"), "[MapExporter] Failed to save PCD: %s", base_path.c_str());
            ok = false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Saved PCD: %s.pcd (%zu pts)",
                     base_path.c_str(), cloud->size());
        }
    }
    if (ply) {
        ok &= savePLY(cloud, base_path + ".ply");
    }
    if (las) {
        ok &= saveLAS(cloud, base_path + ".las");
    }
    return ok;
}

bool MapExporter::exportTiles(const CloudXYZIPtr& cloud,
                               const std::string& tile_dir,
                               double tile_size) const {
    utils::createDirectories(tile_dir);
    if (!cloud || cloud->empty()) return false;

    std::map<std::pair<int,int>, CloudXYZIPtr> tiles;

    for (const auto& pt : cloud->points) {
        int tx = static_cast<int>(std::floor(pt.x / tile_size));
        int ty = static_cast<int>(std::floor(pt.y / tile_size));
        auto key = std::make_pair(tx, ty);
        if (!tiles.count(key)) {
            tiles[key] = std::make_shared<CloudXYZI>();
        }
        tiles[key]->push_back(pt);
    }

    // Save tiles + index
    nlohmann::json tile_index;
    for (const auto& [key, tile_cloud] : tiles) {
        std::string tile_name = "tile_" + std::to_string(key.first) +
                                "_" + std::to_string(key.second);
        std::string tile_path = tile_dir + "/" + tile_name + ".pcd";
        pcl::io::savePCDFileBinary(tile_path, *tile_cloud);

        nlohmann::json tile_meta;
        tile_meta["tx"]   = key.first;
        tile_meta["ty"]   = key.second;
        tile_meta["x_min"] = key.first  * tile_size;
        tile_meta["y_min"] = key.second * tile_size;
        tile_meta["x_max"] = (key.first + 1)  * tile_size;
        tile_meta["y_max"] = (key.second + 1) * tile_size;
        tile_meta["num_points"] = tile_cloud->size();
        tile_meta["file"] = tile_name + ".pcd";
        tile_index.push_back(tile_meta);
    }

    std::ofstream ofs(tile_dir + "/tile_index.json");
    ofs << tile_index.dump(2);

    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Saved %zu tiles to %s", tiles.size(), tile_dir.c_str());
    return true;
}

bool MapExporter::exportTrajectory(const std::vector<SubMap::Ptr>& submaps,
                                    const std::string& traj_dir,
                                    bool tum, bool kitti, bool json_poses) const {
    // Collect all keyframes in order
    std::vector<std::pair<double, KeyFrame::Ptr>> kf_list;
    for (const auto& sm : submaps) {
        for (const auto& kf : sm->keyframes) {
            kf_list.push_back({kf->timestamp, kf});
        }
    }
    std::sort(kf_list.begin(), kf_list.end());

    std::vector<double>  timestamps;
    std::vector<Pose3d>  poses_raw;
    std::vector<Pose3d>  poses_opt;

    for (const auto& [ts, kf] : kf_list) {
        timestamps.push_back(ts);
        poses_raw.push_back(kf->T_w_b);
        poses_opt.push_back(kf->T_w_b_optimized);
    }

    if (tum) {
        utils::savePosesTUM(traj_dir + "/optimized_trajectory_tum.txt",
                            timestamps, poses_opt);
        utils::savePosesTUM(traj_dir + "/raw_trajectory_tum.txt",
                            timestamps, poses_raw);
    }
    if (kitti) {
        utils::savePosesKITTI(traj_dir + "/optimized_trajectory_kitti.txt", poses_opt);
        utils::savePosesKITTI(traj_dir + "/raw_trajectory_kitti.txt", poses_raw);
    }

    if (json_poses) {
        nlohmann::json json_arr;
        for (const auto& [ts, kf] : kf_list) {
            nlohmann::json entry;
            entry["id"]        = kf->id;
            entry["timestamp"] = ts;
            entry["submap_id"] = kf->submap_id;
            entry["has_gps"]   = kf->has_valid_gps;
            const auto& p = kf->T_w_b_optimized.translation();
            Eigen::Quaterniond q(kf->T_w_b_optimized.rotation());
            entry["position"] = {p.x(), p.y(), p.z()};
            entry["orientation"] = {q.x(), q.y(), q.z(), q.w()};
            json_arr.push_back(entry);
        }
        std::ofstream ofs(traj_dir + "/keyframe_poses.json");
        ofs << json_arr.dump(2);
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Trajectory exported: %zu keyframes", kf_list.size());
    return true;
}

bool MapExporter::exportLoopReport(const std::vector<LoopConstraint::Ptr>& loops,
                                    const std::string& path) const {
    nlohmann::json report;
    for (const auto& lc : loops) {
        nlohmann::json entry;
        entry["submap_i"]     = lc->submap_i;
        entry["submap_j"]     = lc->submap_j;
        entry["inlier_ratio"] = lc->inlier_ratio;
        entry["rmse"]         = lc->rmse;
        entry["overlap_score"] = lc->overlap_score;
        entry["is_inter_session"] = lc->is_inter_session;
        entry["status"]       = static_cast<int>(lc->status);
        const auto& t = lc->delta_T.translation();
        entry["delta_t"] = {t.x(), t.y(), t.z()};
        report.push_back(entry);
    }
    std::ofstream ofs(path);
    ofs << report.dump(2);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Loop report: %zu loops → %s", loops.size(), path.c_str());
    return true;
}

bool MapExporter::savePLY(const CloudXYZIPtr& cloud, const std::string& path) const {
    int ret = pcl::io::savePLYFileBinary(path, *cloud);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"), "[MapExporter] Failed to save PLY: %s", path.c_str());
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Saved PLY: %s (%zu pts)", path.c_str(), cloud->size());
    return true;
}

bool MapExporter::saveLAS(const CloudXYZIPtr& cloud, const std::string& path) const {
    // LAS format requires libLAS or PDAL; write minimal ASCII LAS-like format
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << std::fixed << std::setprecision(4);
    ofs << "x y z intensity\n";
    for (const auto& pt : cloud->points) {
        ofs << pt.x << " " << pt.y << " " << pt.z << " " << pt.intensity << "\n";
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapExporter] Saved LAS (ASCII): %s (%zu pts)", path.c_str(), cloud->size());
    return true;
}

}  // namespace automap_pro
