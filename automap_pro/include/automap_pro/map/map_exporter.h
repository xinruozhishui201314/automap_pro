#pragma once

#include <string>
#include <vector>
#include <map>

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/pose_graph.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// MapExporter: exports global map in multiple formats,
// handles tiling and trajectory export
// ──────────────────────────────────────────────────────────
class MapExporter {
public:
    struct ExportOptions {
        std::string output_dir;
        bool save_pcd      = true;
        bool save_ply      = true;
        bool save_las      = false;
        bool tiling        = true;
        double tile_size   = 100.0;
        bool save_traj_tum  = true;
        bool save_traj_kitti = true;
        bool save_poses_json = true;
        bool save_loop_report = true;
        bool save_pose_graph  = true;
        bool save_submaps    = true;
    };

    MapExporter();
    ~MapExporter() = default;

    bool exportAll(const CloudXYZIPtr& global_cloud,
                   const std::vector<SubMap::Ptr>& submaps,
                   const std::vector<LoopConstraint::Ptr>& loop_constraints,
                   const PoseGraph& pose_graph,
                   const ExportOptions& opts);

    // Export a single point cloud in all configured formats
    bool exportCloud(const CloudXYZIPtr& cloud, const std::string& base_path,
                     bool pcd, bool ply, bool las) const;

    // Tile the point cloud and save tiles
    bool exportTiles(const CloudXYZIPtr& cloud,
                     const std::string& tile_dir,
                     double tile_size) const;

    // Save trajectory files
    bool exportTrajectory(const std::vector<SubMap::Ptr>& submaps,
                           const std::string& traj_dir,
                           bool tum, bool kitti, bool json_poses) const;

    // Save loop closure report
    bool exportLoopReport(const std::vector<LoopConstraint::Ptr>& loops,
                           const std::string& path) const;

private:
    bool savePLY(const CloudXYZIPtr& cloud, const std::string& path) const;
    bool saveLAS(const CloudXYZIPtr& cloud, const std::string& path) const;
};

}  // namespace automap_pro
