#pragma once

#include <map>
#include <vector>
#include <mutex>
#include <functional>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// PoseGraph: maintains nodes and edges for optimization
// ──────────────────────────────────────────────────────────
class PoseGraph {
public:
    PoseGraph() = default;
    ~PoseGraph() = default;

    // Nodes
    void addNode(int id, NodeType type, const Pose3d& pose, bool fixed = false);
    void updateNodePose(int id, const Pose3d& pose);
    bool hasNode(int id) const;
    PoseNode getNode(int id) const;
    std::vector<PoseNode> allNodes() const;

    // Edges
    void addOdomEdge(int from, int to, const Pose3d& measurement,
                     const Mat66d& information);
    void addLoopEdge(int from, int to, const Pose3d& measurement,
                     const Mat66d& information);
    void addGPSEdge (int node_id, const Eigen::Vector3d& gps_pos,
                     const Eigen::Matrix3d& gps_cov);
    void addSubmapEdge(int from, int to, const Pose3d& measurement,
                       const Mat66d& information);

    std::vector<PoseEdge> allEdges() const;
    std::vector<PoseEdge> edgesOfType(EdgeType t) const;

    int numNodes()      const;
    int numEdges()      const;
    int numLoopEdges()  const;

    // Get optimized poses
    std::map<int, Pose3d> getOptimizedPoses() const;

    // Save/load as g2o format
    bool saveG2O (const std::string& path) const;
    bool loadG2O (const std::string& path);

    void clear();

private:
    mutable std::mutex mutex_;
    std::map<int, PoseNode> nodes_;
    std::vector<PoseEdge>   edges_;
};

}  // namespace automap_pro
