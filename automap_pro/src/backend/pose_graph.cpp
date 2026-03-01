#include "automap_pro/backend/pose_graph.h"
#include "automap_pro/core/utils.h"

#include <fstream>
#include <iomanip>

namespace automap_pro {

void PoseGraph::addNode(int id, NodeType type, const Pose3d& pose, bool fixed) {
    std::lock_guard<std::mutex> lk(mutex_);
    PoseNode node;
    node.id             = id;
    node.type           = type;
    node.pose           = pose;
    node.pose_optimized = pose;
    node.fixed          = fixed;
    nodes_[id]          = node;
}

void PoseGraph::updateNodePose(int id, const Pose3d& pose) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = nodes_.find(id);
    if (it != nodes_.end()) {
        it->second.pose_optimized = pose;
    }
}

bool PoseGraph::hasNode(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return nodes_.count(id) > 0;
}

PoseNode PoseGraph::getNode(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = nodes_.find(id);
    if (it != nodes_.end()) return it->second;
    return {};
}

std::vector<PoseNode> PoseGraph::allNodes() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<PoseNode> out;
    for (const auto& [id, n] : nodes_) out.push_back(n);
    return out;
}

void PoseGraph::addOdomEdge(int from, int to, const Pose3d& meas, const Mat66d& info) {
    std::lock_guard<std::mutex> lk(mutex_);
    PoseEdge e;
    e.from        = from;
    e.to          = to;
    e.type        = EdgeType::ODOMETRY;
    e.measurement = meas;
    e.information = info;
    edges_.push_back(e);
}

void PoseGraph::addLoopEdge(int from, int to, const Pose3d& meas, const Mat66d& info) {
    std::lock_guard<std::mutex> lk(mutex_);
    PoseEdge e;
    e.from        = from;
    e.to          = to;
    e.type        = EdgeType::LOOP;
    e.measurement = meas;
    e.information = info;
    edges_.push_back(e);
}

void PoseGraph::addGPSEdge(int node_id, const Eigen::Vector3d& gps_pos,
                             const Eigen::Matrix3d& gps_cov) {
    std::lock_guard<std::mutex> lk(mutex_);
    PoseEdge e;
    e.from = node_id;
    e.to   = -1;   // unary
    e.type = EdgeType::GPS;
    // Store GPS position in translation part of measurement
    e.measurement.translation() = gps_pos;
    e.measurement.linear()      = Eigen::Matrix3d::Identity();
    // Inflate 3x3 cov to 6x6
    e.information = Mat66d::Zero();
    e.information.block<3,3>(0,0) = gps_cov.inverse();
    edges_.push_back(e);
}

void PoseGraph::addSubmapEdge(int from, int to, const Pose3d& meas, const Mat66d& info) {
    std::lock_guard<std::mutex> lk(mutex_);
    PoseEdge e;
    e.from        = from;
    e.to          = to;
    e.type        = EdgeType::SUBMAP_ODOM;
    e.measurement = meas;
    e.information = info;
    edges_.push_back(e);
}

std::vector<PoseEdge> PoseGraph::allEdges() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return edges_;
}

std::vector<PoseEdge> PoseGraph::edgesOfType(EdgeType t) const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<PoseEdge> out;
    for (const auto& e : edges_) if (e.type == t) out.push_back(e);
    return out;
}

int PoseGraph::numNodes() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(nodes_.size());
}

int PoseGraph::numEdges() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(edges_.size());
}

int PoseGraph::numLoopEdges() const {
    std::lock_guard<std::mutex> lk(mutex_);
    int cnt = 0;
    for (const auto& e : edges_) if (e.type == EdgeType::LOOP) cnt++;
    return cnt;
}

std::map<int, Pose3d> PoseGraph::getOptimizedPoses() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::map<int, Pose3d> out;
    for (const auto& [id, n] : nodes_) out[id] = n.pose_optimized;
    return out;
}

bool PoseGraph::saveG2O(const std::string& path) const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << std::fixed << std::setprecision(9);

    for (const auto& [id, node] : nodes_) {
        const auto& T = node.pose_optimized;
        Eigen::Quaterniond q(T.rotation());
        const auto& p = T.translation();
        ofs << "VERTEX_SE3:QUAT " << id << " "
            << p.x() << " " << p.y() << " " << p.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        if (node.fixed) ofs << "FIX " << id << "\n";
    }

    for (const auto& edge : edges_) {
        if (edge.to < 0) continue;  // GPS unary: skip for g2o
        const auto& T = edge.measurement;
        Eigen::Quaterniond q(T.rotation());
        const auto& p = T.translation();
        ofs << "EDGE_SE3:QUAT " << edge.from << " " << edge.to << " "
            << p.x() << " " << p.y() << " " << p.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        // Upper-triangle of information matrix (6x6)
        const auto& I = edge.information;
        for (int r = 0; r < 6; ++r)
            for (int c = r; c < 6; ++c)
                ofs << I(r,c) << " ";
        ofs << "\n";
    }
    return true;
}

void PoseGraph::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    nodes_.clear();
    edges_.clear();
}

}  // namespace automap_pro
