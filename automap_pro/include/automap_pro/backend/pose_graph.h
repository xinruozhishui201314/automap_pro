#pragma once

#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <functional>
#include <unordered_map>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// PoseGraph: 位姿图数据结构
// 节点: 关键帧或子图位姿
// 边: Odom约束 / Loop约束 / GPS约束
// ─────────────────────────────────────────────────────────────────────────────

enum class EdgeType {
    ODOM,       // 前端里程计边
    LOOP,       // 回环检测边
    GPS,        // GPS绝对约束边
    PRIOR       // 先验约束边
};

struct GraphNode {
    int      id        = -1;
    Pose3d   pose      = Pose3d::Identity();
    Pose3d   pose_opt  = Pose3d::Identity();
    bool     fixed     = false;
    double   timestamp = 0.0;
    
    using Ptr = std::shared_ptr<GraphNode>;
};

struct GraphEdge {
    int        id        = -1;     // 边的唯一ID
    int        from    = -1;
    int        to      = -1;  // to < 0 表示 unary factor (GPS)
    EdgeType   type   = EdgeType::ODOM;
    Pose3d     measurement = Pose3d::Identity();
    Mat66d     information = Mat66d::Identity();
    float      confidence = 1.0f;  // [0,1] 用于回环约束过滤
    double     timestamp = 0.0;
    
    using Ptr = std::shared_ptr<GraphEdge>;
};

class PoseGraph {
public:
    PoseGraph() = default;
    ~PoseGraph() = default;

    // 节点管理
    int addNode(const GraphNode& node);
    bool updateNodePose(int id, const Pose3d& pose);
    bool updateNodePoseOpt(int id, const Pose3d& pose);
    GraphNode::Ptr getNode(int id) const;
    std::vector<GraphNode> allNodes() const;
    int nodeCount() const;

    // 边管理
    int addEdge(const GraphEdge& edge);
    GraphEdge::Ptr getEdge(int id) const;
    std::vector<GraphEdge> allEdges() const;
    std::vector<GraphEdge> edgesBetween(int from, int to) const;
    std::vector<GraphEdge> odomEdgesFrom(int from) const;
    std::vector<GraphEdge> loopEdgesFrom(int from) const;
    int edgeCount() const;

    // 图查询
    bool hasNode(int id) const;
    bool hasEdge(int id) const;
    int getLastNodeId() const;
    std::vector<int> getNodeIds() const;
    std::vector<int> getNeighborNodes(int node_id) const;
    
    // 路径查询（最短路径）
    std::vector<int> findPath(int start, int end) const;
    double computePathLength(const std::vector<int>& path) const;

    // 清理
    void clear();
    void removeOldNodes(double max_age_seconds);
    void removeEdge(int id);

    // 回调
    using NodeUpdateCallback = std::function<void(int, const Pose3d&)>;
    void registerNodeUpdateCallback(NodeUpdateCallback cb);

private:
    mutable std::mutex mutex_;
    
    std::unordered_map<int, GraphNode> nodes_;
    std::unordered_map<int, GraphEdge> edges_;
    
    std::map<int, std::vector<int>> adjacency_list_;  // from -> [to, to, ...]
    std::map<int, std::vector<int>> reverse_adjacency_list_;  // to -> [from, from, ...]
    
    int node_id_counter_ = 0;
    int edge_id_counter_ = 0;
    
    std::vector<NodeUpdateCallback> node_update_cbs_;
    
    void updateAdjacencyLists();
};

} // namespace automap_pro
