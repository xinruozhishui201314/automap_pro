#pragma once
/**
 * @file backend/pose_graph.h
 * @brief 后端优化：iSAM2、HBA、GPS/回环因子、任务队列与坐标管理。
 */


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

/** 节点类型（用于 HBAWrapper 层级图：子图级 / 关键帧级） */
enum class NodeType {
    SUBMAP,
    KEYFRAME
};

struct GraphNode {
    int       id        = -1;
    NodeType  type      = NodeType::SUBMAP;  // 用于 HBAWrapper 区分层级
    Pose3d    pose      = Pose3d::Identity();
    Pose3d    pose_opt  = Pose3d::Identity();
    bool      fixed     = false;
    double    timestamp = 0.0;

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
    /** 便捷接口：按 id/type/pose/fixed 添加节点（HBAWrapper 等使用） */
    int addNode(int id, NodeType type, const Pose3d& pose, bool fixed = false);
    bool updateNodePose(int id, const Pose3d& pose);
    bool updateNodePoseOpt(int id, const Pose3d& pose);
    GraphNode::Ptr getNode(int id) const;
    std::vector<GraphNode> allNodes() const;
    int nodeCount() const;

    // 边管理
    int addEdge(const GraphEdge& edge);
    /** 便捷接口：添加里程计边（HBAWrapper 等使用） */
    int addOdomEdge(int from, int to, const Pose3d& rel_pose, const Mat66d& information);
    /** 便捷接口：添加回环边 */
    int addLoopEdge(int from, int to, const Pose3d& rel_pose, const Mat66d& information);
    /** 便捷接口：添加 GPS 一元边（to 用 -1 表示） */
    int addGPSEdge(int node_id, const Eigen::Vector3d& gps_pos, const Eigen::Matrix3d& gps_cov);
    GraphEdge::Ptr getEdge(int id) const;
    std::vector<GraphEdge> allEdges() const;
    std::vector<GraphEdge> edgesBetween(int from, int to) const;
    std::vector<GraphEdge> odomEdgesFrom(int from) const;
    std::vector<GraphEdge> loopEdgesFrom(int from) const;
    int edgeCount() const;

    /** 回环边数量（HBAWrapper 日志用） */
    int numLoopEdges() const;
    /** 与 nodeCount() 一致，别名（HBAWrapper 使用） */
    int numNodes() const { return nodeCount(); }
    /** 与 edgeCount() 一致，别名（HBAWrapper 使用） */
    int numEdges() const { return edgeCount(); }
    /** 优化后位姿映射 id -> pose（写回后为 pose，否则 pose_opt 若存在） */
    std::map<int, Pose3d> getOptimizedPoses() const;

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
