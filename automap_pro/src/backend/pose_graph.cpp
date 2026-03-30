/**
 * @file backend/pose_graph.cpp
 * @brief 后端优化与因子实现。
 */
#include "automap_pro/backend/pose_graph.h"
#include "automap_pro/core/logger.h"
#include <queue>
#include <algorithm>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 节点管理
// ─────────────────────────────────────────────────────────────────────────────

int PoseGraph::addNode(const GraphNode& node) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    GraphNode new_node = node;
    if (new_node.id < 0) {
        new_node.id = node_id_counter_++;
    } else {
        node_id_counter_ = std::max(node_id_counter_, new_node.id + 1);
    }
    
    nodes_[new_node.id] = new_node;
    return new_node.id;
}

int PoseGraph::addNode(int id, NodeType type, const Pose3d& pose, bool fixed) {
    GraphNode node;
    node.id = id;
    node.type = type;
    node.pose = pose;
    node.pose_opt = pose;
    node.fixed = fixed;
    node.timestamp = 0.0;
    return addNode(node);
}

bool PoseGraph::updateNodePose(int id, const Pose3d& pose) {
    // 复制回调列表（在锁外调用，避免死锁）
    std::vector<NodeUpdateCallback> callbacks_copy;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        
        auto it = nodes_.find(id);
        if (it == nodes_.end()) return false;
        
        it->second.pose = pose;
        
        // 复制回调列表
        callbacks_copy = node_update_cbs_;
    }
    
    // 在锁外触发回调
    for (auto& cb : callbacks_copy) {
        try {
            cb(id, pose);
        } catch (const std::exception& e) {
            ALOG_ERROR("PoseGraph", "Node update callback exception: %s", e.what());
        } catch (...) {
            ALOG_ERROR("PoseGraph", "Node update callback unknown exception");
        }
    }
    return true;
}

bool PoseGraph::updateNodePoseOpt(int id, const Pose3d& pose) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto it = nodes_.find(id);
    if (it == nodes_.end()) return false;
    
    it->second.pose_opt = pose;
    return true;
}

GraphNode::Ptr PoseGraph::getNode(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto it = nodes_.find(id);
    if (it == nodes_.end()) return nullptr;
    
    return std::make_shared<GraphNode>(it->second);
}

std::vector<GraphNode> PoseGraph::allNodes() const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<GraphNode> nodes;
    nodes.reserve(nodes_.size());
    
    for (const auto& [id, node] : nodes_) {
        nodes.push_back(node);
    }
    return nodes;
}

int PoseGraph::nodeCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(nodes_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 边管理
// ─────────────────────────────────────────────────────────────────────────────

int PoseGraph::addEdge(const GraphEdge& edge) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    GraphEdge new_edge = edge;
    if (new_edge.id < 0) {
        new_edge.id = edge_id_counter_++;
    } else {
        edge_id_counter_ = std::max(edge_id_counter_, new_edge.id + 1);
    }
    
    edges_[new_edge.id] = new_edge;
    
    // 更新邻接表（仅双向边）
    if (new_edge.to >= 0) {
        adjacency_list_[new_edge.from].push_back(new_edge.to);
        reverse_adjacency_list_[new_edge.to].push_back(new_edge.from);
    }
    
    return new_edge.id;
}

int PoseGraph::addOdomEdge(int from, int to, const Pose3d& rel_pose, const Mat66d& information) {
    GraphEdge e;
    e.from = from;
    e.to = to;
    e.type = EdgeType::ODOM;
    e.measurement = rel_pose;
    e.information = information;
    return addEdge(e);
}

int PoseGraph::addLoopEdge(int from, int to, const Pose3d& rel_pose, const Mat66d& information) {
    GraphEdge e;
    e.from = from;
    e.to = to;
    e.type = EdgeType::LOOP;
    e.measurement = rel_pose;
    e.information = information;
    return addEdge(e);
}

int PoseGraph::addGPSEdge(int node_id, const Eigen::Vector3d& gps_pos, const Eigen::Matrix3d& gps_cov) {
    GraphEdge e;
    e.from = node_id;
    e.to = -1;  // unary
    e.type = EdgeType::GPS;
    e.measurement = Pose3d::Identity();
    e.measurement.translation() = gps_pos;
    e.information = Mat66d::Identity();

    // 使用 CompleteOrthogonalDecomposition 进行数值稳定的伪逆计算
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> cod(gps_cov);
    Eigen::Matrix3d pos_info;

    if (cod.isInvertible()) {
        // 矩阵可逆，使用 solve 方法求逆 (A^(-1) = solve(I))
        pos_info = cod.solve(Eigen::Matrix3d::Identity());
    } else {
        // 矩阵奇异或接近奇异，使用伪逆
        ALOG_WARN("PoseGraph", "GPS covariance matrix is singular or near-singular, using pseudo-inverse");
        pos_info = cod.pseudoInverse();
    }

    // 检查求逆结果是否有效
    if (!pos_info.allFinite()) {
        ALOG_WARN("PoseGraph", "GPS covariance inverse contains NaN/Inf, using default");
        pos_info = Eigen::Matrix3d::Identity() * 1e-3;
    }

    e.information.block<3,3>(0,0) = pos_info;
    e.information.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 1e-6;  // 旋转弱约束
    return addEdge(e);
}

GraphEdge::Ptr PoseGraph::getEdge(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto it = edges_.find(id);
    if (it == edges_.end()) return nullptr;
    
    return std::make_shared<GraphEdge>(it->second);
}

std::vector<GraphEdge> PoseGraph::allEdges() const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<GraphEdge> edges;
    edges.reserve(edges_.size());
    
    for (const auto& [id, edge] : edges_) {
        edges.push_back(edge);
    }
    return edges;
}

std::vector<GraphEdge> PoseGraph::edgesBetween(int from, int to) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<GraphEdge> result;
    for (const auto& [id, edge] : edges_) {
        if (edge.from == from && edge.to == to) {
            result.push_back(edge);
        }
    }
    return result;
}

std::vector<GraphEdge> PoseGraph::odomEdgesFrom(int from) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<GraphEdge> result;
    for (const auto& [id, edge] : edges_) {
        if (edge.from == from && edge.type == EdgeType::ODOM && edge.to >= 0) {
            result.push_back(edge);
        }
    }
    return result;
}

std::vector<GraphEdge> PoseGraph::loopEdgesFrom(int from) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<GraphEdge> result;
    for (const auto& [id, edge] : edges_) {
        if (edge.from == from && edge.type == EdgeType::LOOP) {
            result.push_back(edge);
        }
    }
    return result;
}

int PoseGraph::edgeCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(edges_.size());
}

int PoseGraph::numLoopEdges() const {
    std::lock_guard<std::mutex> lk(mutex_);
    int n = 0;
    for (const auto& [id, edge] : edges_) {
        if (edge.type == EdgeType::LOOP) n++;
    }
    return n;
}

std::map<int, Pose3d> PoseGraph::getOptimizedPoses() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::map<int, Pose3d> out;
    for (const auto& [id, node] : nodes_) {
        out[id] = node.pose;  // 优化器写回后 pose 即为优化结果
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// 图查询
// ─────────────────────────────────────────────────────────────────────────────

bool PoseGraph::hasNode(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return nodes_.find(id) != nodes_.end();
}

bool PoseGraph::hasEdge(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return edges_.find(id) != edges_.end();
}

int PoseGraph::getLastNodeId() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (nodes_.empty()) return -1;
    
    int max_id = -1;
    for (const auto& [id, node] : nodes_) {
        if (id > max_id) max_id = id;
    }
    return max_id;
}

std::vector<int> PoseGraph::getNodeIds() const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<int> ids;
    ids.reserve(nodes_.size());
    
    for (const auto& [id, node] : nodes_) {
        ids.push_back(id);
    }
    return ids;
}

std::vector<int> PoseGraph::getNeighborNodes(int node_id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::vector<int> neighbors;
    
    // 正向邻接
    auto it = adjacency_list_.find(node_id);
    if (it != adjacency_list_.end()) {
        for (int neighbor : it->second) {
            neighbors.push_back(neighbor);
        }
    }
    
    // 反向邻接
    auto rit = reverse_adjacency_list_.find(node_id);
    if (rit != reverse_adjacency_list_.end()) {
        for (int neighbor : rit->second) {
            neighbors.push_back(neighbor);
        }
    }
    
    // 去重
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    
    return neighbors;
}

std::vector<int> PoseGraph::findPath(int start, int end) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 修复死锁：直接在已持锁状态下检查节点是否存在，不调用 hasNode()
    auto start_it = nodes_.find(start);
    auto end_it = nodes_.find(end);
    if (start_it == nodes_.end() || end_it == nodes_.end()) return {};
    if (start == end) return {start};
    
    // BFS 寻找最短路径
    // 修复越界：使用 map 替代 vector，避免 ID 不连续导致的越界
    std::queue<int> q;
    std::map<int, int> parent;      // child -> parent
    std::map<int, bool> visited;    // 使用 map 替代 vector，支持非连续 ID
    
    q.push(start);
    visited[start] = true;
    parent[start] = -1;
    
    bool found = false;
    while (!q.empty() && !found) {
        int current = q.front();
        q.pop();
        
        // 获取邻居
        std::vector<int> neighbors;
        auto it = adjacency_list_.find(current);
        if (it != adjacency_list_.end()) {
            neighbors = it->second;
        }
        
        for (int neighbor : neighbors) {
            // 检查邻居节点是否存在（避免引用已删除的节点）
            if (nodes_.find(neighbor) == nodes_.end()) continue;
            
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
                
                if (neighbor == end) {
                    found = true;
                    break;
                }
            }
        }
    }
    
    if (!found) return {};
    
    // 重建路径
    std::vector<int> path;
    int current = end;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());
    
    return path;
}

double PoseGraph::computePathLength(const std::vector<int>& path) const {
    if (path.size() < 2) return 0.0;
    
    std::lock_guard<std::mutex> lk(mutex_);
    double total_length = 0.0;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int from_id = path[i];
        int to_id = path[i + 1];
        
        auto from_it = nodes_.find(from_id);
        auto to_it = nodes_.find(to_id);
        
        if (from_it != nodes_.end() && to_it != nodes_.end()) {
            double dist = (from_it->second.pose.translation() - 
                          to_it->second.pose.translation()).norm();
            total_length += dist;
        }
    }
    
    return total_length;
}

// ─────────────────────────────────────────────────────────────────────────────
// 清理
// ─────────────────────────────────────────────────────────────────────────────

void PoseGraph::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    nodes_.clear();
    edges_.clear();
    adjacency_list_.clear();
    reverse_adjacency_list_.clear();
    node_id_counter_ = 0;
    edge_id_counter_ = 0;
}

void PoseGraph::removeOldNodes(double max_age_seconds) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    double current_time = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::vector<int> nodes_to_remove;
    for (const auto& [id, node] : nodes_) {
        double age = current_time - node.timestamp;
        if (age > max_age_seconds && !node.fixed) {
            nodes_to_remove.push_back(id);
        }
    }
    
    // 先收集所有要删除的边（避免遍历 edges_ 时修改）
    std::vector<int> all_edges_to_remove;
    for (int id : nodes_to_remove) {
        for (const auto& [eid, edge] : edges_) {
            if (edge.from == id || edge.to == id) {
                all_edges_to_remove.push_back(eid);
            }
        }
    }
    std::sort(all_edges_to_remove.begin(), all_edges_to_remove.end());
    all_edges_to_remove.erase(std::unique(all_edges_to_remove.begin(), all_edges_to_remove.end()), all_edges_to_remove.end());
    
    // 删除边并同步更新邻接表（不调用 removeEdge，避免重复加锁）
    for (int eid : all_edges_to_remove) {
        auto it = edges_.find(eid);
        if (it == edges_.end()) continue;
        int from = it->second.from;
        int to = it->second.to;
        edges_.erase(it);
        auto adj_it = adjacency_list_.find(from);
        if (adj_it != adjacency_list_.end()) {
            auto& neighbors = adj_it->second;
            neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), to), neighbors.end());
        }
        auto rev_it = reverse_adjacency_list_.find(to);
        if (rev_it != reverse_adjacency_list_.end()) {
            auto& rev_neighbors = rev_it->second;
            rev_neighbors.erase(std::remove(rev_neighbors.begin(), rev_neighbors.end(), from), rev_neighbors.end());
        }
    }
    
    for (int id : nodes_to_remove) {
        nodes_.erase(id);
        adjacency_list_.erase(id);
        reverse_adjacency_list_.erase(id);
    }
}

void PoseGraph::removeEdge(int id) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto it = edges_.find(id);
    if (it != edges_.end()) {
        int from = it->second.from;
        int to = it->second.to;
        
        // 从邻接表中删除
        auto adj_it = adjacency_list_.find(from);
        if (adj_it != adjacency_list_.end()) {
            auto& neighbors = adj_it->second;
            // 仅删除一条 (from,to) 邻接关系，保持“多边”与邻接表的一致性
            auto pos = std::find(neighbors.begin(), neighbors.end(), to);
            if (pos != neighbors.end()) {
                neighbors.erase(pos);
            }
        }
        
        auto rev_it = reverse_adjacency_list_.find(to);
        if (rev_it != reverse_adjacency_list_.end()) {
            auto& rev_neighbors = rev_it->second;
            auto pos = std::find(rev_neighbors.begin(), rev_neighbors.end(), from);
            if (pos != rev_neighbors.end()) {
                rev_neighbors.erase(pos);
            }
        }
        
        edges_.erase(it);
    }
}

void PoseGraph::registerNodeUpdateCallback(NodeUpdateCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    node_update_cbs_.push_back(std::move(cb));
}

} // namespace automap_pro
