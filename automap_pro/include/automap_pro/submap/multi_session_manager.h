#pragma once

#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/core/data_types.h"
#include <memory>
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace automap_pro {

/**
 * @brief 会话信息
 */
struct SessionInfo {
    uint64_t session_id;
    std::string session_name;
    double start_time;
    double end_time;
    int num_submaps;
    int num_keyframes;
    std::string data_path;
    Pose3d start_pose;
    Pose3d end_pose;
    double total_distance;
    bool is_active;
    bool is_optimized;
    std::vector<int> submap_ids;
    
    // 元数据
    std::string sensor_config;      // 传感器配置
    std::string environment_type;    // 环境类型（室内/室外/混合）
    double map_size;               // 地图大小（m^2）
};

/**
 * @brief 跨会话约束
 */
struct CrossSessionConstraint {
    uint64_t session_id1;
    int submap_id1;
    uint64_t session_id2;
    int submap_id2;
    Eigen::Matrix4d transform;       // session1到session2的变换
    Eigen::Matrix6d information;    // 信息矩阵
    double fitness_score;            // 匹配分数
    bool is_verified;                // 是否已验证
    double timestamp;
};

/**
 * @brief 多会话管理器
 * 
 * 功能：
 * 1. 多会话建图支持
 * 2. 跨会话回环检测
 * 3. 会话间位姿对齐
 * 4. 会话持久化与加载
 * 5. 多会话地图合并
 * 6. 会话管理API（创建/删除/查询）
 * 
 * 使用场景：
 * - M2DGR数据集多日建图
 * - 大规模环境分段建图
 * - 地图更新与维护
 * - 多机器人协同建图
 */
class MultiSessionManager : public rclcpp::Node {
public:
    explicit MultiSessionManager(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("multi_session_manager", node_options) {
        
        // 声明参数
        this->declare_parameter("session_root_dir", std::string("/data/automap_sessions"));
        this->declare_parameter("max_sessions", 100);
        this->declare_parameter("auto_save_interval", 60.0);  // 自动保存间隔（秒）
        this->declare_parameter("enable_cross_session_loop", true);
        this->declare_parameter("cross_session_search_radius", 200.0);  // 跨会话搜索半径（米）
        this->declare_parameter("cross_session_min_overlap", 0.15);        // 跨会话最小重叠度
        this->declare_parameter("auto_merge_sessions", false);               // 自动合并会话
        this->declare_parameter("merge_threshold", 0.8);                   // 合并阈值
        
        // 获取参数
        session_root_dir_ = this->get_parameter("session_root_dir").as_string();
        max_sessions_ = this->get_parameter("max_sessions").as_int();
        auto_save_interval_ = this->get_parameter("auto_save_interval").as_double();
        enable_cross_session_loop_ = this->get_parameter("enable_cross_session_loop").as_bool();
        cross_session_search_radius_ = this->get_parameter("cross_session_search_radius").as_double();
        cross_session_min_overlap_ = this->get_parameter("cross_session_min_overlap").as_double();
        auto_merge_sessions_ = this->get_parameter("auto_merge_sessions").as_bool();
        merge_threshold_ = this->get_parameter("merge_threshold").as_double();
        
        RCLCPP_INFO(this->get_logger(), "[MultiSessionManager] Initializing...");
        RCLCPP_INFO(this->get_logger(), "[MultiSessionManager] Session root: %s", 
                    session_root_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "[MultiSessionManager] Cross-session loop: %s", 
                    enable_cross_session_loop_ ? "enabled" : "disabled");
        
        // 创建会话目录
        ensureSessionDirectory();
        
        // 加载现有会话
        loadExistingSessions();
        
        // 创建发布者
        session_list_pub_ = this->create_publisher<automap_pro::msg::SessionList>(
            "/session_manager/session_list", 10);
        
        cross_constraint_pub_ = this->create_publisher<automap_pro::msg::CrossSessionConstraint>(
            "/session_manager/cross_constraint", 10);
        
        // 创建服务
        create_session_srv_ = this->create_service<automap_pro::srv::CreateSession>(
            "/session_manager/create_session",
            std::bind(&MultiSessionManager::createSessionService, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        load_session_srv_ = this->create_service<automap_pro::srv::LoadSession>(
            "/session_manager/load_session",
            std::bind(&MultiSessionManager::loadSessionService, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        delete_session_srv_ = this->create_service<automap_pro::srv::DeleteSession>(
            "/session_manager/delete_session",
            std::bind(&MultiSessionManager::deleteSessionService, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        get_session_info_srv_ = this->create_service<automap_pro::srv::GetSessionInfo>(
            "/session_manager/get_session_info",
            std::bind(&MultiSessionManager::getSessionInfoService, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        merge_sessions_srv_ = this->create_service<automap_pro::srv::MergeSessions>(
            "/session_manager/merge_sessions",
            std::bind(&MultiSessionManager::mergeSessionsService, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // 初始化状态
        active_session_id_ = 0;
        session_id_counter_ = 0;
        last_auto_save_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "[MultiSessionManager] Ready!");
    }
    
    /**
     * @brief 设置子图管理器
     */
    void setSubmapManager(SubMapManager* submap_manager) {
        submap_manager_ = submap_manager;
    }
    
    /**
     * @brief 创建新会话
     */
    uint64_t createSession(const std::string& name = "",
                          const std::string& sensor_config = "");
    
    /**
     * @brief 切换活跃会话
     */
    bool switchSession(uint64_t session_id);
    
    /**
     * @brief 结束当前会话
     */
    bool finishCurrentSession();
    
    /**
     * @brief 加载会话
     */
    bool loadSession(uint64_t session_id);
    
    /**
     * @brief 删除会话
     */
    bool deleteSession(uint64_t session_id);
    
    /**
     * @brief 获取会话信息
     */
    SessionInfo getSessionInfo(uint64_t session_id) const;
    
    /**
     * @brief 获取所有会话
     */
    std::vector<SessionInfo> getAllSessions() const;
    
    /**
     * @brief 获取活跃会话ID
     */
    uint64_t getActiveSessionId() const { return active_session_id_; }
    
    /**
     * @brief 获取活跃会话信息
     */
    SessionInfo getActiveSessionInfo() const;
    
    /**
     * @brief 添加跨会话约束
     */
    bool addCrossSessionConstraint(const CrossSessionConstraint& constraint);
    
    /**
     * @brief 获取跨会话约束
     */
    std::vector<CrossSessionConstraint> getCrossSessionConstraints(
        uint64_t session_id) const;
    
    /**
     * @brief 跨会话回环检测
     */
    std::vector<CrossSessionConstraint> detectCrossSessionLoops(
        uint64_t current_session_id);
    
    /**
     * @brief 合并多个会话
     */
    bool mergeSessions(const std::vector<uint64_t>& session_ids,
                      const std::string& merged_name = "");
    
    /**
     * @brief 保存会话
     */
    bool saveSession(uint64_t session_id);
    
    /**
     * @brief 自动保存所有会话
     */
    void autoSaveSessions();
    
    /**
     * @brief 保存所有会话到磁盘
     */
    bool saveAllSessions();
    
    /**
     * @brief 优化会话（使用跨会话约束）
     */
    bool optimizeSessions(const std::vector<uint64_t>& session_ids);
    
    /**
     * @brief 构建全局地图（多会话）
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr buildGlobalMap(
        const std::vector<uint64_t>& session_ids,
        float voxel_size = 0.2f) const;

private:
    /**
     * @brief 确保会话目录存在
     */
    void ensureSessionDirectory();
    
    /**
     * @brief 加载现有会话
     */
    void loadExistingSessions();
    
    /**
     * @brief 生成会话ID
     */
    uint64_t generateSessionId();
    
    /**
     * @brief 保存会话到YAML
     */
    bool saveSessionToYAML(const SessionInfo& session);
    
    /**
     * @brief 从YAML加载会话
     */
    bool loadSessionFromYAML(const std::string& filepath,
                            SessionInfo& session);
    
    /**
     * @brief 检查会话是否重叠
     */
    bool checkSessionOverlap(const SessionInfo& session1,
                            const SessionInfo& session2) const;
    
    /**
     * @brief 计算会话距离
     */
    double computeSessionDistance(const SessionInfo& session1,
                                  const SessionInfo& session2) const;
    
    /**
     * @brief 发布会话列表
     */
    void publishSessionList();
    
    /**
     * @brief 发布跨会话约束
     */
    void publishCrossConstraints();
    
    // ROS2服务回调
    void createSessionService(
        const std::shared_ptr<automap_pro::srv::CreateSession::Request> request,
        std::shared_ptr<automap_pro::srv::CreateSession::Response> response);
    
    void loadSessionService(
        const std::shared_ptr<automap_pro::srv::LoadSession::Request> request,
        std::shared_ptr<automap_pro::srv::LoadSession::Response> response);
    
    void deleteSessionService(
        const std::shared_ptr<automap_pro::srv::DeleteSession::Request> request,
        std::shared_ptr<automap_pro::srv::DeleteSession::Response> response);
    
    void getSessionInfoService(
        const std::shared_ptr<automap_pro::srv::GetSessionInfo::Request> request,
        std::shared_ptr<automap_pro::srv::GetSessionInfo::Response> response);
    
    void mergeSessionsService(
        const std::shared_ptr<automap_pro::srv::MergeSessions::Request> request,
        std::shared_ptr<automap_pro::srv::MergeSessions::Response> response);
    
    // 基础组件
    SubMapManager* submap_manager_;
    
    // ROS2节点组件
    rclcpp::Publisher<automap_pro::msg::SessionList>::SharedPtr session_list_pub_;
    rclcpp::Publisher<automap_pro::msg::CrossSessionConstraint>::SharedPtr cross_constraint_pub_;
    rclcpp::Service<automap_pro::srv::CreateSession>::SharedPtr create_session_srv_;
    rclcpp::Service<automap_pro::srv::LoadSession>::SharedPtr load_session_srv_;
    rclcpp::Service<automap_pro::srv::DeleteSession>::SharedPtr delete_session_srv_;
    rclcpp::Service<automap_pro::srv::GetSessionInfo>::SharedPtr get_session_info_srv_;
    rclcpp::Service<automap_pro::srv::MergeSessions>::SharedPtr merge_sessions_srv_;
    
    // 会话数据
    std::unordered_map<uint64_t, SessionInfo> sessions_;
    mutable std::mutex sessions_mutex_;
    
    // 跨会话约束
    std::vector<CrossSessionConstraint> cross_constraints_;
    mutable std::mutex constraints_mutex_;
    
    // 状态
    uint64_t active_session_id_;
    std::atomic<uint64_t> session_id_counter_;
    rclcpp::Time last_auto_save_time_;
    
    // 配置参数
    std::string session_root_dir_;
    int max_sessions_;
    double auto_save_interval_;
    bool enable_cross_session_loop_;
    double cross_session_search_radius_;
    double cross_session_min_overlap_;
    bool auto_merge_sessions_;
    double merge_threshold_;
};

}  // namespace automap_pro
