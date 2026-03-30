#pragma once
/**
 * @file submap/enhanced_submap_manager.h
 * @brief 子图/会话：生命周期、合并、持久化与 MS-Mapping 桥接。
 */


#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/core/data_types.h"
#include <memory>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <functional>
#include <Eigen/Dense>

namespace automap_pro {

/**
 * @brief 子图质量评估结果
 */
struct SubmapQuality {
    double geometric_quality;      // 几何质量（平面度/角点丰富度）
    double temporal_coverage;       // 时间覆盖率
    double spatial_coverage;        // 空间覆盖率
    double loop_density;           // 回环密度
    double overall_score;          // 综合质量分数
    
    bool is_low_quality() const { return overall_score < 0.5; }
    bool is_high_quality() const { return overall_score > 0.8; }
};

/**
 * @brief 子图配对（用于跨子图匹配）
 */
struct SubmapPair {
    int id1;
    int id2;
    double overlap_ratio;
    double distance_m;
    
    bool operator==(const SubmapPair& other) const {
        return (id1 == other.id1 && id2 == other.id2) ||
               (id1 == other.id2 && id2 == other.id1);
    }
};

/**
 * @brief 增强的子图管理器
 * 
 * V1优化功能：
 * 1. 自适应子图大小调整（基于地图复杂度）
 * 2. 子图质量评估与优化
 * 3. 智能子图合并策略
 * 4. 子图缓存与预加载
 * 5. 大规模建图的内存管理
 * 6. 子图级别LOD(Level of Detail)
 * 
 * 使用场景：
 * - M2DGR数据集大规模建图
 * - 多会话建图与回环
 * - 内存受限环境下的建图
 */
class EnhancedSubmapManager {
public:
    explicit EnhancedSubmapManager(SubMapManager* base_manager);
    
    ~EnhancedSubmapManager();
    
    /**
     * @brief 初始化
     */
    void init(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief 添加关键帧（增强版）
     */
    void addKeyFrameEnhanced(const KeyFrame::Ptr& kf);
    
    /**
     * @brief 获取子图质量
     */
    SubmapQuality evaluateSubmapQuality(int submap_id);
    
    /**
     * @brief 自适应调整子图大小
     */
    void adaptSubmapSize();
    
    /**
     * @brief 智能合并子图
     */
    void smartMergeSubmaps();
    
    /**
     * @brief 优化子图缓存
     */
    void optimizeSubmapCache();
    
    /**
     * @brief 构建LOD点云
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr buildLODMap(
        float voxel_size,
        int submap_id = -1) const;  // -1表示所有子图
    
    /**
     * @brief 获取子图配对列表（用于回环检测）
     */
    std::vector<SubmapPair> getCandidatePairs(
        int current_submap_id,
        double min_overlap = 0.1,
        double max_distance = 100.0) const;
    
    /**
     * @brief 内存使用统计
     */
    struct MemoryStats {
        size_t total_bytes;
        size_t cached_bytes;
        size_t loaded_bytes;
        int cached_count;
        int loaded_count;
        double memory_usage_percent;
    };
    MemoryStats getMemoryStats() const;
    
    /**
     * @brief 释放内存
     */
    void releaseMemory(double target_usage_percent = 80.0);
    
    /**
     * @brief 预加载子图（用于回环检测）
     */
    void preloadSubmap(int submap_id);
    
    /**
     * @brief 卸载子图（释放内存）
     */
    void unloadSubmap(int submap_id);
    
    /**
     * @brief 获取子图配对历史
     */
    std::vector<SubmapPair> getLoopHistory() const { return loop_history_; }
    
    /**
     * @brief 注册质量回调
     */
    using QualityCallback = std::function<void(int, const SubmapQuality&)>;
    void registerQualityCallback(QualityCallback cb) {
        quality_cbs_.push_back(std::move(cb));
    }

private:
    /**
     * @brief 评估几何质量
     */
    double evaluateGeometricQuality(const SubMap::Ptr& submap);
    
    /**
     * @brief 评估时间覆盖率
     */
    double evaluateTemporalCoverage(const SubMap::Ptr& submap);
    
    /**
     * @brief 评估空间覆盖率
     */
    double evaluateSpatialCoverage(const SubMap::Ptr& submap);
    
    /**
     * @brief 评估回环密度
     */
    double evaluateLoopDensity(int submap_id);
    
    /**
     * @brief 计算子图重叠度
     */
    double computeSubmapOverlap(const SubMap::Ptr& submap1,
                                const SubMap::Ptr& submap2) const;
    
    /**
     * @brief 计算子图距离
     */
    double computeSubmapDistance(const SubMap::Ptr& submap1,
                                 const SubMap::Ptr& submap2) const;
    
    /**
     * @brief 检查是否需要合并
     */
    bool shouldMerge(const SubMap::Ptr& submap1,
                     const SubMap::Ptr& submap2) const;
    
    /**
     * @brief 合并两个子图
     */
    SubMap::Ptr mergeSubmaps(const SubMap::Ptr& submap1,
                             const SubMap::Ptr& submap2);
    
    /**
     * @brief 更新缓存策略
     */
    void updateCachePolicy();
    
    /**
     * @brief LRU缓存管理
     */
    void manageLRUCache();
    
    /**
     * @brief 基础子图管理器
     */
    SubMapManager* base_manager_;
    
    // ROS2节点
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    
    // 子图质量缓存
    std::unordered_map<int, SubmapQuality> quality_cache_;
    mutable std::mutex quality_mutex_;
    
    // 子图配对缓存
    std::unordered_map<int, std::vector<SubmapPair>> pair_cache_;
    mutable std::mutex pair_mutex_;
    
    // 回环历史
    std::vector<SubmapPair> loop_history_;
    
    // 缓存管理
    std::deque<int> lru_queue_;  // LRU访问队列
    std::unordered_map<int, bool> cached_flag_;  // 是否在缓存中
    mutable std::mutex cache_mutex_;
    
    // 配置参数
    int max_keyframes_adaptive_;     // 自适应最大关键帧数
    int min_keyframes_;             // 最小关键帧数
    double quality_threshold_;       // 质量阈值
    double overlap_threshold_;       // 重叠度阈值
    double distance_threshold_;      // 距离阈值
    size_t max_cache_size_bytes_;   // 最大缓存大小（字节）
    double cache_release_percent_;  // 缓存释放阈值
    
    // 回调
    std::vector<QualityCallback> quality_cbs_;
};

}  // namespace automap_pro
