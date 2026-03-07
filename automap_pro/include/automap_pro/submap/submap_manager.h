#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/core/error_code.h"
#include <automap_pro/msg/sub_map_event_msg.hpp>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <atomic>
#include <vector>
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

/**
 * SubMap 生命周期管理器（MS-Mapping 增量式建图核心）
 *
 * 子图状态机：
 *   ACTIVE → (isFull) → FROZEN → (iSAM2优化) → OPTIMIZED → (持久化) → ARCHIVED
 *
 * MS-Mapping 增量式建图特性：
 *   1. 同一 session 内支持跨子图回环（历史描述子数据库）
 *   2. 跨 session 回环：通过 SessionManager 加载历史描述子
 *   3. 子图位姿优化后，点云自动重新投影（reproject）
 *   4. 新 session 与历史 session 点云融合（map merging）
 */
class SubMapManager {
public:
    explicit SubMapManager();

    void init(rclcpp::Node::SharedPtr node);

    // ── 主要接口 ──────────────────────────────────────────────────────────

    /**
     * 添加新关键帧到当前活跃子图
     * 内部自动处理子图切分和冻结
     */
    void addKeyFrame(const KeyFrame::Ptr& kf);

    /**
     * 子图锚定位姿更新（由 iSAM2 优化后回调）
     * 同时更新子图内所有关键帧的 T_w_b_optimized
     */
    void updateSubmapPose(int submap_id, const Pose3d& new_pose);

    /** 批量更新子图位姿（来自 HBA 结果） */
    void updateAllFromHBA(const HBAResult& result);

    /**
     * 启动新会话（增量建图）
     * @param session_id  新 session ID（由 SessionManager 分配）
     */
    void startNewSession(uint64_t session_id);

    // ── 查询接口 ──────────────────────────────────────────────────────────

    SubMap::Ptr         getActiveSubmap() const;
    SubMap::Ptr         getSubmap(int id)  const;
    std::vector<SubMap::Ptr> getAllSubmaps() const;
    std::vector<SubMap::Ptr> getFrozenSubmaps() const;
    int                 submapCount()  const;
    int                 keyframeCount()const;

    // ── 持久化 ────────────────────────────────────────────────────────────

    /** 将已优化子图序列化到磁盘（增量式建图的核心：允许跨 session 加载） */
    bool archiveSubmap(const SubMap::Ptr& submap, const std::string& dir);

    /** 从磁盘恢复子图（跨 session 加载历史地图） */
    bool loadArchivedSubmap(const std::string& dir, int submap_id, SubMap::Ptr& out);

    // ── 回调注册 ──────────────────────────────────────────────────────────
    void registerSubmapFrozenCallback(SubMapFrozenCallback cb) {
        frozen_cbs_.push_back(std::move(cb));
    }

    // ── 全局点云构建 ──────────────────────────────────────────────────────

    /** 构建全局合并点云（使用优化后位姿） */
    CloudXYZIPtr buildGlobalMap(float voxel_size = 0.2f) const;

    /** 更新子图的 GPS 重力中心 */
    void updateGPSGravityCenter(const KeyFrame::Ptr& kf);

    /** 发布错误事件到 ROS2 话题 */
    void publishErrorEvent(int submap_id, const automap_pro::ErrorDetail& error);

private:
    std::vector<SubMap::Ptr>  submaps_;
    SubMap::Ptr               active_submap_;
    mutable std::mutex        mutex_;
    std::atomic<int>          submap_id_counter_{0};
    uint64_t                  current_session_id_ = 0;

    // 关键帧计数（全局）
    std::atomic<uint64_t>     kf_id_counter_{0};

    // ✅ 修复：VoxelGrid 对象池缓存（按分辨率索引，mutable 以便 const 方法可延迟填充）
    mutable std::unordered_map<float, std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZI>>> voxel_grid_cache_;
    mutable std::mutex vg_cache_mutex_;
    
    // 常用分辨率
    static constexpr float RES_MATCH = 0.4f;
    static constexpr float RES_MERGE = 0.2f;
    
    // 获取缓存的 VoxelGrid 对象（const 版本供 mergeCloudToSubmap 使用）
    pcl::VoxelGrid<pcl::PointXYZI>* getVoxelGrid(float leaf_size) const {
        std::lock_guard<std::mutex> lk(vg_cache_mutex_);
        
        auto it = voxel_grid_cache_.find(leaf_size);
        if (it != voxel_grid_cache_.end()) {
            return it->second.get();
        }
        
        // 创建并缓存新对象
        auto vg = std::make_unique<pcl::VoxelGrid<pcl::PointXYZI>>();
        vg->setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid_cache_[leaf_size] = std::move(vg);
        
        return voxel_grid_cache_[leaf_size].get();
    }

    // ✅ 修复：点云对象池（避免频繁内存分配，mutable 以便 const 方法可借用）
    static constexpr size_t CLOUD_POOL_SIZE = 10;
    mutable std::array<CloudXYZIPtr, CLOUD_POOL_SIZE> cloud_pool_;
    mutable std::atomic<size_t> pool_index_{0};
    mutable std::mutex pool_mutex_;
    
    // 从对象池获取点云（const 版本供 mergeCloudToSubmap 使用）
    CloudXYZIPtr getCloudFromPool() const {
        std::lock_guard<std::mutex> lk(pool_mutex_);
        size_t idx = pool_index_.fetch_add(1) % CLOUD_POOL_SIZE;
        
        if (!cloud_pool_[idx] || cloud_pool_[idx]->empty()) {
            cloud_pool_[idx] = std::make_shared<CloudXYZI>();
        }
        
        return cloud_pool_[idx];
    }

    // 参数
    int    max_kf_       = 100;
    double max_spatial_  = 100.0;   // 米
    double max_temporal_ = 60.0;    // 秒
    double match_res_    = 0.4;     // 降采样分辨率（用于回环匹配）
    double merge_res_    = 0.2;     // 降采样分辨率（合并地图，至少 0.2 避免 PCL 溢出）

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<automap_pro::msg::SubMapEventMsg>::SharedPtr event_pub_;

    std::vector<SubMapFrozenCallback> frozen_cbs_;

    // ── 私有方法 ──────────────────────────────────────────────────────────
    SubMap::Ptr createNewSubmap(const KeyFrame::Ptr& first_kf);
    /** 无参版本仅用于兼容；实际冻结请用传入 submap 的版本，且须在未持 mutex_ 时调用，避免回调内 getFrozenSubmaps 死锁 */
    void        freezeActiveSubmap();
    /** 对指定子图执行冻结并触发回调；调用方不得持有 mutex_（回调中会调用 getFrozenSubmaps） */
    void        freezeActiveSubmap(const SubMap::Ptr& sm);
    bool        isFull(const SubMap::Ptr& sm) const;
    void        mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const;
    void        publishEvent(const SubMap::Ptr& sm, const std::string& event);
};

} // namespace automap_pro
