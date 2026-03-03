#pragma once

#include "automap_pro/core/data_types.h"
#include <automap_pro/msg/sub_map_event_msg.hpp>
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
    CloudXYZIPtr buildGlobalMap(float voxel_size = 0.1f) const;

private:
    std::vector<SubMap::Ptr>  submaps_;
    SubMap::Ptr               active_submap_;
    mutable std::mutex        mutex_;
    std::atomic<int>          submap_id_counter_{0};
    uint64_t                  current_session_id_ = 0;

    // 关键帧计数（全局）
    std::atomic<uint64_t>     kf_id_counter_{0};

    // 参数
    int    max_kf_       = 100;
    double max_spatial_  = 100.0;   // 米
    double max_temporal_ = 60.0;    // 秒
    double match_res_    = 0.4;     // 降采样分辨率（用于回环匹配）
    double merge_res_    = 0.1;     // 降采样分辨率（合并地图）

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<automap_pro::msg::SubMapEventMsg>::SharedPtr event_pub_;

    std::vector<SubMapFrozenCallback> frozen_cbs_;

    // ── 私有方法 ──────────────────────────────────────────────────────────
    SubMap::Ptr createNewSubmap(const KeyFrame::Ptr& first_kf);
    void        freezeActiveSubmap();
    bool        isFull(const SubMap::Ptr& sm) const;
    void        mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const;
    void        publishEvent(const SubMap::Ptr& sm, const std::string& event);
};

} // namespace automap_pro
