#pragma once

#include "automap_pro/core/data_types.h"
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <mutex>
#include <atomic>

namespace automap_pro {

// 前向声明
class SubMapManager;
class GPSManager;

/**
 * @brief GPS因子信息结构
 */
struct GPSFactorInfo {
    int submap_id = -1;
    double timestamp = 0.0;
    Eigen::Vector3d pos_map = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    GPSQuality quality = GPSQuality::INVALID;
    bool is_compensated = false;  // 是否为补偿添加（vs 实时添加）
};

/**
 * @brief 延迟GPS补偿器
 *
 * 核心职责：
 *   1. 管理GPS未对齐期间创建的子图队列
 *   2. GPS对齐成功后，批量为历史子图补充GPS约束
 *   3. 每次回环优化后，自动检查并补偿涉及的子图
 *   4. 确保同一子图不会重复添加GPS因子
 *
 * 设计原则：
 *   - GPS对齐与回环检测完全独立，互不阻塞
 *   - 补偿操作异步执行，不影响实时建图性能
 *   - 幂等性：同一子图多次调用补偿不会重复添加因子
 *
 * 典型工作流程：
 *   1. 子图冻结时调用 registerSubmap()，若GPS未对齐则加入待补偿队列
 *   2. GPS对齐成功时调用 onGPSAligned()，标记已对齐并收集历史子图
 *   3. 回环优化完成时调用 onPoseOptimized()，自动补偿涉及的待补偿子图
 *   4. 新子图冻结时若已对齐，立即补偿
 *
 * 坐标系约定：仅在 GPS 已对齐后添加因子；enuToMap 使用 align_result_（与 GPSManager 对齐结果一致），
 * 将 ENU 转为 map 系，保证与 iSAM2 节点同系。
 */
class DelayedGPSCompensator {
public:
    using Ptr = std::shared_ptr<DelayedGPSCompensator>;
    
    // GPS因子回调类型（返回是否成功添加）
    using GpsFactorCallback = std::function<bool(int sm_id, const Eigen::Vector3d& pos_map,
                                                  const Eigen::Matrix3d& cov, bool is_compensated)>;
    
    // 子图查询回调（获取子图的GPS测量）
    using SubmapGPSQueryCallback = std::function<std::optional<GPSMeasurement>(int sm_id)>;
    
    // 位姿查询回调（获取子图当前位姿）
    using SubmapPoseQueryCallback = std::function<std::optional<Pose3d>(int sm_id)>;

    explicit DelayedGPSCompensator();
    ~DelayedGPSCompensator() = default;

    // ── 配置接口 ─────────────────────────────────────────────────────────────
    
    void setEnabled(bool enabled) { enabled_.store(enabled); }
    bool isEnabled() const { return enabled_.load(); }
    
    void setCompensateOnLoop(bool enable) { compensate_on_loop_.store(enable); }
    void setCompensateOnAlign(bool enable) { compensate_on_align_.store(enable); }
    void setBatchSize(int size) { batch_size_.store(size); }
    void setMaxPendingSubmaps(int max_count) { max_pending_submaps_.store(max_count); }

    // ── 核心接口 ─────────────────────────────────────────────────────────────

    /**
     * @brief GPS对齐成功回调
     * @param result SVD对齐结果
     * @note 调用后，所有后续子图将自动补偿GPS约束
     */
    void onGPSAligned(const GPSAlignResult& result);

    /**
     * @brief 收集历史子图到待补偿队列
     * @param all_submaps 所有子图列表
     * @return 收集到的待补偿子图数量
     */
    int collectHistoricalSubmaps(const std::vector<SubMap::Ptr>& all_submaps);

    /**
     * @brief 回环优化后回调
     * @param poses 本次优化涉及的子图位姿
     * @return 实际补偿的子图数量
     * @note 检查待补偿队列中是否有本次优化涉及的子图，若有则补偿
     */
    int onPoseOptimized(const std::unordered_map<int, Pose3d>& poses);

    /**
     * @brief 子图冻结时注册
     * @param sm 子图指针
     * @note 若GPS未对齐则加入待补偿队列；若已对齐则立即补偿
     */
    void registerSubmap(const SubMap::Ptr& sm);

    /**
     * @brief 批量补偿待补偿队列中的子图
     * @param max_count 本次最大补偿数量，-1表示无限制
     * @return 实际补偿的子图数量
     */
    int compensateBatch(int max_count = -1);

    // ── 回调注册 ─────────────────────────────────────────────────────────────

    void registerGpsFactorCallback(GpsFactorCallback cb) {
        gps_factor_cbs_.push_back(std::move(cb));
    }

    void setSubmapGPSQueryCallback(SubmapGPSQueryCallback cb) {
        submap_gps_query_cb_ = std::move(cb);
    }

    void setSubmapPoseQueryCallback(SubmapPoseQueryCallback cb) {
        submap_pose_query_cb_ = std::move(cb);
    }

    // ── 状态查询 ─────────────────────────────────────────────────────────────

    bool isGPSAligned() const { return gps_aligned_.load(); }
    size_t pendingCount() const;
    size_t compensatedCount() const;
    
    /** 获取待补偿子图ID列表（用于诊断） */
    std::vector<int> getPendingSubmapIds() const;
    
    /** 获取已补偿子图ID列表（用于诊断） */
    std::vector<int> getCompensatedSubmapIds() const;
    
    /** 获取子图的GPS因子信息（如果存在） */
    std::optional<GPSFactorInfo> getGPSFactorInfo(int sm_id) const;

    // ── 重置 ────────────────────────────────────────────────────────────────

    /** 重置所有状态（新Session开始时调用） */
    void reset();

private:
    // ── 配置参数 ─────────────────────────────────────────────────────────────
    std::atomic<bool> enabled_{true};
    std::atomic<bool> compensate_on_loop_{true};
    std::atomic<bool> compensate_on_align_{true};
    std::atomic<int> batch_size_{10};
    std::atomic<int> max_pending_submaps_{1000};

    // ── GPS对齐状态 ───────────────────────────────────────────────────────────
    std::atomic<bool> gps_aligned_{false};
    GPSAlignResult align_result_;

    // ── 待补偿/已补偿集合 ─────────────────────────────────────────────────────
    mutable std::mutex mutex_;
    
    // 待补偿的子图ID集合（GPS未对齐时创建的子图）
    std::set<int> pending_gps_submaps_;
    
    // 已补偿的子图ID集合（避免重复补偿）
    std::unordered_set<int> compensated_submaps_;
    
    // 已添加GPS因子的子图记录（包括实时添加和补偿添加）
    std::unordered_map<int, GPSFactorInfo> gps_factors_;

    // ── 回调函数 ─────────────────────────────────────────────────────────────
    std::vector<GpsFactorCallback> gps_factor_cbs_;
    SubmapGPSQueryCallback submap_gps_query_cb_;
    SubmapPoseQueryCallback submap_pose_query_cb_;

    // ── 私有方法 ─────────────────────────────────────────────────────────────
    
    /**
     * @brief 补偿单个子图
     * @param sm_id 子图ID
     * @return 是否成功补偿
     */
    bool compensateSubmap(int sm_id);

    /**
     * @brief 查询子图对应的GPS测量
     * @param sm_id 子图ID
     * @return GPS测量（如果存在）
     */
    std::optional<GPSMeasurement> queryGPSForSubmap(int sm_id);

    /**
     * @brief 计算GPS协方差
     * @param quality GPS质量
     * @param hdop HDOP值
     * @return 3x3协方差矩阵
     */
    Eigen::Matrix3d computeGPSCovariance(GPSQuality quality, double hdop);

    /**
     * @brief 将GPS ENU坐标转换到地图坐标系
     * @param pos_enu ENU坐标
     * @return 地图坐标系坐标
     */
    Eigen::Vector3d enuToMap(const Eigen::Vector3d& pos_enu) const;
};

} // namespace automap_pro
