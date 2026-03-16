#pragma once

/**
 * iSAM2 GPS 因子管理模块
 *
 * 管理 GPS 因子的添加、批量处理、延迟刷新等功能。
 * 包含 GPS 动态协方差调整和异常值检测。
 */

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/isam2_factor_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <vector>
#include <functional>
#include <memory>

namespace automap_pro {

/**
 * GPS 异常值检测结果
 */
struct GPSOutlierResult {
    bool is_outlier = false;
    double residual = 0.0;
    double scale = 1.0;
};

/**
 * GPS 因子管理器接口
 * 
 * 负责 GPS 因子的创建、动态协方差调整、异常值检测等功能。
 */
class ISAM2GPSManager {
public:
    ISAM2GPSManager();
    ~ISAM2GPSManager();

    /**
     * 添加单个 GPS 因子到待处理列表
     *
     * @param sm_id 子图 ID
     * @param pos_map GPS 位置（ENU 坐标）
     * @param cov3x3 位置协方差矩阵
     * @param pending_graph 待提交的因子图（需已持锁）
     * @param factor_count 当前因子计数（引用）
     * @param current_estimate 当前估计值
     * @param node_exists 节点存在性映射
     */
    void addGPSFactor(
        int sm_id,
        const Eigen::Vector3d& pos_map,
        const Eigen::Matrix3d& cov3x3,
        gtsam::NonlinearFactorGraph& pending_graph,
        int& factor_count,
        const gtsam::Values& current_estimate,
        const std::unordered_map<int, bool>& node_exists);

    /**
     * 批量添加 GPS 因子
     *
     * @param factors GPS 因子列表
     * @param pending_graph 待提交的因子图（需已持锁）
     * @param factor_count 当前因子计数（引用）
     * @param node_exists 节点存在性映射
     * @return 实际添加的因子数量
     */
    int addGPSFactorsBatch(
        const std::vector<GPSFactorItem>& factors,
        gtsam::NonlinearFactorGraph& pending_graph,
        int& factor_count,
        const std::unordered_map<int, bool>& node_exists);

    /**
     * 刷新待处理的 GPS 因子
     *
     * @param pending_gps 待处理的 GPS 因子列表
     * @param pending_graph 待提交的因子图（需已持锁）
     * @param factor_count 当前因子计数（引用）
     * @param current_estimate 当前估计值
     * @param node_exists 节点存在性映射
     * @return 本次刷新的因子数量
     */
    int flushPendingGPSFactors(
        std::vector<GPSFactorItem>& pending_gps,
        gtsam::NonlinearFactorGraph& pending_graph,
        int& factor_count,
        const gtsam::Values& current_estimate,
        const std::unordered_map<int, bool>& node_exists);

    /**
     * GPS 异常值检测
     *
     * @param sm_id 子图 ID
     * @param pos_map GPS 位置
     * @param cov3x3 协方差矩阵
     * @param current_estimate 当前估计值
     * @return 异常值检测结果
     */
    GPSOutlierResult detectOutlier(
        int sm_id,
        const Eigen::Vector3d& pos_map,
        const Eigen::Matrix3d& cov3x3,
        const gtsam::Values& current_estimate) const;

    /**
     * 应用动态协方差调整
     *
     * @param pos_map GPS 位置
     * @param cov3x3 原始协方差矩阵
     * @return 调整后的协方差矩阵
     */
    Eigen::Matrix3d applyDynamicCovariance(
        const Eigen::Vector3d& pos_map,
        const Eigen::Matrix3d& cov3x3) const;

    /**
     * 将 GPS 因子添加到因子图
     *
     * @param sm_id 子图 ID
     * @param pos 位置
     * @param cov 协方差
     * @param pending_graph 因子图
     * @param factor_count 因子计数
     */
    static void addGPSToGraph(
        int sm_id,
        const Eigen::Vector3d& pos,
        const Eigen::Matrix3d& cov,
        gtsam::NonlinearFactorGraph& pending_graph,
        int& factor_count);

private:
    /** 获取 GTSAM Symbol */
    static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }
};

}  // namespace automap_pro
