#pragma once

#include <Eigen/Core>

namespace automap_pro {

class ConfigManager;

namespace gps_factor_noise {

/**
 * V1: 按 HDOP / 卫星数放大水平位置协方差（弱信号 → 更大 σ²，GPS 更软）。
 * V2: 若提供当前图优化位姿 z_graph，与 GPS 观测 z_gps 水平不一致时进一步放大 XY 方差
 *     （等效于“与局部激光/里程计链一致性冲突则降权”，不跑额外 ICP）。
 *
 * @param cov_io  入参为测量协方差；出参为注入后端前的协方差（仅改数值，不改结构）。
 * @param hdop    <0 或非有限 → 按最差档处理
 * @param num_satellites  <=0 → 按最差档处理
 * @param z_gps   与 GTSAM GPSFactor 一致的位置（map 系）
 * @param z_graph 若非空，为当前 KF 在图中的平移估计（同坐标系）；空指针则跳过一致性门控
 */
void refineKeyFrameGpsCovariance(Eigen::Matrix3d& cov_io,
                                 const ConfigManager& cfg,
                                 double hdop,
                                 int num_satellites,
                                 const Eigen::Vector3d& z_gps,
                                 const Eigen::Vector3d* z_graph);

/** 应用 gps.factor_weight：weight>1 更强约束 → 缩小协方差（cov /= w²） */
void applyGpsFactorWeight(Eigen::Matrix3d& cov_io, double factor_weight);

}  // namespace gps_factor_noise
}  // namespace automap_pro
