#pragma once

/**
 * iSAM2 因子类型定义模块
 *
 * 定义 iSAM2 优化器使用的所有数据类型，包括：
 * - 里程计因子
 * - 回环因子
 * - GPS 因子
 * - 子图数据
 * - 优化结果等
 */

#include "automap_pro/core/data_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>
#include <vector>

namespace automap_pro {

/**
 * @brief 自定义 GTSAM 圆柱地标因子 (Cylinder Landmark Factor)
 * 约束 Pose3 (KeyFrame) 与 Pose3 (Submap Anchor) 之间的关系，通过圆柱地标。
 * 该因子为二元因子，确保当子图锚点位姿在全局优化中移动时，约束依然有效。
 */
class CylinderFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
public:
    /**
     * @param kfKey 关键帧位姿 Key
     * @param smKey 子图锚点位姿 Key
     * @param point_body 关键帧 body 系下的观测点
     * @param root_submap 子图锚点系下的圆柱底部中心
     * @param ray_submap 子图锚点系下的圆柱轴向向量
     * @param radius 圆柱半径
     * @param model 噪声模型
     */
    CylinderFactor(gtsam::Key kfKey, gtsam::Key smKey, 
                   const gtsam::Point3& point_body, 
                   const gtsam::Point3& root_submap, const gtsam::Unit3& ray_submap, 
                   double radius, const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, kfKey, smKey),
          point_body_(point_body), root_submap_(root_submap), 
          ray_submap_(ray_submap), radius_(radius) {}

    virtual ~CylinderFactor() {}

    /**
     * @brief 计算残差
     * @param T_w_kf 关键帧在世界系下的位姿
     * @param T_w_sm 子图锚点在世界系下的位姿
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& T_w_kf, const gtsam::Pose3& T_w_sm,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
        // 1. 将 body 系下的观测点转换到世界系: p_w = T_w_kf * p_b
        gtsam::Matrix36 Dpw_Dkf;
        gtsam::Point3 p_w = T_w_kf.transformFrom(point_body_, H1 ? &Dpw_Dkf : nullptr);
        
        // 2. 将子图系下的地标参数转换到世界系: 
        // root_w = T_w_sm * root_s
        // ray_w = R_w_sm * ray_s
        gtsam::Matrix36 Drootw_Dsm;
        gtsam::Matrix33 Drootw_Droots; // 不常用，因为 root_s 是常量
        gtsam::Point3 root_world = T_w_sm.transformFrom(root_submap_, H2 ? &Drootw_Dsm : nullptr);
        
        gtsam::Matrix33 Drw_Drsm; // R_w_sm 对 rotation 的导数
        gtsam::Rot3 R_w_sm = T_w_sm.rotation();
        gtsam::Point3 ray_world = R_w_sm.rotate(ray_submap_.point3(), H2 ? &Drw_Drsm : nullptr);
        
        // 3. 计算点到轴线的距离
        gtsam::Point3 pr = p_w - root_world;
        gtsam::Point3 cross_prod = pr.cross(ray_world);
        double dist_to_axis = cross_prod.norm();
        
        gtsam::Vector1 error;
        error << dist_to_axis - radius_;

        if (H1 || H2) {
            if (dist_to_axis < 1e-6) {
                if (H1) *H1 = gtsam::Matrix::Zero(1, 6);
                if (H2) *H2 = gtsam::Matrix::Zero(1, 6);
            } else {
                gtsam::Point3 unit_cross = cross_prod / dist_to_axis;
                gtsam::Point3 de_dpw_vec = ray_world.cross(unit_cross);
                gtsam::Matrix13 de_dpw;
                de_dpw << de_dpw_vec.x(), de_dpw_vec.y(), de_dpw_vec.z();
                
                if (H1) {
                    *H1 = de_dpw * Dpw_Dkf;
                }
                
                if (H2) {
                    // e = |(p_w - T_w_sm*root_s) x (R_w_sm*ray_s)| - R
                    // 对 T_w_sm 求导比较复杂，这里简化处理或使用数值导数，
                    // 但为了性能，我们推导解析导数：
                    // de/d_sm = de/dp_w * dp_w/d_sm (0) + de/droot_w * droot_w/d_sm + de/dray_w * dray_w/d_sm
                    
                    // 1. de/droot_w: 和 de/dp_w 相反
                    gtsam::Matrix13 de_drootw = -de_dpw;
                    gtsam::Matrix16 de_dsm_via_root = de_drootw * Drootw_Dsm;
                    
                    // 2. de/dray_w: 
                    // d(|pr x ray_w|)/dray_w = unit_cross^T * d(pr x ray_w)/dray_w
                    // d(pr x ray_w)/dray_w = [pr]x
                    // 所以 de/dray_w = unit_cross^T * [pr]x = (pr x unit_cross)^T
                    gtsam::Point3 de_drayw_vec = pr.cross(unit_cross);
                    gtsam::Matrix13 de_drayw;
                    de_drayw << de_drayw_vec.x(), de_drayw_vec.y(), de_drayw_vec.z();
                    
                    // dray_w/d_sm = d(R_w_sm * ray_s) / d_sm
                    // 这部分只跟旋转有关，gtsam::Rot3::rotate 提供了对旋转的导数 (3x3)
                    // 我们需要将其扩展到 3x6 (前3维是旋转，后3维是平移)
                    gtsam::Matrix36 Drw_Dsm_full = gtsam::Matrix36::Zero();
                    Drw_Dsm_full.block<3, 3>(0, 0) = Drw_Drsm; // Rot3::rotate 的导数是对其旋转向量的
                    
                    gtsam::Matrix16 de_dsm_via_ray = de_drayw * Drw_Dsm_full;
                    
                    *H2 = de_dsm_via_root + de_dsm_via_ray;
                }
            }
        }
        return error;
    }

private:
    gtsam::Point3 point_body_;
    gtsam::Point3 root_submap_;
    gtsam::Unit3 ray_submap_;
    double radius_;
};

/**
 * 圆柱因子项 (用于关键帧)
 */
struct CylinderFactorItemKF {
    uint64_t kf_id;
    uint64_t sm_id;      // 所属子图 ID (NEW)
    Eigen::Vector3d point_body;
    Eigen::Vector3d root_submap; // 改为相对子图锚点的坐标 (NEW)
    Eigen::Vector3d ray_submap;  // 改为相对子图锚点的方向 (NEW)
    double radius;
    double weight = 1.0;
};

/**
 * 优化任务类型枚举
 */
enum class OptimTaskType {
    LOOP_FACTOR,    // 回环因子任务
    GPS_FACTOR,     // GPS 因子任务
    BATCH_UPDATE,   // 批量更新任务
    REBUILD         // 重建任务（GPS对齐后重建）
};

/**
 * GPS 因子项结构
 */
struct GPSFactorItem {
    int sm_id;
    Eigen::Vector3d pos;
    Eigen::Matrix3d cov;
};

/**
 * 子图间里程计因子结构体（用于保存和恢复）
 */
struct OdomFactorItem {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 回环因子结构体（用于保存和恢复）
 */
struct LoopFactorItem {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 子图数据（用于 GPS 对齐后重建）
 */
struct SubmapData {
    int id;
    Pose3d pose;
    bool is_fixed;
    bool has_gps;
    Eigen::Vector3d gps_center;
    Eigen::Matrix3d gps_cov;
};

/**
 * 关键帧数据结构（用于重建）
 */
struct KeyFrameData {
    int id;
    Pose3d pose;
    bool fixed;
    bool is_first_kf_of_submap;
};

/**
 * 关键帧间里程计因子结构体
 */
struct OdomFactorItemKF {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 关键帧间回环因子结构体
 */
struct LoopFactorItemKF {
    int from_id;
    int to_id;
    Pose3d rel_pose;
    Mat66d info_matrix;
};

/**
 * 优化任务结构体
 */
struct OptimTask {
    OptimTaskType type;
    int from_id = 0;                     // 起始节点 ID
    int to_id = 0;                        // 目标节点 ID
    Pose3d rel_pose;                     // 相对位姿
    Mat66d info_matrix;                   // 信息矩阵
    Eigen::Vector3d gps_pos;              // GPS 位置
    Eigen::Matrix3d gps_cov;             // GPS 协方差
    std::function<void()> action;        // 批量更新回调
    // REBUILD 任务专用字段
    std::vector<SubmapData> submap_data;
    std::vector<OdomFactorItem> odom_factors;
    std::vector<LoopFactorItem> loop_factors;
    std::vector<KeyFrameData> keyframe_data;
    std::vector<OdomFactorItemKF> kf_odom_factors;
    std::vector<LoopFactorItemKF> kf_loop_factors;
};

/**
 * 优化器健康状态
 */
struct ISAM2HealthStatus {
    bool is_healthy = true;
    int consecutive_failures = 0;
    int total_optimizations = 0;
    int failed_optimizations = 0;
    double last_success_time_ms = 0.0;
    std::string last_error_message;
};

}  // namespace automap_pro
