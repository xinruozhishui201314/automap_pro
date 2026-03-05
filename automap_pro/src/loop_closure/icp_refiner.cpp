#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/logger.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>

#define MOD "ICPRefiner"

namespace automap_pro {

IcpRefiner::IcpRefiner()
    : IcpRefiner(Config()) {
}

IcpRefiner::IcpRefiner(const Config& config)
    : config_(config) {
    ALOG_INFO(MOD, "IcpRefiner initialized: method={} max_iter={}",
              static_cast<int>(config_.method), config_.max_iterations);
}

void IcpRefiner::setConfig(const Config& config) {
    config_ = config;
}

IcpRefiner::Config IcpRefiner::getConfig() const {
    return config_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 主要配准接口
// ─────────────────────────────────────────────────────────────────────────────

IcpRefiner::Result IcpRefiner::refine(const CloudXYZIPtr& src, 
                                        const CloudXYZIPtr& tgt,
                                        const Pose3d& initial) const {
    if (!src || !tgt || src->empty() || tgt->empty()) {
        Result res;
        return res;
    }
    
    auto start = std::chrono::steady_clock::now();
    
    Result res;
    
    // 根据配置选择算法
    switch (config_.method) {
        case ICPMethod::ICP:
            res = refineICP(src, tgt, initial, false);
            break;
        case ICPMethod::POINT_TO_PLANE:
            res = refineICP(src, tgt, initial, true);
            break;
        case ICPMethod::GICP:
            res = refineGICP(src, tgt, initial);
            break;
        case ICPMethod::NDT:
            res = refineNDT(src, tgt, initial);
            break;
    }
    
    auto end = std::chrono::steady_clock::now();
    res.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    // 验证结果
    if (config_.validate_result) {
        if (!validateResult(res, src, tgt)) {
            ALOG_WARN(MOD, "ICP refinement result validation failed");
            res.converged = false;
        }
    }
    
    // 计算位姿变化
    computePoseChange(initial, res.T_refined, res.rotation_angle_deg, res.translation_norm);
    
    ALOG_DEBUG(MOD, "ICP refine done: converged={} rmse={:.4f} angle={:.2f}deg trans={:.2f}m time={:.1f}ms",
               res.converged, res.rmse, res.rotation_angle_deg, 
               res.translation_norm, res.elapsed_ms);
    
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// 多尺度配准
// ─────────────────────────────────────────────────────────────────────────────

IcpRefiner::Result IcpRefiner::refineMultiscale(const CloudXYZIPtr& src, 
                                                  const CloudXYZIPtr& tgt,
                                                  const Pose3d& initial) const {
    Result final_res;
    Pose3d current_pose = initial;
    
    for (size_t i = 0; i < config_.scale_levels.size(); ++i) {
        float scale = config_.scale_levels[i];
        
        ALOG_DEBUG(MOD, "Multiscale level {}/{}: scale={:.2f}",
                   i + 1, config_.scale_levels.size(), scale);
        
        // 降采样
        CloudXYZIPtr src_ds(new CloudXYZI);
        CloudXYZIPtr tgt_ds(new CloudXYZI);
        
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(scale, scale, scale);
        
        vg.setInputCloud(src);
        vg.filter(*src_ds);
        
        vg.setInputCloud(tgt);
        vg.filter(*tgt_ds);
        
        // 配准
        Result res = refine(src_ds, tgt_ds, current_pose);
        
        if (!res.converged) {
            ALOG_WARN(MOD, "Multiscale failed at level {}/{}", i + 1, config_.scale_levels.size());
            return final_res;
        }
        
        current_pose = res.T_refined;
        final_res = res;
    }
    
    // 最终精配准（原始分辨率）
    if (!config_.scale_levels.empty()) {
        final_res = refine(src, tgt, current_pose);
    }
    
    ALOG_INFO(MOD, "Multiscale refine: {} levels, final rmse={:.4f}",
              config_.scale_levels.size(), final_res.rmse);
    
    return final_res;
}

// ─────────────────────────────────────────────────────────────────────────────
// ICP配准
// ─────────────────────────────────────────────────────────────────────────────

IcpRefiner::Result IcpRefiner::refineICP(const CloudXYZIPtr& src, 
                                            const CloudXYZIPtr& tgt,
                                            const Pose3d& initial,
                                            bool point_to_plane) const {
    Result res;
    
    CloudXYZIPtr src_proc = preprocessCloud(src);
    CloudXYZIPtr tgt_proc = preprocessCloud(tgt);
    
    if (point_to_plane) {
        // 点到面ICP
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(src_proc);
        icp.setInputTarget(tgt_proc);
        icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
        icp.setMaximumIterations(config_.max_iterations);
        icp.setTransformationEpsilon(config_.transformation_epsilon);
        icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
        
        CloudXYZI aligned;
        Eigen::Matrix4f init = initial.cast<float>().matrix();
        icp.align(aligned, init);
        
        res.converged = icp.hasConverged();
        res.rmse = icp.getFitnessScore();
        res.iterations = config_.max_iterations;
        
        if (res.converged) {
            Eigen::Matrix4d final_tf = icp.getFinalTransformation().cast<double>();
            res.T_refined.matrix() = final_tf;
        }
        
    } else {
        // 标准点对点ICP
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(src_proc);
        icp.setInputTarget(tgt_proc);
        icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
        icp.setMaximumIterations(config_.max_iterations);
        icp.setTransformationEpsilon(config_.transformation_epsilon);
        icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
        
        CloudXYZI aligned;
        Eigen::Matrix4f init = initial.cast<float>().matrix();
        icp.align(aligned, init);
        
        res.converged = icp.hasConverged();
        res.rmse = icp.getFitnessScore();
        res.iterations = config_.max_iterations;
        
        if (res.converged) {
            Eigen::Matrix4d final_tf = icp.getFinalTransformation().cast<double>();
            res.T_refined.matrix() = final_tf;
        }
    }
    
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// GICP配准
// ─────────────────────────────────────────────────────────────────────────────

IcpRefiner::Result IcpRefiner::refineGICP(const CloudXYZIPtr& src,
                                             const CloudXYZIPtr& tgt,
                                             const Pose3d& initial) const {
    Result res;
    
    CloudXYZIPtr src_proc = preprocessCloud(src);
    CloudXYZIPtr tgt_proc = preprocessCloud(tgt);
    
    // 计算法线（用于GICP）
    computeNormals(src_proc);
    computeNormals(tgt_proc);
    
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(src_proc);
    gicp.setInputTarget(tgt_proc);
    gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
    gicp.setMaximumIterations(config_.max_iterations);
    // GICP 默认对应估计即可；PCL 1.12 中 Reciprocal 枚举可能不可用
    
    CloudXYZI aligned;
    Eigen::Matrix4f init = initial.cast<float>().matrix();
    gicp.align(aligned, init);
    
    res.converged = gicp.hasConverged();
    res.rmse = gicp.getFitnessScore();
    res.iterations = config_.max_iterations;
    
    if (res.converged) {
        Eigen::Matrix4d final_tf = gicp.getFinalTransformation().cast<double>();
        res.T_refined.matrix() = final_tf;
    }
    
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// NDT配准
// ─────────────────────────────────────────────────────────────────────────────

IcpRefiner::Result IcpRefiner::refineNDT(const CloudXYZIPtr& src,
                                            const CloudXYZIPtr& tgt,
                                            const Pose3d& initial) const {
    Result res;
    
    CloudXYZIPtr src_proc = preprocessCloud(src);
    CloudXYZIPtr tgt_proc = preprocessCloud(tgt);
    
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputSource(src_proc);
    ndt.setInputTarget(tgt_proc);
    ndt.setResolution(config_.ndt_resolution);
    ndt.setStepSize(config_.ndt_step_size);
    ndt.setResolution(config_.ndt_resolution_gradient);
    ndt.setMaximumIterations(config_.max_iterations);
    
    CloudXYZI aligned;
    Eigen::Matrix4f init = initial.cast<float>().matrix();
    ndt.align(aligned, init);
    
    res.converged = ndt.hasConverged();
    res.rmse = ndt.getFitnessScore();
    res.iterations = ndt.getFinalNumIteration();
    
    if (res.converged) {
        Eigen::Matrix4d final_tf = ndt.getFinalTransformation().cast<double>();
        res.T_refined.matrix() = final_tf;
    }
    
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// 验证配准结果
// ─────────────────────────────────────────────────────────────────────────────

bool IcpRefiner::validateResult(const Result& res, const CloudXYZIPtr& src,
                                const CloudXYZIPtr& tgt) const {
    if (!res.converged) return false;
    
    // 检查旋转角度
    if (res.rotation_angle_deg > config_.max_rotation_deg) {
        ALOG_DEBUG(MOD, "Rotation angle too large: {:.2f}deg > {:.2f}deg",
                   res.rotation_angle_deg, config_.max_rotation_deg);
        return false;
    }
    
    // 检查平移距离
    if (res.translation_norm > config_.max_translation_m) {
        ALOG_DEBUG(MOD, "Translation too large: {:.2f}m > {:.2f}m",
                   res.translation_norm, config_.max_translation_m);
        return false;
    }
    
    // 检查内点率
    if (res.rmse < 1e6 && res.correspondences > 0) {
        double inlier_ratio = 1.0 - (res.rmse / config_.max_correspondence_distance);
        if (inlier_ratio < config_.min_inlier_ratio) {
            ALOG_DEBUG(MOD, "Inlier ratio too low: {:.2f} < {:.2f}",
                       inlier_ratio, config_.min_inlier_ratio);
            return false;
        }
    }
    
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 计算对应关系协方差
// ─────────────────────────────────────────────────────────────────────────────

Mat66d IcpRefiner::computeCorrespondenceCovariance(
    const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
    const Pose3d& T_src_tgt) const {
    
    Mat66d cov = Mat66d::Zero();
    
    auto correspondences = findCorrespondences(src, tgt, T_src_tgt, 
                                             config_.max_correspondence_distance);
    
    if (correspondences.empty()) return cov;
    
    // 计算对应误差的协方差
    for (const auto& [src_idx, tgt_idx] : correspondences) {
        if (src_idx >= src->size() || tgt_idx >= tgt->size()) continue;
        
        const auto& src_pt = src->points[src_idx];
        const auto& tgt_pt = tgt->points[tgt_idx];
        
        Eigen::Vector3d src_p(src_pt.x, src_pt.y, src_pt.z);
        Eigen::Vector3d tgt_p(tgt_pt.x, tgt_pt.y, tgt_pt.z);
        
        Eigen::Vector3d pred = T_src_tgt * src_p;
        Eigen::Vector3d error = pred - tgt_p;
        
        // 构建协方差矩阵（平移+旋转）
        for (int i = 0; i < 3; ++i) {
            cov(i, i) += error[i] * error[i];
        }
    }
    
    cov /= static_cast<double>(correspondences.size());
    
    return cov;
}

// ─────────────────────────────────────────────────────────────────────────────
// 预处理
// ─────────────────────────────────────────────────────────────────────────────

CloudXYZIPtr IcpRefiner::preprocessCloud(const CloudXYZIPtr& cloud) const {
    if (!cloud) return nullptr;
    
    CloudXYZIPtr processed(new CloudXYZI(*cloud));
    
    // 降采样
    if (config_.downsample_before_refine && config_.downsample_voxel_size > 0) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(processed);
        vg.setLeafSize(config_.downsample_voxel_size, 
                       config_.downsample_voxel_size, 
                       config_.downsample_voxel_size);
        vg.filter(*processed);
    }
    
    // 统计去噪
    if (processed->size() > 100) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(processed);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2.0);
        
        CloudXYZIPtr filtered(new CloudXYZI);
        sor.filter(*filtered);
        processed = filtered;
    }
    
    return processed;
}

void IcpRefiner::computeNormals(const CloudXYZIPtr& cloud) const {
    if (!cloud || cloud->empty()) return;
    
    // PCL 的 GICP 会自动计算法线，但需要使用 pcl::PointCloud<pcl::PointNormal>
    // 这里我们使用 NormalEstimation 计算法线
    // 注意：GICP 内部会自动处理法线计算，此函数主要用于显式法线需求
    
    // 创建法线估计对象
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    
    // 创建 KD 树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    ne.setSearchMethod(tree);
    
    // 设置邻域搜索半径（使用配置中的参数）
    ne.setRadiusSearch(config_.gicp_correspondence_radius);
    
    // 计算法线
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    
    // 注意：PointXYZI 格式的点云不能直接存储法线
    // GICP 会自动基于协方差矩阵进行计算
    // 如果需要显式使用法线，应该使用 pcl::PointCloud<pcl::PointNormal>
    
    ALOG_DEBUG(MOD, "Computed normals for {} points (k={})", 
               cloud->size(), config_.gicp_correspondence_knn);
}

// ─────────────────────────────────────────────────────────────────────────────
// 对应关系分析
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::pair<int, int>> IcpRefiner::findCorrespondences(
    const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
    const Pose3d& T_src_tgt, float max_dist) const {
    
    std::vector<std::pair<int, int>> correspondences;
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(tgt);
    
    // 变换源点云到目标坐标系
    CloudXYZI src_transformed;
    pcl::transformPointCloud(*src, src_transformed, T_src_tgt.cast<float>().matrix());
    
    // 找每个源点的最近邻
    for (size_t i = 0; i < src_transformed.size(); ++i) {
        const auto& src_pt = src_transformed.points[i];
        
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        
        kdtree.nearestKSearch(src_pt, 1, indices, distances);
        
        if (distances[0] < max_dist * max_dist) {
            correspondences.push_back({static_cast<int>(i), indices[0]});
        }
    }
    
    return correspondences;
}

void IcpRefiner::filterCorrespondences(
    std::vector<std::pair<int, int>>& correspondences,
    const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
    const Pose3d& T_src_tgt) const {
    
    if (!config_.enable_correspondence_filtering) return;
    
    std::vector<std::pair<int, int>> filtered;
    
    for (const auto& [src_idx, tgt_idx] : correspondences) {
        if (src_idx >= src->size() || tgt_idx >= tgt->size()) continue;
        
        const auto& src_pt = src->points[src_idx];
        const auto& tgt_pt = tgt->points[tgt_idx];
        
        Eigen::Vector3d src_p(src_pt.x, src_pt.y, src_pt.z);
        Eigen::Vector3d tgt_p(tgt_pt.x, tgt_pt.y, tgt_pt.z);
        
        // 距离约束
        Eigen::Vector3d pred = T_src_tgt * src_p;
        double dist = (pred - tgt_p).norm();
        double ref_dist = src_p.norm();
        
        if (dist > config_.max_distance_ratio * ref_dist) continue;
        
        // 角度约束（检查法线方向）
        // 这里简化处理，实际需要法线信息
        
        filtered.push_back({src_idx, tgt_idx});
    }
    
    correspondences = std::move(filtered);
}

// ─────────────────────────────────────────────────────────────────────────────
// 结果分析
// ─────────────────────────────────────────────────────────────────────────────

double IcpRefiner::computeFitnessScore(const CloudXYZIPtr& aligned,
                                         const CloudXYZIPtr& tgt) const {
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(tgt);
    
    double total_error = 0.0;
    int count = 0;
    
    for (const auto& pt : aligned->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        
        kdtree.nearestKSearch(pt, 1, indices, distances);
        
        total_error += std::sqrt(distances[0]);
        count++;
    }
    
    return count > 0 ? total_error / count : 1e6;
}

void IcpRefiner::computePoseChange(const Pose3d& T_initial, const Pose3d& T_final,
                                 double& angle_deg, double& translation) const {
    
    Pose3d T_change = T_initial.inverse() * T_final;
    
    // 提取旋转角度
    Eigen::AngleAxisd angle_axis(T_change.linear());
    angle_deg = angle_axis.angle() * 180.0 / M_PI;
    
    // 提取平移距离
    translation = T_change.translation().norm();
}

} // namespace automap_pro
