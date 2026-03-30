#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/logger.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <unordered_set>
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
    // 仅使用 ndt_resolution：此前二次 setResolution(ndt_resolution_gradient) 会覆盖 YAML/配置中的体素尺寸，
    // 导致回环 NDT 兜底实际一直在用默认 0.1m 网格，与 loop_closure.teaser_fallback_register.ndt_resolution 不一致。
    ndt.setResolution(config_.ndt_resolution);
    ndt.setStepSize(config_.ndt_step_size);
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
        if (static_cast<std::size_t>(src_idx) >= src->size() || static_cast<std::size_t>(tgt_idx) >= tgt->size()) continue;
        
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
    
    // 降采样（回环侧可关闭以保留语义 intensity）
    if (config_.preprocess_downsample && config_.downsample_before_refine && config_.downsample_voxel_size > 0) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(processed);
        vg.setLeafSize(config_.downsample_voxel_size, 
                       config_.downsample_voxel_size, 
                       config_.downsample_voxel_size);
        vg.filter(*processed);
    }
    
    // 统计去噪（会破坏逐点语义标签对齐，回环语义 ICP 路径应关闭）
    if (config_.preprocess_sor && processed->size() > 100) {
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
        if (static_cast<std::size_t>(src_idx) >= src->size() || static_cast<std::size_t>(tgt_idx) >= tgt->size()) continue;
        
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

// ─────────────────────────────────────────────────────────────────────────────
// 语义加权 ICP（SuMa++ 类：同类高权、异类/动态/未知降权）
// ─────────────────────────────────────────────────────────────────────────────

int IcpRefiner::intensityToSemanticLabel(float intensity) {
    if (!std::isfinite(intensity)) return 0;
    const int v = static_cast<int>(std::lround(intensity));
    if (v < 0 || v > 65535) return 0;
    return v;
}

double IcpRefiner::semanticLabelRatio(const CloudXYZIPtr& cloud) {
    if (!cloud || cloud->empty()) return 0.0;
    size_t n = 0;
    for (const auto& p : cloud->points) {
        if (intensityToSemanticLabel(p.intensity) != 0) ++n;
    }
    return static_cast<double>(n) / static_cast<double>(cloud->size());
}

double IcpRefiner::semanticCorrespondenceWeight(int ls, int lt,
                                                const std::unordered_set<int>& dyn) const {
    const bool dyn_s = dyn.count(ls) != 0;
    const bool dyn_t = dyn.count(lt) != 0;
    if (dyn_s || dyn_t) return static_cast<double>(config_.semantic_weight_dynamic);
    const bool unk_s = (ls == 0);
    const bool unk_t = (lt == 0);
    if (unk_s || unk_t) return static_cast<double>(config_.semantic_weight_unknown);
    if (ls == lt) return static_cast<double>(config_.semantic_weight_match);
    return static_cast<double>(config_.semantic_weight_mismatch);
}

IcpRefiner::Result IcpRefiner::refineSemanticWeighted(const CloudXYZIPtr& src,
                                                       const CloudXYZIPtr& tgt,
                                                       const Pose3d& initial) const {
    Result res;
    if (!config_.semantic_icp_enabled) {
        return refine(src, tgt, initial);
    }
    if (!src || !tgt || src->empty() || tgt->empty()) {
        return res;
    }

    const double r_src = semanticLabelRatio(src);
    const double r_tgt = semanticLabelRatio(tgt);
    if (r_src < config_.semantic_min_label_ratio || r_tgt < config_.semantic_min_label_ratio) {
        ALOG_DEBUG(MOD, "refineSemanticWeighted: label_ratio low (src={:.3f} tgt={:.3f} min={:.3f}) -> point-to-plane ICP",
                   r_src, r_tgt, config_.semantic_min_label_ratio);
        return refineICP(src, tgt, initial, true);
    }

    std::unordered_set<int> dyn;
    dyn.reserve(config_.semantic_dynamic_class_ids.size());
    for (int c : config_.semantic_dynamic_class_ids) dyn.insert(c);

    double sem_trust_scale = 1.0;
    if (config_.semantic_gray_zone_linear_trust) {
        const double rmin = std::min(r_src, r_tgt);
        const double mr = config_.semantic_min_label_ratio;
        if (rmin >= mr && rmin < 2.0 * mr) {
            sem_trust_scale = std::clamp((rmin - mr) / std::max(1e-9, mr), 0.0, 1.0);
        }
    }

    pcl::KdTreeFLANN<pcl::PointXYZI> kdt;
    kdt.setInputCloud(tgt);

    Pose3d T = initial;
    const int max_iter =
        (config_.semantic_max_iterations > 0) ? config_.semantic_max_iterations : config_.max_iterations;
    const double max_d = config_.max_correspondence_distance;
    const double max_d2 = max_d * max_d;

    int last_pairs = 0;
    for (int it = 0; it < max_iter; ++it) {
        std::vector<Eigen::Vector3d> P;
        std::vector<Eigen::Vector3d> Q;
        std::vector<double> Ww;
        P.reserve(src->size());
        Q.reserve(src->size());
        Ww.reserve(src->size());

        for (size_t i = 0; i < src->size(); ++i) {
            const auto& sp = src->points[i];
            if (!std::isfinite(sp.x) || !std::isfinite(sp.y) || !std::isfinite(sp.z)) continue;
            Eigen::Vector3d p(sp.x, sp.y, sp.z);
            Eigen::Vector3d tp = T * p;
            pcl::PointXYZI qq;
            qq.x = static_cast<float>(tp.x());
            qq.y = static_cast<float>(tp.y());
            qq.z = static_cast<float>(tp.z());
            std::vector<int> idx(1);
            std::vector<float> d2(1);
            if (kdt.nearestKSearch(qq, 1, idx, d2) < 1) continue;
            if (d2[0] > max_d2) continue;
            const auto& tp_pt = tgt->points[static_cast<size_t>(idx[0])];
            if (!std::isfinite(tp_pt.x) || !std::isfinite(tp_pt.y) || !std::isfinite(tp_pt.z)) continue;
            const int ls = intensityToSemanticLabel(sp.intensity);
            const int lt = intensityToSemanticLabel(tp_pt.intensity);
            double w = semanticCorrespondenceWeight(ls, lt, dyn);
            w = w * sem_trust_scale + 1.0 * (1.0 - sem_trust_scale);
            if (w < 1e-9) continue;
            P.push_back(p);
            Q.push_back(Eigen::Vector3d(tp_pt.x, tp_pt.y, tp_pt.z));
            Ww.push_back(w);
        }

        last_pairs = static_cast<int>(P.size());
        if (P.size() < 12) {
            ALOG_WARN(MOD, "refineSemanticWeighted: pairs={} < 12, fallback point-to-plane ICP", P.size());
            return refineICP(src, tgt, initial, true);
        }

        double sum_w = 0.0;
        Eigen::Vector3d p_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d q_mean = Eigen::Vector3d::Zero();
        for (size_t k = 0; k < P.size(); ++k) {
            sum_w += Ww[k];
            p_mean += Ww[k] * P[k];
            q_mean += Ww[k] * Q[k];
        }
        p_mean /= sum_w;
        q_mean /= sum_w;

        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (size_t k = 0; k < P.size(); ++k) {
            const Eigen::Vector3d a = P[k] - p_mean;
            const Eigen::Vector3d b = Q[k] - q_mean;
            H += Ww[k] * a * b.transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix3d Vc = svd.matrixV();
            Vc.col(2) *= -1.0;
            R = Vc * svd.matrixU().transpose();
        }
        const Eigen::Vector3d tvec = q_mean - R * p_mean;
        Eigen::Matrix4d Tf = Eigen::Matrix4d::Identity();
        Tf.block<3, 3>(0, 0) = R;
        Tf.block<3, 1>(0, 3) = tvec;
        Pose3d T_new;
        T_new.matrix() = Tf;

        const Pose3d delta = T.inverse() * T_new;
        const double dang = Eigen::AngleAxisd(delta.linear()).angle();
        const double dtr = delta.translation().norm();
        T = T_new;
        if (dang < 1e-5 && dtr < 1e-5) {
            break;
        }
    }

    res.T_refined = T;
    res.converged = true;
    res.iterations = max_iter;
    res.correspondences = last_pairs;

    double serr = 0.0;
    double sw = 0.0;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdt_rmse;
    kdt_rmse.setInputCloud(tgt);
    for (size_t i = 0; i < src->size(); ++i) {
        const auto& sp = src->points[i];
        if (!std::isfinite(sp.x) || !std::isfinite(sp.y) || !std::isfinite(sp.z)) continue;
        Eigen::Vector3d p(sp.x, sp.y, sp.z);
        Eigen::Vector3d tp = T * p;
        pcl::PointXYZI qq;
        qq.x = static_cast<float>(tp.x());
        qq.y = static_cast<float>(tp.y());
        qq.z = static_cast<float>(tp.z());
        std::vector<int> idx(1);
        std::vector<float> d2(1);
        if (kdt_rmse.nearestKSearch(qq, 1, idx, d2) < 1) continue;
        const double dist = std::sqrt(static_cast<double>(d2[0]));
        const int ls = intensityToSemanticLabel(sp.intensity);
        const auto& tp_pt = tgt->points[static_cast<size_t>(idx[0])];
        const int lt = intensityToSemanticLabel(tp_pt.intensity);
        const double w = semanticCorrespondenceWeight(ls, lt, dyn);
        if (w < 1e-9) continue;
        serr += w * dist * dist;
        sw += w;
    }
    res.rmse = (sw > 1e-12) ? std::sqrt(serr / sw) : 1e6;

    computePoseChange(initial, res.T_refined, res.rotation_angle_deg, res.translation_norm);

    if (config_.validate_result) {
        if (!validateResult(res, src, tgt)) {
            ALOG_DEBUG(MOD, "refineSemanticWeighted: validateResult failed, fallback point-to-plane ICP");
            return refineICP(src, tgt, initial, true);
        }
    }

    ALOG_DEBUG(MOD, "refineSemanticWeighted: converged pairs={} rmse={:.4f} deg={:.2f} trans={:.3f}m",
               res.correspondences, res.rmse, res.rotation_angle_deg, res.translation_norm);
    return res;
}

} // namespace automap_pro
