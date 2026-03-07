#include "hba_api/hba_api.h"

// 确保 PCL io 在 HBA 头文件之前可用（避免 mypcl.hpp 中 pcl::io 未声明）
#include <pcl/io/pcd_io.h>

// 直接包含 HBA 算法头文件（源码级复用）
// HBA 算法核心位于 automap_ws/src/hba/include/
#include "layer.hpp"
#include "voxel.hpp"
#include "mypcl.hpp"
#include "tools.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <thread>

// HBA 核心函数（由 libhba_core 提供）
extern void distribute_thread(LAYER& layer, LAYER& next_layer);
extern void global_ba(LAYER& layer);

namespace hba_api {

// ─────────────────────────────────────────────────────────────────────────────
// 内部实现（Pimpl）
// ─────────────────────────────────────────────────────────────────────────────
struct HBAOptimizer::Impl {
    Config config;
    std::vector<KeyFrameInput> keyframes;

    explicit Impl(const Config& cfg) : config(cfg) {}

    // 将 KeyFrameInput 转换为 mypcl::pose（HBA 内部格式）
    static mypcl::pose to_mypcl_pose(const KeyFrameInput& kf) {
        mypcl::pose p;
        p.q = kf.rotation;
        p.t = kf.translation;
        return p;
    }

    // 将 mypcl::pose 转换回 Eigen::Isometry3d
    static Eigen::Isometry3d from_mypcl_pose(const mypcl::pose& p) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear()      = p.q.toRotationMatrix();
        T.translation() = p.t;
        return T;
    }

    // 体素降采样并转为 HBA 层所需的 PointXYZI（layer 使用 mypcl::PointType == PointXYZI）
    static pcl::PointCloud<pcl::PointXYZI>::Ptr voxelizeToXYZI(
        const CloudPtr& cloud, double voxel_size)
    {
        using OutPoint = pcl::PointXYZI;
        pcl::PointCloud<OutPoint>::Ptr out(new pcl::PointCloud<OutPoint>);
        if (!cloud || cloud->empty()) return out;
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>);
        vg.filter(*tmp);
        out->resize(tmp->size());
        for (size_t i = 0; i < tmp->size(); ++i) {
            (*out)[i].x = (*tmp)[i].x;
            (*out)[i].y = (*tmp)[i].y;
            (*out)[i].z = (*tmp)[i].z;
            (*out)[i].intensity = (*tmp)[i].intensity;
        }
        return out;
    }

    // 计算 Mean Map Entropy
    static double calc_mme(
        const std::vector<KeyFrameInput>& kfs,
        const std::vector<Eigen::Isometry3d>& poses,
        double voxel_size)
    {
        if (kfs.empty() || poses.size() != kfs.size()) return 0.0;

        // 合并所有点云到世界坐标系
        std::unordered_map<VOXEL_LOC, VOX_FACTOR> voxel_map;
        const double inv_voxel = 1.0 / voxel_size;

        for (size_t i = 0; i < kfs.size(); ++i) {
            if (!kfs[i].cloud || kfs[i].cloud->empty()) continue;
            Eigen::Matrix3d R = poses[i].rotation();
            Eigen::Vector3d t = poses[i].translation();

            for (const auto& pt : kfs[i].cloud->points) {
                Eigen::Vector3d pw = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + t;
                VOXEL_LOC loc(
                    static_cast<int64_t>(std::floor(pw.x() * inv_voxel)),
                    static_cast<int64_t>(std::floor(pw.y() * inv_voxel)),
                    static_cast<int64_t>(std::floor(pw.z() * inv_voxel)));

                auto& vf = voxel_map[loc];
                vf.P += pw * pw.transpose();
                vf.v += pw;
                vf.N += 1;
            }
        }

        // 计算每个体素的最小特征值（平面度）
        double total_entropy = 0.0;
        int count = 0;
        for (const auto& kv : voxel_map) {
            const auto& vf = kv.second;
            if (vf.N < 5) continue;
            Eigen::Vector3d mean = vf.v / vf.N;
            Eigen::Matrix3d cov = vf.P / vf.N - mean * mean.transpose();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
            double lambda_min = es.eigenvalues()(0);  // 最小特征值
            if (lambda_min > 0) {
                total_entropy += std::log(lambda_min);
                count++;
            }
        }
        return count > 0 ? total_entropy / count : 0.0;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// 公共接口实现
// ─────────────────────────────────────────────────────────────────────────────
HBAOptimizer::HBAOptimizer(const Config& cfg)
    : impl_(std::make_unique<Impl>(cfg)) {}

HBAOptimizer::~HBAOptimizer() = default;

void HBAOptimizer::addKeyFrame(const KeyFrameInput& kf) {
    impl_->keyframes.push_back(kf);
}

void HBAOptimizer::setKeyFrames(const std::vector<KeyFrameInput>& kfs) {
    impl_->keyframes = kfs;
}

void HBAOptimizer::clear() {
    impl_->keyframes.clear();
}

size_t HBAOptimizer::keyFrameCount() const {
    return impl_->keyframes.size();
}

Result HBAOptimizer::optimize(ProgressCallback progress_cb) {
    auto t0 = std::chrono::steady_clock::now();
    Result result;

    const auto& kfs = impl_->keyframes;
    if (kfs.empty()) {
        result.error_msg = "No keyframes added";
        return result;
    }

    const int N = static_cast<int>(kfs.size());
    const int total_layers = impl_->config.total_layer_num;
    const int thread_num   = impl_->config.thread_num;
    const double vs = impl_->config.voxel_size;

    try {
        // ── 初始化底层（Layer 0）──────────────────────────────────────────
        std::vector<LAYER> layers(total_layers);
        for (int i = 0; i < total_layers; ++i) {
            layers[i].layer_num  = i + 1;
            layers[i].thread_num = thread_num;
        }

        // 底层 pose_vec：从 KeyFrameInput 构建
        layers[0].pose_vec.resize(N);
        layers[0].pcds.resize(N);
        for (int i = 0; i < N; ++i) {
            layers[0].pose_vec[i] = Impl::to_mypcl_pose(kfs[i]);
            // 点云体素化
            layers[0].pcds[i] = Impl::voxelizeToXYZI(kfs[i].cloud, vs);
        }
        layers[0].init_layer_param();
        layers[0].init_storage(total_layers);

        // 初始化上层
        for (int i = 1; i < total_layers; ++i) {
            int pose_size = (layers[i-1].thread_num - 1) * layers[i-1].part_length;
            pose_size += (layers[i-1].tail == 0)
                ? layers[i-1].left_gap_num
                : layers[i-1].left_gap_num + 1;
            layers[i].init_layer_param(pose_size);
            layers[i].init_storage(total_layers);
        }

        // ── 分层优化（逐层从底到顶）──────────────────────────────────────
        for (int layer_idx = 0; layer_idx < total_layers - 1; ++layer_idx) {
            if (progress_cb) {
                progress_cb(layer_idx, total_layers,
                            100.0f * layer_idx / total_layers);
            }
            distribute_thread(layers[layer_idx], layers[layer_idx + 1]);
        }

        // ── 顶层全局 BA ───────────────────────────────────────────────────
        if (progress_cb) {
            progress_cb(total_layers - 1, total_layers,
                        100.0f * (total_layers - 1) / total_layers);
        }
        global_ba(layers[total_layers - 1]);

        // ── GTSAM 位姿图优化（PGO）────────────────────────────────────────
        // 收集最终底层位姿
        // (write_interpolate_pose 会将顶层优化结果写回底层)
        std::vector<mypcl::pose> final_poses = layers[0].pose_vec;
        // 注意：HBA 原始实现通过 update_next_layer_state 传播到底层
        // 这里简化处理：取底层已更新的 pose_vec

        // GTSAM PGO：添加 BetweenFactor（参考 hba.cpp::pose_graph_optimization）
        // 使用局部变量名避免与 layer.hpp 中宏 WIN_SIZE/GAP 冲突
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values init_vals;

        const int pgo_win_size = impl_->config.win_size;
        const int pgo_gap      = impl_->config.gap;

        for (int i = 0; i < N; ++i) {
            gtsam::Symbol sym('x', i);
            init_vals.insert(sym,
                gtsam::Pose3(gtsam::Rot3(final_poses[i].q.toRotationMatrix()),
                             gtsam::Point3(final_poses[i].t)));
        }

        // 先验（固定第一帧）
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            gtsam::Symbol('x', 0),
            init_vals.at<gtsam::Pose3>(gtsam::Symbol('x', 0)),
            gtsam::noiseModel::Constrained::All(6)));

        // 底层窗口 BetweenFactor
        for (int w = 0; w < (int)layers[0].hessians.size(); ++w) {
            int base = w * pgo_gap;
            for (int i = base; i < std::min(base + pgo_win_size, N) - 1; ++i) {
                if (i + 1 >= N) break;
                // 相对位姿
                Eigen::Matrix3d Ri = final_poses[i].q.toRotationMatrix();
                Eigen::Matrix3d Rj = final_poses[i+1].q.toRotationMatrix();
                Eigen::Vector3d ti = final_poses[i].t;
                Eigen::Vector3d tj = final_poses[i+1].t;
                Eigen::Matrix3d Rij = Ri.transpose() * Rj;
                Eigen::Vector3d tij = Ri.transpose() * (tj - ti);

                gtsam::Pose3 relative_pose(gtsam::Rot3(Rij), gtsam::Point3(tij));
                auto noise = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    gtsam::Symbol('x', i), gtsam::Symbol('x', i+1), relative_pose, noise));
            }
        }

        // GPS 因子（如启用）
        if (impl_->config.enable_gps) {
            // GPS 因子在 GPSManager 外部添加，此处跳过
        }

        // LM 优化
        gtsam::LevenbergMarquardtParams lm_params;
        lm_params.maxIterations = 50;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
        gtsam::Values optimized = optimizer.optimize();

        // ── 提取结果 ─────────────────────────────────────────────────────
        result.optimized_poses.resize(N);
        for (int i = 0; i < N; ++i) {
            try {
                auto p = optimized.at<gtsam::Pose3>(gtsam::Symbol('x', i));
                Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
                T.linear()      = p.rotation().matrix();
                T.translation() = p.translation();
                result.optimized_poses[i] = T;
            } catch (...) {
                result.optimized_poses[i] = Impl::from_mypcl_pose(final_poses[i]);
            }
        }

        auto t1 = std::chrono::steady_clock::now();
        result.elapsed_ms     = std::chrono::duration<double, std::milli>(t1 - t0).count();
        result.total_keyframes = N;
        result.final_mme      = Impl::calc_mme(kfs, result.optimized_poses, vs);
        result.success        = true;

        if (progress_cb) progress_cb(total_layers, total_layers, 100.0f);

    } catch (const std::exception& e) {
        result.error_msg = e.what();
        result.success = false;
    }

    return result;
}

double HBAOptimizer::computeMME(
    const std::vector<KeyFrameInput>& kfs,
    const std::vector<Eigen::Isometry3d>& poses,
    double voxel_size)
{
    return Impl::calc_mme(kfs, poses, voxel_size);
}

} // namespace hba_api
