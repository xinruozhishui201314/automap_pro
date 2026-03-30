/**
 * @file backend/optimizer.cpp
 * @brief 后端优化与因子实现。
 */
#include "automap_pro/backend/optimizer.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/core/logger.h"

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>

#define MOD "Optimizer"

#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/robust.h>
#endif

namespace automap_pro {

void Optimizer::setOptions(const Options& opts) {
    options_ = opts;
}

Optimizer::Result Optimizer::optimize(PoseGraph& graph) const {
#ifdef USE_GTSAM
    return optimizeGTSAM(graph);
#else
    return optimizeGaussNewton(graph);
#endif
}

#ifdef USE_GTSAM
Optimizer::Result Optimizer::optimizeGTSAM(PoseGraph& graph) const {
    utils::Timer timer;
    Result result;

    gtsam::NonlinearFactorGraph factor_graph;
    gtsam::Values initial_values;

    auto nodes = graph.allNodes();
    auto edges = graph.allEdges();

    // 空图或仅先验的保护：与 Gauss-Newton 分支语义保持一致
    if (nodes.empty()) {
        result.success = false;
        result.converged = false;
        result.fail_reason = Result::FailReason::NUMERICAL_ISSUES;
        result.fail_message = "Empty graph: no nodes to optimize";
        result.time_ms = timer.elapsedMs();
        return result;
    }
    if (edges.empty()) {
        // 只有先验约束时，无需真正优化，直接返回
        result.success = true;
        result.converged = true;
        result.fail_reason = Result::FailReason::NONE;
        result.final_cost = 0.0;
        result.iterations = 0;
        result.time_ms = timer.elapsedMs();
        return result;
    }

    std::string params = "nodes=" + std::to_string(nodes.size()) + " edges=" + std::to_string(edges.size());
    GtsamCallScope scope(GtsamCaller::Optimizer, "optimize", params, true);

    using gtsam::symbol_shorthand::X;

    // Add nodes
    for (const auto& node : nodes) {
        const auto& T = node.pose;
        gtsam::Rot3   R(T.rotation());
        gtsam::Point3 p(T.translation());
        gtsam::Pose3  pose(R, p);
        initial_values.insert(X(node.id), pose);

        if (node.fixed) {
            auto prior_noise = gtsam::noiseModel::Constrained::All(6);
            factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(node.id), pose, prior_noise));
        }
    }

    // Add edges
    for (const auto& edge : edges) {
        if (edge.to < 0) {
            // GPS unary factor: 位置约束强，旋转约束弱（使用 PriorFactor<Pose3>）
            gtsam::Pose3 gps_prior(gtsam::Rot3(), gtsam::Point3(edge.measurement.translation()));
            // 6x6 协方差：位置用 information 的逆（带秩/条件数检查），旋转用大方差（弱约束）
            Eigen::Matrix3d pos_info = edge.information.block<3,3>(0,0);
            Eigen::Matrix3d pos_cov;

            // 数值稳定性保护：SVD 检查秩与条件数，退化/病态时采用保守协方差
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(pos_info, Eigen::ComputeFullU | Eigen::ComputeFullV);
            const int rank = svd.rank();
            const double sv_max = svd.singularValues()(0);
            const double sv_min = svd.singularValues()(2);
            const double cond   = (sv_min > 1e-12) ? (sv_max / sv_min) : 1e12;

            bool use_safe_cov = (rank < 3) || !std::isfinite(cond) || cond > 1e6;
            if (use_safe_cov) {
                ALOG_WARN(MOD,
                          "GPS information matrix ill-conditioned (rank={}/3, cond={:.2e}), "
                          "using conservative covariance",
                          rank, cond);
                pos_cov = 1e2 * Eigen::Matrix3d::Identity();
            } else {
                // 使用 LDLT 求逆，若失败再回退
                Eigen::LLT<Eigen::Matrix3d> llt(pos_info);
                if (llt.info() != Eigen::Success) {
                    ALOG_WARN(MOD,
                              "GPS information LLT decomposition failed, using conservative covariance");
                    pos_cov = 1e2 * Eigen::Matrix3d::Identity();
                } else {
                    pos_cov = llt.solve(Eigen::Matrix3d::Identity());
                    if (!pos_cov.allFinite()) {
                        ALOG_WARN(MOD,
                                  "GPS covariance contains NaN/Inf, using conservative covariance");
                        pos_cov = 1e2 * Eigen::Matrix3d::Identity();
                    }
                }
            }

            gtsam::Matrix66 cov6 = gtsam::Matrix66::Identity();
            cov6.block<3,3>(0,0) = pos_cov;
            cov6.block<3,3>(3,3) = 1e6 * Eigen::Matrix3d::Identity();  // 旋转弱约束
            auto gps_noise = gtsam::noiseModel::Gaussian::Covariance(cov6);
            if (options_.use_robust_kernel) {
                auto robust = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(options_.robust_kernel_delta),
                    gps_noise);
                factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    X(edge.from), gps_prior, robust));
            } else {
                factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    X(edge.from), gps_prior, gps_noise));
            }
            continue;
        }

        const auto& T  = edge.measurement;
        gtsam::Rot3   R(T.rotation());
        gtsam::Point3 p(T.translation());
        gtsam::Pose3  rel(R, p);

        auto noise = gtsam::noiseModel::Gaussian::Information(edge.information);
        if (options_.use_robust_kernel && edge.type == EdgeType::LOOP) {
            auto robust = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(options_.robust_kernel_delta),
                noise);
            factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(edge.from), X(edge.to), rel, robust));
        } else {
            factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(edge.from), X(edge.to), rel, noise));
        }
    }

    gtsam::LevenbergMarquardtParams params;
    params.maxIterations          = options_.max_iterations;
    params.relativeErrorTol       = options_.convergence_threshold;
    params.absoluteErrorTol       = options_.convergence_threshold;
    params.verbosity              = options_.verbose ?
        gtsam::NonlinearOptimizerParams::ERROR : gtsam::NonlinearOptimizerParams::SILENT;

    try {
        gtsam::LevenbergMarquardtOptimizer lm(factor_graph, initial_values, params);
        auto optimized = lm.optimize();

        // ─────────────────────────────────────────────────────────────
        // 求解失败检测
        // ─────────────────────────────────────────────────────────────

        // 检查最终代价是否过大（发散）
        double final_error = lm.error();
        if (options_.check_numerical_issues && final_error > options_.max_cost_threshold) {
            scope.setSuccess(false);
            result.success = false;
            result.converged = false;
            result.fail_reason = Result::FailReason::COST_DIVERGED;
            result.fail_message = fmt::format(
                "Optimization diverged: final_cost={} exceeds threshold {}",
                final_error, options_.max_cost_threshold);
            ALOG_WARN(MOD, "Optimization diverged: cost={} exceeds threshold {}",
                final_error, options_.max_cost_threshold);
            RCLCPP_WARN(rclcpp::get_logger("automap_pro"),
                "[Optimizer] Optimization diverged: cost=%e (threshold=%e)", 
                final_error, options_.max_cost_threshold);
            result.time_ms = timer.elapsedMs();
            return result;
        }

        // 检查迭代次数：与 Gauss-Newton 统一语义，未收敛则不写回
        int actual_iterations = static_cast<int>(lm.iterations());
        bool gtsam_converged = (actual_iterations < options_.max_iterations);
        if (!gtsam_converged) {
            scope.setSuccess(false);
            result.success = false;
            result.converged = false;
            result.fail_reason = Result::FailReason::MAX_ITERATIONS_REACHED;
            result.fail_message = fmt::format(
                "Reached max iterations ({}) without convergence", options_.max_iterations);
            ALOG_WARN(MOD, "GTSAM max iterations reached: {} iters, not writing back", actual_iterations);
            RCLCPP_WARN(rclcpp::get_logger("automap_pro"),
                "[Optimizer] GTSAM max iterations reached: %d iters, not writing back", actual_iterations);
            result.final_cost = final_error;
            result.iterations = actual_iterations;
            result.time_ms = timer.elapsedMs();
            return result;
        }

        // 检查优化值是否有效（数值问题）
        bool has_numerical_issues = false;
        for (const auto& node : nodes) {
            if (optimized.exists(X(node.id))) {
                auto pose = optimized.at<gtsam::Pose3>(X(node.id));
                // 检查 NaN/Inf
                if (!std::isfinite(pose.x()) || !std::isfinite(pose.y()) ||
                    !std::isfinite(pose.z()) ||
                    !std::isfinite(pose.rotation().quaternion().w()) ||
                    !std::isfinite(pose.rotation().quaternion().x()) ||
                    !std::isfinite(pose.rotation().quaternion().y()) ||
                    !std::isfinite(pose.rotation().quaternion().z())) {
                    has_numerical_issues = true;
                    break;
                }
            }
        }

        if (has_numerical_issues) {
            scope.setSuccess(false);
            result.success = false;
            result.converged = false;
            result.fail_reason = Result::FailReason::NUMERICAL_ISSUES;
            result.fail_message = "Numerical issues detected (NaN/Inf) in optimized poses";
            ALOG_ERROR(MOD, "Numerical issues detected in optimized poses");
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[Optimizer] Numerical issues detected in optimized poses");
            result.time_ms = timer.elapsedMs();
            return result;
        }

        // 写回优化结果（仅收敛时）
        for (const auto& node : nodes) {
            if (optimized.exists(X(node.id))) {
                auto pose = optimized.at<gtsam::Pose3>(X(node.id));
                Pose3d T_opt;
                T_opt.linear()      = pose.rotation().matrix();
                T_opt.translation() = pose.translation();
                graph.updateNodePose(node.id, T_opt);
            }
        }

        result.success    = true;
        result.converged  = true;
        result.final_cost = lm.error();
        result.iterations = static_cast<int>(lm.iterations());
        scope.setSuccess(true);
    } catch (const std::exception& e) {
        scope.setSuccess(false);
        ALOG_ERROR(MOD, "GTSAM optimization failed: {}", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"), 
            "[Optimizer] GTSAM optimization failed: %s", e.what());
        result.success = false;
        result.converged = false;
        result.fail_reason = Result::FailReason::LINEAR_SYSTEM_SINGULAR;
        result.fail_message = std::string("Exception: ") + e.what();
    }

    result.time_ms = timer.elapsedMs();
    return result;
}
#endif

Optimizer::Result Optimizer::optimizeGaussNewton(PoseGraph& graph) const {
    utils::Timer timer;
    Result result;

    auto nodes = graph.allNodes();
    auto edges = graph.allEdges();

    if (nodes.empty()) {
        result.success = false;
        result.fail_reason = Result::FailReason::NUMERICAL_ISSUES;
        result.fail_message = "Empty graph: no nodes to optimize";
        result.time_ms = timer.elapsedMs();
        return result;
    }

    // Build index map
    std::map<int, int> id_to_idx;
    int idx = 0;
    for (const auto& n : nodes) { id_to_idx[n.id] = idx++; }
    int n_nodes = static_cast<int>(nodes.size());

    // Current poses
    std::vector<Pose3d> poses(n_nodes);
    for (const auto& n : nodes) {
        poses[id_to_idx[n.id]] = n.pose;
    }

    // Fixed node index
    int fixed_idx = -1;
    for (const auto& n : nodes) {
        if (n.fixed) { fixed_idx = id_to_idx[n.id]; break; }
    }
    if (fixed_idx < 0) fixed_idx = 0;  // fix first if none fixed

    const int dof = 6;
    const int N   = n_nodes * dof;

    bool converged = false;

    for (int iter = 0; iter < options_.max_iterations; ++iter) {
        Eigen::VectorXd b_vec = Eigen::VectorXd::Zero(N);
        Eigen::MatrixXd H_mat = Eigen::MatrixXd::Zero(N, N);

        for (const auto& edge : edges) {
            if (edge.to < 0) continue;  // skip GPS unary for now
            auto it_from = id_to_idx.find(edge.from);
            auto it_to   = id_to_idx.find(edge.to);
            if (it_from == id_to_idx.end() || it_to == id_to_idx.end()) continue;

            int fi = it_from->second;
            int ti = it_to->second;

            const Pose3d& T_from = poses[fi];
            const Pose3d& T_to   = poses[ti];
            const Pose3d& T_meas = edge.measurement;

            // Error: e = log(T_meas^-1 * T_from^-1 * T_to)
            Pose3d T_pred = T_from.inverse() * T_to;
            Pose3d T_err  = T_meas.inverse() * T_pred;
            Vec6d  e      = utils::poseToVec6d(T_err);

            // Jacobians (approximate)
            Eigen::MatrixXd J_from = -Eigen::MatrixXd::Identity(dof, dof);
            Eigen::MatrixXd J_to   =  Eigen::MatrixXd::Identity(dof, dof);

            const auto& Omega = edge.information;
            H_mat.block(fi*dof, fi*dof, dof, dof) += J_from.transpose() * Omega * J_from;
            H_mat.block(ti*dof, ti*dof, dof, dof) += J_to.transpose()   * Omega * J_to;
            H_mat.block(fi*dof, ti*dof, dof, dof) += J_from.transpose() * Omega * J_to;
            H_mat.block(ti*dof, fi*dof, dof, dof) += J_to.transpose()   * Omega * J_from;

            b_vec.segment(fi*dof, dof) += J_from.transpose() * Omega * e;
            b_vec.segment(ti*dof, dof) += J_to.transpose()   * Omega * e;
        }

        // Fix reference node
        for (int d = 0; d < dof; ++d) {
            H_mat(fixed_idx*dof + d, fixed_idx*dof + d) += 1e10;
        }

        // Solve H * dx = -b
        Eigen::VectorXd dx = H_mat.ldlt().solve(-b_vec);

        // ─────────────────────────────────────────────────────────────
        // 求解失败检测
        // ─────────────────────────────────────────────────────────────
        if (!dx.allFinite()) {
            result.success = false;
            result.fail_reason = Result::FailReason::NUMERICAL_ISSUES;
            result.fail_message = "GaussNewton optimization produced NaN/Inf values";
            ALOG_ERROR(MOD, "GaussNewton optimization produced NaN/Inf in solution vector");
            result.time_ms = timer.elapsedMs();
            return result;
        }

        // Update poses
        for (int i = 0; i < n_nodes; ++i) {
            Vec6d delta = dx.segment<6>(i * dof);
            Pose3d delta_T = utils::vec6dToPose(delta);
            poses[i] = poses[i] * delta_T;
        }

        double update_norm = dx.norm();
        if (update_norm < options_.convergence_threshold) {
            result.iterations = iter + 1;
            converged = true;
            break;
        }
    }

    if (converged) {
        // 仅在收敛时写回优化结果
        for (const auto& n : nodes) {
            int i = id_to_idx[n.id];
            graph.updateNodePose(n.id, poses[i]);
        }
        result.success = true;
        result.converged = true;
        result.fail_reason = Result::FailReason::NONE;
    } else {
        // 未在最大迭代次数前收敛：不写回，保留原图
        result.success = false;
        result.converged = false;
        result.fail_reason = Result::FailReason::MAX_ITERATIONS_REACHED;
        result.fail_message = "GaussNewton reached max iterations without convergence";
        result.iterations = options_.max_iterations;
        ALOG_WARN(MOD, "GaussNewton reached max iterations (%d) without convergence",
                  options_.max_iterations);
    }

    result.time_ms = timer.elapsedMs();
    return result;
}

}  // namespace automap_pro
