#include "automap_pro/v3/semantic_processor.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/config_manager.h"
#include <stdexcept>

// sloam_rec：include 根目录为包内 thrid_party/sloam_rec/sloam/include（由 CMake 注入）
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
#include <segmentation/inference.h>
#endif
#include <segmentation/trellis.h>
#include <helpers/definitions.h>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>

namespace automap_pro::v3 {

struct CylinderFittingCost {
    CylinderFittingCost(const Eigen::Vector3d& point) : point_(point) {}

    template <typename T>
    bool operator()(const T* const root, const T* const ray, const T* const radius, T* residual) const {
        Eigen::Matrix<T, 3, 1> p{T(point_.x()), T(point_.y()), T(point_.z())};
        Eigen::Matrix<T, 3, 1> r{root[0], root[1], root[2]};
        Eigen::Matrix<T, 3, 1> v{ray[0], ray[1], ray[2]};
        T rad = radius[0];

        // Distance from point p to line (r, v)
        // d = |(p - r) x v| / |v|
        Eigen::Matrix<T, 3, 1> pr = p - r;
        Eigen::Matrix<T, 3, 1> cross = pr.cross(v);
        T v_norm = v.norm();
        if (v_norm < T(1e-6)) return false;
        
        T dist_to_axis = cross.norm() / v_norm;
        residual[0] = dist_to_axis - rad;
        return true;
    }

    Eigen::Vector3d point_;
};

SemanticProcessor::SemanticProcessor(const Config& config) : config_(config) {
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
    try {
        // Initialize SLOAM components
        segmentator_ = std::make_shared<seg::Segmentation>(
            config.model_path, config.fov_up, config.fov_down, 
            config.img_w, config.img_h, 1, config.do_destagger);

        instance_detector_ = std::make_unique<Instance>();
        Instance::Params params;
        params.beam_cluster_threshold = config.beam_cluster_threshold;
        params.max_dist_to_centroid = config.max_dist_to_centroid;
        params.min_vertex_size = config.min_vertex_size;
        params.min_landmark_size = config.min_landmark_size;
        params.min_landmark_height = config.min_landmark_height;
        instance_detector_->set_params(params);
        
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
            "[SEMANTIC][Processor][INIT] step=ok model_path=%s fov=[%.1f,%.1f] img=%dx%d",
            config.model_path.c_str(), config.fov_up, config.fov_down, config.img_w, config.img_h);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), 
            "[SEMANTIC][Processor][INIT] step=FAILED model_path=%s error=%s → abort startup",
            config.model_path.c_str(), e.what());
        throw std::runtime_error(std::string("SemanticProcessor init failed: ") + e.what());
    }
#else
    (void)config;
    throw std::runtime_error("SemanticProcessor requires AUTOMAP_USE_SLOAM_SEMANTIC=1; stub mode is forbidden");
#endif
}

SemanticProcessor::~SemanticProcessor() = default;

std::vector<CylinderLandmark::Ptr> SemanticProcessor::process(const CloudXYZIConstPtr& cloud) {
    std::vector<CylinderLandmark::Ptr> landmarks;
#ifndef AUTOMAP_USE_SLOAM_SEMANTIC
    (void)cloud;
    return landmarks;
#else
    if (!segmentator_) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=segmentator_null (feature disabled)");
        return landmarks;
    }
    if (!cloud || cloud->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=cloud_null_or_empty");
        return landmarks;
    }

    std::lock_guard<std::recursive_mutex> lock(resource_mutex_);

    try {
        // 🏛️ [坐标系对齐] sloam_rec 内部使用 yaw = -atan2(y, x)
        // 标准 ROS 使用 atan2(y, x)。为了对齐，我们将 y 轴取反后再送入投影。
        CloudXYZIPtr flipped_cloud(new CloudXYZI());
        flipped_cloud->header = cloud->header;
        flipped_cloud->width = cloud->width;
        flipped_cloud->height = cloud->height;
        flipped_cloud->points.reserve(cloud->size());
        for (const auto& p : cloud->points) {
            auto p_f = p;
            p_f.y = -p.y;
            flipped_cloud->push_back(p_f);
        }

        // 1. Semantic Segmentation
        cv::Mat mask = cv::Mat::zeros(flipped_cloud->height, flipped_cloud->width, CV_8U);
        segmentator_->run(flipped_cloud, mask);

        // 2. Mask Tree Points
        CloudXYZIPtr tree_cloud(new CloudXYZI());
        segmentator_->maskCloud(flipped_cloud, mask, tree_cloud, 255, true);

        if (tree_cloud->empty()) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=mask_tree pts=%zu tree_pts=0 → no tree points",
                cloud->size());
            return landmarks;
        }

        // 3. Instance Segmentation (Clustering)
        std::vector<std::vector<TreeVertex>> tree_clusters;
        instance_detector_->computeGraph(flipped_cloud, tree_cloud, tree_clusters);

        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=instance tree_pts=%zu clusters=%zu",
            tree_cloud->size(), tree_clusters.size());

        // 4. Cylinder Fitting for each cluster
        for (const auto& cluster : tree_clusters) {
            std::vector<PointT> points;
            for (const auto& v : cluster) {
                for (const auto& p : v.points) {
                    points.push_back(p);
                }
            }

            if (points.size() < 10) continue;

            auto landmark = fitCylinder(points);
            if (landmark) {
                // 🏛️ [坐标系对齐] 将拟合结果从翻转坐标系转回 body 系
                landmark->root.y() = -landmark->root.y();
                landmark->ray.y() = -landmark->ray.y();
                landmarks.push_back(landmark);
            }
        }

        if (landmarks.empty()) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=done clusters=%zu fitted=0 (all rejected)",
                tree_clusters.size());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=done clusters=%zu fitted=%zu",
                tree_clusters.size(), landmarks.size());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=EXCEPTION error=%s cloud_size=%zu",
            e.what(), cloud ? cloud->size() : 0);
    }

    return landmarks;
#endif
}

bool SemanticProcessor::hasRuntimeCapability() const {
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
    return segmentator_ != nullptr;
#else
    return false;
#endif
}

CylinderLandmark::Ptr SemanticProcessor::fitCylinder(const std::vector<pcl::PointXYZI>& points) {
#ifndef AUTOMAP_USE_SLOAM_SEMANTIC
    (void)points;
    return nullptr;
#else
    // 1. Initial Guess using RANSAC line fitting
    CloudXYZIPtr tree(new CloudXYZI());
    for (const auto& p : points) tree->push_back(p);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(tree);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=ransac_reject pts=%zu inliers=0", points.size());
        return nullptr;
    }

    double root[3] = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
    double ray[3] = {coefficients->values[3], coefficients->values[4], coefficients->values[5]};
    
    // Initial radius estimate: median distance to axis
    std::vector<double> distances;
    Eigen::Vector3d r(root[0], root[1], root[2]);
    Eigen::Vector3d v(ray[0], ray[1], ray[2]);
    v.normalize();
    
    for (const auto& p : points) {
        Eigen::Vector3d pt(p.x, p.y, p.z);
        double d = (pt - r).cross(v).norm();
        distances.push_back(d);
    }
    std::sort(distances.begin(), distances.end());
    double radius = distances[distances.size() / 2];

    // 2. Ceres Optimization
    ceres::Problem problem;
    for (const auto& p : points) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CylinderFittingCost, 1, 3, 3, 1>(
                new CylinderFittingCost(Eigen::Vector3d(p.x, p.y, p.z))),
            nullptr,
            root, ray, &radius);
    }

    problem.SetManifold(ray, new ceres::SphereManifold<3>());

    // 约束半径不为负
    problem.SetParameterLowerBound(&radius, 0, 0.005);
    problem.SetParameterUpperBound(&radius, 0, config_.max_tree_radius);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.termination_type != ceres::CONVERGENCE && summary.termination_type != ceres::USER_SUCCESS) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=ceres_fail pts=%zu term=%d iters=%d",
            points.size(), static_cast<int>(summary.termination_type), summary.iterations.size());
        return nullptr;
    }

    auto landmark = std::make_shared<CylinderLandmark>();
    landmark->root = Eigen::Vector3d(root[0], root[1], root[2]);
    landmark->ray = Eigen::Vector3d(ray[0], ray[1], ray[2]).normalized();
    landmark->radius = radius;
    landmark->confidence = static_cast<double>(inliers->indices.size()) / points.size();

    // Sanity checks
    if (landmark->radius > config_.max_tree_radius || landmark->radius < 0.01) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=reject_radius radius=%.3f max=%.2f",
            landmark->radius, config_.max_tree_radius);
        return nullptr;
    }

    // Check axis orientation (tilt from vertical)
    double tilt = std::acos(std::abs(landmark->ray.dot(Eigen::Vector3d::UnitZ()))) * 180.0 / M_PI;
    if (tilt > config_.max_axis_theta) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=reject_tilt tilt=%.1f deg max=%.1f",
            tilt, config_.max_axis_theta);
        return nullptr;
    }

    return landmark;
#endif
}

} // namespace automap_pro::v3
