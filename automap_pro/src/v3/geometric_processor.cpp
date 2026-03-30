/**
 * @file v3/geometric_processor.cpp
 * @brief V3 流水线模块实现。
 */
#include "automap_pro/v3/geometric_processor.h"
#include <patchworkpp/patchworkpp.hpp>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_set>
#include <vector>

namespace automap_pro::v3 {
namespace {
std::atomic<uint64_t> g_geo_frame_counter{0};

std::string toLowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool geoInfoEnabled(const GeometricProcessorConfig& cfg) {
    return toLowerCopy(cfg.log_level) != "off";
}

bool geoDebugEnabled(const GeometricProcessorConfig& cfg) {
    const std::string level = toLowerCopy(cfg.log_level);
    if (level == "off") return false;
    return level == "debug" || cfg.log_detail;
}

rclcpp::Clock& geoThrottleClock() {
    static rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock;
}

/// Debug dump: merged multi-frame body cloud or primitive-classifier input (current body frame, binary compressed PCD).
void maybeSaveGeoSemanticDebugPcd(const GeometricProcessorConfig& cfg,
                                  uint64_t frame_idx,
                                  double ts,
                                  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                                  const char* tag) {
    if (!cfg.accumulator.save_debug_pcd) {
        return;
    }
    const std::string& dir = cfg.accumulator.save_merged_cloud_dir;
    if (dir.empty() || !cloud || cloud->empty() || !tag) {
        return;
    }
    const int every_n = std::max(1, std::min(100000, cfg.accumulator.save_merged_cloud_every_n));
    if ((frame_idx % static_cast<uint64_t>(every_n)) != 0u) {
        return;
    }
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            10000,
            "[GEOMETRIC][SAVE_ACCUM] mkdir failed path=%s msg=%s",
            dir.c_str(),
            ec.message().c_str());
        return;
    }
    char fname[384];
    std::snprintf(fname,
        sizeof(fname),
        "geo_sem_%s_idx%lu_ts%.6f_pts%zu.pcd",
        tag,
        static_cast<unsigned long>(frame_idx),
        ts,
        cloud->size());
    const fs::path out_path = fs::path(dir) / fname;
    const int rc = pcl::io::savePCDFileBinaryCompressed(out_path.string(), *cloud);
    if (rc != 0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[GEOMETRIC][SAVE_ACCUM] save failed rc=%d path=%s",
            rc,
            out_path.string().c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[GEOMETRIC][SAVE_ACCUM] wrote %s pts=%zu tag=%s every_n=%d",
        out_path.string().c_str(),
        cloud->size(),
        tag,
        every_n);
}

/** Patchwork++ 中 sensor_height 为正，且 RNR 使用 z < -sensor_height - 0.8，即默认假设车体系 z 向上、路面在传感器下方（z 为负）。 */
std::vector<float> collectGroundHeightSamples(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    float min_xy_m,
    float max_xy_m,
    float max_z_over_r) {
    std::vector<float> zs;
    if (!cloud || cloud->empty()) {
        return zs;
    }
    zs.reserve(std::min(cloud->size(), size_t{120000}));
    const double zor = static_cast<double>(max_z_over_r);
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        const double r = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
        if (r < static_cast<double>(min_xy_m) || r > static_cast<double>(max_xy_m)) {
            continue;
        }
        if (zor > 1e-9 && static_cast<double>(std::abs(p.z)) > zor * r) {
            continue;
        }
        zs.push_back(p.z);
    }
    return zs;
}

Eigen::Vector3d odomUpUnit(int axis) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    const int a = std::clamp(axis, 0, 2);
    e(a) = 1.0;
    return e;
}

/// T_odom_b: p_odom = R * p_body + t  =>  v_body = R^T * v_odom
Eigen::Vector3d gravityUpInBody(const Eigen::Isometry3d& T_odom_b, int axis) {
    const Eigen::Vector3d e_odom = odomUpUnit(axis);
    const Eigen::Matrix3d R = T_odom_b.linear();
    Eigen::Vector3d v = R.transpose() * e_odom;
    const double n = v.norm();
    if (n < 1e-9) {
        return Eigen::Vector3d::UnitZ();
    }
    return v / n;
}

void rotateCloudPoints(const pcl::PointCloud<pcl::PointXYZI>& in,
                       const Eigen::Matrix3d& R,
                       pcl::PointCloud<pcl::PointXYZI>& out) {
    out.points.clear();
    out.header = in.header;
    out.points.reserve(in.size());
    for (const auto& p : in.points) {
        Eigen::Vector3d v(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        v = R * v;
        pcl::PointXYZI q = p;
        q.x = static_cast<float>(v.x());
        q.y = static_cast<float>(v.y());
        q.z = static_cast<float>(v.z());
        out.push_back(q);
    }
    out.width = static_cast<uint32_t>(out.points.size());
    out.height = 1;
}

void applyInverseRotationToCloud(const Eigen::Matrix3d& R_inv, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (!cloud) {
        return;
    }
    for (auto& p : cloud->points) {
        Eigen::Vector3d v(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        v = R_inv * v;
        p.x = static_cast<float>(v.x());
        p.y = static_cast<float>(v.y());
        p.z = static_cast<float>(v.z());
    }
}

/**
 * Patchwork nonground 已在当前帧 body；按水平环带（可选体素）缩小 primitive 输入，减轻大范围关键帧上 RANSAC/聚类的「全局稀疏」。
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr applyPrimitiveRoiPipeline(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,
    const GeometricProcessorConfig::PrimitiveRoi& roi,
    size_t& dropped_xy_or_nan,
    size_t& removed_by_voxel,
    size_t& final_size) {
    dropped_xy_or_nan = 0;
    removed_by_voxel = 0;
    final_size = 0;
    if (!roi.enabled || !in || in->empty()) {
        final_size = in ? in->size() : 0;
        return in;
    }

    double r_outer = static_cast<double>(roi.body_xy_radius_m);
    const double ring_max = static_cast<double>(roi.ring_max_xy_m);
    if (ring_max > 1e-9) {
        if (r_outer > 1e-9) {
            r_outer = std::min(r_outer, ring_max);
        } else {
            r_outer = ring_max;
        }
    }
    const double r_min = static_cast<double>(roi.ring_min_xy_m);
    const bool has_outer = r_outer > 1e-9;
    const bool has_inner = r_min > 1e-9;
    const bool do_voxel = roi.voxel_leaf_m > 1e-6f;

    if (!has_outer && !has_inner && !do_voxel) {
        final_size = in->size();
        return in;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr spatial;
    if (has_outer || has_inner) {
        spatial.reset(new pcl::PointCloud<pcl::PointXYZI>());
        spatial->header = in->header;
        spatial->points.reserve(in->size());
        for (const auto& p : in->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                ++dropped_xy_or_nan;
                continue;
            }
            const double r = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
            if (has_inner && r < r_min) {
                ++dropped_xy_or_nan;
                continue;
            }
            if (has_outer && r > r_outer) {
                ++dropped_xy_or_nan;
                continue;
            }
            spatial->points.push_back(p);
        }
        spatial->width = static_cast<uint32_t>(spatial->points.size());
        spatial->height = 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr src_for_voxel = spatial ? spatial : in;
    if (!do_voxel) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out = spatial ? spatial : in;
        final_size = out ? out->size() : 0;
        return out;
    }

    if (!src_for_voxel || src_for_voxel->empty()) {
        final_size = 0;
        return spatial;
    }

    const size_t before_v = static_cast<size_t>(src_for_voxel->size());
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(src_for_voxel);
    vg.setLeafSize(roi.voxel_leaf_m, roi.voxel_leaf_m, roi.voxel_leaf_m);
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxed(new pcl::PointCloud<pcl::PointXYZI>());
    vg.filter(*voxed);
    voxed->width = static_cast<uint32_t>(voxed->points.size());
    voxed->height = 1;
    if (before_v > voxed->size()) {
        removed_by_voxel = before_v - static_cast<size_t>(voxed->size());
    }
    final_size = voxed->size();
    return voxed;
}

struct GeoRvPatch {
    std::vector<int> point_indices;
    float score = 0.f;
    uint8_t label = 0;
    bool from_nn = false;
};

inline bool bodyPointToRangeUv(const pcl::PointXYZI& p,
                               int image_w,
                               int image_h,
                               float elev_min_deg,
                               float elev_max_deg,
                               float min_range_m,
                               float max_range_m,
                               int& u,
                               int& v,
                               float& range_m) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return false;
    }
    const double x = static_cast<double>(p.x);
    const double y = static_cast<double>(p.y);
    const double z = static_cast<double>(p.z);
    const double xy = std::hypot(x, y);
    range_m = static_cast<float>(std::sqrt(x * x + y * y + z * z));
    if (range_m < min_range_m || range_m > max_range_m) {
        return false;
    }
    if (xy < 1e-4) {
        return false;
    }
    const double elev_deg = std::atan2(z, xy) * (180.0 / M_PI);
    if (elev_deg < static_cast<double>(elev_min_deg) || elev_deg > static_cast<double>(elev_max_deg)) {
        return false;
    }
    const double az = std::atan2(y, x);
    const double az01 = (az + M_PI) / (2.0 * M_PI);
    u = static_cast<int>(std::floor(az01 * static_cast<double>(image_w)));
    v = static_cast<int>(
        std::floor((elev_deg - static_cast<double>(elev_min_deg)) /
                   (static_cast<double>(elev_max_deg) - static_cast<double>(elev_min_deg)) * static_cast<double>(image_h)));
    if (u < 0 || v < 0 || u >= image_w || v >= image_h) {
        return false;
    }
    return true;
}

static void subsamplePatchIndices(std::vector<int>& idx, size_t max_pts) {
    if (max_pts < 3 || idx.size() <= max_pts) {
        return;
    }
    const size_t stride = (idx.size() + max_pts - 1) / max_pts;
    std::vector<int> out;
    out.reserve(max_pts);
    for (size_t i = 0; i < idx.size(); i += stride) {
        out.push_back(idx[i]);
    }
    idx.swap(out);
}

static void processPrimitiveClusterCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster,
    const GeometricProcessorConfig& config,
    const Eigen::Vector3d& up,
    float rv_score,
    uint8_t rv_label,
    bool rv_from_nn,
    size_t& eval_clusters,
    size_t& rejected_eigen,
    size_t& rejected_line_ransac,
    size_t& rejected_plane_ransac,
    size_t& rejected_plane_verticality,
    size_t& unknown_feature_type,
    size_t& plane_full_cluster_fallback,
    std::vector<GeometricResult::Primitive>& out_primitives) {
    if (!cluster || cluster->size() < static_cast<size_t>(std::max(3, config.euclidean_cluster.min_points))) {
        return;
    }

    const int line_min_inliers = config.wall_ransac.line_min_inliers > 0 ? config.wall_ransac.line_min_inliers
                                                                         : config.wall_ransac.min_inliers;
    const float line_dist_thresh = config.wall_ransac.line_distance_threshold > 0.0f
        ? config.wall_ransac.line_distance_threshold
        : config.wall_ransac.distance_threshold;
    const int max_line_passes = std::max(1, std::min(8, config.max_lines_per_cluster));
    const int min_cluster_pts = std::max(3, config.euclidean_cluster.min_points);
    const int plane_min_inliers = config.wall_ransac.plane_min_inliers > 0 ? config.wall_ransac.plane_min_inliers
                                                                           : config.wall_ransac.min_inliers;

    pcl::PointCloud<pcl::PointXYZI>::Ptr work(new pcl::PointCloud<pcl::PointXYZI>(*cluster));
    int lines_from_this_cluster = 0;

    for (int pass = 0; pass < max_line_passes && work->size() >= static_cast<size_t>(min_cluster_pts); ++pass) {
        ++eval_clusters;
        Eigen::Vector4f centroid;
        Eigen::Matrix3f covariance_matrix;
        pcl::compute3DCentroid(*work, centroid);
        pcl::computeCovarianceMatrix(*work, centroid, covariance_matrix);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix);
        Eigen::Vector3f eigenvalues = solver.eigenvalues();

        const float l1 = eigenvalues(0);
        const float l2 = eigenvalues(1);
        const float l3 = eigenvalues(2);
        if (l3 <= 1e-6f) {
            ++rejected_eigen;
            break;
        }

        float linearity = (l3 - l2) / l3;
        float planarity = (l2 - l1) / l3;

        if (!(linearity > config.primitive_classifier.linearity_threshold)) {
            break;
        }

        pcl::ModelCoefficients::Ptr line_coeffs(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(line_dist_thresh);
        seg.setInputCloud(work);
        seg.segment(*line_inliers, *line_coeffs);

        if (line_inliers->indices.size() < static_cast<size_t>(line_min_inliers)) {
            ++rejected_line_ransac;
            break;
        }

        GeometricResult::Primitive prim;
        prim.type = GeometricResult::Primitive::Type::LINE;
        prim.linearity = linearity;
        prim.planarity = planarity;
        prim.model_coeffs.setZero();
        for (int i = 0; i < 6; ++i) {
            prim.model_coeffs(i) = line_coeffs->values[i];
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_pts(new pcl::PointCloud<pcl::PointXYZI>());
        inlier_pts->reserve(line_inliers->indices.size());
        for (int li : line_inliers->indices) {
            if (li >= 0 && static_cast<size_t>(li) < work->size()) {
                inlier_pts->push_back((*work)[static_cast<size_t>(li)]);
            }
        }
        prim.points = inlier_pts;
        prim.residual = static_cast<double>(line_inliers->indices.size()) / static_cast<double>(work->size());
        prim.range_view_score = rv_score;
        prim.range_view_label = rv_label;
        prim.range_view_from_nn = rv_from_nn;
        out_primitives.push_back(prim);
        ++lines_from_this_cluster;

        std::vector<bool> keep(work->size(), true);
        for (int li : line_inliers->indices) {
            if (li >= 0 && static_cast<size_t>(li) < keep.size()) {
                keep[static_cast<size_t>(li)] = false;
            }
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr rest(new pcl::PointCloud<pcl::PointXYZI>());
        for (size_t i = 0; i < work->size(); ++i) {
            if (keep[i]) {
                rest->push_back((*work)[i]);
            }
        }
        work = rest;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud;
    if (lines_from_this_cluster == 0) {
        plane_cloud = cluster;
    } else if (work && work->size() >= static_cast<size_t>(plane_min_inliers)) {
        plane_cloud = work;
    } else if (lines_from_this_cluster > 0 && cluster->size() >= static_cast<size_t>(plane_min_inliers)) {
        plane_cloud = cluster;
        ++plane_full_cluster_fallback;
    }

    if (plane_cloud && plane_cloud->size() >= static_cast<size_t>(min_cluster_pts)) {
        ++eval_clusters;
        Eigen::Vector4f centroid2;
        Eigen::Matrix3f cov2;
        pcl::compute3DCentroid(*plane_cloud, centroid2);
        pcl::computeCovarianceMatrix(*plane_cloud, centroid2, cov2);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver2(cov2);
        Eigen::Vector3f eigenvalues2 = solver2.eigenvalues();

        const float l1 = eigenvalues2(0);
        const float l2 = eigenvalues2(1);
        const float l3 = eigenvalues2(2);
        if (l3 <= 1e-6f) {
            ++rejected_eigen;
            return;
        }

        float linearity = (l3 - l2) / l3;
        float planarity = (l2 - l1) / l3;

        GeometricResult::Primitive prim;
        prim.points = plane_cloud;
        prim.linearity = linearity;
        prim.planarity = planarity;
        prim.type = GeometricResult::Primitive::Type::UNKNOWN;
        prim.model_coeffs.setZero();
        prim.residual = 0.0;
        prim.range_view_score = rv_score;
        prim.range_view_label = rv_label;
        prim.range_view_from_nn = rv_from_nn;

        if (planarity > config.primitive_classifier.planarity_threshold) {
            prim.type = GeometricResult::Primitive::Type::PLANE;

            pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZI> seg_pl;
            seg_pl.setOptimizeCoefficients(true);
            seg_pl.setModelType(pcl::SACMODEL_PLANE);
            seg_pl.setMethodType(pcl::SAC_RANSAC);
            seg_pl.setDistanceThreshold(config.wall_ransac.plane_distance_threshold > 0.0f
                ? config.wall_ransac.plane_distance_threshold
                : config.wall_ransac.distance_threshold);
            seg_pl.setInputCloud(plane_cloud);
            seg_pl.segment(*plane_inliers, *plane_coeffs);

            if (plane_inliers->indices.size() >= static_cast<size_t>(plane_min_inliers)) {
                for (int i = 0; i < 4; ++i) {
                    prim.model_coeffs(i) = plane_coeffs->values[i];
                }

                Eigen::Vector3d normal(plane_coeffs->values[0], plane_coeffs->values[1], plane_coeffs->values[2]);
                normal.normalize();
                const double angle_to_up =
                    std::acos(std::clamp(std::abs(normal.dot(up)), 0.0, 1.0)) * 180.0 / M_PI;

                if (std::abs(angle_to_up - 90.0) < config.wall_ransac.max_normal_tilt_deg) {
                    prim.residual = static_cast<double>(plane_inliers->indices.size()) /
                        static_cast<double>(std::max<size_t>(1, plane_cloud->size()));
                } else {
                    prim.type = GeometricResult::Primitive::Type::UNKNOWN;
                    ++rejected_plane_verticality;
                }
            } else {
                prim.type = GeometricResult::Primitive::Type::UNKNOWN;
                ++rejected_plane_ransac;
            }
        } else {
            ++unknown_feature_type;
        }

        if (prim.type != GeometricResult::Primitive::Type::UNKNOWN) {
            out_primitives.push_back(prim);
        }
    }
}

static void collectGradientRvPatches(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                     const GeometricProcessorConfig::RangeView& rv,
                                     std::vector<GeoRvPatch>& out_patches) {
    out_patches.clear();
    const int W = std::max(32, rv.image_width);
    const int H = std::max(16, rv.image_height);
    const size_t ncells = static_cast<size_t>(W) * static_cast<size_t>(H);
    std::vector<std::vector<int>> buckets(ncells);
    for (size_t i = 0; i < cloud->size(); ++i) {
        int u = 0;
        int v = 0;
        float range_m = 0.f;
        if (!bodyPointToRangeUv(
                cloud->points[i], W, H, rv.elev_min_deg, rv.elev_max_deg, rv.min_range_m, rv.max_range_m, u, v, range_m)) {
            continue;
        }
        buckets[static_cast<size_t>(u) + static_cast<size_t>(v) * static_cast<size_t>(W)].push_back(static_cast<int>(i));
    }

    cv::Mat rimg(H, W, CV_32F, cv::Scalar(0));
    cv::Mat valid(H, W, CV_8U, cv::Scalar(0));
    for (int row = 0; row < H; ++row) {
        for (int col = 0; col < W; ++col) {
            const auto& cell = buckets[static_cast<size_t>(col) + static_cast<size_t>(row) * static_cast<size_t>(W)];
            if (cell.empty()) {
                continue;
            }
            float best = std::numeric_limits<float>::infinity();
            for (int id : cell) {
                const auto& p = cloud->points[static_cast<size_t>(id)];
                int uu = 0;
                int vv = 0;
                float rm = 0.f;
                if (bodyPointToRangeUv(
                        p, W, H, rv.elev_min_deg, rv.elev_max_deg, rv.min_range_m, rv.max_range_m, uu, vv, rm)) {
                    best = std::min(best, rm);
                }
            }
            if (std::isfinite(best)) {
                rimg.at<float>(row, col) = best;
                valid.at<uint8_t>(row, col) = 255;
            }
        }
    }

    cv::Mat gx;
    cv::Mat gy;
    cv::Sobel(rimg, gx, CV_32F, 1, 0, 3);
    cv::Sobel(rimg, gy, CV_32F, 0, 1, 3);
    cv::Mat mag;
    cv::magnitude(gx, gy, mag);
    double max_mag = 0.0;
    cv::minMaxLoc(mag, nullptr, &max_mag, nullptr, nullptr, valid);
    max_mag = std::max(max_mag, 1e-6);
    const float thresh = static_cast<float>(rv.grad_mag_norm_thresh) * static_cast<float>(max_mag);
    cv::Mat strong;
    cv::compare(mag, thresh, strong, cv::CMP_GT);
    cv::bitwise_and(strong, valid, strong);
    if (rv.dilate_iterations > 0) {
        cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(strong, strong, k, cv::Point(-1, -1), rv.dilate_iterations);
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int ncomp = cv::connectedComponentsWithStats(strong, labels, stats, centroids, 8);
    struct CompInfo {
        int id;
        int area;
        float score_hint;
    };
    std::vector<CompInfo> comps;
    for (int c = 1; c < ncomp; ++c) {
        const int area = stats.at<int>(c, cv::CC_STAT_AREA);
        if (area < rv.min_cc_pixels || area > rv.max_cc_pixels) {
            continue;
        }
        const int bw = stats.at<int>(c, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(c, cv::CC_STAT_HEIGHT);
        const float asp = bw > 0 ? static_cast<float>(bh) / static_cast<float>(bw) : 0.f;
        uint8_t lbl = 0;
        if (bw >= rv.wall_min_width_u && asp <= rv.wall_max_aspect_h_over_w) {
            lbl = 1;
        } else if (bw <= rv.trunk_max_width_u && asp >= rv.trunk_min_aspect_h_over_w) {
            lbl = 2;
        }
        if (lbl == 0) {
            continue;
        }
        double sm = 0.0;
        int cnt = 0;
        for (int row = 0; row < H; ++row) {
            for (int col = 0; col < W; ++col) {
                if (labels.at<int>(row, col) != c) {
                    continue;
                }
                sm += static_cast<double>(mag.at<float>(row, col));
                ++cnt;
            }
        }
        const float score = cnt > 0 ? static_cast<float>(std::min(1.0, (sm / static_cast<double>(cnt)) / max_mag)) : 0.f;
        comps.push_back(CompInfo{c, area, score});
    }
    std::sort(comps.begin(), comps.end(), [](const CompInfo& a, const CompInfo& b) {
        return a.area > b.area;
    });
    const int maxp = std::max(1, rv.max_patches_per_frame);
    for (size_t ci = 0; ci < comps.size() && static_cast<int>(out_patches.size()) < maxp; ++ci) {
        const int c = comps[ci].id;
        const int bx = stats.at<int>(c, cv::CC_STAT_LEFT);
        const int by = stats.at<int>(c, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(c, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(c, cv::CC_STAT_HEIGHT);
        const int x0 = std::max(0, bx - rv.bbox_margin_u);
        const int y0 = std::max(0, by - rv.bbox_margin_v);
        const int x1 = std::min(W - 1, bx + bw - 1 + rv.bbox_margin_u);
        const int y1 = std::min(H - 1, by + bh - 1 + rv.bbox_margin_v);
        std::unordered_set<int> uniq;
        for (int row = y0; row <= y1; ++row) {
            for (int col = x0; col <= x1; ++col) {
                for (int pid :
                    buckets[static_cast<size_t>(col) + static_cast<size_t>(row) * static_cast<size_t>(W)]) {
                    uniq.insert(pid);
                }
            }
        }
        if (uniq.size() < static_cast<size_t>(std::max(3, 8))) {
            continue;
        }
        GeoRvPatch patch;
        patch.point_indices.assign(uniq.begin(), uniq.end());
        patch.score = comps[ci].score_hint;
        const int bw2 = stats.at<int>(c, cv::CC_STAT_WIDTH);
        const int bh2 = stats.at<int>(c, cv::CC_STAT_HEIGHT);
        const float asp2 = bw2 > 0 ? static_cast<float>(bh2) / static_cast<float>(bw2) : 0.f;
        if (bw2 >= rv.wall_min_width_u && asp2 <= rv.wall_max_aspect_h_over_w) {
            patch.label = 1;
        } else {
            patch.label = 2;
        }
        patch.from_nn = false;
        subsamplePatchIndices(patch.point_indices, static_cast<size_t>(std::max(100, rv.max_patch_points)));
        out_patches.push_back(std::move(patch));
    }
}

static std::vector<GeoRvPatch> collectOnnxRvPatches(GeometricProcessor::Impl& impl,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const GeometricProcessorConfig::RangeView& rv,
    bool& ran_ok);

} // namespace

struct GeometricProcessor::Impl {
    std::unique_ptr<PatchWorkpp<pcl::PointXYZI>> patchwork;
    cv::dnn::Net rv_onnx_net;
    bool rv_onnx_attempted = false;
    bool rv_onnx_ok = false;

    Impl() {
        rclcpp::NodeOptions options;
        patchwork = std::make_unique<PatchWorkpp<pcl::PointXYZI>>(options);
    }

    void tryLoadRvOnnx(const std::string& path);
};

void GeometricProcessor::Impl::tryLoadRvOnnx(const std::string& path) {
    if (rv_onnx_attempted) {
        return;
    }
    rv_onnx_attempted = true;
    rv_onnx_ok = false;
    if (path.empty()) {
        return;
    }
    try {
        rv_onnx_net = cv::dnn::readNetFromONNX(path);
        if (rv_onnx_net.empty()) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 8000,
                "[GEOMETRIC][RV][ONNX] readNetFromONNX returned empty: %s", path.c_str());
            return;
        }
        rv_onnx_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        rv_onnx_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        rv_onnx_ok = true;
    } catch (const cv::Exception& e) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 8000,
            "[GEOMETRIC][RV][ONNX] readNetFromONNX failed path=%s err=%s", path.c_str(), e.what());
    }
}

namespace {

std::vector<GeoRvPatch> collectOnnxRvPatches(GeometricProcessor::Impl& impl,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const GeometricProcessorConfig::RangeView& rv,
    bool& ran_ok) {
    ran_ok = false;
    std::vector<GeoRvPatch> out;
    impl.tryLoadRvOnnx(rv.onnx_model_path);
    if (!impl.rv_onnx_ok || impl.rv_onnx_net.empty()) {
        return out;
    }
    const int W = std::max(32, rv.image_width);
    const int H = std::max(16, rv.image_height);
    const size_t ncells = static_cast<size_t>(W) * static_cast<size_t>(H);
    std::vector<std::vector<int>> buckets(ncells);
    for (size_t i = 0; i < cloud->size(); ++i) {
        int u = 0;
        int v = 0;
        float range_m = 0.f;
        if (!bodyPointToRangeUv(
                cloud->points[i], W, H, rv.elev_min_deg, rv.elev_max_deg, rv.min_range_m, rv.max_range_m, u, v, range_m)) {
            continue;
        }
        buckets[static_cast<size_t>(u) + static_cast<size_t>(v) * static_cast<size_t>(W)].push_back(static_cast<int>(i));
    }
    cv::Mat rimg(H, W, CV_32F, cv::Scalar(0));
    cv::Mat znorm(H, W, CV_32F, cv::Scalar(0));
    cv::Mat mask(H, W, CV_8U, cv::Scalar(0));
    for (int row = 0; row < H; ++row) {
        for (int col = 0; col < W; ++col) {
            const auto& cell = buckets[static_cast<size_t>(col) + static_cast<size_t>(row) * static_cast<size_t>(W)];
            if (cell.empty()) {
                continue;
            }
            float best_r = std::numeric_limits<float>::infinity();
            float pick_z = 0.f;
            for (int id : cell) {
                const auto& p = cloud->points[static_cast<size_t>(id)];
                const float px = p.x;
                const float py = p.y;
                const float pz = p.z;
                const float rr = std::sqrt(px * px + py * py + pz * pz);
                if (rr < best_r) {
                    best_r = rr;
                    pick_z = pz;
                }
            }
            if (std::isfinite(best_r)) {
                rimg.at<float>(row, col) = best_r / std::max(rv.max_range_m, 1e-3f);
                const float xy = std::sqrt(std::max(1e-8f, best_r * best_r - pick_z * pick_z));
                znorm.at<float>(row, col) = std::atan2(pick_z, xy) / static_cast<float>(M_PI);
                mask.at<uint8_t>(row, col) = 255;
            }
        }
    }
    cv::Mat mf;
    mask.convertTo(mf, CV_32F, 1.f / 255.f);
    cv::Mat c0;
    cv::Mat c1;
    cv::Mat c2;
    cv::multiply(rimg, mf, c0);
    cv::multiply(znorm, mf, c1);
    cv::multiply(rimg, mf, c2);
    std::vector<cv::Mat> chans = {c0, c1, c2};
    cv::Mat merged;
    cv::merge(chans, merged);
    const int in_w = std::max(32, rv.onnx_input_width);
    const int in_h = std::max(16, rv.onnx_input_height);
    cv::Mat resized;
    cv::resize(merged, resized, cv::Size(in_w, in_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0, cv::Size(in_w, in_h), cv::Scalar(0, 0, 0), false, false);
    try {
        impl.rv_onnx_net.setInput(blob);
        cv::Mat out_blob = impl.rv_onnx_net.forward();
        if (out_blob.dims != 4) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 5000,
                "[GEOMETRIC][RV][ONNX] forward output dims=%d expected NCHW=4", out_blob.dims);
            return out;
        }
        const int dC = out_blob.size[1];
        const int dH = out_blob.size[2];
        const int dW = out_blob.size[3];
        if (dC <= 1 || dH <= 0 || dW <= 0) {
            return out;
        }
        cv::Mat cls_up(H, W, CV_8U, cv::Scalar(0));
        cv::Mat conf_up(H, W, CV_32F, cv::Scalar(0));
        const float* base = reinterpret_cast<const float*>(out_blob.data);
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const int yl = std::min(dH - 1, (y * dH) / H);
                const int xl = std::min(dW - 1, (x * dW) / W);
                int best = 0;
                float bestv = -1e30f;
                for (int c = 0; c < dC; ++c) {
                    const float v = base[static_cast<size_t>(c) * static_cast<size_t>(dH) * static_cast<size_t>(dW) +
                                        static_cast<size_t>(yl) * static_cast<size_t>(dW) + static_cast<size_t>(xl)];
                    if (v > bestv) {
                        bestv = v;
                        best = c;
                    }
                }
                cls_up.at<uint8_t>(y, x) = static_cast<uint8_t>(best);
                conf_up.at<float>(y, x) = std::isfinite(bestv) ? std::min(1.f, 1.f / (1.f + std::exp(-bestv))) : 0.5f;
            }
        }

        const int wall_id = std::clamp(rv.onnx_wall_class_id, 0, dC - 1);
        const int trunk_id = std::clamp(rv.onnx_trunk_class_id, 0, dC - 1);
        cv::Mat wall_mask;
        cv::compare(cls_up, cv::Scalar(wall_id), wall_mask, cv::CMP_EQ);
        cv::bitwise_and(wall_mask, mask, wall_mask);
        cv::Mat trunk_mask;
        cv::compare(cls_up, cv::Scalar(trunk_id), trunk_mask, cv::CMP_EQ);
        cv::bitwise_and(trunk_mask, mask, trunk_mask);

        auto mask_to_patches = [&](const cv::Mat& m, uint8_t lb) {
            cv::Mat lab;
            cv::Mat st;
            cv::Mat cent;
            const int nc = cv::connectedComponentsWithStats(m, lab, st, cent, 8);
            struct Tmp {
                int id;
                int area;
            };
            std::vector<Tmp> cc;
            for (int t = 1; t < nc; ++t) {
                const int ar = st.at<int>(t, cv::CC_STAT_AREA);
                if (ar >= rv.min_cc_pixels && ar <= rv.max_cc_pixels) {
                    cc.push_back(Tmp{t, ar});
                }
            }
            std::sort(cc.begin(), cc.end(), [](const Tmp& a, const Tmp& b) { return a.area > b.area; });
            for (size_t i = 0; i < cc.size() && static_cast<int>(out.size()) < rv.max_patches_per_frame; ++i) {
                const int cid = cc[i].id;
                const int bx = st.at<int>(cid, cv::CC_STAT_LEFT);
                const int by = st.at<int>(cid, cv::CC_STAT_TOP);
                const int bwcc = st.at<int>(cid, cv::CC_STAT_WIDTH);
                const int bhcc = st.at<int>(cid, cv::CC_STAT_HEIGHT);
                const int x0 = std::max(0, bx - rv.bbox_margin_u);
                const int y0 = std::max(0, by - rv.bbox_margin_v);
                const int x1 = std::min(W - 1, bx + bwcc - 1 + rv.bbox_margin_u);
                const int y1 = std::min(H - 1, by + bhcc - 1 + rv.bbox_margin_v);
                std::unordered_set<int> uniq;
                double conf_sum = 0.0;
                int conf_n = 0;
                for (int row = y0; row <= y1; ++row) {
                    for (int col = x0; col <= x1; ++col) {
                        for (int pid :
                            buckets[static_cast<size_t>(col) + static_cast<size_t>(row) * static_cast<size_t>(W)]) {
                            uniq.insert(pid);
                        }
                        if (lab.at<int>(row, col) == cid) {
                            conf_sum += static_cast<double>(conf_up.at<float>(row, col));
                            ++conf_n;
                        }
                    }
                }
                if (uniq.size() < static_cast<size_t>(std::max(3, 8))) {
                    continue;
                }
                GeoRvPatch patch;
                patch.point_indices.assign(uniq.begin(), uniq.end());
                patch.score = conf_n > 0 ? static_cast<float>(conf_sum / static_cast<double>(conf_n)) : 0.6f;
                patch.label = lb;
                patch.from_nn = true;
                subsamplePatchIndices(patch.point_indices, static_cast<size_t>(std::max(100, rv.max_patch_points)));
                out.push_back(std::move(patch));
            }
        };
        mask_to_patches(wall_mask, 3);
        mask_to_patches(trunk_mask, 4);
        ran_ok = !out.empty();
    } catch (const cv::Exception& e) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 5000,
            "[GEOMETRIC][RV][ONNX] cv::Exception: %s", e.what());
    }
    return out;
}

} // namespace

FrameAccumulator::FrameAccumulator(int max_frames, bool tag_intensity_with_scan_seq)
    : max_frames_(max_frames), tag_intensity_with_scan_seq_(tag_intensity_with_scan_seq) {}

void FrameAccumulator::addFrame(double ts,
                                  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                                  const Eigen::Isometry3d& T_odom_b,
                                  uint64_t scan_seq) {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.push_back(Frame{ts, cloud, T_odom_b, scan_seq});
    if (frames_.size() > static_cast<size_t>(max_frames_)) {
        frames_.pop_front();
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr FrameAccumulator::accumulate(const Eigen::Isometry3d& T_curr) {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    if (frames_.empty()) return accumulated;

    Eigen::Isometry3d T_curr_inv = T_curr.inverse();
    for (const auto& frame : frames_) {
        Eigen::Isometry3d T_rel = T_curr_inv * frame.T_odom_b;
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*(frame.cloud), *transformed, T_rel.matrix().cast<float>());
        if (tag_intensity_with_scan_seq_) {
            const float tag = static_cast<float>(static_cast<double>(frame.scan_seq));
            for (auto& p : transformed->points) {
                p.intensity = tag;
            }
        }
        *accumulated += *transformed;
    }
    return accumulated;
}

GeometricProcessor::GeometricProcessor(const GeometricProcessorConfig& config)
    : config_(config), impl_(std::make_unique<Impl>()) {
    
    if (config_.accumulator.enabled) {
        accumulator_ = std::make_unique<FrameAccumulator>(
            config_.accumulator.max_frames, config_.accumulator.tag_intensity_with_scan_seq);
    }

    // 配置 Patchwork++
    auto& pw = impl_->patchwork;
    last_patchwork_sensor_height_m_ = static_cast<double>(config_.patchwork.sensor_height);
    pw->set_parameter(rclcpp::Parameter("sensor_height", static_cast<double>(config_.patchwork.sensor_height)));
    pw->set_parameter(rclcpp::Parameter("num_iter", config_.patchwork.num_iter));
    pw->set_parameter(rclcpp::Parameter("th_dist", static_cast<double>(config_.patchwork.th_dist)));
    pw->set_parameter(rclcpp::Parameter("max_range", static_cast<double>(config_.patchwork.max_range)));
    if (geoInfoEnabled(config_)) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[GEOMETRIC][INIT] log(level=%s detail=%d) patchwork(sensor_height=%.2f auto_height=%d xy=[%.1f,%.1f] pct=%.2f ema_a=%.2f z/r_max=%.2f "
            "odom_gravity=%d up_axis=%d level_pw=%d num_iter=%d th_dist=%.3f max_range=%.1f) "
            "accumulator(enabled=%d max_frames=%d tag_scan_intensity=%d dbg_pcd=%d save_dir=%d every_n=%d accum_body=%d prim_in=%d) primitive_roi(en=%d R=%.1f rmin=%.1f rmax=%.1f vx=%.3f) "
            "range_view(en=%d mode=%s img=%dx%d grad_th=%.3f dilate=%d onnx_in=%dx%d fb_full=%d fusion=%.2f) "
            "primitive_classifier(enabled=%d lin_th=%.3f pla_th=%.3f) "
            "wall_ransac(enabled=%d dist=%.3f min_inliers=%d line(dist=%.3f,min=%d) plane(dist=%.3f,min=%d) tilt_deg=%.1f) "
            "euclidean_cluster(tol=%.3f min_pts=%d max_pts=%d) max_lines_per_cluster=%d",
            config_.log_level.c_str(), config_.log_detail ? 1 : 0,
            config_.patchwork.sensor_height,
            config_.patchwork.auto_sensor_height ? 1 : 0,
            config_.patchwork.auto_height_min_xy_m,
            config_.patchwork.auto_height_max_xy_m,
            config_.patchwork.auto_height_percentile,
            config_.patchwork.auto_height_ema_alpha,
            config_.patchwork.auto_height_max_z_over_r,
            config_.patchwork.use_odom_gravity ? 1 : 0,
            config_.patchwork.odom_up_axis,
            config_.patchwork.level_cloud_for_patchwork ? 1 : 0,
            config_.patchwork.num_iter, config_.patchwork.th_dist, config_.patchwork.max_range,
            config_.accumulator.enabled ? 1 : 0, config_.accumulator.max_frames,
            config_.accumulator.tag_intensity_with_scan_seq ? 1 : 0,
            config_.accumulator.save_debug_pcd ? 1 : 0,
            (!config_.accumulator.save_merged_cloud_dir.empty() && config_.accumulator.save_debug_pcd) ? 1 : 0,
            config_.accumulator.save_merged_cloud_every_n,
            config_.accumulator.save_accum_body_pcd ? 1 : 0,
            config_.accumulator.save_primitive_input_cloud ? 1 : 0,
            config_.primitive_roi.enabled ? 1 : 0,
            config_.primitive_roi.body_xy_radius_m,
            config_.primitive_roi.ring_min_xy_m,
            config_.primitive_roi.ring_max_xy_m,
            config_.primitive_roi.voxel_leaf_m,
            config_.range_view.enabled ? 1 : 0,
            config_.range_view.mode.c_str(),
            config_.range_view.image_width,
            config_.range_view.image_height,
            config_.range_view.grad_mag_norm_thresh,
            config_.range_view.dilate_iterations,
            config_.range_view.onnx_input_width,
            config_.range_view.onnx_input_height,
            config_.range_view.fallback_full_cloud ? 1 : 0,
            config_.range_view.fusion_rv_boost_scale,
            config_.primitive_classifier.enabled ? 1 : 0, config_.primitive_classifier.linearity_threshold, config_.primitive_classifier.planarity_threshold,
            config_.wall_ransac.enabled ? 1 : 0, config_.wall_ransac.distance_threshold, config_.wall_ransac.min_inliers,
            config_.wall_ransac.line_distance_threshold, config_.wall_ransac.line_min_inliers,
            config_.wall_ransac.plane_distance_threshold, config_.wall_ransac.plane_min_inliers,
            config_.wall_ransac.max_normal_tilt_deg,
            config_.euclidean_cluster.tolerance_m, config_.euclidean_cluster.min_points, config_.euclidean_cluster.max_points,
            config_.max_lines_per_cluster);
    }
}

GeometricProcessor::~GeometricProcessor() = default;

void GeometricProcessor::applyPatchworkAutoSensorHeight_(
    uint64_t frame_idx,
    double ts,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
    const auto& pw = config_.patchwork;
    const double cfg_h = static_cast<double>(pw.sensor_height);

    if (!pw.auto_sensor_height) {
        last_patchwork_sensor_height_m_ = cfg_h;
        impl_->patchwork->set_parameter(rclcpp::Parameter("sensor_height", cfg_h));
        return;
    }

    float min_xy = pw.auto_height_min_xy_m;
    float max_xy = pw.auto_height_max_xy_m;
    if (min_xy > max_xy) {
        std::swap(min_xy, max_xy);
    }

    std::vector<float> zs = collectGroundHeightSamples(cloud, min_xy, max_xy, pw.auto_height_max_z_over_r);
    // Multi-ring fallback: sparse first frames / narrow FOV often miss the configured [min_xy,max_xy] annulus.
    // Near-field band (≈1–min(20m,max_xy)) boosts sample count without opening to full range (reduces facade contamination vs 2–90m only).
    auto try_more_samples = [&](float rmin, float rmax) {
        if (static_cast<int>(zs.size()) >= pw.auto_height_min_samples) {
            return;
        }
        if (rmin > rmax) {
            std::swap(rmin, rmax);
        }
        std::vector<float> extra = collectGroundHeightSamples(cloud, rmin, rmax, pw.auto_height_max_z_over_r);
        if (extra.size() > zs.size()) {
            zs = std::move(extra);
        }
    };
    try_more_samples(2.0f, std::min(pw.max_range, 90.f));
    try_more_samples(1.0f, std::min(20.0f, max_xy));

    const char* src = "cfg_fallback";
    double h_applied = cfg_h;

    if (zs.size() < 50) {
        if (geoInfoEnabled(config_)) {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("automap_system"),
                geoThrottleClock(),
                3000,
                "[GEOMETRIC][AUTO_HEIGHT] idx=%lu ts=%.3f reason=insufficient_samples n=%zu min_need~%d -> patchwork sensor_height=%.3f (yaml fallback)",
                static_cast<unsigned long>(frame_idx),
                ts,
                zs.size(),
                pw.auto_height_min_samples,
                cfg_h);
        }
    } else {
        const double pct = std::clamp(static_cast<double>(pw.auto_height_percentile), 0.01, 0.45);
        const size_t k = static_cast<size_t>(std::floor(pct * static_cast<double>(zs.size() - 1)));
        std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(k), zs.end());
        const float z_q = zs[k];
        double h_est = (z_q < 0.0f) ? static_cast<double>(-z_q) : static_cast<double>(std::abs(z_q));

        double lo = static_cast<double>(std::min(pw.auto_height_clamp_min_m, pw.auto_height_clamp_max_m));
        double hi = static_cast<double>(std::max(pw.auto_height_clamp_min_m, pw.auto_height_clamp_max_m));
        if (hi <= lo) {
            hi = lo + 0.1;
        }
        h_est = std::clamp(h_est, lo, hi);

        const double alpha = static_cast<double>(pw.auto_height_ema_alpha);
        if (!patchwork_height_ema_inited_ || alpha <= 0.0 || alpha >= 1.0) {
            patchwork_height_ema_ = h_est;
            patchwork_height_ema_inited_ = true;
            h_applied = h_est;
            src = (alpha <= 0.0 || alpha >= 1.0) ? "percentile_raw" : "percentile_ema_init";
        } else {
            patchwork_height_ema_ = (1.0 - alpha) * patchwork_height_ema_ + alpha * h_est;
            h_applied = patchwork_height_ema_;
            src = "percentile_ema";
        }

        if (geoInfoEnabled(config_)) {
            RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("automap_system"),
                geoThrottleClock(),
                2000,
                "[GEOMETRIC][AUTO_HEIGHT] idx=%lu ts=%.3f source=%s samples=%zu z_pct=%.3f h_est=%.3f ema=%.3f applied=%.3f yaml_fallback=%.3f alpha=%.3f",
                static_cast<unsigned long>(frame_idx),
                ts,
                src,
                zs.size(),
                static_cast<double>(z_q),
                h_est,
                patchwork_height_ema_,
                h_applied,
                cfg_h,
                alpha);
        }
    }

    last_patchwork_sensor_height_m_ = h_applied;
    impl_->patchwork->set_parameter(rclcpp::Parameter("sensor_height", h_applied));
}

GeometricResult GeometricProcessor::process(double ts, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const Eigen::Isometry3d& T_odom_b) {
    GeometricResult result;
    result.ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    result.nonground_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    const uint64_t frame_idx = g_geo_frame_counter.fetch_add(1, std::memory_order_relaxed) + 1;
    const size_t input_pts = cloud ? cloud->size() : 0;
    if (geoDebugEnabled(config_)) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[GEOMETRIC][FRAME][DETAIL] step=start idx=%lu ts=%.3f input_pts=%zu",
            static_cast<unsigned long>(frame_idx), ts, input_pts);
    }
    if (!cloud || cloud->empty()) {
        result.gravity_up_body = config_.patchwork.use_odom_gravity
            ? gravityUpInBody(T_odom_b, config_.patchwork.odom_up_axis)
            : Eigen::Vector3d::UnitZ();
        if (geoInfoEnabled(config_)) {
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 1000,
                "[GEOMETRIC][FRAME] step=skip idx=%lu ts=%.3f reason=empty_cloud",
                static_cast<unsigned long>(frame_idx), ts);
        }
        return result;
    }

    if (geoInfoEnabled(config_)) {
        double x_min = std::numeric_limits<double>::infinity();
        double x_max = -std::numeric_limits<double>::infinity();
        double y_min = std::numeric_limits<double>::infinity();
        double y_max = -std::numeric_limits<double>::infinity();
        double z_min = std::numeric_limits<double>::infinity();
        double z_max = -std::numeric_limits<double>::infinity();
        for (const auto& p : cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }
            x_min = std::min(x_min, static_cast<double>(p.x));
            x_max = std::max(x_max, static_cast<double>(p.x));
            y_min = std::min(y_min, static_cast<double>(p.y));
            y_max = std::max(y_max, static_cast<double>(p.y));
            z_min = std::min(z_min, static_cast<double>(p.z));
            z_max = std::max(z_max, static_cast<double>(p.z));
        }
        const double xy_span = std::max(x_max - x_min, y_max - y_min);
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            2000,
            "[GEOMETRIC][INPUT_EXTENT] idx=%lu ts=%.3f pts=%zu xyz=[x:%.2f..%.2f y:%.2f..%.2f z:%.2f..%.2f] xy_span=%.2f "
            "(expect BODY near ego; very large span may mean world cloud mislabeled as body)",
            static_cast<unsigned long>(frame_idx),
            ts,
            input_pts,
            x_min,
            x_max,
            y_min,
            y_max,
            z_min,
            z_max,
            xy_span);
    }

    if (accumulator_) {
        accumulator_->addFrame(ts, cloud, T_odom_b, frame_idx);
    }

    Eigen::Vector3d gravity_up = Eigen::Vector3d::UnitZ();
    if (config_.patchwork.use_odom_gravity) {
        gravity_up = gravityUpInBody(T_odom_b, config_.patchwork.odom_up_axis);
    }
    result.gravity_up_body = gravity_up;

    Eigen::Matrix3d R_level = Eigen::Matrix3d::Identity();
    bool did_level = false;
    double level_misalign = 0.0;  // 1-|up·e_z|，供 [GEO_PIPELINE] 诊断重力拉平是否触发
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr pw_cloud = cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr leveled_storage;
    if (config_.patchwork.level_cloud_for_patchwork && config_.patchwork.use_odom_gravity) {
        level_misalign = 1.0 - std::abs(gravity_up.dot(Eigen::Vector3d::UnitZ()));
        if (level_misalign > 1e-6) {
            const Eigen::Quaterniond q_align =
                Eigen::Quaterniond::FromTwoVectors(gravity_up, Eigen::Vector3d::UnitZ());
            R_level = q_align.toRotationMatrix();
            leveled_storage.reset(new pcl::PointCloud<pcl::PointXYZI>());
            rotateCloudPoints(*cloud, R_level, *leveled_storage);
            pw_cloud = leveled_storage;
            did_level = true;
        }
    }

    applyPatchworkAutoSensorHeight_(frame_idx, ts, pw_cloud);

    // 1. 地面分割 (Patchwork++)
    double time_taken = 0.0;
    // 注意：Patchwork++ 默认接收的是 body 系点云；level_cloud_for_patchwork 时在「伪水平」body 中分割后再旋回
    impl_->patchwork->estimate_ground(*pw_cloud, *(result.ground_cloud), *(result.nonground_cloud), time_taken);
    if (did_level) {
        const Eigen::Matrix3d R_inv = R_level.transpose();
        applyInverseRotationToCloud(R_inv, result.ground_cloud);
        applyInverseRotationToCloud(R_inv, result.nonground_cloud);
    }
    const size_t pw_ground_sz = result.ground_cloud ? result.ground_cloud->size() : 0;
    const size_t pw_nonground_sz = result.nonground_cloud ? result.nonground_cloud->size() : 0;
    // 统一前缀 grep: GEO_PIPELINE — 前 25 帧 + 每 200 帧一条，便于对齐语义帧号与几何 idx
    if (geoInfoEnabled(config_) &&
        (frame_idx <= 25u || (frame_idx % 200u) == 0u)) {
        const double g_ratio = input_pts > 0 ? static_cast<double>(pw_ground_sz) / static_cast<double>(input_pts) : 0.0;
        RCLCPP_INFO(
            rclcpp::get_logger("automap_system"),
            "[GEO_PIPELINE][PW_RESULT] idx=%lu ts=%.3f in=%zu ground=%zu nonground=%zu g_ratio=%.4f "
            "pw_ms=%.2f pw_sensor_h_m=%.3f did_level=%d misalign=%.5f up_b=[%.3f,%.3f,%.3f] grav_en=%d up_ax=%d level_cfg=%d",
            static_cast<unsigned long>(frame_idx),
            ts,
            input_pts,
            pw_ground_sz,
            pw_nonground_sz,
            g_ratio,
            time_taken,
            last_patchwork_sensor_height_m_,
            did_level ? 1 : 0,
            level_misalign,
            result.gravity_up_body.x(),
            result.gravity_up_body.y(),
            result.gravity_up_body.z(),
            config_.patchwork.use_odom_gravity ? 1 : 0,
            config_.patchwork.odom_up_axis,
            config_.patchwork.level_cloud_for_patchwork ? 1 : 0);
    }
    if (geoInfoEnabled(config_)) {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 1000,
            "[GEOMETRIC][PATCHWORK] idx=%lu ts=%.3f input=%zu ground=%zu nonground=%zu time_ms=%.2f",
            static_cast<unsigned long>(frame_idx), ts, input_pts,
            pw_ground_sz,
            pw_nonground_sz,
            time_taken);
    }
    if (geoInfoEnabled(config_) && input_pts >= 500 && pw_ground_sz == 0) {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            8000,
            "[GEOMETRIC][EVIDENCE] idx=%lu ts=%.3f chain=patchwork ground_pts=0 nonground=%zu input=%zu "
            "pw_sensor_height_m=%.3f auto_height=%d use_odom_gravity=%d level_pw=%d | "
            "likely: wrong body frame / z sign vs Patchwork, or sensor_height far from true",
            static_cast<unsigned long>(frame_idx),
            ts,
            pw_nonground_sz,
            input_pts,
            last_patchwork_sensor_height_m_,
            config_.patchwork.auto_sensor_height ? 1 : 0,
            config_.patchwork.use_odom_gravity ? 1 : 0,
            config_.patchwork.level_cloud_for_patchwork ? 1 : 0);
    }

    // 2. 基元分类 (Zhou22ral)
    size_t primitive_input_pts = pw_nonground_sz;
    if (config_.primitive_classifier.enabled) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr proc_cloud = result.nonground_cloud;

        // 如果开启累加，则对非地面点云进行更稠密的辨识
        if (accumulator_ && config_.accumulator.max_frames > 1) {
            proc_cloud = accumulator_->accumulate(T_odom_b);
            if (proc_cloud && !proc_cloud->empty()) {
                if (config_.accumulator.save_accum_body_pcd) {
                    maybeSaveGeoSemanticDebugPcd(config_, frame_idx, ts, proc_cloud, "accum_body");
                }
                const size_t acc_merge_pts = proc_cloud->size();
                pcl::PointCloud<pcl::PointXYZI>::Ptr acc_input = proc_cloud;
                pcl::PointCloud<pcl::PointXYZI>::Ptr acc_leveled;
                if (did_level) {
                    acc_leveled.reset(new pcl::PointCloud<pcl::PointXYZI>());
                    rotateCloudPoints(*proc_cloud, R_level, *acc_leveled);
                    acc_input = acc_leveled;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr acc_ground(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr acc_nonground(new pcl::PointCloud<pcl::PointXYZI>());
                double acc_time_taken = 0.0;
                impl_->patchwork->estimate_ground(*acc_input, *acc_ground, *acc_nonground, acc_time_taken);
                if (did_level) {
                    const Eigen::Matrix3d R_inv = R_level.transpose();
                    applyInverseRotationToCloud(R_inv, acc_ground);
                    applyInverseRotationToCloud(R_inv, acc_nonground);
                }
                proc_cloud = acc_nonground;
                primitive_input_pts = proc_cloud ? proc_cloud->size() : 0;
                if (geoDebugEnabled(config_)) {
                    RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 1000,
                        "[GEOMETRIC][ACCUMULATOR][DETAIL] idx=%lu ts=%.3f accumulated_pts=%zu acc_ground=%zu acc_nonground=%zu acc_patchwork_ms=%.2f",
                        static_cast<unsigned long>(frame_idx), ts,
                        acc_merge_pts,
                        acc_ground ? acc_ground->size() : 0,
                        acc_nonground ? acc_nonground->size() : 0,
                        acc_time_taken);
                }
            } else {
                primitive_input_pts = 0;
            }
        }

        size_t prim_roi_dropped = 0;
        size_t prim_roi_voxel_rm = 0;
        if (config_.primitive_roi.enabled && proc_cloud && !proc_cloud->empty()) {
            const size_t before_roi = proc_cloud->size();
            size_t roi_final = before_roi;
            proc_cloud =
                applyPrimitiveRoiPipeline(proc_cloud, config_.primitive_roi, prim_roi_dropped, prim_roi_voxel_rm, roi_final);
            primitive_input_pts = proc_cloud ? proc_cloud->size() : 0;
            if (geoInfoEnabled(config_) && (frame_idx <= 25u || (frame_idx % 200u) == 0u)) {
                double log_eff_outer = static_cast<double>(config_.primitive_roi.body_xy_radius_m);
                const double log_ring_max = static_cast<double>(config_.primitive_roi.ring_max_xy_m);
                if (log_ring_max > 1e-9) {
                    if (log_eff_outer > 1e-9) {
                        log_eff_outer = std::min(log_eff_outer, log_ring_max);
                    } else {
                        log_eff_outer = log_ring_max;
                    }
                }
                RCLCPP_INFO(
                    rclcpp::get_logger("automap_system"),
                    "[GEO_PIPELINE][PRIM_ROI] idx=%lu ts=%.3f before=%zu after=%zu dropped_xy_nan=%zu voxel_removed=%zu "
                    "cfg(body_R=%.1f ring=[%.1f,%.1f] eff_outer=%.1f voxel=%.3f)",
                    static_cast<unsigned long>(frame_idx),
                    ts,
                    before_roi,
                    primitive_input_pts,
                    prim_roi_dropped,
                    prim_roi_voxel_rm,
                    config_.primitive_roi.body_xy_radius_m,
                    config_.primitive_roi.ring_min_xy_m,
                    config_.primitive_roi.ring_max_xy_m,
                    log_eff_outer,
                    config_.primitive_roi.voxel_leaf_m);
            }
            if (geoDebugEnabled(config_)) {
                RCLCPP_DEBUG_THROTTLE(
                    rclcpp::get_logger("automap_system"),
                    geoThrottleClock(),
                    2000,
                    "[GEOMETRIC][PRIM_ROI][DETAIL] idx=%lu before=%zu after=%zu drop=%zu voxel_rm=%zu",
                    static_cast<unsigned long>(frame_idx),
                    before_roi,
                    primitive_input_pts,
                    prim_roi_dropped,
                    prim_roi_voxel_rm);
            }
            if (geoInfoEnabled(config_) && before_roi >= 200 && primitive_input_pts == 0) {
                RCLCPP_WARN_THROTTLE(
                    rclcpp::get_logger("automap_system"),
                    geoThrottleClock(),
                    10000,
                    "[GEOMETRIC][EVIDENCE] idx=%lu ts=%.3f chain=prim_roi nonground_in=%zu after_roi=0 | "
                    "check body_xy_radius_m / ring_min_xy_m vs cloud extent",
                    static_cast<unsigned long>(frame_idx),
                    ts,
                    before_roi);
            }
        }

        if (config_.accumulator.save_primitive_input_cloud) {
            maybeSaveGeoSemanticDebugPcd(config_, frame_idx, ts, proc_cloud, "prim_input");
        }
        classifyPrimitives(proc_cloud, result.primitives, result.gravity_up_body, frame_idx);
    } else if (geoInfoEnabled(config_)) {
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            5000,
            "[GEOMETRIC][SKIP] idx=%lu ts=%.3f reason=primitive_classifier_disabled (no line/plane/wall extraction)",
            static_cast<unsigned long>(frame_idx),
            ts);
    }

    size_t line_cnt = 0;
    size_t plane_cnt = 0;
    for (const auto& p : result.primitives) {
        if (p.type == GeometricResult::Primitive::Type::LINE) ++line_cnt;
        else if (p.type == GeometricResult::Primitive::Type::PLANE) ++plane_cnt;
    }
    if (geoInfoEnabled(config_)) {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("automap_system"), geoThrottleClock(), 1000,
            "[GEOMETRIC][FRAME] step=done idx=%lu ts=%.3f input=%zu nonground=%zu primitives=%zu lines=%zu planes=%zu",
            static_cast<unsigned long>(frame_idx), ts, input_pts,
            result.nonground_cloud ? result.nonground_cloud->size() : 0,
            result.primitives.size(), line_cnt, plane_cnt);
        const double g_ratio = input_pts > 0 ? static_cast<double>(pw_ground_sz) / static_cast<double>(input_pts) : 0.0;
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            2000,
            "[GEOMETRIC][CHAIN] idx=%lu ts=%.3f in=%zu g=%zu ng=%zu g_ratio=%.4f pw_ms=%.2f pw_sensor_h_m=%.3f "
            "grav=%d up_ax=%d level_pw=%d did_level=%d up_b=[%.3f,%.3f,%.3f] prim_en=%d prim_in=%zu L=%zu P=%zu",
            static_cast<unsigned long>(frame_idx),
            ts,
            input_pts,
            pw_ground_sz,
            pw_nonground_sz,
            g_ratio,
            time_taken,
            last_patchwork_sensor_height_m_,
            config_.patchwork.use_odom_gravity ? 1 : 0,
            config_.patchwork.odom_up_axis,
            config_.patchwork.level_cloud_for_patchwork ? 1 : 0,
            did_level ? 1 : 0,
            result.gravity_up_body.x(),
            result.gravity_up_body.y(),
            result.gravity_up_body.z(),
            config_.primitive_classifier.enabled ? 1 : 0,
            primitive_input_pts,
            line_cnt,
            plane_cnt);
    }
    if (geoInfoEnabled(config_) && config_.primitive_classifier.enabled && primitive_input_pts >= 800 && plane_cnt == 0 &&
        line_cnt == 0) {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            8000,
            "[GEOMETRIC][EVIDENCE] idx=%lu ts=%.3f chain=primitives prim_in=%zu lines=0 planes=0 | "
            "see [GEOMETRIC][PRIMITIVE] reject counts (cluster/eigen/RANSAC/verticality)",
            static_cast<unsigned long>(frame_idx),
            ts,
            primitive_input_pts);
    }

    return result;
}

void GeometricProcessor::classifyPrimitives(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                            std::vector<GeometricResult::Primitive>& out_primitives,
                                            const Eigen::Vector3d& gravity_up_body,
                                            uint64_t frame_idx) {
    if (!cloud || cloud->empty()) {
        if (geoInfoEnabled(config_)) {
            RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("automap_system"),
                geoThrottleClock(),
                3000,
                "[GEOMETRIC][SKIP] classifyPrimitives reason=empty_nonground_cloud (no walls/lines)");
        }
        return;
    }

    Eigen::Vector3d up = gravity_up_body;
    if (up.norm() < 1e-9) {
        up = Eigen::Vector3d::UnitZ();
    } else {
        up.normalize();
    }

    size_t eval_clusters = 0;
    size_t rejected_eigen = 0;
    size_t rejected_line_ransac = 0;
    size_t rejected_plane_ransac = 0;
    size_t rejected_plane_verticality = 0;
    size_t unknown_feature_type = 0;
    size_t plane_full_cluster_fallback = 0;

    std::vector<GeoRvPatch> rv_patches;
    const std::string rv_mode_lc = toLowerCopy(config_.range_view.mode);
    bool used_rv_pipeline = false;
    if (config_.range_view.enabled && rv_mode_lc != "none" && rv_mode_lc != "off") {
        if (rv_mode_lc == "onnx" || rv_mode_lc == "hybrid") {
            bool onnx_ran = false;
            std::vector<GeoRvPatch> onnx_p = collectOnnxRvPatches(*impl_, cloud, config_.range_view, onnx_ran);
            if (!onnx_p.empty()) {
                rv_patches = std::move(onnx_p);
                used_rv_pipeline = true;
            }
        }
        if ((rv_mode_lc == "gradient" || rv_mode_lc == "hybrid" || rv_mode_lc == "onnx") && rv_patches.empty()) {
            collectGradientRvPatches(cloud, config_.range_view, rv_patches);
            used_rv_pipeline = !rv_patches.empty();
        }
        if (geoInfoEnabled(config_) && (frame_idx <= 25u || (frame_idx % 200u) == 0u)) {
            size_t nn_c = 0;
            for (const auto& p : rv_patches) {
                if (p.from_nn) {
                    ++nn_c;
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[GEO_PIPELINE][RV] idx=%lu cloud_pts=%zu patches=%zu onnx_like=%zu mode=%s WxH=%dx%d",
                static_cast<unsigned long>(frame_idx),
                cloud->size(),
                rv_patches.size(),
                static_cast<unsigned long>(nn_c),
                config_.range_view.mode.c_str(),
                config_.range_view.image_width,
                config_.range_view.image_height);
        }
    }

    auto runFullEuclideanPipeline = [&]() {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(config_.euclidean_cluster.tolerance_m);
        ec.setMinClusterSize(config_.euclidean_cluster.min_points);
        ec.setMaxClusterSize(config_.euclidean_cluster.max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        if (geoInfoEnabled(config_) && !cloud->empty() && cluster_indices.empty()) {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("automap_system"),
                geoThrottleClock(),
                6000,
                "[GEO_PIPELINE][PRIM] chain=cluster cloud_pts=%zu clusters=0 | euclidean_cluster(tol=%.3f min=%d max=%d) "
                "-> no line/plane candidates (walls/trees need clusters)",
                cloud->size(),
                config_.euclidean_cluster.tolerance_m,
                config_.euclidean_cluster.min_points,
                config_.euclidean_cluster.max_points);
        }
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& idx : indices.indices) {
                cluster->push_back((*cloud)[idx]);
            }
            processPrimitiveClusterCloud(cluster,
                config_,
                up,
                0.f,
                0,
                false,
                eval_clusters,
                rejected_eigen,
                rejected_line_ransac,
                rejected_plane_ransac,
                rejected_plane_verticality,
                unknown_feature_type,
                plane_full_cluster_fallback,
                out_primitives);
        }
        return cluster_indices.size();
    };

    size_t top_level_clusters = 0;
    if (used_rv_pipeline && !rv_patches.empty()) {
        for (const auto& patch : rv_patches) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr patch_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            patch_cloud->header = cloud->header;
            for (int pid : patch.point_indices) {
                if (pid >= 0 && static_cast<size_t>(pid) < cloud->size()) {
                    patch_cloud->push_back((*cloud)[static_cast<size_t>(pid)]);
                }
            }
            patch_cloud->width = static_cast<uint32_t>(patch_cloud->size());
            patch_cloud->height = 1;
            if (patch_cloud->size() < static_cast<size_t>(std::max(3, config_.euclidean_cluster.min_points))) {
                continue;
            }
            pcl::search::KdTree<pcl::PointXYZI>::Ptr ptree(new pcl::search::KdTree<pcl::PointXYZI>);
            ptree->setInputCloud(patch_cloud);
            std::vector<pcl::PointIndices> local_ix;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> pec;
            pec.setClusterTolerance(config_.euclidean_cluster.tolerance_m);
            pec.setMinClusterSize(std::max(3, config_.euclidean_cluster.min_points));
            pec.setMaxClusterSize(config_.euclidean_cluster.max_points);
            pec.setSearchMethod(ptree);
            pec.setInputCloud(patch_cloud);
            pec.extract(local_ix);
            ++top_level_clusters;
            if (local_ix.empty()) {
                processPrimitiveClusterCloud(patch_cloud,
                    config_,
                    up,
                    patch.score,
                    patch.label,
                    patch.from_nn,
                    eval_clusters,
                    rejected_eigen,
                    rejected_line_ransac,
                    rejected_plane_ransac,
                    rejected_plane_verticality,
                    unknown_feature_type,
                    plane_full_cluster_fallback,
                    out_primitives);
            } else {
                for (const auto& li : local_ix) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr sub(new pcl::PointCloud<pcl::PointXYZI>());
                    sub->header = cloud->header;
                    for (int idx : li.indices) {
                        if (idx >= 0 && static_cast<size_t>(idx) < patch_cloud->size()) {
                            sub->push_back((*patch_cloud)[static_cast<size_t>(idx)]);
                        }
                    }
                    sub->width = static_cast<uint32_t>(sub->size());
                    sub->height = 1;
                    processPrimitiveClusterCloud(sub,
                        config_,
                        up,
                        patch.score,
                        patch.label,
                        patch.from_nn,
                        eval_clusters,
                        rejected_eigen,
                        rejected_line_ransac,
                        rejected_plane_ransac,
                        rejected_plane_verticality,
                        unknown_feature_type,
                        plane_full_cluster_fallback,
                        out_primitives);
                }
            }
        }
    }

    if (!used_rv_pipeline || rv_patches.empty()) {
        top_level_clusters = runFullEuclideanPipeline();
    } else if (config_.range_view.fallback_full_cloud && out_primitives.empty() && cloud->size() >= 200) {
        runFullEuclideanPipeline();
    }

    size_t line_cnt = 0;
    size_t plane_cnt = 0;
    for (const auto& p : out_primitives) {
        if (p.type == GeometricResult::Primitive::Type::LINE) {
            ++line_cnt;
        } else if (p.type == GeometricResult::Primitive::Type::PLANE) {
            ++plane_cnt;
        }
    }
    if (geoInfoEnabled(config_)) {
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("automap_system"),
            geoThrottleClock(),
            1000,
            "[GEOMETRIC][PRIMITIVE] cloud_pts=%zu rv_top=%zu eval=%zu out(lines/planes/total)=(%zu/%zu/%zu) "
            "reject(eigen/line_ransac/plane_ransac/plane_verticality/unknown_feature)=(%zu/%zu/%zu/%zu/%zu) "
            "plane_full_cluster_fallback=%zu",
            cloud->size(),
            top_level_clusters,
            eval_clusters,
            line_cnt,
            plane_cnt,
            out_primitives.size(),
            rejected_eigen,
            rejected_line_ransac,
            rejected_plane_ransac,
            rejected_plane_verticality,
            unknown_feature_type,
            plane_full_cluster_fallback);
    }
}

} // namespace automap_pro::v3
