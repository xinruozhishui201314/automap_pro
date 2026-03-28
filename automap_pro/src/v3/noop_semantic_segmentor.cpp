#include "automap_pro/v3/semantic_segmentor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace automap_pro::v3 {

namespace {

struct ProjectionIndex {
    int x = 0;
    int y = 0;
    bool valid = false;
};

ProjectionIndex projectPoint(const pcl::PointXYZI& p, int img_w, int img_h, double fov_up_rad, double fov_down_rad) {
    ProjectionIndex idx;
    const double range = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z);
    if (!(range > 1e-6)) {
        return idx;
    }
    const double yaw = -std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    const double fov = std::abs(fov_up_rad) + std::abs(fov_down_rad);

    int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(img_w)));
    int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov)) *
                                             static_cast<double>(img_h)));
    proj_x = std::clamp(proj_x, 0, img_w - 1);
    proj_y = std::clamp(proj_y, 0, img_h - 1);
    idx.x = proj_x;
    idx.y = proj_y;
    idx.valid = true;
    return idx;
}

}  // namespace

/** Placeholder segmentor when semantic.mode=geometric_only: no NN / Python worker. */
class NoopGeometricSemanticSegmentor final : public ISemanticSegmentor {
public:
    explicit NoopGeometricSemanticSegmentor(const SegmentorConfig& cfg)
        : cfg_(cfg),
          fov_up_rad_(static_cast<double>(cfg.fov_up) * M_PI / 180.0),
          fov_down_rad_(static_cast<double>(cfg.fov_down) * M_PI / 180.0) {}

    const char* name() const override { return "noop_geometric"; }

    bool isReady() const override { return true; }

    void run(const CloudXYZIConstPtr& /*cloud*/, cv::Mat& mask, SemanticSegResult* result) override {
        if (mask.rows != cfg_.img_h || mask.cols != cfg_.img_w) {
            mask = cv::Mat::zeros(cfg_.img_h, cfg_.img_w, CV_8U);
        } else {
            mask.setTo(0);
        }
        if (result) {
            result->success = true;
            result->backend_name = name();
            result->message = "geometric_only: inference skipped";
            result->inference_ms = 0.0;
            result->per_point_labels.clear();
        }
    }

    void maskCloud(const CloudXYZIConstPtr& cloud, const cv::Mat& mask, CloudXYZIPtr& out_cloud, int tree_label,
                   bool dense_for_clustering) override {
        out_cloud.reset(new CloudXYZI());
        if (!cloud || cloud->empty()) {
            return;
        }

        const int label = tree_label >= 0 ? tree_label : 255;
        if (dense_for_clustering) {
            out_cloud->width = static_cast<uint32_t>(cfg_.img_w);
            out_cloud->height = static_cast<uint32_t>(cfg_.img_h);
            out_cloud->is_dense = false;
            out_cloud->points.assign(static_cast<size_t>(cfg_.img_w * cfg_.img_h),
                                     pcl::PointXYZI(std::numeric_limits<float>::quiet_NaN(),
                                                    std::numeric_limits<float>::quiet_NaN(),
                                                    std::numeric_limits<float>::quiet_NaN(),
                                                    std::numeric_limits<float>::quiet_NaN()));
        } else {
            out_cloud->height = 1;
            out_cloud->is_dense = false;
            out_cloud->points.reserve(cloud->size());
        }

        for (const auto& p : cloud->points) {
            const auto proj = projectPoint(p, cfg_.img_w, cfg_.img_h, fov_up_rad_, fov_down_rad_);
            if (!proj.valid) {
                continue;
            }
            if (mask.at<uint8_t>(proj.y, proj.x) != static_cast<uint8_t>(label)) {
                continue;
            }
            if (dense_for_clustering) {
                out_cloud->points[static_cast<size_t>(proj.y * cfg_.img_w + proj.x)] = p;
            } else {
                out_cloud->points.push_back(p);
            }
        }

        if (!dense_for_clustering) {
            out_cloud->width = static_cast<uint32_t>(out_cloud->points.size());
        }
    }

private:
    SegmentorConfig cfg_;
    double fov_up_rad_ = 0.0;
    double fov_down_rad_ = 0.0;
};

std::unique_ptr<ISemanticSegmentor> CreateNoopGeometricSemanticSegmentor(const SegmentorConfig& cfg) {
    return std::make_unique<NoopGeometricSemanticSegmentor>(cfg);
}

}  // namespace automap_pro::v3
