#pragma once

#include "automap_pro/core/data_types.h"
#include <memory>
#include <vector>
#include <string>
#include <mutex>

// Forward declarations from sloam_rec
namespace seg {
    class Segmentation;
}
class Instance;
struct FeatureModelParams;

namespace automap_pro::v3 {

/**
 * @brief SemanticProcessor: Wraps semantic segmentation and instance clustering
 * to extract tree landmarks from point clouds.
 */
class SemanticProcessor {
public:
    using Ptr = std::shared_ptr<SemanticProcessor>;

    struct Config {
        std::string model_path;
        float fov_up = 22.5f;
        float fov_down = -22.5f;
        int img_w = 2048;
        int img_h = 64;
        bool do_destagger = true;
        
        // Clustering params
        float beam_cluster_threshold = 0.1f;
        float max_dist_to_centroid = 0.2f;
        int min_vertex_size = 2;
        int min_landmark_size = 4;
        float min_landmark_height = 1.0f;

        // Fitting params
        float default_tree_radius = 0.1f;
        float max_tree_radius = 0.5f;
        float max_axis_theta = 15.0f; // degrees from vertical
    };

    explicit SemanticProcessor(const Config& config);
    ~SemanticProcessor();

    /**
     * @brief Extracts cylindrical landmarks from a point cloud
     * @param cloud The input point cloud
     * @return A vector of extracted CylinderLandmarks
     */
    std::vector<CylinderLandmark::Ptr> process(const CloudXYZIConstPtr& cloud);

private:
    Config config_;
    std::shared_ptr<seg::Segmentation> segmentator_;
    std::unique_ptr<Instance> instance_detector_;
    std::unique_ptr<FeatureModelParams> fm_params_;
    mutable std::recursive_mutex resource_mutex_;
    CylinderLandmark::Ptr fitCylinder(const std::vector<pcl::PointXYZI>& points);
};

} // namespace automap_pro::v3
