/**
 * @file v3/sloam_semantic_segmentor.cpp
 * @brief V3 流水线模块实现。
 */
#include "automap_pro/v3/semantic_segmentor.h"

#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
#include <segmentation/inference.h>
#endif

#include <stdexcept>
#include <chrono>

namespace automap_pro::v3 {

namespace {

/** SLOAM Segmentation API takes non-const Ptr; we only have ConstPtr. Alias shares ownership; SLOAM run/maskCloud only read points. */
inline CloudXYZIPtr sloamInputCloudPtr(const CloudXYZIConstPtr& cloud) {
    if (!cloud) {
        return nullptr;
    }
    return CloudXYZIPtr(cloud, const_cast<CloudXYZI*>(cloud.get()));
}

}  // namespace

class SloamSemanticSegmentor final : public ISemanticSegmentor {
public:
    explicit SloamSemanticSegmentor(const SegmentorConfig& cfg) {
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
        segmentator_ = std::make_shared<seg::Segmentation>(
            cfg.model_path, cfg.fov_up, cfg.fov_down, cfg.img_w, cfg.img_h, cfg.input_channels,
            cfg.num_classes, cfg.tree_class_id, cfg.input_mean, cfg.input_std, cfg.do_destagger);
#else
        (void)cfg;
        throw std::runtime_error("SLOAM semantic backend requires AUTOMAP_USE_SLOAM_SEMANTIC=1");
#endif
    }

    const char* name() const override { return "sloam"; }
    bool isReady() const override {
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
        return segmentator_ != nullptr;
#else
        return false;
#endif
    }

    void run(const CloudXYZIConstPtr& cloud, cv::Mat& mask, SemanticSegResult* result) override {
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
        const auto t0 = std::chrono::steady_clock::now();
        segmentator_->run(sloamInputCloudPtr(cloud), mask);
        if (result != nullptr) {
            result->per_point_labels.clear();
            const auto t1 = std::chrono::steady_clock::now();
            result->success = true;
            result->backend_name = name();
            result->inference_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
#else
        (void)cloud;
        (void)mask;
        (void)result;
#endif
    }

    void maskCloud(const CloudXYZIConstPtr& cloud, const cv::Mat& mask, CloudXYZIPtr& out_cloud,
                   int tree_label, bool dense_for_clustering) override {
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
        segmentator_->maskCloud(sloamInputCloudPtr(cloud), mask, out_cloud,
                                static_cast<unsigned char>(tree_label), dense_for_clustering);
#else
        (void)cloud;
        (void)mask;
        (void)out_cloud;
        (void)tree_label;
        (void)dense_for_clustering;
#endif
    }

private:
#ifdef AUTOMAP_USE_SLOAM_SEMANTIC
    std::shared_ptr<seg::Segmentation> segmentator_;
#endif
};

std::unique_ptr<ISemanticSegmentor> CreateSloamSemanticSegmentor(const SegmentorConfig& cfg) {
    return std::make_unique<SloamSemanticSegmentor>(cfg);
}

}  // namespace automap_pro::v3
