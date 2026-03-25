#include "automap_pro/v3/semantic_segmentor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#ifdef USE_TORCH
#include <torch/script.h>
#include <torch/torch.h>
#endif

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
    if (!(range > 1e-6)) return idx;
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

class Lsk3dnetSemanticSegmentor final : public ISemanticSegmentor {
public:
    explicit Lsk3dnetSemanticSegmentor(const SegmentorConfig& cfg)
        : cfg_(cfg),
          fov_up_rad_(static_cast<double>(cfg.fov_up) * M_PI / 180.0),
          fov_down_rad_(static_cast<double>(cfg.fov_down) * M_PI / 180.0) {
#ifndef USE_TORCH
        (void)cfg_;
        throw std::runtime_error("LSK3DNet backend requires USE_TORCH=1");
#else
        if (cfg_.lsk3dnet_model_path.empty()) {
            throw std::runtime_error("semantic.lsk3dnet.model_path is empty");
        }
        module_ = torch::jit::load(cfg_.lsk3dnet_model_path);
        module_.eval();

        if (cfg_.lsk3dnet_device.rfind("cuda", 0) == 0 && torch::cuda::is_available()) {
            device_ = torch::Device(cfg_.lsk3dnet_device);
        } else {
            device_ = torch::kCPU;
        }
        module_.to(device_);
#endif
    }

    const char* name() const override { return "lsk3dnet"; }
    bool isReady() const override {
#ifdef USE_TORCH
        return true;
#else
        return false;
#endif
    }

    void run(const CloudXYZIConstPtr& cloud, cv::Mat& mask, SemanticSegResult* result) override {
#ifndef USE_TORCH
        (void)cloud;
        (void)mask;
        if (result != nullptr) {
            result->success = false;
            result->backend_name = name();
            result->message = "USE_TORCH disabled";
        }
        return;
#else
        if (!cloud || cloud->empty()) {
            mask = cv::Mat::zeros(cfg_.img_h, cfg_.img_w, CV_8U);
            if (result != nullptr) {
                result->success = true;
                result->backend_name = name();
                result->message = "empty cloud";
                result->inference_ms = 0.0;
            }
            return;
        }
        const auto t0 = std::chrono::steady_clock::now();
        const int channels = std::max(4, cfg_.input_channels > 0 ? cfg_.input_channels : 4);
        std::vector<float> input(static_cast<size_t>(channels * cfg_.img_h * cfg_.img_w), 0.0f);
        std::vector<int> hit_count(static_cast<size_t>(cfg_.img_h * cfg_.img_w), 0);

        for (const auto& p : cloud->points) {
            const auto proj = projectPoint(p, cfg_.img_w, cfg_.img_h, fov_up_rad_, fov_down_rad_);
            if (!proj.valid) continue;
            const int pix = proj.y * cfg_.img_w + proj.x;
            const int cnt = ++hit_count[static_cast<size_t>(pix)];
            auto set_channel = [&](int c, float v) {
                if (c >= channels) return;
                const size_t idx = static_cast<size_t>(c) * cfg_.img_h * cfg_.img_w + static_cast<size_t>(pix);
                input[idx] += (v - meanAt(c)) / stdAt(c);
            };
            set_channel(0, p.x);
            set_channel(1, p.y);
            set_channel(2, p.z);
            set_channel(3, p.intensity);
            if (channels > 4) {
                const float range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                set_channel(4, range);
            }
            (void)cnt;
        }

        for (int y = 0; y < cfg_.img_h; ++y) {
            for (int x = 0; x < cfg_.img_w; ++x) {
                const int pix = y * cfg_.img_w + x;
                const int cnt = hit_count[static_cast<size_t>(pix)];
                if (cnt <= 1) continue;
                for (int c = 0; c < channels; ++c) {
                    const size_t idx = static_cast<size_t>(c) * cfg_.img_h * cfg_.img_w + static_cast<size_t>(pix);
                    input[idx] /= static_cast<float>(cnt);
                }
            }
        }

        torch::NoGradGuard no_grad;
        auto tensor = torch::from_blob(input.data(), {1, channels, cfg_.img_h, cfg_.img_w}, torch::kFloat32).clone();
        tensor = tensor.to(device_);
        auto out_iv = module_.forward({tensor});
        torch::Tensor out = out_iv.toTensor();
        out = out.to(torch::kCPU);

        torch::Tensor class_map;
        if (out.dim() == 4) {
            class_map = out.argmax(1, false).squeeze(0).to(torch::kU8);
        } else if (out.dim() == 3) {
            class_map = out.squeeze(0).to(torch::kU8);
        } else if (out.dim() == 2) {
            class_map = out.to(torch::kU8).reshape({cfg_.img_h, cfg_.img_w});
        } else {
            throw std::runtime_error("Unsupported LSK3DNet output dim: " + std::to_string(out.dim()));
        }
        if (class_map.size(0) != cfg_.img_h || class_map.size(1) != cfg_.img_w) {
            throw std::runtime_error("LSK3DNet output shape mismatch");
        }

        mask = cv::Mat(cfg_.img_h, cfg_.img_w, CV_8U);
        std::memcpy(mask.data, class_map.contiguous().data_ptr<uint8_t>(), static_cast<size_t>(cfg_.img_h * cfg_.img_w));

        if (result != nullptr) {
            const auto t1 = std::chrono::steady_clock::now();
            result->success = true;
            result->backend_name = name();
            result->inference_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
#endif
    }

    void maskCloud(const CloudXYZIConstPtr& cloud, const cv::Mat& mask, CloudXYZIPtr& out_cloud,
                   int tree_label, bool dense_for_clustering) override {
        out_cloud.reset(new CloudXYZI());
        if (!cloud || cloud->empty()) return;

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
            if (!proj.valid) continue;
            if (mask.at<uint8_t>(proj.y, proj.x) != static_cast<uint8_t>(label)) continue;
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
    float meanAt(int c) const {
        if (c >= 0 && c < static_cast<int>(cfg_.input_mean.size())) return cfg_.input_mean[static_cast<size_t>(c)];
        return 0.0f;
    }
    float stdAt(int c) const {
        if (c >= 0 && c < static_cast<int>(cfg_.input_std.size())) {
            return std::max(1e-6f, cfg_.input_std[static_cast<size_t>(c)]);
        }
        return 1.0f;
    }

    SegmentorConfig cfg_;
    double fov_up_rad_ = 0.0;
    double fov_down_rad_ = 0.0;
#ifdef USE_TORCH
    torch::jit::script::Module module_;
    torch::Device device_{torch::kCPU};
#endif
};

std::unique_ptr<ISemanticSegmentor> CreateLsk3dnetSemanticSegmentor(const SegmentorConfig& cfg) {
    return std::make_unique<Lsk3dnetSemanticSegmentor>(cfg);
}

}  // namespace automap_pro::v3
