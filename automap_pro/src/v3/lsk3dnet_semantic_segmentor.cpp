#include "automap_pro/v3/semantic_segmentor.h"
#include "automap_pro/core/ort_wrapper.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef USE_TORCH
#include <torch/script.h>
#include <torch/torch.h>
#include <torch/version.h>
#endif

#include <rclcpp/rclcpp.hpp>

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
        if (cfg_.lsk3dnet_model_path.empty()) {
            throw std::runtime_error("semantic.lsk3dnet.model_path is empty");
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][LSK3DNET][LOAD] step=begin device_req=%s model=%s img=%dx%d fov=[%.1f,%.1f]",
            cfg_.lsk3dnet_device.c_str(), cfg_.lsk3dnet_model_path.c_str(),
            cfg_.img_w, cfg_.img_h, cfg_.fov_up, cfg_.fov_down);

        std::filesystem::path model_path(cfg_.lsk3dnet_model_path);
        if (model_path.extension() == ".onnx") {
            onnx_model_ = std::make_shared<OnnxSession>();
            bool use_cuda = (cfg_.lsk3dnet_device.rfind("cuda", 0) == 0);
            if (onnx_model_->loadModel(cfg_.lsk3dnet_model_path, use_cuda)) {
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][LSK3DNET][LOAD] step=ok_onnx model=%s device=%s",
                    cfg_.lsk3dnet_model_path.c_str(), use_cuda ? "CUDA/TensorRT" : "CPU");
                return;
            } else {
                onnx_model_ = nullptr;
                throw std::runtime_error("Failed to load ONNX model: " + cfg_.lsk3dnet_model_path);
            }
        }

#ifndef USE_TORCH
        throw std::runtime_error("TorchScript backend requires USE_TORCH=1 (or provide .onnx model)");
#else
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][LSK3DNET][LOAD] torch_version=%s", TORCH_VERSION);
        // 🛑 [FIX] 针对 RTX 5080 (Blackwell) 稳定性和性能修复
        at::globalContext().setUserEnabledMkldnn(false);
#if defined(TORCH_VERSION_MAJOR) && (TORCH_VERSION_MAJOR < 2)
        torch::jit::setTensorExprFuserEnabled(false);
#endif
        torch::set_num_threads(1);

        module_ = torch::jit::load(cfg_.lsk3dnet_model_path);
        module_.eval();

        // Hard requirement: LSK3DNet must run on CUDA. If CUDA is not active, exit fast with clear logs.
        if (cfg_.lsk3dnet_device.rfind("cuda", 0) != 0) {
            throw std::runtime_error(
                "LSK3DNet requires CUDA device. Set semantic.lsk3dnet.device to 'cuda:0' (got '" + cfg_.lsk3dnet_device + "')");
        }
        const bool cuda_ok = torch::cuda::is_available();
        if (!cuda_ok) {
            std::ostringstream oss;
            oss << "[SEMANTIC][LSK3DNET][FATAL] CUDA is NOT available in LibTorch runtime.\n"
                << "  requested_device=" << cfg_.lsk3dnet_device << "\n"
                << "  torch_cuda_is_available=" << (cuda_ok ? "true" : "false") << "\n"
                << "  torch_cuda_device_count=" << torch::cuda::device_count() << "\n"
                << "  hint: ensure CUDA-capable LibTorch is installed and container has GPU access (--gpus all).";
            std::cerr << oss.str() << std::endl;
            throw std::runtime_error("LSK3DNet requires CUDA, but torch::cuda::is_available()==false");
        }
        device_ = torch::Device(cfg_.lsk3dnet_device);
        module_.to(device_);

        // Warmup a tiny forward to prove CUDA path is working and measure initial latency.
        try {
            torch::NoGradGuard guard;
            const int channels = std::max(4, cfg_.input_channels > 0 ? cfg_.input_channels : 4);
            auto t0 = std::chrono::steady_clock::now();
            auto x = torch::zeros({1, channels, cfg_.img_h, cfg_.img_w},
                                  torch::TensorOptions().dtype(torch::kFloat32).device(device_));
            auto out = module_.forward({x}).toTensor();
            (void)out;
            auto t1 = std::chrono::steady_clock::now();
            const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][LSK3DNET][LOAD] step=ok device=%s cuda_available=1 warmup_ms=%.2f",
                device_.str().c_str(), ms);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][LSK3DNET][LOAD] step=warmup_failed device=%s error=%s",
                device_.str().c_str(), e.what());
            throw;
        }
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
            result->per_point_labels.clear();
            result->success = false;
            result->backend_name = name();
            result->message = "USE_TORCH disabled";
        }
        return;
#else
        if (!cloud || cloud->empty()) {
            mask = cv::Mat::zeros(cfg_.img_h, cfg_.img_w, CV_8U);
            if (result != nullptr) {
                result->per_point_labels.clear();
                result->success = true;
                result->backend_name = name();
                result->message = "empty cloud";
                result->inference_ms = 0.0;
            }
            return;
        }
        if (result != nullptr) {
            result->per_point_labels.clear();
        }
        const auto t0 = std::chrono::steady_clock::now();
        const int channels = std::max(4, cfg_.input_channels > 0 ? cfg_.input_channels : 4);
        std::vector<float> input(static_cast<size_t>(channels * cfg_.img_h * cfg_.img_w), 0.0f);
        std::vector<int> hit_count(static_cast<size_t>(cfg_.img_h * cfg_.img_w), 0);

        const int img_h = cfg_.img_h;
        const int img_w = cfg_.img_w;
        const size_t img_size = static_cast<size_t>(img_h * img_w);

        #pragma omp parallel for
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& p = cloud->points[i];
            const auto proj = projectPoint(p, img_w, img_h, fov_up_rad_, fov_down_rad_);
            if (!proj.valid) continue;
            const int pix = proj.y * img_w + proj.x;
            
            #pragma omp atomic
            ++hit_count[static_cast<size_t>(pix)];

            for (int c = 0; c < channels; ++c) {
                float val = 0.0f;
                if (c == 0) val = p.x;
                else if (c == 1) val = p.y;
                else if (c == 2) val = p.z;
                else if (c == 3) val = p.intensity;
                else if (c == 4) val = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                
                const size_t idx = static_cast<size_t>(c) * img_size + static_cast<size_t>(pix);
                float norm_val = (val - meanAt(c)) / stdAt(c);
                
                #pragma omp atomic
                input[idx] += norm_val;
            }
        }

        #pragma omp parallel for
        for (int y = 0; y < img_h; ++y) {
            for (int x = 0; x < img_w; ++x) {
                const int pix = y * img_w + x;
                const int cnt = hit_count[static_cast<size_t>(pix)];
                if (cnt <= 1) continue;
                for (int c = 0; c < channels; ++c) {
                    const size_t idx = static_cast<size_t>(c) * img_size + static_cast<size_t>(pix);
                    input[idx] /= static_cast<float>(cnt);
                }
            }
        }

        if (onnx_model_) {
            std::vector<int64_t> input_dims = {1, static_cast<int64_t>(channels), static_cast<int64_t>(img_h), static_cast<int64_t>(img_w)};
            std::vector<float> input_data = input;
            
            Ort::Value input_tensor = onnx_model_->createTensor<float>(
                input_data.data(), input_data.size(), input_dims);
            
            std::vector<Ort::Value> inputs;
            inputs.push_back(std::move(input_tensor));
            
            auto outputs = onnx_model_->forward(inputs);
            if (outputs.empty()) throw std::runtime_error("ONNX inference failed");
            
            float* out_ptr = outputs[0].GetTensorMutableData<float>();
            auto out_dims = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
            
            mask = cv::Mat(img_h, img_w, CV_8U);
            if (out_dims.size() == 4) { // [1, C, H, W]
                int C = static_cast<int>(out_dims[1]);
                for (int y = 0; y < img_h; ++y) {
                    for (int x = 0; x < img_w; ++x) {
                        float max_val = -std::numeric_limits<float>::infinity();
                        int max_idx = 0;
                        for (int c = 0; c < C; ++c) {
                            float v = out_ptr[c * img_h * img_w + y * img_w + x];
                            if (v > max_val) {
                                max_val = v;
                                max_idx = c;
                            }
                        }
                        mask.at<uint8_t>(y, x) = static_cast<uint8_t>(max_idx);
                    }
                }
            } else if (out_dims.size() == 3) { // [1, H, W] labels
                for (int i = 0; i < img_h * img_w; ++i) {
                    mask.data[i] = static_cast<uint8_t>(out_ptr[i]);
                }
            }
            
            if (result != nullptr) {
                const auto t1 = std::chrono::steady_clock::now();
                result->success = true;
                result->backend_name = name();
                result->message = "ONNX/TensorRT";
                result->inference_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            }
            return;
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
            result->message = std::string("device=") + device_.str();
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
    OnnxSession::Ptr onnx_model_ = nullptr;
#ifdef USE_TORCH
    torch::jit::script::Module module_;
    torch::Device device_{torch::kCPU};
#endif
};

std::unique_ptr<ISemanticSegmentor> CreateLsk3dnetSemanticSegmentor(const SegmentorConfig& cfg) {
    return std::make_unique<Lsk3dnetSemanticSegmentor>(cfg);
}

}  // namespace automap_pro::v3
