/**
 * @file overlap_transformer_infer.cpp
 * @brief Range 图投影、TorchScript 前向、L2 归一化描述子、余弦 Top-K 检索与 FPFH fallback。
 */
#include "automap_pro/loop_closure/overlap_transformer_infer.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "OT_Infer"

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#ifdef USE_TORCH
#include <ATen/Context.h>
#include <torch/script.h>
#include <torch/version.h>
#endif

namespace automap_pro {

namespace {
bool isNvrtcArchError(const std::exception& e) {
    const std::string msg = e.what();
    return msg.find("nvrtc") != std::string::npos &&
           msg.find("gpu-architecture") != std::string::npos;
}
}  // namespace

OverlapTransformerInfer::OverlapTransformerInfer() {
    const auto& cfg = ConfigManager::instance();
    proj_H_    = cfg.rangeImageH();
    proj_W_    = cfg.rangeImageW();
    fov_up_    = cfg.fovUp();
    fov_down_  = cfg.fovDown();
    max_range_ = cfg.maxRange();
}

bool OverlapTransformerInfer::loadModel(const std::string& model_path, bool use_cuda) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (model_path.empty()) {
        ALOG_WARN(MOD, "[OT] model_path empty; overlapTransformer.pt will not be loaded (use ScanContext or fallback).");
        model_loaded_ = false;
        return false;
    }
    // 解析为绝对路径，避免相对路径受 CWD 影响
    std::string path_to_load = model_path;
    try {
        std::filesystem::path p(model_path);
        if (p.is_relative()) {
            p = std::filesystem::absolute(p);
            path_to_load = p.string();
        }
        if (!std::filesystem::exists(p) || !std::filesystem::is_regular_file(p)) {
            // 回退：安装空间下模型可能在 share/automap_pro/models/
            std::string fallback;
            try {
                // 尝试 .pt 和 .onnx 两种后缀
                std::string base = ament_index_cpp::get_package_share_directory("automap_pro") + "/models/overlapTransformer";
                if (std::filesystem::exists(base + ".onnx")) {
                    fallback = base + ".onnx";
                } else if (std::filesystem::exists(base + ".pt")) {
                    fallback = base + ".pt";
                }
                
                if (!fallback.empty() && std::filesystem::exists(fallback) && std::filesystem::is_regular_file(fallback)) {
                    path_to_load = fallback;
                    p = std::filesystem::path(path_to_load);
                    ALOG_INFO(MOD, "[OT] Using fallback model path (install space): {}", path_to_load);
                } else {
                    fallback.clear();
                }
            } catch (const std::exception&) {
                fallback.clear();
            }
            if (fallback.empty() || !std::filesystem::exists(p) || !std::filesystem::is_regular_file(p)) {
                ALOG_ERROR(MOD, "[OT] Model file not found or not a file: {} (model NOT loaded)", path_to_load);
                model_loaded_ = false;
                return false;
            }
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[OT] Path check failed for '{}': {} (model NOT loaded)", model_path, e.what());
        model_loaded_ = false;
        return false;
    }

    // 优先处理 ONNX 模型（支持 TensorRT 加速）
    std::filesystem::path p_final(path_to_load);
    if (p_final.extension() == ".onnx") {
        ALOG_INFO(MOD, "[OT] Loading OverlapTransformer model (ONNX): {} (H={} W={})", path_to_load, proj_H_, proj_W_);
        onnx_model_ = std::make_shared<OnnxSession>();
        if (onnx_model_->loadModel(path_to_load, use_cuda)) {
            model_loaded_ = true;
            use_cuda_ = use_cuda;
            ALOG_INFO(MOD, "[OT] ONNX Model loaded OK: path={} device={} H={} W={}",
                      path_to_load, use_cuda_ ? "CUDA/TensorRT" : "CPU", proj_H_, proj_W_);
            return true;
        } else {
            onnx_model_ = nullptr;
            model_loaded_ = false;
            return false;
        }
    }

#ifdef USE_TORCH
    ALOG_INFO(MOD, "[OT] Loading OverlapTransformer model (LibTorch): {} (H={} W={})", path_to_load, proj_H_, proj_W_);
    try {
        // 禁用 MKLDNN/DNNL 后端，避免 dnnl::impl::lru_primitive_cache_t::get 多线程/缓存竞态导致 SIGSEGV（full.log 中崩溃栈）
        at::globalContext().setUserEnabledMkldnn(false);
        ALOG_INFO(MOD, "[OT] MKLDNN disabled (use native CPU impl to avoid DNNL cache crash)");

        // 🛑 [FIX] 针对 RTX 5080 (Blackwell) 崩溃修复：在旧版 LibTorch 关闭 TensorExpr fuser。
        // 新版 LibTorch 已移除此 API，需条件编译避免构建失败。
#if defined(TORCH_VERSION_MAJOR) && (TORCH_VERSION_MAJOR < 2)
        torch::jit::setTensorExprFuserEnabled(false);
        ALOG_INFO(MOD, "[OT] TensorExpr fuser disabled (legacy LibTorch API)");
#else
        ALOG_INFO(MOD, "[OT] Skip TensorExpr fuser toggle: API removed in this LibTorch version");
#endif

        // 限制 LibTorch 内部线程数为 1，进一步降低竞态风险
        torch::set_num_threads(1);
        // 加载 TorchScript .pt 模型（由 gen_libtorch_model.py 生成）
        model_ = torch::jit::load(path_to_load);
        model_.eval();

        // 设备选择
        use_cuda_ = use_cuda && torch::cuda::is_available();
        torch::Device device(use_cuda_ ? torch::kCUDA : torch::kCPU);
        model_.to(device);

        // 预热（消除首次推理延迟，维度：[1,1,H,W]）
        auto dummy = torch::zeros({1, 1, proj_H_, proj_W_}, torch::kFloat);
        if (use_cuda_) dummy = dummy.to(torch::kCUDA);
        {
            torch::NoGradGuard no_grad;
            model_.forward({dummy});
        }
        model_loaded_ = true;

        ALOG_INFO(MOD, "[OT] Model loaded OK: path={} device={} H={} W={} (inference will use this model)",
                  path_to_load, use_cuda_ ? "CUDA" : "CPU", proj_H_, proj_W_);
        return true;
    } catch (const c10::Error& e) {
        ALOG_ERROR(MOD, "[OT] Failed to load model '{}': {} (overlapTransformer.pt NOT loaded)", path_to_load, e.what());
        model_loaded_ = false;
        return false;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[OT] Exception loading model: {} (overlapTransformer.pt NOT loaded)", e.what());
        model_loaded_ = false;
        return false;
    }
#else
    (void)model_path;
    (void)use_cuda;
    model_loaded_ = false;
    ALOG_WARN(MOD,
        "[OT] overlapTransformer.pt NOT USED: LibTorch not compiled (USE_TORCH off). "
        "Loop closure uses ScanContext or fallback descriptor. "
        "To use OT: build with LibTorch and set loop_closure.overlap_transformer.model_path.");
    return false;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// 生成范围图（完整移植自 OT_libtorch/ws/fast_ot.cpp gen_range_image）
// ─────────────────────────────────────────────────────────────────────────────
std::vector<float> OverlapTransformerInfer::generateRangeImage(
    const CloudXYZIPtr& cloud) const
{
    const int len = proj_H_ * proj_W_;
    std::vector<float> img(len, 0.0f);  // 初始化为0而非-1，便于后续补齐

    const float fov_up_rad   = fov_up_   * static_cast<float>(M_PI) / 180.0f;
    const float fov_down_rad = fov_down_ * static_cast<float>(M_PI) / 180.0f;
    const float fov_range    = std::abs(fov_down_rad) + std::abs(fov_up_rad);

    // ✅ 第一遍：将点投影到范围图（取最近深度）
    std::vector<bool> valid_pixel(len, false);  // 标记有效像素
    for (const auto& pt : cloud->points) {
        float depth = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (depth >= max_range_ || depth <= 0.0f) continue;

        float yaw   = -std::atan2(pt.y, pt.x);
        float pitch = std::asin(pt.z / depth);

        float proj_x = 0.5f * (yaw / static_cast<float>(M_PI) + 1.0f);
        float proj_y = 1.0f - (pitch + std::abs(fov_down_rad)) / fov_range;

        proj_x *= proj_W_;
        proj_y *= proj_H_;

        int col = std::max(0, std::min(proj_W_ - 1, static_cast<int>(std::floor(proj_x))));
        int row = std::max(0, std::min(proj_H_ - 1, static_cast<int>(std::floor(proj_y))));

        int idx = row * proj_W_ + col;
        if (img[idx] <= 0.0f || depth < img[idx]) {
            img[idx] = depth;
            valid_pixel[idx] = true;
        }
    }

    // ✅ 第二遍：快速补齐稀疏像素（行列扫描，O(H×W) 而非 O(H×W×9)）
    // 水平传播（从左到右）
    for (int row = 0; row < proj_H_; ++row) {
        for (int col = 1; col < proj_W_; ++col) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int left_idx = idx - 1;
            if (valid_pixel[left_idx] && img[left_idx] > 0.0f) {
                img[idx] = img[left_idx];
                valid_pixel[idx] = true;
            }
        }
    }
    // 水平传播（从右到左，补齐右侧空洞）
    for (int row = 0; row < proj_H_; ++row) {
        for (int col = proj_W_ - 2; col >= 0; --col) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int right_idx = idx + 1;
            if (valid_pixel[right_idx] && img[right_idx] > 0.0f) {
                img[idx] = img[right_idx];
                valid_pixel[idx] = true;
            }
        }
    }
    
    // 垂直传播（从上到下）
    for (int col = 0; col < proj_W_; ++col) {
        for (int row = 1; row < proj_H_; ++row) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int up_idx = idx - proj_W_;
            if (valid_pixel[up_idx] && img[up_idx] > 0.0f) {
                img[idx] = img[up_idx];
                valid_pixel[idx] = true;
            }
        }
    }
    // 垂直传播（从下到上，补齐下侧空洞）
    for (int col = 0; col < proj_W_; ++col) {
        for (int row = proj_H_ - 2; row >= 0; --row) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int down_idx = idx + proj_W_;
            if (valid_pixel[down_idx] && img[down_idx] > 0.0f) {
                img[idx] = img[down_idx];
                valid_pixel[idx] = true;
            }
        }
    }

    return img;
}

// ─────────────────────────────────────────────────────────────────────────────
// LibTorch 推理（Level 1）
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_TORCH
Eigen::VectorXf OverlapTransformerInfer::inferWithTorch(
    const std::vector<float>& range_img) const
{
    const unsigned tid = logThreadId();
    const long lwp = logLwp();

    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=infer_tensor_enter tid={} lwp={}", tid, lwp);

    torch::Device device(use_cuda_ ? torch::kCUDA : torch::kCPU);

    // 先拷贝到局部缓冲区，避免多线程下 range_img 被修改导致悬空指针
    std::vector<float> range_img_copy(range_img.begin(), range_img.end());
    torch::Tensor t = torch::from_blob(
        range_img_copy.data(),
        {1, 1, proj_H_, proj_W_},
        torch::kFloat32).clone();

    if (use_cuda_) t = t.to(device);

    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=forward_enter tid={} lwp={} (崩溃在 DNNL 时本 step 为最后一条)",
              tid, lwp);
    std::cerr << "[OT_CRASH_LOC] step=forward_enter lwp=" << lwp << std::endl;

    torch::Tensor result;
    {
        torch::NoGradGuard no_grad;
        result = model_.forward({t}).toTensor();
    }

    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=forward_exit tid={} lwp={}", tid, lwp);

    // L2 归一化（参考 OverlapTransformer 原始实现）
    result = torch::nn::functional::normalize(
        result,
        torch::nn::functional::NormalizeFuncOptions().p(2).dim(1));

    if (use_cuda_) result = result.to(torch::kCPU);
    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=to_cpu_done tid={} lwp={}", tid, lwp);

    result = result.squeeze();   // [256]

    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=eigen_mapped tid={} lwp={} dim={}", tid, lwp, result.numel());

    // Tensor → Eigen::VectorXf
    return Eigen::Map<Eigen::VectorXf>(result.data_ptr<float>(), result.numel());
}
#endif

Eigen::VectorXf OverlapTransformerInfer::inferWithOnnx(
    const std::vector<float>& range_img) const
{
    if (!onnx_model_) return Eigen::VectorXf::Zero(256);

    std::vector<int64_t> input_dims = {1, 1, static_cast<int64_t>(proj_H_), static_cast<int64_t>(proj_W_)};
    
    // ONNX Runtime 需要 float* 数据。直接使用 range_img 的副本（推理是并行的，需确保数据生命周期）
    std::vector<float> input_data = range_img; 

    Ort::Value input_tensor = onnx_model_->createTensor<float>(
        input_data.data(), input_data.size(), input_dims);

    std::vector<Ort::Value> inputs;
    inputs.push_back(std::move(input_tensor));

    auto outputs = onnx_model_->forward(inputs);
    if (outputs.empty()) {
        ALOG_ERROR(MOD, "[OT] ONNX inference returned no outputs");
        return Eigen::VectorXf::Zero(256);
    }

    // 假设输出是 [1, 256] 或 [256]
    float* output_data = outputs[0].GetTensorMutableData<float>();
    auto tensor_info = outputs[0].GetTensorTypeAndShapeInfo();
    size_t output_size = tensor_info.GetElementCount();

    Eigen::VectorXf desc = Eigen::Map<Eigen::VectorXf>(output_data, output_size);
    
    // L2 归一化（ONNX 模型内部可能没包含 normalize 算子）
    float norm = desc.norm();
    if (norm > 1e-6f) {
        desc /= norm;
    }

    return desc;
}

// ─────────────────────────────────────────────────────────────────────────────
// FPFH 全局直方图 fallback（Level 3）
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXf OverlapTransformerInfer::computeFallbackDescriptor(
    const CloudXYZIPtr& cloud) const
{
    const int dim = 256;
    Eigen::VectorXf desc = Eigen::VectorXf::Zero(dim);
    if (!cloud || cloud->empty()) return desc;

    // 使用深度直方图（与 OverlapTransformer Python fallback 一致）
    auto range_img = generateRangeImage(cloud);
    int bins = dim;
    float bin_width = max_range_ / bins;

    for (float d : range_img) {
        if (d <= 0.0f) continue;
        int bin = std::min(bins - 1, static_cast<int>(d / bin_width));
        desc(bin) += 1.0f;
    }

    // L2 归一化
    float norm = desc.norm();
    if (norm > 1e-6f) desc /= norm;
    return desc;
}

// ─────────────────────────────────────────────────────────────────────────────
// 主入口：计算描述子
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXf OverlapTransformerInfer::computeDescriptor(
    const CloudXYZIPtr& cloud) const
{
    const unsigned tid = logThreadId();
    const long lwp = logLwp();
    const int call_index = total_inferences_ + 1;

    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=compute_enter tid={} lwp={} call={} pts={} (grep OT_CRASH_LOC 定位崩溃步骤)",
              tid, lwp, call_index, cloud ? cloud->size() : 0u);

    auto t_start = std::chrono::steady_clock::now();
    
    auto t_phase = std::chrono::steady_clock::now();
    
    Eigen::VectorXf desc;

    if (model_loaded_) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto range_img = generateRangeImage(cloud);
        auto t_range = std::chrono::steady_clock::now();
        double range_ms = std::chrono::duration<double, std::milli>(t_range - t_phase).count();
        ALOG_INFO(MOD, "[LOOP_DESC][OT] phase=range_image_done pts={} range_ms={:.1f} (OverlapTransformer 范围图生成)",
                  cloud ? cloud->size() : 0u, range_ms);

        ALOG_INFO(MOD, "[OT_CRASH_LOC] step=about_to_infer tid={} lwp={} call={} (若崩溃在 DNNL/forward，上一条即本 step)",
                  tid, lwp, call_index);
        std::cerr << "[OT_CRASH_LOC] step=about_to_forward tid=" << tid << " lwp=" << lwp << " call=" << call_index << std::endl;

        if (onnx_model_) {
            desc = inferWithOnnx(range_img);
            ALOG_INFO(MOD, "[OT_CRASH_LOC] step=infer_returned_onnx tid={} lwp={} call={}", tid, lwp, call_index);
        } else {
#ifdef USE_TORCH
            try {
                desc = inferWithTorch(range_img);
                ALOG_INFO(MOD, "[OT_CRASH_LOC] step=infer_returned tid={} lwp={} call={}", tid, lwp, call_index);
            } catch (const std::exception& e) {
                // ... existing exception handling ...
                ALOG_ERROR(MOD,
                    "[OT][EXCEPTION] inferWithTorch failed: {} (device={} call={} pts={})",
                    e.what(), use_cuda_ ? "CUDA" : "CPU", call_index, cloud ? cloud->size() : 0u);

                // Blackwell + older LibTorch may throw NVRTC arch errors in TE kernels.
                // Degrade to CPU model in-process so the pipeline continues.
                if (use_cuda_ && isNvrtcArchError(e)) {
                    ALOG_WARN(MOD, "[OT][RECOVER] NVRTC arch mismatch detected, fallback to CPU torch inference");
                    try {
                        model_.to(torch::kCPU);
                        use_cuda_ = false;
                        desc = inferWithTorch(range_img);
                        ALOG_WARN(MOD, "[OT][RECOVER] CPU retry succeeded after NVRTC arch error");
                    } catch (const std::exception& e2) {
                        ALOG_ERROR(MOD,
                            "[OT][RECOVER] CPU retry failed: {} ; fallback to histogram descriptor", e2.what());
                        desc = computeFallbackDescriptor(cloud);
                    }
                } else {
                    desc = computeFallbackDescriptor(cloud);
                }
            }
#else
            desc = computeFallbackDescriptor(cloud);
#endif
        }

        auto t_infer = std::chrono::steady_clock::now();
        double infer_ms = std::chrono::duration<double, std::milli>(t_infer - t_range).count();
        ALOG_INFO(MOD, "[LOOP_DESC][OT] phase=infer_done infer_ms={:.1f} dim=256 device={} (模型推理描述子)",
                  infer_ms, use_cuda_ ? "CUDA" : "CPU");

        if (!first_inference_logged_) {
            first_inference_logged_ = true;
            ALOG_INFO(MOD, "[OT] inference state: model=loaded device={} dim=256 (first inference done, range_ms={:.1f} infer_ms={:.1f})",
                      use_cuda_ ? "CUDA" : "CPU", range_ms, infer_ms);
        }
        
        if (range_ms > 10.0 || infer_ms > 20.0) {
            ALOG_WARN(MOD, "[OT][PERF] Range generation: {:.1f}ms, Inference: {:.1f}ms", 
                      range_ms, infer_ms);
        }
    } else {
        desc = computeFallbackDescriptor(cloud);
        ALOG_INFO(MOD, "[LOOP_DESC][OT] phase=fallback_descriptor pts={} (模型未加载，使用直方图 fallback)",
                  cloud ? cloud->size() : 0u);
    }

    // ✅ 验证 L2 归一化
    float norm = desc.norm();
    if (norm > 1e-6f) {
        desc /= norm;
    } else {
        ALOG_WARN(MOD, "Descriptor norm too small ({:.6f}), using uniform fallback", norm);
        desc = Eigen::VectorXf::Constant(256, 1.0f / std::sqrt(256.0f));
    }

    auto t_end = std::chrono::steady_clock::now();
    last_infer_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    total_inferences_++;
    ALOG_INFO(MOD, "[OT_CRASH_LOC] step=compute_exit tid={} lwp={} call={} total_ms={:.1f} (描述子计算完成)",
              logThreadId(), logLwp(), total_inferences_, last_infer_ms_);
    ALOG_INFO(MOD, "[LOOP_DESC][OT] phase=descriptor_done total_ms={:.1f} pts={} calls={} (描述子计算完成)",
              last_infer_ms_, cloud ? cloud->size() : 0, total_inferences_);
    
    if (last_infer_ms_ > 30.0) {
        ALOG_WARN(MOD, "[OT][PERF] Descriptor compute SLOW: {:.1f}ms (target: <20ms for 50Hz) pts={}", 
                  last_infer_ms_, cloud ? cloud->size() : 0);
    } else if (total_inferences_ % 50 == 0) {
        // 每50次输出一次，便于从日志确认推理持续进行
        ALOG_DEBUG(MOD, "[OT][PERF] descriptor calls={} last_ms={:.1f}",
                   total_inferences_, last_infer_ms_);
    }
    
    return desc;
}

// ─────────────────────────────────────────────────────────────────────────────
// 候选检索（余弦相似度 Top-K + 多重过滤）
// ─────────────────────────────────────────────────────────────────────────────
std::vector<OverlapTransformerInfer::Candidate> OverlapTransformerInfer::retrieve(
    const Eigen::VectorXf& query_desc,
    const std::vector<std::shared_ptr<SubMap>>& db_submaps,
    int    top_k,
    float  threshold,
    int    min_submap_gap,
    double min_time_gap,
    double gps_radius_m,
    const Eigen::Vector3d& query_gps_pos,
    bool   query_has_gps) const
{
    std::vector<std::pair<float, int>> scored;
    float query_norm_cache = query_desc.norm();
    if (query_norm_cache < 1e-6f) {
        query_norm_cache = 1.0f;
    }

    int db_with_desc = 0;
    for (int i = 0; i < (int)db_submaps.size(); ++i) {
        const auto& sm = db_submaps[i];
        if (!sm->has_descriptor) continue;
        db_with_desc++;

        float score = query_desc.dot(sm->overlap_descriptor) /
                      (query_norm_cache * sm->overlap_descriptor_norm + 1e-8f);
        if (score < threshold) continue;

        if (gps_radius_m > 0.0 && query_has_gps && sm->has_valid_gps) {
            double dist = (query_gps_pos - sm->gps_center).norm();
            if (dist > gps_radius_m) continue;
        }

        scored.push_back({score, i});
    }

    ALOG_INFO(MOD, "[LOOP_RETRIEVE][OT] phase=score_scan db_size={} db_with_desc={} threshold={:.3f} passed={} top_k={} (余弦相似度过滤)",
              (int)db_submaps.size(), db_with_desc, threshold, (int)scored.size(), top_k);

    std::partial_sort(scored.begin(),
                      scored.begin() + std::min(top_k, (int)scored.size()),
                      scored.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<Candidate> result;
    for (int i = 0; i < std::min(top_k, (int)scored.size()); ++i) {
        const auto& sm = db_submaps[scored[i].second];
        result.push_back({sm->id, sm->session_id, scored[i].first});
    }
    if (!result.empty()) {
        std::ostringstream oss;
        for (size_t i = 0; i < result.size(); ++i) {
            if (i > 0) oss << ",";
            oss << "(id=" << result[i].submap_id << " score=" << std::fixed << std::setprecision(3) << result[i].score << ")";
        }
        ALOG_INFO(MOD, "[LOOP_RETRIEVE][OT] phase=topk_result count={} candidates=[{}] (回环候选列表)",
                  result.size(), oss.str());
    }
    return result;
}

} // namespace automap_pro
