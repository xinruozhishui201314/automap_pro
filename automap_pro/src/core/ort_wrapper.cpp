/**
 * @file core/ort_wrapper.cpp
 * @brief 核心实现。
 */
#include "automap_pro/core/ort_wrapper.h"
#include "automap_pro/core/logger.h"
#include <iostream>
#include <filesystem>

#define MOD "ORT_Wrapper"

namespace automap_pro {

OnnxSession::OnnxSession() {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "AutoMap_ORT");
}

OnnxSession::~OnnxSession() = default;

bool OnnxSession::loadModel(const std::string& model_path, bool use_cuda, int device_id) {
    if (!std::filesystem::exists(model_path)) {
        ALOG_ERROR(MOD, "Model file not found: {}", model_path);
        return false;
    }

    try {
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

        if (use_cuda) {
            use_cuda_ = true;
            // 尝试添加 TensorRT EP，RTX 5080 必须用这个才能发挥极致性能
            try {
                OrtTensorRTProviderOptions trt_options{};
                trt_options.device_id = device_id;
                trt_options.trt_fp16_enable = 1; // 开启 FP16 进一步加速
                trt_options.trt_max_workspace_size = 1LL << 30; // 1GB workspace
                // 注意：如果环境下没装 TensorRT，这行会抛异常，此时回退到 CUDA EP
                session_options.AppendExecutionProvider_TensorRT(trt_options);
                ALOG_INFO(MOD, "TensorRT Execution Provider added for device: {}", device_id);
            } catch (...) {
                ALOG_WARN(MOD, "TensorRT EP not available, falling back to CUDA EP");
                OrtCUDAProviderOptions cuda_options{};
                cuda_options.device_id = device_id;
                session_options.AppendExecutionProvider_CUDA(cuda_options);
                ALOG_INFO(MOD, "CUDA Execution Provider added for device: {}", device_id);
            }
        }

        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);

        // 获取输入输出信息
        Ort::AllocatorWithDefaultOptions allocator;
        size_t num_input_nodes = session_->GetInputCount();
        input_names_.clear();
        input_names_ptr_.clear();
        input_dims_.clear();
        for (size_t i = 0; i < num_input_nodes; i++) {
            char* input_name = session_->GetInputName(i, allocator);
            input_names_.push_back(input_name);
            allocator.Free(input_name);
            input_names_ptr_.push_back(input_names_.back().c_str());

            Ort::TypeInfo type_info = session_->GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            input_dims_.push_back(tensor_info.GetShape());
        }

        size_t num_output_nodes = session_->GetOutputCount();
        output_names_.clear();
        output_names_ptr_.clear();
        output_dims_.clear();
        for (size_t i = 0; i < num_output_nodes; i++) {
            char* output_name = session_->GetOutputName(i, allocator);
            output_names_.push_back(output_name);
            allocator.Free(output_name);
            output_names_ptr_.push_back(output_names_.back().c_str());

            Ort::TypeInfo type_info = session_->GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            output_dims_.push_back(tensor_info.GetShape());
        }

        ALOG_INFO(MOD, "Model loaded successfully: {} (Input: {}, Output: {})", 
                  model_path, input_names_.size(), output_names_.size());
        return true;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "Failed to load model: {}", e.what());
        return false;
    }
}

std::vector<Ort::Value> OnnxSession::forward(std::vector<Ort::Value>& input_tensors) {
    if (!session_) return {};

    try {
        return session_->Run(Ort::RunOptions{nullptr}, 
                             input_names_ptr_.data(), 
                             input_tensors.data(), 
                             input_tensors.size(), 
                             output_names_ptr_.data(), 
                             output_names_ptr_.size());
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "Inference failed: {}", e.what());
        return {};
    }
}

} // namespace automap_pro
