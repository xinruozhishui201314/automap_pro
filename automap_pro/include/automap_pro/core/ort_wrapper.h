#pragma once
/**
 * @file core/ort_wrapper.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */


#include <string>
#include <vector>
#include <memory>
#include <onnxruntime_cxx_api.h>

namespace automap_pro {

/**
 * @brief ONNX Runtime Session 封装，支持 TensorRT 加速。
 */
class OnnxSession {
public:
    using Ptr = std::shared_ptr<OnnxSession>;

    OnnxSession();
    ~OnnxSession();

    /**
     * @brief 加载 ONNX 模型。
     * @param model_path 模型文件路径 (.onnx)
     * @param use_cuda 是否使用 CUDA (及 TensorRT) 加速
     * @param device_id 设备 ID (默认 0)
     * @return 是否加载成功
     */
    bool loadModel(const std::string& model_path, bool use_cuda = true, int device_id = 0);

    /**
     * @brief 执行推理。
     * @param input_tensors 输入张量列表
     * @return 输出张量列表
     */
    std::vector<Ort::Value> forward(std::vector<Ort::Value>& input_tensors);

    /**
     * @brief 创建输入张量助手（支持 float32）。
     */
    template <typename T>
    Ort::Value createTensor(T* data, size_t size, const std::vector<int64_t>& dims);

    // 获取输入输出信息
    const std::vector<std::string>& inputNames() const { return input_names_; }
    const std::vector<std::string>& outputNames() const { return output_names_; }
    const std::vector<std::vector<int64_t>>& inputDims() const { return input_dims_; }
    const std::vector<std::vector<int64_t>>& outputDims() const { return output_dims_; }

    bool isLoaded() const { return session_ != nullptr; }

private:
    std::unique_ptr<Ort::Env> env_ = nullptr;
    std::unique_ptr<Ort::Session> session_ = nullptr;
    Ort::MemoryInfo memory_info_ = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
    
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> input_names_ptr_;
    std::vector<const char*> output_names_ptr_;
    std::vector<std::vector<int64_t>> input_dims_;
    std::vector<std::vector<int64_t>> output_dims_;

    bool use_cuda_ = false;
};

template <typename T>
Ort::Value OnnxSession::createTensor(T* data, size_t size, const std::vector<int64_t>& dims) {
    return Ort::Value::CreateTensor<T>(memory_info_, data, size, dims.data(), dims.size());
}

} // namespace automap_pro
