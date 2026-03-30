#pragma once
/**
 * @file core/frame_types.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */


/**
 * 帧数据类型定义
 *
 * 定义系统中的基础帧类型，避免头文件循环依赖。
 */

#include "automap_pro/core/data_types.h"

namespace automap_pro {

/**
 * 待处理帧
 */
struct FrameToProcess {
    double ts = 0.0;
    CloudXYZIPtr cloud;
    CloudXYZIPtr cloud_ds;  // 可选：feeder 预计算体素降采样，worker 有则直接用
};

/**
 * 帧就绪回调
 */
using FrameReadyCallback = std::function<void(FrameToProcess&&)>;

}  // namespace automap_pro
