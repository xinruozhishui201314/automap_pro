#pragma once

/**
 * 帧处理器模块
 *
 * 负责点云数据的接收、体素下采样和帧队列管理。
 * 封装了 feederLoop 的核心逻辑，提供更清晰的接口。
 */

#include "automap_pro/core/frame_types.h"
#include "automap_pro/core/data_types.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>

namespace automap_pro {

/**
 * 帧就绪回调
 */
using FrameReadyCallback = std::function<void(FrameToProcess&&)>;

/**
 * 帧处理器
 *
 * 负责：
 * - 从 ingress_queue 消费原始帧
 * - 执行体素下采样
 * - 将处理后的帧推送到 frame_queue
 */
class FrameProcessor {
public:
    /**
     * 构造函数
     */
    FrameProcessor();
    
    ~FrameProcessor();
    
    // 禁止拷贝
    FrameProcessor(const FrameProcessor&) = delete;
    FrameProcessor& operator=(const FrameProcessor&) = delete;
    
    /**
     * 初始化
     * @param max_ingress_queue_size 入口队列最大大小
     * @param max_frame_queue_size 处理后队列最大大小
     */
    void init(size_t max_ingress_queue_size = 100, 
              size_t max_frame_queue_size = 500);
    
    /**
     * 启动处理线程
     */
    void start();
    
    /**
     * 停止处理线程
     */
    void stop();
    
    /**
     * 推送原始帧到入口队列 (Wait-Free if queue not full)
     * @return 是否成功入队
     */
    bool pushFrame(double ts, const CloudXYZIPtr& cloud);
    
    /**
     * 注册帧就绪回调
     */
    void registerCallback(FrameReadyCallback cb);
    
    /**
     * 获取入口队列大小
     */
    size_t ingressQueueSize() const;
    
    /**
     * 获取处理队列大小
     */
    size_t frameQueueSize() const;
    
    /**
     * 设置关闭标志
     */
    void setShutdownFlag(const std::atomic<bool>* flag);
    
    /**
     * 获取处理帧计数
     */
    int processedFrameCount() const;

    /**
     * 获取入口队列未响应计数
     */
    int backpressureDropCount() const;

    /**
     * 尝试从处理队列获取一帧（供外部 worker 调用）
     * @param timeout_ms 超时时间（毫秒）
     * @param out_frame 输出帧
     * @return 是否成功获取到帧
     */
    bool tryPopFrame(int timeout_ms, FrameToProcess& out_frame);
    
private:
    /**
     * Feeder 线程主循环
     */
    void feederLoop();
    
    /**
     * 执行体素下采样
     */
    CloudXYZIPtr downsample(const CloudXYZIPtr& cloud, float resolution);
    
    // 队列 (V2 优化: SPSC 无锁队列，容量固定为 8192)
    // ingress_queue: onCloud -> feederLoop
    boost::lockfree::spsc_queue<FrameToProcess, boost::lockfree::capacity<8192>> ingress_queue_;
    // frame_queue: feederLoop -> backendWorkerLoop
    boost::lockfree::spsc_queue<FrameToProcess, boost::lockfree::capacity<8192>> frame_queue_;
    
    // 同步原语 (仅用于 wait_for 机制，热路径 push/pop 为无锁)
    std::mutex ingress_not_empty_mutex_;
    std::condition_variable ingress_not_empty_cv_;
    
    std::mutex frame_queue_not_empty_mutex_;
    std::condition_variable frame_queue_not_empty_cv_;
    
    // 配置
    size_t max_ingress_queue_size_;
    size_t max_frame_queue_size_;
    
    // 线程
    std::thread feeder_thread_;
    std::atomic<bool> running_{false};
    const std::atomic<bool>* shutdown_flag_ = nullptr;
    
    // 回调
    FrameReadyCallback callback_;
    
    // 统计
    std::atomic<int> frame_count_{0};
    std::atomic<int> backpressure_drop_count_{0};
    std::atomic<size_t> ingress_size_{0};
    std::atomic<size_t> frame_queue_size_{0};
};

}  // namespace automap_pro
