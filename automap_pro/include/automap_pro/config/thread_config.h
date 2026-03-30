#pragma once
/**
 * @file config/thread_config.h
 * @brief 线程与运行时配置片段。
 */


/**
 * 线程与队列配置模块
 *
 * 统一管理所有线程相关的配置参数，包括：
 * - 队列最大容量
 * - 超时设置
 * - 心跳监控阈值
 * - 背压控制参数
 */

#include <chrono>

namespace automap_pro {

/**
 * 线程与队列配置
 *
 * 所有线程和队列相关的配置参数集中在此管理
 */
struct ThreadConfig {
    // ==================== 队列配置 ====================
    
    /** 入口队列最大大小 */
    size_t max_ingress_queue_size{100};
    
    /** 帧处理队列最大大小 */
    size_t max_frame_queue_size{500};
    
    /** 优化任务队列最大大小 */
    size_t max_opt_task_queue_size{64};
    
    /** 回环触发队列最大大小 */
    size_t max_loop_trigger_queue_size{64};
    
    /** GPS 队列最大大小 */
    size_t max_gps_queue_size{200};
    
    /** 可视化队列最大大小 */
    size_t max_viz_queue_size{2};
    
    // ==================== 超时配置 ====================
    
    /** 队列等待超时（默认 500ms） */
    std::chrono::milliseconds queue_wait_timeout{500};
    
    /** 后端处理超时（默认 2000ms） */
    std::chrono::milliseconds backend_process_timeout{2000};
    
    /** 体素下采样超时（默认 5000ms） */
    std::chrono::milliseconds voxel_downsample_timeout{5000};
    
    // ==================== 心跳配置 ====================
    
    /** 心跳检查间隔（默认 30s） */
    std::chrono::seconds heartbeat_interval{30};
    
    /** 心跳告警阈值（默认 10s） */
    std::chrono::milliseconds heartbeat_warn{10000};
    
    /** 心跳错误阈值（默认 30s） */
    std::chrono::milliseconds heartbeat_error{30000};
    
    // ==================== 背压配置 ====================
    
    /** 最大等待次数（默认 10 次） */
    int backpressure_max_waits{10};
    
    /** 每次等待时长（秒，默认 1s） */
    int backpressure_wait_sec{1};
    
    // ==================== 其他配置 ====================
    
    /** 传感器空闲超时（秒，默认 5s） */
    double sensor_idle_timeout_sec{5.0};
    
    /** 获取单例实例 */
    static const ThreadConfig& instance();
    
    /** 获取可变实例（用于运行时修改） */
    static ThreadConfig& mutableInstance();
    
    /**
     * 从 ConfigManager 加载配置
     * 注意：此方法需要在 ConfigManager 初始化后调用
     */
    static void loadFromConfigManager();
    
private:
    ThreadConfig() = default;
    static ThreadConfig* instance_;
};

}  // namespace automap_pro
