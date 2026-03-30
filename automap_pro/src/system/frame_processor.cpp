/**
 * @file system/frame_processor.cpp
 * @brief 系统节点与 ROS 服务实现。
 */
#include "automap_pro/system/frame_processor.h"

#include "automap_pro/core/config_manager.h"
#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/backend/optimizer.h"

#include <pcl/common/io.h>

namespace automap_pro {

FrameProcessor::FrameProcessor()
    : max_ingress_queue_size_(100)
    , max_frame_queue_size_(500)
    , submap_match_res_(ConfigManager::instance().isLoaded() ? static_cast<float>(ConfigManager::instance().submapMatchRes()) : 0.4f)
    , shutdown_flag_(nullptr) {
}

FrameProcessor::~FrameProcessor() {
    stop();
}

void FrameProcessor::init(size_t max_ingress_queue_size, 
                         size_t max_frame_queue_size) {
    max_ingress_queue_size_ = max_ingress_queue_size;
    max_frame_queue_size_ = max_frame_queue_size;
    if (ConfigManager::instance().isLoaded()) {
        submap_match_res_ = static_cast<float>(ConfigManager::instance().submapMatchRes());
    }
}

void FrameProcessor::start() {
    if (running_.load()) {
        return;
    }
    running_.store(true);
    feeder_thread_ = std::thread([this]() {
#ifdef __linux__
        pthread_setname_np(pthread_self(), "frame_feeder");
#endif
        feederLoop();
    });
}

void FrameProcessor::stop() {
    if (!running_.load()) {
        return;
    }
    running_.store(false);
    ingress_not_empty_cv_.notify_all();
    frame_queue_not_empty_cv_.notify_all();
    if (feeder_thread_.joinable()) {
        feeder_thread_.join();
    }
}

bool FrameProcessor::pushFrame(double ts, const CloudXYZIPtr& cloud) {
    // 检查硬上限（SPSC 队列物理容量 8192）
    if (ingress_size_.load(std::memory_order_relaxed) >= 8192) {
        ALOG_ERROR("FrameProcessor", "SPSC ingress_queue physical limit reached (8192)!");
        return false;
    }

    // 检查逻辑上限（从配置读取）
    if (ingress_size_.load(std::memory_order_relaxed) >= max_ingress_queue_size_) {
        ALOG_WARN("FrameProcessor", "ingress_queue logical limit reached ({}), dropping frame ts={:.3f}", max_ingress_queue_size_, ts);
        return false;
    }
    
    FrameToProcess f;
    f.ts = ts;
    f.cloud = cloud;
    
    if (ingress_queue_.push(f)) {
        ingress_size_.fetch_add(1, std::memory_order_relaxed);
        ingress_not_empty_cv_.notify_one();
        return true;
    }
    
    return false;
}

void FrameProcessor::registerCallback(FrameReadyCallback cb) {
    callback_ = std::move(cb);
}

size_t FrameProcessor::ingressQueueSize() const {
    return ingress_size_.load(std::memory_order_relaxed);
}

size_t FrameProcessor::frameQueueSize() const {
    return frame_queue_size_.load(std::memory_order_relaxed);
}

void FrameProcessor::setShutdownFlag(const std::atomic<bool>* flag) {
    shutdown_flag_ = flag;
}

int FrameProcessor::processedFrameCount() const {
    return frame_count_.load();
}

int FrameProcessor::backpressureDropCount() const {
    return backpressure_drop_count_.load();
}

bool FrameProcessor::tryPopFrame(int timeout_ms, FrameToProcess& out_frame) {
    // 快速路径：如果有数据，直接弹出 (Wait-Free)
    if (frame_queue_.pop(out_frame)) {
        frame_queue_size_.fetch_sub(1, std::memory_order_relaxed);
        return true;
    }

    // 慢速路径：等待条件变量
    std::unique_lock<std::mutex> lock(frame_queue_not_empty_mutex_);
    bool has_data = frame_queue_not_empty_cv_.wait_for(
        lock, std::chrono::milliseconds(timeout_ms), [this] {
            return !frame_queue_.empty() || !running_.load();
        });

    if (!has_data || !running_.load()) {
        return false;
    }

    if (frame_queue_.pop(out_frame)) {
        frame_queue_size_.fetch_sub(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

void FrameProcessor::feederLoop() {
    ALOG_INFO("FrameProcessor", "feeder thread started (max_ingress={}, max_frame={})",
              max_ingress_queue_size_, max_frame_queue_size_);

    static std::atomic<int64_t> total_voxel_time_ms{0};
    static std::atomic<int> processed_count{0};

    while (running_.load(std::memory_order_acquire)) {
        if (shutdown_flag_ && shutdown_flag_->load(std::memory_order_acquire)) {
            break;
        }
        FrameToProcess f;
        bool has_data = false;

        // 快速尝试从 ingress_queue 弹出
        if (ingress_queue_.pop(f)) {
            ingress_size_.fetch_sub(1, std::memory_order_relaxed);
            has_data = true;
        } else {
            // 没数据，进入等待
            std::unique_lock<std::mutex> lock(ingress_not_empty_mutex_);
            bool woke = ingress_not_empty_cv_.wait_for(
                lock, std::chrono::milliseconds(500), [this]() {
                    return !running_.load(std::memory_order_acquire) || 
                           (shutdown_flag_ && shutdown_flag_->load(std::memory_order_acquire)) ||
                           !ingress_queue_.empty();
                });

            if (!running_.load(std::memory_order_acquire)) break;
            if (shutdown_flag_ && shutdown_flag_->load(std::memory_order_acquire)) break;
            if (!woke || !ingress_queue_.pop(f)) continue;
            ingress_size_.fetch_sub(1, std::memory_order_relaxed);
            has_data = true;
        }

        if (!has_data) continue;

        const int frame_seq = frame_count_.fetch_add(1) + 1;

        // 体素下采样
        if (f.cloud && !f.cloud->empty()) {
            float ds_res = submap_match_res_;
            if (ds_res <= 0) {
                ds_res = 0.5f;
            }

            const int voxel_timeout_ms = 5000;
            bool timed_out = false;
            auto t_start = std::chrono::steady_clock::now();

            try {
                f.cloud_ds = utils::voxelDownsampleWithTimeout(f.cloud, ds_res, voxel_timeout_ms, &timed_out);
                auto t_end = std::chrono::steady_clock::now();
                double voxel_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

                total_voxel_time_ms.fetch_add(static_cast<int64_t>(voxel_ms));
                processed_count.fetch_add(1);

                if (timed_out) {
                    ALOG_ERROR("FrameProcessor", "[COMPUTE_TIMEOUT] voxelDownsampleWithTimeout timed out seq={} pts={} limit_ms={}",
                               frame_seq, f.cloud->size(), voxel_timeout_ms);
                }
                if (frame_seq <= 5 || frame_seq % 100 == 0 || voxel_ms > 100.0 || timed_out) {
                    const int64_t avg_voxel = processed_count.load() > 0 ? total_voxel_time_ms.load() / processed_count.load() : 0;
                    ALOG_INFO("FrameProcessor", "[{}] voxel: pts={}->{} ms={:.1f} avg={}ms timeout={} ingress_q={} frame_q={}",
                             frame_seq, f.cloud->size(), f.cloud_ds ? f.cloud_ds->size() : 0,
                             voxel_ms, avg_voxel, timed_out ? 1 : 0,
                             ingress_size_.load(), frame_queue_size_.load());
                }
            } catch (const std::exception& e) {
                ALOG_ERROR("FrameProcessor", "voxelDownsample exception: {}", e.what());
                f.cloud_ds = nullptr;
            }
        }

        // 推送到处理队列 (frame_queue)
        // 注意：SPSC 满时，如果是离线建图，我们希望慢下来；如果是实时建图，且积压严重，则丢弃最老帧
        if (frame_queue_.push(f)) {
            frame_queue_size_.fetch_add(1, std::memory_order_relaxed);
            frame_queue_not_empty_cv_.notify_one();
        } else {
            // 队列满：丢弃当前最老的一帧，腾出空间
            FrameToProcess old_f;
            if (frame_queue_.pop(old_f)) {
                frame_queue_size_.fetch_sub(1, std::memory_order_relaxed);
                backpressure_drop_count_.fetch_add(1);
                if (frame_queue_.push(f)) {
                    frame_queue_size_.fetch_add(1, std::memory_order_relaxed);
                }
                ALOG_WARN("FrameProcessor", "frame_queue full, dropped oldest. total_drops={}", backpressure_drop_count_.load());
            }
        }
    }

    ALOG_INFO("FrameProcessor", "feeder thread exiting (total_processed={})", processed_count.load());
}

CloudXYZIPtr FrameProcessor::downsample(const CloudXYZIPtr& cloud, float resolution) {
    if (!cloud || cloud->empty()) {
        return nullptr;
    }
    return utils::voxelDownsample(cloud, resolution);
}

}  // namespace automap_pro
