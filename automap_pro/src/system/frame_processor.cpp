#include "automap_pro/system/frame_processor.h"

#include "automap_pro/backend/incremental_optimizer.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"
#include "automap_pro/backend/optimizer.h"

#include <pcl/common/io.h>

namespace automap_pro {

FrameProcessor::FrameProcessor()
    : max_ingress_queue_size_(100)
    , max_frame_queue_size_(500)
    , shutdown_flag_(nullptr) {
}

FrameProcessor::~FrameProcessor() {
    stop();
}

void FrameProcessor::init(size_t max_ingress_queue_size, 
                         size_t max_frame_queue_size) {
    max_ingress_queue_size_ = max_ingress_queue_size;
    max_frame_queue_size_ = max_frame_queue_size;
}

void FrameProcessor::start() {
    if (running_.load()) {
        return;
    }
    running_.store(true);
    feeder_thread_ = std::thread([this]() {
#ifdef __linux__
        pthread_setname_np(pthread_self(), "frame_processor");
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
    frame_queue_cv_.notify_all();
    if (feeder_thread_.joinable()) {
        feeder_thread_.join();
    }
}

bool FrameProcessor::pushFrame(double ts, const CloudXYZIPtr& cloud) {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    
    if (ingress_queue_.size() >= max_ingress_queue_size_) {
        ALOG_WARN("FrameProcessor", "ingress_queue full, dropping frame ts=%.3f", ts);
        return false;
    }
    
    FrameToProcess f;
    f.ts = ts;
    f.cloud = cloud;
    ingress_queue_.push(std::move(f));
    ingress_not_empty_cv_.notify_one();
    
    return true;
}

void FrameProcessor::registerCallback(FrameReadyCallback cb) {
    callback_ = std::move(cb);
}

size_t FrameProcessor::ingressQueueSize() const {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    return ingress_queue_.size();
}

size_t FrameProcessor::frameQueueSize() const {
    std::lock_guard<std::mutex> lock(frame_queue_mutex_);
    return frame_queue_.size();
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
    std::unique_lock<std::mutex> lock(frame_queue_mutex_);
    bool has_data = frame_queue_cv_.wait_for(
        lock, std::chrono::milliseconds(timeout_ms), [this] {
            return !frame_queue_.empty() || !running_.load();
        });

    if (!has_data || frame_queue_.empty()) {
        return false;
    }

    out_frame = std::move(frame_queue_.front());
    frame_queue_.pop();
    frame_queue_not_full_cv_.notify_one();
    return true;
}

void FrameProcessor::feederLoop() {
    const std::atomic<bool>* shutdown = shutdown_flag_ ? shutdown_flag_ : &running_;

    ALOG_INFO("FrameProcessor", "feeder thread started (max_ingress=%zu, max_frame=%zu)",
              max_ingress_queue_size_, max_frame_queue_size_);

    static std::atomic<int64_t> total_voxel_time_ms{0};
    static std::atomic<int> processed_count{0};

    while (!shutdown->load(std::memory_order_acquire)) {
        FrameToProcess f;
        {
            std::unique_lock<std::mutex> lock(ingress_mutex_);
            const bool has_data = ingress_not_empty_cv_.wait_for(
                lock, std::chrono::milliseconds(500), [this, shutdown]() {
                    return shutdown->load(std::memory_order_acquire) || !ingress_queue_.empty();
                });

            if (shutdown->load(std::memory_order_acquire)) break;
            if (!has_data || ingress_queue_.empty()) continue;

            f = std::move(ingress_queue_.front());
            ingress_queue_.pop();
            ingress_not_full_cv_.notify_one();
        }

        const int frame_seq = frame_count_.fetch_add(1) + 1;

        // 体素下采样
        if (f.cloud && !f.cloud->empty()) {
            float ds_res = static_cast<float>(ConfigManager::instance().submapMatchRes());
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
                    ALOG_ERROR("FrameProcessor", "[COMPUTE_TIMEOUT] voxelDownsampleWithTimeout timed out seq=%d pts=%zu limit_ms=%d (建图时请重点关注)",
                               frame_seq, f.cloud->size(), voxel_timeout_ms);
                }
                if (frame_seq <= 5 || frame_seq % 100 == 0 || voxel_ms > 100.0 || timed_out) {
                    const int64_t avg_voxel = processed_count.load() > 0 ? total_voxel_time_ms.load() / processed_count.load() : 0;
                    ALOG_INFO("FrameProcessor", "[%d] voxel: pts=%zu->%zu ms=%.1f avg=%ldms timeout=%d queue_i=%zu q=%zu",
                             frame_seq, f.cloud->size(), f.cloud_ds ? f.cloud_ds->size() : 0,
                             voxel_ms, avg_voxel, timed_out ? 1 : 0,
                             ingress_queue_.size(), frame_queue_.size());
                }
            } catch (const std::exception& e) {
                ALOG_ERROR("FrameProcessor", "voxelDownsample exception: %s", e.what());
                f.cloud_ds = nullptr;
            }
        }

        // 推送到处理队列
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);

            int max_waits = ConfigManager::instance().backpressureMaxWaits();
            int wait_sec = ConfigManager::instance().backpressureWaitSec();
            if (max_waits <= 0) max_waits = 10;
            if (wait_sec <= 0) wait_sec = 1;

            const size_t queue_before = frame_queue_.size();
            int wait_count = 0;
            while (frame_queue_.size() >= max_frame_queue_size_ &&
                   !shutdown->load(std::memory_order_acquire)) {
                if (wait_count >= max_waits) {
                    backpressure_drop_count_.fetch_add(1);
                    ALOG_ERROR("FrameProcessor", "backpressure force drop, total=%d queue=%zu",
                              backpressure_drop_count_.load(), frame_queue_.size());
                    frame_queue_.pop();
                    break;
                }
                frame_queue_not_full_cv_.wait_for(lock, std::chrono::seconds(wait_sec));
                wait_count++;
            }

            if (!shutdown->load(std::memory_order_acquire)) {
                frame_queue_.push(std::move(f));
                frame_queue_cv_.notify_one();
                const size_t queue_after = frame_queue_.size();
                if (frame_seq <= 5 || frame_seq % 50 == 0) {
                    ALOG_INFO("FrameProcessor", "[%d] pushed queue=%zu->%zu", frame_seq, queue_before, queue_after);
                }
            }
        }
    }

    ALOG_INFO("FrameProcessor", "feeder thread exiting (total_processed=%d)", processed_count.load());
}

CloudXYZIPtr FrameProcessor::downsample(const CloudXYZIPtr& cloud, float resolution) {
    if (!cloud || cloud->empty()) {
        return nullptr;
    }
    return utils::voxelDownsample(cloud, resolution);
}

}  // namespace automap_pro
