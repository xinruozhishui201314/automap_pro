#pragma once

#include "automap_pro/v3/module_base.h"
#include <atomic>
#include <cstdint>
#include <deque>
#include <unordered_map>

namespace automap_pro::v3 {

/**
 * @brief 动态点过滤模块
 *
 * 链路位置：
 * FrontEndModule -> SyncedFrameEvent -> DynamicFilterModule
 * -> FilteredFrameEventOptionalDs -> FilteredFrameEventRequiredDs -> MappingModule
 */
class DynamicFilterModule : public ModuleBase {
public:
    using Ptr = std::shared_ptr<DynamicFilterModule>;

    DynamicFilterModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node);
    bool isIdle() const override;
    std::vector<std::pair<std::string, size_t>> queueDepths() const override;
    std::string idleDetail() const override;

protected:
    void run() override;

private:
    struct VoxelKey {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            size_t h = 1469598103934665603ull;
            auto mix = [&h](int v) {
                h ^= static_cast<size_t>(v);
                h *= 1099511628211ull;
            };
            mix(k.x);
            mix(k.y);
            mix(k.z);
            return h;
        }
    };

    struct VoxelStat {
        uint32_t observations = 0;
        uint64_t last_seen_seq = 0;
        uint64_t lru_stamp = 0;
    };

    void processOne(const SyncedFrameEvent& in);
    FilteredFrameEventOptionalDs passthrough(const SyncedFrameEvent& in, FilterFallbackReason reason) const;
    FilteredFrameEventRequiredDs makeRequiredEvent(const FilteredFrameEventOptionalDs& in) const;
    VoxelKey voxelize(const pcl::PointXYZI& p) const;
    bool pointInRange(const pcl::PointXYZI& p) const;
    void logKpiIfNeeded();
    void touchVoxelLru(const VoxelKey& key, VoxelStat& stat);
    void enforceHardCapacity();
    void cleanupStaleVoxelsByAge();
    void applyFaultInjectionIfEnabled(FilteredFrameEventRequiredDs& ev);

    rclcpp::Node::SharedPtr node_;
    std::deque<SyncedFrameEvent> input_queue_;
    mutable std::mutex queue_mutex_;

    std::unordered_map<VoxelKey, VoxelStat, VoxelKeyHash> voxel_stats_;
    uint64_t seq_id_ = 0;
    uint64_t lru_tick_ = 0;
    std::deque<std::pair<VoxelKey, uint64_t>> lru_queue_;

    bool enabled_ = false;
    bool shadow_mode_ = true;
    bool drop_if_queue_full_ = true;
    int queue_max_size_ = 512;
    int min_static_observations_ = 2;
    int voxel_capacity_ = 200000;
    double voxel_size_ = 0.30;
    double min_range_m_ = 0.2;
    double max_range_m_ = 80.0;
    bool fault_injection_enabled_ = false;
    std::string fault_injection_mode_ = "none";
    int fault_injection_every_n_frames_ = 0;
    int fault_injection_max_count_ = 0;
    std::atomic<uint64_t> fault_injection_done_count_{0};

    // KPI counters
    std::atomic<uint64_t> kpi_processed_total_{0};
    std::atomic<uint64_t> kpi_filtered_applied_total_{0};
    std::atomic<uint64_t> kpi_shadow_passthrough_total_{0};
    std::atomic<uint64_t> kpi_queue_drop_total_{0};
    std::atomic<uint64_t> kpi_fallback_total_{0};
    std::atomic<uint64_t> kpi_contract_drop_total_{0};
    std::atomic<uint64_t> kpi_input_points_total_{0};
    std::atomic<uint64_t> kpi_output_points_total_{0};
    std::atomic<double> kpi_latency_sum_ms_{0.0};
    std::atomic<double> kpi_latency_max_ms_{0.0};
};

} // namespace automap_pro::v3
