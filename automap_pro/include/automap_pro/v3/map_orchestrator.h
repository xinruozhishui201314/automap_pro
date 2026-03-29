#pragma once

#include "automap_pro/v3/module_base.h"
#include <atomic>
#include <deque>
#include <unordered_map>

namespace automap_pro::v3 {

class MapOrchestrator : public ModuleBase {
public:
    using Ptr = std::shared_ptr<MapOrchestrator>;

    MapOrchestrator(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node);

    bool isIdle() const override;
    std::vector<std::pair<std::string, size_t>> queueDepths() const override;
    std::string idleDetail() const override;

protected:
    void run() override;

private:
    struct Item {
        std::string event_name;
        EventMeta meta;
    };

    void enqueueObserved(const char* event_name, const EventMeta& meta);
    void observeMeta(const char* event_name, const EventMeta& meta);
    void maybePublishAdvice(const char* event_name, const EventMeta& meta, const char* reason, bool takeover);

    rclcpp::Node::SharedPtr node_;
    mutable std::mutex queue_mutex_;
    std::deque<Item> queue_;
    std::unordered_map<std::string, uint64_t> last_producer_seq_;
    std::atomic<uint64_t> observed_total_{0};
    std::atomic<uint64_t> out_of_order_total_{0};
    std::atomic<uint64_t> stale_total_{0};
    std::atomic<uint64_t> advice_seq_{0};
    bool takeover_enabled_{false};
};

}  // namespace automap_pro::v3

