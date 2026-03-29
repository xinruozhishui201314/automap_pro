#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <typeindex>
#include <vector>
#include <memory>
#include <any>
#include <future>
#include <thread>

namespace automap_pro::v3 {

/**
 * @brief 核心事件总线 (Core Event Bus)
 * 
 * 职责：
 * 1. 解耦各个模块的通信
 * 2. 支持强类型的发布/订阅
 * 3. 线程安全
 */
class EventBus {
public:
    using Ptr = std::shared_ptr<EventBus>;

    /**
     * @brief 订阅一个事件类型
     * @tparam TEvent 事件数据类型
     * @param handler 回调函数
     */
    template <typename TEvent>
    void subscribe(std::function<void(const TEvent&)> handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& handlers = handlers_[typeid(TEvent)];
        handlers.push_back([handler](const std::any& any_event) {
            handler(std::any_cast<const TEvent&>(any_event));
        });
    }

    /**
     * @brief 异步订阅（每事件 spawn 一线程并 detach，不阻塞 publish）
     * @note 不保证同类型事件的回调执行顺序；需严格 FIFO / 与 Registry 因果序时，应使用 subscribe + 模块内队列，
     *       由单 worker 顺序处理（参见 VisualizationModule）。
     */
    template <typename TEvent>
    void subscribeAsync(std::function<void(const TEvent&)> handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& handlers = handlers_[typeid(TEvent)];
        handlers.push_back([handler](const std::any& any_event) {
            // 异步分发必须不阻塞 publish：避免临时 future 析构导致同步等待。
            auto event_copy = std::any_cast<const TEvent&>(any_event);
            std::thread([handler, event_copy]() {
                handler(event_copy);
            }).detach();
        });
    }

    /**
     * @brief 发布一个事件
     * @tparam TEvent 事件数据类型
     * @param event 事件数据对象
     */
    template <typename TEvent>
    void publish(const TEvent& event) {
        std::vector<EventHandler> handlers_to_call;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = handlers_.find(typeid(TEvent));
            if (it != handlers_.end()) {
                handlers_to_call = it->second;
            }
        }
        for (auto& handler : handlers_to_call) {
            handler(event);
        }
    }

    /**
     * @brief 清除所有订阅者
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        handlers_.clear();
    }

private:
    using EventHandler = std::function<void(const std::any&)>;
    std::map<std::type_index, std::vector<EventHandler>> handlers_;
    std::mutex mutex_;
};

} // namespace automap_pro::v3
