/**
 * @file landmark_id.cpp
 * @brief 全局原子序列 @f$\texttt{g\_landmark\_id\_seq}@f$ 的实现与 compare-exchange 播种。
 */
#include "automap_pro/core/landmark_id.h"

#include <algorithm>
#include <atomic>

namespace automap_pro {
namespace {

std::atomic<uint64_t> g_landmark_id_seq{1};

}  // namespace

uint64_t allocateLandmarkId() {
    return g_landmark_id_seq.fetch_add(1, std::memory_order_relaxed);
}

void seedLandmarkIdSeq(uint64_t min_next_id) {
    uint64_t cur = g_landmark_id_seq.load(std::memory_order_relaxed);
    while (cur < min_next_id) {
        if (g_landmark_id_seq.compare_exchange_weak(cur, min_next_id, std::memory_order_relaxed)) {
            return;
        }
    }
}

}  // namespace automap_pro
