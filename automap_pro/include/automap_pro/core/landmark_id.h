#pragma once
/**
 * @file landmark_id.h
 * @brief 进程内单调递增的地标实例 ID 分配（圆柱/平面等共用序列）。
 */
#include <cstdint>

namespace automap_pro {

/** @brief 分配下一个唯一 ID（原子递增）。 */
uint64_t allocateLandmarkId();

/** @brief 持久化加载后将序列抬升到 @f$\ge \texttt{min\_next\_id}@f$，避免与归档 ID 碰撞。 */
void seedLandmarkIdSeq(uint64_t min_next_id);

}  // namespace automap_pro
