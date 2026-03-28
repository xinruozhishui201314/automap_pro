#pragma once

#include <cstdint>

namespace automap_pro {

/// Monotonic process-wide landmark instance id (trees and planes share the same sequence).
uint64_t allocateLandmarkId();

/// After loading persisted landmarks, bump the sequence so new ids never collide with archived values.
void seedLandmarkIdSeq(uint64_t min_next_id);

}  // namespace automap_pro
