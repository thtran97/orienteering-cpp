#pragma once

#include <cstdint>
#include <vector>

namespace oplib {

// Basic semantic types for clarity
using NodeId = int32_t;
using Reward = double;
using Time = double;  // Cost to travel from i to j; also available time budget

/**
 * @brief Defines how time values are scaled in the model.
 */
enum class ScalingMode {
    RAW,           // Use raw double-precision Euclidean distances
    SCALED_INTEGER // Use floor(raw * scale) to match existing benchmarks
};

// Common structures
struct TimeWindow {
    Time opening = 0.0;
    Time closing = 1e18; // Default to "infinity"

    bool is_within(Time t) const {
        return t >= opening && t <= closing;
    }
};

} // namespace oplib
