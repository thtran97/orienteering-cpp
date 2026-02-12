#pragma once

#include <cstdint>
#include <vector>

namespace oplib {

// Basic semantic types for clarity
using NodeId = int32_t;
using Reward = double;
using Distance = double;
using Time = double;

/**
 * @brief Defines how distances and times are handled in the model.
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
