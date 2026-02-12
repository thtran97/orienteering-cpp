#pragma once

#include <cstdint>
#include <vector>

namespace oplib {

// Basic semantic types for clarity
using NodeId = int32_t;
using Reward = double;
using Distance = double;
using Time = double;

// Common structures
struct TimeWindow {
    Time opening;
    Time closing;

    bool is_within(Time t) const {
        return t >= opening && t <= closing;
    }
};

} // namespace oplib
