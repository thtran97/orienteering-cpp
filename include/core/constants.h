#pragma once

#include <climits>

#include "types.h"

namespace oplib::constants {

// Numeric Constants
inline constexpr NodeId INVALID_NODE = -1;
inline constexpr Reward UNSET_REWARD = -1.0;
inline constexpr Distance INF_DISTANCE = 1e18; // Use a large double instead of INT_MAX for doubles
inline constexpr Time INF_TIME = 1e18;

// Default Algorithm Parameters
inline constexpr int DEFAULT_SEED = 42;
inline constexpr int DEFAULT_TIMELIMIT_SECONDS = 60;
inline constexpr int DEFAULT_MAX_ITERATIONS = 1000;
inline constexpr int DEFAULT_RCL_SIZE = 5;
inline constexpr double DEFAULT_REMOVAL_RATIO = 0.4;
inline constexpr int DEFAULT_ALPHA = 2;
inline constexpr int MAX_NB_VEHICLES = 10;

} // namespace oplib::constants
