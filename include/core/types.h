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

/*
* @brief Represents a time window with opening and closing times.
 * 
 * Used for problems with time window constraints (OPTW, TOPTW).
 */
struct TimeWindow {
    Time opening = 0.0;
    Time closing = 1e18; // Default to "infinity"

    bool is_within(Time t) const {
        return t >= opening && t <= closing;
    }
};

/**
 * @brief Insertion strategy modes for the greedy solver.
 * 
 * CUSTOMER_WISE: for each customer, evaluate all possible insertions across all vehicles and positions, and pick the best one.
 * 
 * VEHICLE_WISE: for each vehicle, evaluate all possible insertions across all customers and positions, and pick the best one.
 */
enum class InsertionStrategyMode {
    CUSTOMER_WISE,
    VEHICLE_WISE
};

/**
 * @brief Evaluation strategies for scoring insertion moves in the greedy solver.
 */
enum class MoveEvaluatorType {
    REWARD,
    REWARD_PER_TIME,
    REWARD_PER_DISTANCE,
    SQUARED_REWARD_PER_DISTANCE
};

} // namespace oplib
