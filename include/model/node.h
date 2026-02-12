#pragma once

#include "../core/types.h"
#include <string>

namespace oplib::model {

/**
 * @brief Represents a node (customer or depot) in the problem.
 */
struct Node {
    NodeId id = -1;
    double x = 0.0;
    double y = 0.0;
    Reward reward = 0.0;
    Time service_time = 0.0;
    TimeWindow tw;

    // Preprocessing data: list of (neighbor_id, heuristic_value)
    // Used by constructive heuristics and local search.
    std::vector<std::pair<NodeId, double>> neighbors;

    std::string to_string() const {
        return "Node " + std::to_string(id) + " (" + std::to_string(x) + "," + std::to_string(y) + ") R=" + std::to_string(reward);
    }
};

} // namespace oplib::model
