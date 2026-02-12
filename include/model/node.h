#pragma once

#include "../core/types.h"
#include <string>

namespace oplib::model {

/**
 * @brief Represents a node (customer or depot) in the problem.
 */
struct Node {
    NodeId id;
    double x = 0.0;
    double y = 0.0;
    Reward reward = 0.0;
    Time service_time = 0.0;
    TimeWindow tw = {0.0, 1e18};

    std::string to_string() const {
        return "Node " + std::to_string(id) + " (" + std::to_string(x) + "," + std::to_string(y) + ") R=" + std::to_string(reward);
    }
};

} // namespace oplib::model
