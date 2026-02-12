#pragma once

#include "op.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of the Orienteering Problem with Time Windows (OPTW).
 * Single vehicle, distance budget, and time window constraints.
 */
class OPTWProblem : public OPProblem {
public:
    OPTWProblem(std::string name, Distance budget)
        : OPProblem(std::move(name), budget) {}

    bool has_time_windows() const override { return true; }
    
    const TimeWindow& get_time_window(NodeId i) const {
        return nodes[i].tw;
    }

    Time get_service_time(NodeId i) const {
        return nodes[i].service_time;
    }
};

} // namespace oplib::model::variants
