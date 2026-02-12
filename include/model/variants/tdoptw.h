#pragma once

#include "optw.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of Time-Dependent Orienteering Problem with Time Windows (TDOPTW).
 */
class TDOPTWProblem : public OPTWProblem {
public:
    TDOPTWProblem(std::string name, Time tmax)
        : OPTWProblem(std::move(name), tmax) {}

    bool is_time_dependent() const override { return true; }

    Time get_travel_time(NodeId i, NodeId j, Time departure_time) const override {
        int slot = find_time_slot(departure_time);
        return transition_matrix[i][j][slot];
    }

    void set_transition_matrix(std::vector<std::vector<std::vector<Time>>> matrix) {
        transition_matrix = std::move(matrix);
    }

    void set_slot_duration(Time duration) {
        slot_duration = duration;
    }

    void set_start_time(Time start) {
        start_time = start;
    }

private:
    int find_time_slot(Time t) const {
        int slot = static_cast<int>((t - start_time) / slot_duration);
        if (slot < 0) return 0;
        if (slot >= static_cast<int>(transition_matrix[0][0].size())) 
            return static_cast<int>(transition_matrix[0][0].size()) - 1;
        return slot;
    }

    Time start_time = 0.0;
    Time slot_duration = 15.0; // Minutes
    std::vector<std::vector<std::vector<Time>>> transition_matrix; // Node x Node x TimeSlot
};

} // namespace oplib::model::variants
