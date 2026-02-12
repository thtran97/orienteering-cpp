#pragma once

#include "op.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of Time-Dependent Orienteering Problem (TDOP).
 */
class TDOPProblem : public OPProblem {
public:
    TDOPProblem(std::string name, Distance budget)
        : OPProblem(std::move(name), budget) {}

    bool is_time_dependent() const override { return true; }

    Time get_travel_time(NodeId i, NodeId j, Time departure_time) const override {
        int slot = find_time_slot(departure_time);
        int category = arc_categories[i][j];
        
        // Bounds checks to prevent Segfault
        if (category < 0 || category >= static_cast<int>(speed_matrix.size())) return get_distance(i, j);
        if (slot < 0 || slot >= static_cast<int>(speed_matrix[category].size())) return get_distance(i, j);
        
        double speed = speed_matrix[category][slot];
        if (speed <= 0.0) return get_distance(i, j); // Prevent division by zero
        
        return get_distance(i, j) / speed;
    }

    void set_speed_matrix(std::vector<std::vector<double>> matrix) {
        speed_matrix = std::move(matrix);
    }

    void set_arc_categories(std::vector<std::vector<int>> categories) {
        arc_categories = std::move(categories);
    }

    void set_time_slots(std::vector<Time> slots) {
        time_slot_boundaries = std::move(slots);
    }

private:
    int find_time_slot(Time t) const {
        for (size_t i = 0; i < time_slot_boundaries.size(); ++i) {
            if (t < time_slot_boundaries[i]) return static_cast<int>(i);
        }
        return static_cast<int>(time_slot_boundaries.size());
    }

    std::vector<Time> time_slot_boundaries;
    std::vector<std::vector<double>> speed_matrix; // TimeSlot x Category
    std::vector<std::vector<int>> arc_categories;  // Node x Node
};

} // namespace oplib::model::variants
