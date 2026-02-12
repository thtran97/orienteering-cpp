#pragma once

#include "op.h"
#include "../../core/routing_utils.h"
#include <algorithm>

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of Time-Dependent Orienteering Problem (TDOP).
 */
class TDOPProblem : public OPProblem {
public:
    TDOPProblem(std::string name, Distance budget)
        : OPProblem(std::move(name), budget) {}

    bool is_time_dependent() const override { return true; }

    // Override to compute time-dependent travel time based on departure time and interpolation coefficients
    Time get_travel_time(NodeId i, NodeId j, Time departure_time) const override {
        int slot = find_time_slot(departure_time);
        int category = arc_categories[i][j];
        
        // Bounds checks to prevent Segfault
        if (category < 0 || category >= static_cast<int>(speed_matrix.size())) return get_distance(i, j);
        if (slot < 0 || slot >= static_cast<int>(speed_matrix[category].size())) return get_distance(i, j);
        
        // Interpolation logic: ceil(departure * a + b)
        if (!coeff_a.empty() && slot < static_cast<int>(coeff_a[i][j].size())) {
            return std::ceil(departure_time * coeff_a[i][j][slot]) + coeff_b[i][j][slot];
        }

        double speed = speed_matrix[category][slot];
        if (speed <= 0.0) return get_distance(i, j); // Prevent division by zero
        
        return get_distance(i, j) / speed;
    }

    // Setters for time-dependent parameters
    void set_coefficients(std::vector<std::vector<std::vector<double>>> a, 
                          std::vector<std::vector<std::vector<double>>> b) {
        coeff_a = std::move(a);
        coeff_b = std::move(b);
    }

    // Setters for speed matrix
    void set_speed_matrix(std::vector<std::vector<double>> matrix) {
        speed_matrix = std::move(matrix);
    }

    // Setters for arc categories
    void set_arc_categories(std::vector<std::vector<int>> categories) {
        arc_categories = std::move(categories);
    }

    // Setters for time slot boundaries
    void set_time_slots(std::vector<Time> slots) {
        time_slot_boundaries = std::move(slots);
    }

    // Override to compute latest departure time at i to arrive at j by arrival_time_at_j considering time-dependency
    Time estimate_departure_time(NodeId i, NodeId j, Time arrival_time_at_j) const override {
        if (!coeff_a.empty() && !time_slot_boundaries.empty()) {
            double slot_duration = time_slot_boundaries[0]; // Assuming uniform slots for now or search
            // If not uniform, RoutingUtils needs to be more complex.
            // For now, just use the backward search.
            return oplib::utils::RoutingUtils::compute_departure_time_td(*this, i, j, arrival_time_at_j, coeff_a[i][j], coeff_b[i][j], slot_duration);
        }
        return Problem::estimate_departure_time(i, j, arrival_time_at_j);
    }

private:

    // Helper to find the time slot index for a given departure time
    int find_time_slot(Time t) const {
        for (size_t i = 0; i < time_slot_boundaries.size(); ++i) {
            if (t < time_slot_boundaries[i]) return static_cast<int>(i);
        }
        return static_cast<int>(time_slot_boundaries.size());
    }

    std::vector<Time> time_slot_boundaries;         // Time boundaries for slots
    std::vector<std::vector<double>> speed_matrix;  // Category x TimeSlot
    std::vector<std::vector<int>> arc_categories;   // Node x Node

    // Piecewise linear interpolation coefficients (Node x Node x TimeSlot)
    std::vector<std::vector<std::vector<double>>> coeff_a;
    std::vector<std::vector<std::vector<double>>> coeff_b;
};

} // namespace oplib::model::variants
