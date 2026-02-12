#pragma once

#include "optw.h"
#include "../../core/routing_utils.h"
#include <algorithm>

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
        
        // Interpolation logic: ceil(departure * a + b)
        if (!coeff_a.empty() && slot < static_cast<int>(coeff_a[i][j].size())) {
            return std::ceil(departure_time * coeff_a[i][j][slot]) + coeff_b[i][j][slot];
        }
        
        return transition_matrix[i][j][slot];
    }

    // Setters for time-dependent parameters
    void set_transition_matrix(std::vector<std::vector<std::vector<Time>>> matrix) {
        transition_matrix = std::move(matrix);
    }

    // Setters for interpolation coefficients
    void set_coefficients(std::vector<std::vector<std::vector<double>>> a, 
                          std::vector<std::vector<std::vector<double>>> b) {
        coeff_a = std::move(a);
        coeff_b = std::move(b);
    }

    // Setters for free flow travel times 
    void set_free_flow_matrix(std::vector<std::vector<Distance>> matrix) {
        free_flow_matrix = std::move(matrix);
    }

    void set_slot_duration(Time duration) {
        slot_duration = duration;
    }

    void set_start_time(Time start) {
        start_time = start;
    }

    // Override to compute latest departure time at i to arrive at j by arrival_time_at_j considering time-dependency
    Time estimate_departure_time(NodeId i, NodeId j, Time arrival_time_at_j) const override {
        if (!coeff_a.empty()) {
            return oplib::utils::RoutingUtils::compute_departure_time_td(*this, i, j, arrival_time_at_j, coeff_a[i][j], coeff_b[i][j], slot_duration);
        }
        return Problem::estimate_departure_time(i, j, arrival_time_at_j);
    }

private:
    
// Helper to find the time slot index for a given departure time
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
    
    // Piecewise linear interpolation coefficients
    std::vector<std::vector<std::vector<double>>> coeff_a;
    std::vector<std::vector<std::vector<double>>> coeff_b;

    // Optional free flow travel times
    std::vector<std::vector<Distance>> free_flow_matrix;
};

} // namespace oplib::model::variants
