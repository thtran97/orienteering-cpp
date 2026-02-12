#pragma once

#include "types.h"
#include "../model/problem.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace oplib::utils {

/**
 * @brief Utility functions for routing calculations.
 */
class RoutingUtils {
public:
    /**
     * @brief Computes transition time based on interpolation coefficients.
     * So how it works? 
     * 
     * For a given departure time, we determine the corresponding time slot and apply the formula: 
     * $travel_time = ceil(departure_time * coeff_a[slot]) + coeff_b[slot]$
     * 
     * This allows us to model time-dependent travel times based on piecewise linear functions defined by the coefficients.
     */
    static Time compute_travel_time_td(Time departure_time, 
                                      int slot,
                                      const std::vector<double>& coeff_a,
                                      const std::vector<double>& coeff_b) {
        if (slot < 0 || slot >= static_cast<int>(coeff_a.size())) {
            return 0.0;
        }
        return std::ceil(departure_time * coeff_a[slot]) + coeff_b[slot];
    }

    /**
     * @brief Computes latest departure time at i to arrive at j by arrival_time_at_j.
     * 
     * This function performs a backward search through time slots 
     * to find the latest feasible departure time at node i that allows arrival at node j by the specified time, considering time windows and service times. 
     * It assumes the FIFO property holds for travel times.
     * 
     * Intuitively, "Given that I need to be at node j by arrival_time_at_j, when is the latest I can leave node i, considering the time-dependent travel times and constraints at node i?"
     */
    static Time compute_departure_time_td(const model::Problem& problem,
                                          NodeId i, NodeId j,
                                          Time arrival_time_at_j,
                                          const std::vector<double>& coeff_a,
                                          const std::vector<double>& coeff_b,
                                          Time slot_duration) {
        
        const auto& tw_i = problem.get_time_window(i);
        Time latest_depart_at_i = tw_i.closing + problem.get_service_time(i); 
        
        // Initial feasibility check
        if (latest_depart_at_i + problem.get_travel_time(i, j, latest_depart_at_i) <= arrival_time_at_j) {
            return latest_depart_at_i;
        }

        // Search backward through time slots
        // This assumes FIFO property holds
        int slot_begin = static_cast<int>(std::floor(tw_i.opening / slot_duration));
        int slot_end = static_cast<int>(std::floor(latest_depart_at_i / slot_duration));

        for (int t = slot_end; t >= slot_begin; --t) {
            if (t < 0 || t >= static_cast<int>(coeff_a.size())) continue;
            
            // Expected arrival if departing at start of slot t
            Time start_of_slot = t * slot_duration;
            Time arrival_at_start = start_of_slot + compute_travel_time_td(start_of_slot, t, coeff_a, coeff_b);
            
            if (arrival_at_start < arrival_time_at_j) {
                // If we are here, the latest departure is within this slot
                // arrival = depart * (1 + a) + b  =>  depart = (arrival - b) / (1 + a)
                Time depart = (arrival_time_at_j - coeff_b[t]) / (1.0 + coeff_a[t]);
                return std::min(depart, latest_depart_at_i);
            }
        }

        return -1.0; // Infeasible
    }
};

} // namespace oplib::utils
