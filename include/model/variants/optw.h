#pragma once

#include "op.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of the Orienteering Problem with Time Windows (OPTW).
 * Single vehicle, time budget, and time window constraints.
 */
class OPTWProblem : public OPProblem {
public:
    OPTWProblem(std::string name, Time budget)
        : OPProblem(std::move(name), budget) {}

    bool has_time_windows() const override { return true; }
    
    const TimeWindow& get_time_window(NodeId i) const override {return nodes[i].tw;}
    Time get_service_time(NodeId i) const override {return nodes[i].service_time;}

    // Preprocessing to prune infeasible arcs and compute heuristic values for neighbors
    void preprocessing() override {
        size_t n = nodes.size();
        allowed_arcs.assign(n, std::vector<bool>(n, true));
        NodeId sink = get_sink_depot();
        double tmax = budget; // Use budget as tmax for TW variants

        for (size_t i = 0; i < n; ++i) {
            allowed_arcs[i][i] = false;
            nodes[i].neighbors.clear();
            if (static_cast<NodeId>(i) == sink) continue;

            const auto& tw_i = get_time_window(i);
            double service_i = get_service_time(i);

            for (size_t j = 0; j < n; ++j) {
                if (i == j) continue;

                Time travel_time_ij = get_distance(i, j);
                const auto& tw_j = get_time_window(j);
                
                bool feasible = true;
                
                // Arrive at j before closing?
                double early_depart_i = tw_i.opening + service_i;
                double arrival_j = early_depart_i + travel_time_ij;
                if (arrival_j > tw_j.closing) feasible = false;

                if (feasible) {
                    // Return to sink after j before tmax?
                    double departure_j = std::max(arrival_j, tw_j.opening) + get_service_time(j);
                    double arrival_sink = departure_j + get_distance(j, sink);
                    if (arrival_sink > tmax) feasible = false;
                }

                if (feasible) {
                    double heuristic = static_cast<double>(get_reward(j)) / (service_i + travel_time_ij + 1e-6);
                    nodes[i].neighbors.push_back({static_cast<NodeId>(j), heuristic});
                } else {
                    allowed_arcs[i][j] = false;
                }
            }
            std::sort(nodes[i].neighbors.rbegin(), nodes[i].neighbors.rend(),
                      [](const auto& a, const auto& b) { return a.second < b.second; });
        }
    }
};

} // namespace oplib::model::variants
