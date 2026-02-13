#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

#include "model/problem.h"
#include "model/node.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of the Orienteering Problem (OP).
 * Single vehicle, limited by a time budget.
 */
class OPProblem : public Problem {
public:
    OPProblem(std::string name, Time budget)
        : name(std::move(name)), budget(budget) {}

    // Problem interface
    std::string get_name() const override { return name; }
    NodeId get_num_nodes() const override { return static_cast<NodeId>(nodes.size()); }
    NodeId get_source_depot() const override { return 0; }
    NodeId get_sink_depot() const override { return static_cast<NodeId>(nodes.size() - 1); }

    ScalingMode get_scaling_mode() const override { return scaling_mode; }
    double get_time_scale() const override { return time_scale; }
    
    void set_scaling(ScalingMode mode, double scale) {
        scaling_mode = mode;
        time_scale = scale;
    }

    Reward get_reward(NodeId i) const override { return nodes[i].reward; }
    Time get_distance(NodeId i, NodeId j) const override { return travel_time_matrix[i][j]; }

    // Constants
    Time get_budget() const override { return budget; } 

    // Builders
    void add_node(const Node& node) { nodes.push_back(node); }
    
    // Finalize the problem by computing the travel time matrix 
    void finalize() {
        size_t n = nodes.size();
        travel_time_matrix.assign(n, std::vector<Time>(n, 0.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                double dx = nodes[i].x - nodes[j].x;
                double dy = nodes[i].y - nodes[j].y;
                double raw_dist = std::sqrt(dx*dx + dy*dy);
                
                if (scaling_mode == ScalingMode::SCALED_INTEGER) {
                    travel_time_matrix[i][j] = std::floor(raw_dist * time_scale);
                } else {
                    travel_time_matrix[i][j] = raw_dist;
                }
            }
        }
    }

    // Preprocessing to prune infeasible arcs and compute heuristic values for neighbors
    virtual void preprocessing() {
        size_t n = nodes.size();
        allowed_arcs.assign(n, std::vector<bool>(n, true));
        NodeId source = get_source_depot();
        NodeId sink = get_sink_depot();

        for (size_t i = 0; i < n; ++i) {
            allowed_arcs[i][i] = false; // No self-loops
            nodes[i].neighbors.clear();
            
            if (static_cast<NodeId>(i) == sink) continue;

            for (size_t j = 0; j < n; ++j) {
                if (i == j) continue;
                
                // Basic budget pruning for OP
                bool feasible = true;
                Time travel_time_ij = get_distance(i, j);
                Time travel_time_to_sink = get_distance(j, sink);
                
                if (travel_time_ij + travel_time_to_sink > budget) {
                    feasible = false;
                }

                if (feasible) {
                    // Heuristic value: reward / travel_time
                    // Using service_time of i + travel_time of ij
                    double heuristic = static_cast<double>(get_reward(j)) / (get_service_time(i) + travel_time_ij + 1e-6);
                    nodes[i].neighbors.push_back({static_cast<NodeId>(j), heuristic});
                } else {
                    allowed_arcs[i][j] = false;
                }
            }

            // Sort neighbors by heuristic value descending
            std::sort(nodes[i].neighbors.begin(), nodes[i].neighbors.end(), 
                      [](const auto& a, const auto& b) { return a.second > b.second; });
        }
    }

    bool exists_arc(NodeId i, NodeId j) const {
        if (i < 0 || i >= static_cast<NodeId>(allowed_arcs.size())) return false;
        if (j < 0 || j >= static_cast<NodeId>(allowed_arcs[i].size())) return false;
        return allowed_arcs[i][j];
    }

protected:
    std::string name;                                      // instance name
    Time budget;                                           // maximum allowed time budget
    ScalingMode scaling_mode = ScalingMode::RAW;           // Scaling mode for time values (affects get_distance and heuristic calculations)
    double time_scale = 1.0;                               // Scale factor for time when using SCALED_INTEGER mode      
    std::vector<Node> nodes;                               // list of nodes (including depots)
    std::vector<std::vector<Time>> travel_time_matrix;     // precomputed travel time matrix for efficiency
    std::vector<std::vector<bool>> allowed_arcs;           // pruned arc matrix
};

} // namespace oplib::model::variants
