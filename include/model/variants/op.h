#pragma once

#include "../problem.h"
#include "../node.h"
#include <cmath>
#include <vector>

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of the Orienteering Problem (OP).
 * Single vehicle, limited by a distance/cost budget.
 */
class OPProblem : public Problem {
public:
    OPProblem(std::string name, Distance budget)
        : name(std::move(name)), budget(budget) {}

    // Problem interface
    std::string get_name() const override { return name; }
    NodeId get_num_nodes() const override { return static_cast<NodeId>(nodes.size()); }
    NodeId get_source_depot() const override { return 0; }
    NodeId get_sink_depot() const override { return static_cast<NodeId>(nodes.size() - 1); }

    Reward get_reward(NodeId i) const override { return nodes[i].reward; }
    Distance get_distance(NodeId i, NodeId j) const override { return distance_matrix[i][j]; }

    // Constants
    Distance get_budget() const { return budget; }

    // Builders
    void add_node(const Node& node) { nodes.push_back(node); }
    
    // Finalize the problem by computing the distance matrix after all nodes are added
    void finalize() {
        size_t n = nodes.size();
        distance_matrix.assign(n, std::vector<Distance>(n, 0.0));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                double dx = nodes[i].x - nodes[j].x;
                double dy = nodes[i].y - nodes[j].y;
                distance_matrix[i][j] = std::sqrt(dx*dx + dy*dy);
            }
        }
    }

protected:
    std::string name;                                   // instance name
    Distance budget;                                    // maximum allowed travel distance/cost
    std::vector<Node> nodes;                            // list of nodes (including depots)
    std::vector<std::vector<Distance>> distance_matrix; // precomputed distance matrix for efficiency
};

} // namespace oplib::model::variants
