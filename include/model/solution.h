#pragma once

#include <vector>
#include <numeric>

#include "core/types.h"

namespace oplib::model {

/**
 * @brief Represents a solution to an Orienteering Problem.
 * 
 * A solution consists of one or more routes (paths) taken by vehicles.
 * Each route is a sequence of NodeIds.
 */
class Solution {
public:
    Solution() = default;
    explicit Solution(int num_vehicles) : routes(num_vehicles) {}

    // ==== Data Access ====
    const std::vector<std::vector<NodeId>>& get_routes() const { return routes; }
    std::vector<std::vector<NodeId>>& get_routes() { return routes; }
    
    const std::vector<NodeId>& get_route(int vehicle_idx) const { return routes[vehicle_idx]; }
    std::vector<NodeId>& get_route(int vehicle_idx) { return routes[vehicle_idx]; }

    int get_num_vehicles() const { return static_cast<int>(routes.size()); }

    // ==== Solution Metadata ====
    Reward total_reward = 0.0;
    Time total_time = 0.0;
    Distance total_distance = 0.0;

    // ==== Utility ====

    // Clears all routes and resets metadata
    void clear() {
        for (auto& route : routes) {
            route.clear();
        }
        total_reward = 0.0;
        total_time = 0.0;
        total_distance = 0.0;
    }

    // Checks if all routes are empty
    bool is_empty() const {
        for (const auto& route : routes) {
            if (!route.empty()) return false;
        }
        return true;
    }
    
    // Insert a node into a specific route at a given position
    void insert_node(int vehicle_idx, int pos, NodeId node_id) {
        routes[vehicle_idx].insert(routes[vehicle_idx].begin() + pos, node_id);
    }

    // Remove a node from a specific route at a given position
    void remove_node(int vehicle_idx, int pos) {
        routes[vehicle_idx].erase(routes[vehicle_idx].begin() + pos);
    }

private:
    std::vector<std::vector<NodeId>> routes; // record of routes for each vehicle, where each route is a sequence of NodeIds
};

} // namespace oplib::model
