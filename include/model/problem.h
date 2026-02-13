#pragma once

#include <string>
#include <vector>
#include <memory>

#include "core/types.h"

namespace oplib::model {

/**
 * @brief Abstract base class for all Orienteering Problem variants.
 * 
 * This class defines the common interface that solvers use to interact with
 * any problem instance, regardless of its specific constraints (Time Windows, 
 * Team/Multi-vehicle, Time-dependency, etc.).
 */
class Problem {
public:
    virtual ~Problem() = default;

    // Basic Structure
    virtual std::string get_name() const = 0;
    virtual NodeId get_num_nodes() const = 0;
    virtual NodeId get_source_depot() const = 0;
    virtual NodeId get_sink_depot() const = 0;
    virtual int get_num_vehicles() const { return 1; } 

    // Scaling and Precision
    virtual ScalingMode get_scaling_mode() const { return ScalingMode::RAW; }
    virtual double get_time_scale() const { return 1.0; }
    bool is_scaled() const { return get_scaling_mode() == ScalingMode::SCALED_INTEGER; }

    // Abstract check for problem-specific constraints
    // This allows checkers/solvers to work with base types
    virtual bool has_time_windows() const { return false; }  // each node has a time window
    virtual bool is_time_dependent() const { return false; } // travel times depend on departure time
    virtual bool is_multi_vehicle() const { return false; }  // multiple routes/vehicles allowed
    
    // Reward and distance
    virtual Reward get_reward(NodeId i) const = 0;
    virtual Distance get_distance(NodeId i, NodeId j) const = 0;

    // Constraint budgets (for constructive heuristics and feasibility checking)
    // Returns the maximum budget allowed for distance/time. Default is unconstrained.
    virtual Distance get_budget() const { return 1e18; }
    
    // Time budget for time-window constrained problems (available time to complete tour)
    virtual Time get_time_budget() const { return 1e18; }
    
    // Constraints & Metadata (only relevant for problems with time windows)
    virtual const TimeWindow& get_time_window(NodeId i) const { return default_time_window; }
    virtual Time get_service_time(NodeId i) const { return 0.0; }
    
    // Logic utilities
    // Default is time-independent, which does not use departure_time. Time-dependent variants should override this.
    virtual Time get_travel_time(NodeId i, NodeId j, Time /*departure_time*/ = 0.0) const {
        return get_distance(i, j); 
    }

    // Inverse of travel time: find latest departure at i to arrive at j by arrival_time_at_j
    virtual Time estimate_departure_time(NodeId i, NodeId j, Time arrival_time_at_j) const {
        return arrival_time_at_j - get_travel_time(i, j);
    }

private:
    static constexpr TimeWindow default_time_window = {0.0, 1e18}; // Default to "infinity"
};

} // namespace oplib::model
