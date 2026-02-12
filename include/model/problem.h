#pragma once

#include "../core/types.h"
#include <string>
#include <vector>
#include <memory>

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

    // Scaling and Precision
    virtual ScalingMode get_scaling_mode() const { return ScalingMode::RAW; }
    virtual double get_time_scale() const { return 1.0; }
    bool is_scaled() const { return get_scaling_mode() == ScalingMode::SCALED_INTEGER; }

    // Rewards and Costs
    virtual Reward get_reward(NodeId i) const = 0;
    virtual Distance get_distance(NodeId i, NodeId j) const = 0;

    // Constraints & Metadata
    virtual const TimeWindow& get_time_window(NodeId i) const = 0;
    virtual Time get_service_time(NodeId i) const = 0;
    
    // Abstract check for problem-specific constraints
    // This allows checkers/solvers to work with base types
    virtual bool has_time_windows() const { return false; }  // each node has a time window
    virtual bool is_time_dependent() const { return false; } // travel times depend on departure time
    virtual bool is_multi_vehicle() const { return false; }  // multiple routes/vehicles allowed

    // Logic utilities
    // Default is time-independent, which does not use departure_time. Time-dependent variants should override this.
    virtual Time get_travel_time(NodeId i, NodeId j, Time /*departure_time*/ = 0.0) const {
        return get_distance(i, j); 
    }

    // Inverse of travel time: find latest departure at i to arrive at j by arrival_time_at_j
    virtual Time estimate_departure_time(NodeId i, NodeId j, Time arrival_time_at_j) const {
        return arrival_time_at_j - get_travel_time(i, j);
    }
};

} // namespace oplib::model
