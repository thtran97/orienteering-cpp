#pragma once

#include <vector>
#include <memory>
#include "model/problem.h"
#include "model/solution.h"
#include "core/types.h"

namespace oplib::solver::constructive {

using oplib::model::Problem;
using oplib::model::Solution;

/**
 * @brief Result of feasibility checking for an insertion move.
 * 
 * Captures whether the move is feasible and the computed costs (travel time and actual time)
 * that result from the insertion. These costs are needed for move evaluation.
 */
struct FeasibilityResult {
    bool feasible = false;
    Time arrival_time_at_customer = 0.0;      // arrival time at the inserted customer
    Time departure_time_from_customer = 0.0;  // departure time after service at customer
    Time travel_time_cost = 0.0;              // additional travel time incurred by insertion
};

/**
 * @brief Abstract base class for checking feasibility of insertion moves.
 * 
 * Different problem variants have different feasibility constraints:
 * - Distance variants (OP, TOP): check if total distance stays within budget
 * - Time-window variants (OPTW, TOPTW): check if arrival times respect time windows
 * - Time-dependent variants (TDOP, TDOPTW): use time-dependent travel times in feasibility check
 * 
 * This abstraction allows solvers to work with different constraint types
 * polymorphically without explicit casting or branching on problem type.
 */
class FeasibilityChecker {
public:
    virtual ~FeasibilityChecker() = default;

    /**
     * @brief Check if inserting a customer at a given position in a route is feasible.
     * 
     * @param problem The problem instance
     * @param solution Current solution state
     * @param vehicle Vehicle index
     * @param customer Customer node ID to insert
     * @param position Position in the route where to insert (between route[position-1] and route[position])
     * @param arrival_times Arrival times at each position in current route
     * @param departure_times Departure times at each position in current route
     * @param cumulative_time Current cumulative travel time of the route
     * @return FeasibilityResult containing feasibility status and computed costs
     */
    virtual FeasibilityResult check_feasibility(
        const Problem& problem,
        const Solution& solution,
        int vehicle,
        NodeId customer,
        int position,
        const std::vector<Time>& arrival_times,
        const std::vector<Time>& departure_times,
        Time cumulative_time
    ) const = 0;

    /**
     * @brief Get a human-readable name for this checker (for logging/debugging).
     */
    virtual std::string get_name() const = 0;
};

/**
 * @brief Feasibility checker for time-constrained problems (OP, TOP).
 * 
 * Checks if the total travel time of the route stays within the problem's time budget.
 * Does not validate time windows (since these variants don't have them).
 */
class DistanceFeasibilityChecker : public FeasibilityChecker {
public:
    FeasibilityResult check_feasibility(
        const Problem& problem,
        const Solution& solution,
        int vehicle,
        NodeId customer,
        int position,
        const std::vector<Time>& arrival_times,
        const std::vector<Time>& departure_times,
        Time cumulative_time
    ) const override;

    std::string get_name() const override { return "DistanceFeasibilityChecker"; }
};

/**
 * @brief Feasibility checker for time-window constrained problems (OPTW, TOPTW).
 * 
 * Checks both:
 * - Total time stays within budget
 * - Arrival times respect time windows at all nodes in the route
 * - Service can be completed within the time windows of all affected nodes
 */
class TimeWindowFeasibilityChecker : public FeasibilityChecker {
public:
    FeasibilityResult check_feasibility(
        const Problem& problem,
        const Solution& solution,
        int vehicle,
        NodeId customer,
        int position,
        const std::vector<Time>& arrival_times,
        const std::vector<Time>& departure_times,
        Time cumulative_time
    ) const override;

    std::string get_name() const override { return "TimeWindowFeasibilityChecker"; }

protected:
    /**
     * @brief Compute arrival and departure times for a given edge.
     * 
     * @param problem Problem instance
     * @param prev_node Previous node in route
     * @param current_node Current node to visit
     * @param departure_from_prev Departure time from previous node
     * @return Pair of (arrival_time_at_current, departure_time_from_current)
     */
    virtual std::pair<Time, Time> compute_times(
        const Problem& problem,
        NodeId prev_node,
        NodeId current_node,
        Time departure_from_prev
    ) const;
};

/**
 * @brief Feasibility checker for time-dependent variants (TDOPTW).
 * 
 * Extends the time-window checker to use time-dependent travel times
 * that depend on the departure time from the origin node.
 */
class TimeDependentFeasibilityChecker : public TimeWindowFeasibilityChecker {
public:
    std::string get_name() const override { return "TimeDependentFeasibilityChecker"; }

protected:
    std::pair<Time, Time> compute_times(
        const Problem& problem,
        NodeId prev_node,
        NodeId current_node,
        Time departure_from_prev
    ) const override;
};

} // namespace oplib::solver::constructive
