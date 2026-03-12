#pragma once

#include <memory>
#include <set>
#include "solver/solver.h"
#include "solver/constructive/move_evaluator.h"

namespace oplib::solver::constructive {


/**
 * @brief Configuration parameters specific to the greedy solver.
 * 
 * Extends the base SolverConfig with greedy-specific options for
 * insertion strategy, move evaluation, and caching.
 */
struct GreedySolverConfig : public SolverConfig {
    MoveEvaluatorType move_evaluator = MoveEvaluatorType::REWARD;
};

/**
 * @brief Tracks time information for a single vehicle's route.
 * 
 * Maintains arrival/departure times at each position and cumulative travel time.
 * Used to efficiently check feasibility and compute insertion costs incrementally.
 */
struct RouteContext {
    std::vector<Time> arrival_times;    // arrival_times[i] = arrival at route[i]
    std::vector<Time> departure_times;  // departure_times[i] = departure from route[i]
    std::vector<Time> max_shift;        // max_shift[i] = maximum time shift allowed at position i without violating time windows of subsequent nodes 
    Time cumulative_time;               // sum of travel time on current route
    
    RouteContext() : cumulative_time(0.0) {} // default constructor initializes cumulative_time to 0
};

/**
 * @brief Represents a candidate insertion move.
 * 
 * Captures the customer to insert, vehicle to insert into, position in route,
 * computed costs, and feasibility status.
 */
struct InsertionMove {
    // -- Identifiers for the move
    NodeId customer = -1;
    int vehicle = -1;
    int position = -1;
    // -- Evaluation and feasibility results
    bool feasible = false;
    double score = -1.0;            // score computed by move evaluator (higher is better)
    Time time_shift = 0.0;          // additional travel time incurred
    Time arrival_time = 0.0;        // computed arrival at customer
    Time departure_time = 0.0;      // computed departure from customer
};

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
 * @brief Cache of infeasible (customer, vehicle, position) triplets.
 * 
 * Avoids re-evaluating insertion feasibility for triplets that have already
 * been found infeasible. Cache is updated when a customer is inserted.
 */
class InfeasibilityCache {
public:
    /**
     * @brief Check if an insertion is cached as infeasible.
     */
    bool is_infeasible(NodeId customer, int vehicle, int position) const {
        return infeasible_triplets.count({customer, vehicle, position}) > 0;
    }

    /**
     * @brief Mark an insertion as infeasible.
     */
    void mark_infeasible(NodeId customer, int vehicle, int position) {
        infeasible_triplets.insert({customer, vehicle, position});
    }

    /**
     * @brief Clear all cache entries for a given customer.
     * 
     * Called when a customer is inserted into a route, invalidating all
     * cached (customer, *, *) triplets since the route structure changed.
     */
    void invalidate_customer(NodeId customer) {
        // Remove all triplets with this customer
        auto it = infeasible_triplets.begin();
        while (it != infeasible_triplets.end()) {
            if (std::get<0>(*it) == customer) {
                it = infeasible_triplets.erase(it);
            } else {
                ++it;
            }
        }
    }

    /**
     * @brief Clear the entire cache.
     */
    void clear() {
        infeasible_triplets.clear();
    }

private:
    std::set<std::tuple<NodeId, int, int>> infeasible_triplets;  // (customer, vehicle, position)
};

/**
 * @brief Modular greedy construction heuristic for all OP variants.
 * 
 * This heuristic builds routes by iteratively inserting the "best" available 
 * customer into current routes until no more feasible insertions exist.
 * 
 * Variant supports:
 * - OP: Orienteering Problem (single vehicle, distance budget)
 * - OPTW: OP with Time Windows
 * - TOP: Team Orienteering Problem (multiple vehicles)
 * - TOPTW: Team OP with Time Windows
 * - TDOP: Time-Dependent OP
 * - TDOPTW: Time-Dependent OP with Time Windows
 */
class GreedySolver : public Solver {
public:
    /**
     * @brief Constructor with dependency injection.
     * 
     * @param strategy_mode Insertion strategy (CUSTOMER_WISE or VEHICLE_WISE)
     */
    GreedySolver(){};

    std::string get_name() const override { return "GreedySolver"; }
    
    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override;
    
    /**
     * @brief Solve with greedy-specific configuration.
     * 
     * Preferred method when using the greedy solver directly, as it provides
     * access to all greedy-specific options.
     */
    model::Solution solve(const model::Problem& problem, const GreedySolverConfig& config);

    /**
     * @brief Continue greedy construction from a partial (pre-filled) solution.
     * 
     * Reconstructs RouteContext from the existing routes, marks those nodes as
     * visited, then continues the standard greedy insertion loop.
     * Used by LNS repair to re-fill a solution after a destroy step.
     */
    model::Solution solve_from_partial(
        const model::Problem& problem,
        model::Solution partial_solution,
        const GreedySolverConfig& config
    );

protected:
    std::unique_ptr<MoveEvaluator> move_evaluator;
    InfeasibilityCache infeasibility_cache;

    /**
     * @brief Find the best insertion move across all unvisited customers and route positions.
     * 
     * Evaluates all feasible (vehicle, customer, position) triplets and returns the one
     * with the highest score according to the move evaluator.
     */
    InsertionMove find_best_insertion(
        const model::Problem& problem, 
        const model::Solution& solution, 
        const std::vector<bool>& visited,
        const std::vector<RouteContext>& route_contexts
    );

    /**
     * @brief Evaluate a single insertion candidate (customer, vehicle, position) triplet.
     * 
     * Checks cache, feasibility, computes score, and updates best_move if this candidate
     * is better than the current best. Helper method to eliminate code duplication between
     * different insertion strategies.
     */
    void evaluate_insertion_candidate(
        InsertionMove& best_move,
        NodeId c,
        int v,
        int pos,
        const model::Problem& problem,
        const model::Solution& solution,
        const std::vector<RouteContext>& route_contexts
    );

public:

    /**
     * @brief Check if inserting a customer at a given position in a route is feasible using max_shift.
     * 
     * Uses the max_shift from the route context to determine feasibility:
     * - max_shift[pos-1] tracks how much time delay position pos-1 can tolerate
     * - If insertion causes time_shift <= max_shift[pos-1], it's feasible
     * - Computes exact arrival/departure times at the inserted customer
     * 
     * @return FeasibilityResult with feasibility status and insertion times
     */
    FeasibilityResult check_insertion_feasible(
        const model::Problem& problem,
        const model::Solution& solution,
        int vehicle,
        NodeId customer,
        int position,
        const std::vector<RouteContext>& route_contexts
    ) const;

    /**
     * @brief Compute arrival and departure times for inserting a customer between two nodes.
     * 
     * @return Pair of (arrival_time_at_customer, departure_time_from_customer)
     */
    std::pair<Time, Time> compute_insertion_times(
        const model::Problem& problem,
        NodeId prev,
        NodeId customer,
        Time departure_from_prev
    ) const;

    /**
     * @brief Apply an insertion move to the solution.
     * 
     * Updates route, arrival/departure times, cumulative distance, and visited flags.
     * Also invalidates infeasibility cache entries for the inserted customer.
     */
    void apply_insertion(
        const model::Problem& problem,
        model::Solution& solution,
        const InsertionMove& move,
        std::vector<RouteContext>& route_contexts,
        std::vector<bool>& visited
    );

    /**
     * @brief Find all feasible insertion candidates for the current solution.
     * 
     * Generates all (customer, vehicle, position) triplets and evaluates their feasibility.
     * Returns all feasible moves sorted by score (highest first).
     */
    std::vector<InsertionMove> find_all_feasible_insertions(
        const model::Problem& problem,
        const model::Solution& solution,
        const std::vector<bool>& visited,
        const std::vector<RouteContext>& route_contexts
    );

    /**
     * @brief Rebuild RouteContext from an existing route.
     * 
     * Traverses a route and recomputes all arrival_times, departure_times, and max_shift values.
     * Called by solve_from_partial to initialize context for partially-filled solutions.
     * 
     * FUTURE OPTIMIZATION: Instead of rebuilding from scratch, we could incrementally update
     * the route context of the partial solution based on the context of the full solution
     * (before destroy). This would be more efficient than full traversal.
     */
    RouteContext rebuild_route_context(
        const model::Problem& problem,
        const std::vector<NodeId>& route
    ) const;

}; // class GreedySolver

} // namespace oplib::solver::constructive
