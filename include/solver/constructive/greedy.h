#pragma once

#include <memory>
#include <set>
#include "solver/solver.h"
#include "solver/constructive/feasibility_checker.h"
#include "solver/constructive/move_evaluator.h"

namespace oplib::solver::constructive {


/**
 * @brief Configuration parameters specific to the greedy solver.
 * 
 * Extends the base SolverConfig with greedy-specific options for
 * insertion strategy, move evaluation, and caching.
 */
struct GreedySolverConfig : public SolverConfig {
    InsertionStrategyMode insertion_strategy = InsertionStrategyMode::CUSTOMER_WISE;
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
 * Key features:
 * - Modular design: pluggable feasibility checking, move evaluation, and insertion strategy
 * - Incremental tracking: maintains arrival/departure times and distances incrementally
 * - Caching: avoids re-evaluating infeasible insertions
 * - Variant support: works with all problem types (OP, OPTW, TOP, TOPTW, TDOP, TDOPTW)
 * 
 * Supports:
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
    GreedySolver(
        InsertionStrategyMode strategy_mode = InsertionStrategyMode::CUSTOMER_WISE // TODO: unused for now
    ) : strategy_mode(strategy_mode) {};

    std::string get_name() const override { return "GreedySolver"; }
    
    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override;
    
    /**
     * @brief Solve with greedy-specific configuration.
     * 
     * Preferred method when using the greedy solver directly, as it provides
     * access to all greedy-specific options.
     */
    model::Solution solve(const model::Problem& problem, const GreedySolverConfig& config);

private:
    InsertionStrategyMode strategy_mode;
    std::unique_ptr<FeasibilityChecker> feasibility_checker;
    std::unique_ptr<MoveEvaluator> move_evaluator;
    InfeasibilityCache infeasibility_cache;

    /**
     * @brief Select appropriate feasibility checker based on problem type.
     */
    std::unique_ptr<FeasibilityChecker> select_feasibility_checker(
        const model::Problem& problem
    );

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
     * @brief Compute arrival and departure times for inserting a customer between two nodes.
     */
    std::pair<Time, Time> compute_arrival_departure_times(
        const model::Problem& problem,
        NodeId prev,
        NodeId customer,
        NodeId next,
        Time departure_from_prev
    );
};

} // namespace oplib::solver::constructive
