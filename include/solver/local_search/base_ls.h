#pragma once

#include <vector>
#include <queue>
#include <array>
#include <limits>
#include <utility>

#include "core/types.h"
#include "core/random.h"
#include "model/problem.h"
#include "model/solution.h"

namespace oplib::solver::local_search {

/**
 * @brief Per-route timing state used by local search utilities.
 *
 * Maintains arrival/departure times and max-shift slack at each position.
 * Updated incrementally as insertions, removals and moves are applied.
 */
struct RouteContext {
    std::vector<Time> arrival_times;   // arrival_times[i]   = arrival at route[i]
    std::vector<Time> departure_times; // departure_times[i]  = departure from route[i]
    std::vector<Time> max_shift;       // max_shift[i] = max extra delay tolerable at i
    Time cumulative_time = 0.0;        // total travel time accumulated on this route
};

/**
 * @brief Configuration for the base local-search primitives.
 */
struct LSConfig {
    int    alpha          = 2;    // reward exponent in heuristic score: reward^alpha/(shift+eps)
    int    rcl_size       = 5;    // RCL cap: top-K candidates kept; selection by weighted roulette
    double removal_ratio  = 0.4;  // fraction of route customers removed by destroy()
};

/**
 * @brief Stateless utility class providing core local-search primitives.
 *
 * Operates on model::Solution + per-vehicle RouteContext vectors in-place.
 * Intended to be composed (not inherited) by concrete metaheuristic solvers.
 *
 * Ported from toptwLib/lib/src/solver/local_search/base_LS.cpp and adapted to
 * the orienteering-cpp model API (virtual Problem, double Time, RouteContext).
 *
 * Thread-safety: not thread-safe (rng_ is mutated).
 */
class BaseLSUtils {
public:
    static constexpr Time INF = 1e18;

    /**
     * @param problem  The problem instance (held by const reference; must outlive this object).
     * @param rng      Shared random number generator (caller owns, must outlive this object).
     */
    BaseLSUtils(const model::Problem& problem, oplib::utils::Random& rng);

    // -----------------------------------------------------------------------
    // Initialisation
    // -----------------------------------------------------------------------

    /**
     * @brief Fill solution with empty depot-to-depot routes and reset contexts.
     *
     * After this call every vehicle route is {source_depot, sink_depot} and
     * the corresponding RouteContext is consistent with that empty route.
     * visited[i] is set to true for the depots and false for all customers.
     */
    void init(model::Solution& solution,
              std::vector<bool>& visited,
              std::vector<RouteContext>& contexts) const;

    /**
     * @brief Recompute a RouteContext from scratch for the given route.
     *
     * Performs a forward pass (arrival/departure times) followed by a backward
     * pass (max_shift values).  Use after any move that invalidates the context.
     */
    void recompute_context(const std::vector<NodeId>& route,
                           RouteContext& context) const;

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /**
     * @brief RCL-based greedy construction across all vehicles.
     *
     * Repeatedly selects the best feasible insertion from a Restricted Candidate
     * List (scored by reward^alpha / time_shift) until no more insertions are
     * possible.  Handles multi-vehicle problems by searching across all routes.
     *
     * @param solution   Modified in-place: customers inserted into routes.
     * @param visited    Modified in-place: inserted customers marked as visited.
     * @param contexts   Modified in-place: RouteContexts kept in sync.
     * @param config     alpha controls score weighting; rcl_size caps the candidate list.
     */
    void repair(model::Solution& solution,
                std::vector<bool>& visited,
                std::vector<RouteContext>& contexts,
                const LSConfig& config);

    // -----------------------------------------------------------------------
    // Perturbation / Destruction
    // -----------------------------------------------------------------------

    /**
     * @brief Remove a random fraction of customers from a single vehicle's route.
     *
     * Internally calls shake() with a random position and random length in
     * [1, max(1, removal_ratio * route_customers)].
     *
     * @param removal_ratio Fraction of route customers to remove (0 < r < 1).
     */
    void destroy(model::Solution& solution,
                 std::vector<bool>& visited,
                 std::vector<RouteContext>& contexts,
                 int vehicle,
                 double removal_ratio);

    /**
     * @brief Remove `length` consecutive customers starting at `position`.
     *
     * Wraps around the route if position + length exceeds the last customer.
     * Removes customers from visited and updates the RouteContext.
     *
     * @param position 1-based index into the route (depot is 0).
     * @param length   Number of customers to remove (>= 1).
     */
    void shake(model::Solution& solution,
               std::vector<bool>& visited,
               std::vector<RouteContext>& contexts,
               int vehicle,
               int position,
               int length);

    // -----------------------------------------------------------------------
    // Local Search
    // -----------------------------------------------------------------------

    /**
     * @brief Apply swap + 2-opt moves that reduce the route makespan.
     *
     * Iterates until no improving move is found.  Uses first-improvement:
     * restarts the search as soon as one improving move is applied.
     *
     * @return true if at least one improving move was applied.
     */
    bool minimize_makespan(model::Solution& solution,
                           std::vector<RouteContext>& contexts,
                           int vehicle);

    // -----------------------------------------------------------------------
    // Feasibility checking (public for use by concrete solvers)
    // -----------------------------------------------------------------------

    /**
     * @brief Check whether inserting `customer` at `position` in `vehicle` is feasible.
     *
     * Feasibility accounts for: customer TW, next-node TW, max_shift propagation,
     * and route budget.
     *
     * @return The additional travel time incurred (time_shift >= 0), or INF if infeasible.
     */
    double check_insertion(const model::Solution& solution,
                           const std::vector<RouteContext>& contexts,
                           int vehicle,
                           NodeId customer,
                           int position) const;

    /**
     * @brief Public wrapper for apply_insertion — allows external solvers to commit
     *        an insertion move obtained via check_insertion().
     */
    void apply_insertion_public(model::Solution& solution,
                                std::vector<RouteContext>& contexts,
                                std::vector<bool>& visited,
                                int vehicle,
                                NodeId customer,
                                int position,
                                Time time_shift);

private:
    const model::Problem& problem_;
    oplib::utils::Random& rng_;

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /**
     * @brief Commit an insertion move and keep RouteContext in sync.
     *
     * Forward pass: recompute arrival/departure for successors.
     * Backward pass: recompute max_shift for predecessors.
     */
    void apply_insertion(model::Solution& solution,
                         std::vector<RouteContext>& contexts,
                         std::vector<bool>& visited,
                         int vehicle,
                         NodeId customer,
                         int position,
                         Time time_shift);

    /**
     * @brief Remove the customer at `position` from the route and update context.
     *
     * Marks the removed customer as unvisited.
     */
    void remove_customer_at(model::Solution& solution,
                            std::vector<RouteContext>& contexts,
                            std::vector<bool>& visited,
                            int vehicle,
                            int position);

    /**
     * @brief Heuristic score for RCL: reward^alpha / (time_shift + epsilon).
     */
    double heuristic_score(NodeId customer, Time time_shift, int alpha) const;

    /**
     * @brief Check a swap move between pos1 and pos2; returns shift at next[pos2] or INF.
     *
     * A negative shift means the move reduces makespan.
     */
    Time check_swap(const model::Solution& solution,
                    const std::vector<RouteContext>& contexts,
                    int vehicle,
                    int pos1,
                    int pos2) const;

    /**
     * @brief Check a 2-opt move (reverse [pos1, pos2]); returns shift at next[pos2] or INF.
     */
    Time check_2opt(const model::Solution& solution,
                    const std::vector<RouteContext>& contexts,
                    int vehicle,
                    int pos1,
                    int pos2) const;

    void apply_swap(model::Solution& solution,
                    std::vector<RouteContext>& contexts,
                    int vehicle,
                    int pos1,
                    int pos2);

    void apply_2opt(model::Solution& solution,
                    std::vector<RouteContext>& contexts,
                    int vehicle,
                    int pos1,
                    int pos2);
};

} // namespace oplib::solver::local_search
