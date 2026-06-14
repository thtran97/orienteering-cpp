#pragma once

#include <vector>

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the ILS + Route Recombination solver.
 */
struct ILSRouteRecombinationSolverConfig : public SolverConfig {
    int    alpha             = oplib::constants::DEFAULT_ALPHA;
    int    rcl_size          = oplib::constants::DEFAULT_RCL_SIZE;
    int    restart_threshold = 10;   ///< no-improvement trigger
    int    pool_size         = 10;   ///< max elite-pool size
    double similarity_threshold = 0.5; ///< Hamming overlap threshold for diversity
};

/**
 * @brief ILS09 extended with an elite pool and route recombination (path relinking).
 *
 * After each improvement the solution is added to the elite pool.
 * After no_impr ≥ restart_threshold the best solution is combined with a
 * randomly chosen pool member via a greedy path-relinking move:
 *   - customers in guiding_sol but not in current_sol are inserted if feasible.
 *   - the combined solution becomes the new current.
 *
 * Inspired by toptwLib/lib/src/solver/local_search/ILS_with_route_recombination.cpp.
 * Avoids the boost::dynamic_bitset dependency by using std::vector<bool>.
 */
class ILSRouteRecombinationSolver : public Solver {
public:
    std::string get_name() const override { return "ILSRouteRecombination"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem&                       problem,
                          const ILSRouteRecombinationSolverConfig&    config);

    /**
     * @brief Route-recombination operator (set-packing over a pool of routes).
     *
     * Gathers every route from the pooled solutions, greedily selects the
     * highest-reward set of customer-disjoint routes (a max-weight set-packing
     * heuristic), assigns each selected route to a vehicle, then repairs any
     * leftover customers. Because vehicles are independent in TOP/TOPTW, a route
     * feasible in its origin solution stays feasible when reassigned, so the
     * recombined solution is always feasible.
     *
     * Public + static so it can be unit-tested in isolation. This is the
     * route-level counterpart to toptwLib's route_recombinator/combinator.
     */
    static model::Solution recombine_routes(
        const model::Problem&               problem,
        const std::vector<model::Solution>& pool,
        local_search::BaseLSUtils&          ls,
        const local_search::LSConfig&       ls_cfg);

private:
    /// Add solution to elite pool if sufficiently different from existing members.
    void add_to_pool(std::vector<model::Solution>& pool,
                     const model::Solution&         sol,
                     int                            pool_size,
                     double                         similarity_threshold) const;

    /// Compute Hamming similarity: fraction of customers shared between two solutions.
    double hamming_similarity(const model::Solution& a, const model::Solution& b,
                              int num_nodes) const;

    /// Recompute the visited flags and per-vehicle RouteContext for `sol`.
    static void rebuild_bookkeeping(local_search::BaseLSUtils&                ls,
                                    const model::Problem&                     problem,
                                    const model::Solution&                    sol,
                                    std::vector<bool>&                        visited,
                                    std::vector<local_search::RouteContext>&  ctx);
};

} // namespace oplib::solver::metaheuristic
