#pragma once

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the Large Neighbourhood Search solver.
 */
struct LNSSolverConfig : public SolverConfig {
    int    alpha          = oplib::constants::DEFAULT_ALPHA;
    int    rcl_size       = oplib::constants::DEFAULT_RCL_SIZE;
    double removal_ratio  = oplib::constants::DEFAULT_REMOVAL_RATIO; ///< fraction removed per destroy
};

/**
 * @brief Large Neighbourhood Search (LNS) metaheuristic.
 *
 * Algorithm:
 *  1. init() + repair() — build initial solution.
 *  2. Loop (max_iterations × max_cpu_time):
 *       for each vehicle: destroy() — remove a random fraction of customers.
 *       repair() — greedily reinsert all unvisited customers.
 *       if improved: update best.
 *
 * Ported from toptwLib/lib/src/solver/local_search/LNS.cpp.
 */
class LNSSolver : public Solver {
public:
    std::string get_name() const override { return "LNS"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const LNSSolverConfig& config);
};

} // namespace oplib::solver::metaheuristic
