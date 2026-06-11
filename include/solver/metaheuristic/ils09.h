#pragma once

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the ILS09 solver.
 */
struct ILS09SolverConfig : public SolverConfig {
    int alpha             = oplib::constants::DEFAULT_ALPHA;
    int rcl_size          = oplib::constants::DEFAULT_RCL_SIZE;
    int restart_threshold = 10; ///< no-improvement count before restart from best
};

/**
 * @brief Iterated Local Search 2009 (ILS09) metaheuristic.
 *
 * Algorithm:
 *  1. construct() — repair() + minimize_makespan() per vehicle.
 *  2. Loop (bounded by max_iterations and max_cpu_time):
 *       perturb: shake each vehicle at a random position with current shake_length.
 *       construct again.
 *       acceptance: if improved → best=current, shake_length=1, no_impr=0.
 *                   else         → no_impr++.
 *       if no_impr ≥ restart_threshold → restore best, shake_length++.
 *
 * Ported from toptwLib/lib/src/solver/local_search/ILS09.cpp.
 */
class ILS09Solver : public Solver {
public:
    std::string get_name() const override { return "ILS09"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem&   problem,
                          const ILS09SolverConfig& config);
};

} // namespace oplib::solver::metaheuristic
