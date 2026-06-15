#pragma once

#include "solver/metaheuristic/base_ils.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the ILS09 solver.
 *
 * Inherits alpha, rcl_size, and restart_threshold from BaseILSSolverConfig.
 * Kept as a named type so callers can construct it explicitly.
 */
struct ILS09SolverConfig : public BaseILSSolverConfig {};

/**
 * @brief Iterated Local Search 2009 (ILS09) metaheuristic.
 * 
 * Vansteenwegen, P., Souffriau, W., Vanden Berghe, G., & Van Oudheusden, D. (2009). Iterated local search for the team orienteering problem with time windows. Computers & Operations Research, 36(12), 3281-3290.
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
class ILS09Solver : public BaseILSSolver {
public:
    std::string get_name() const override { return "ILS09"; }

    using BaseILSSolver::solve;
    model::Solution solve(const model::Problem& problem, const ILS09SolverConfig& config);

protected:
    model::Solution do_solve(const model::Problem& problem, const BaseILSSolverConfig& config) override;
};

} // namespace oplib::solver::metaheuristic
