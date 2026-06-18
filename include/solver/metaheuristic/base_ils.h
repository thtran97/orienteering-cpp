#pragma once

#include <vector>

#include "core/constants.h"
#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Shared configuration for all ILS variants.
 */
struct BaseILSSolverConfig : public SolverConfig {
    virtual ~BaseILSSolverConfig() = default;
    int alpha             = oplib::constants::DEFAULT_ALPHA;
    int rcl_size          = oplib::constants::DEFAULT_RCL_SIZE;
    int restart_threshold = 10;

    mutable int iterations_out = 0; ///< total iterations executed (set by do_solve)
};

/**
 * @brief Abstract base for ILS-family solvers.
 *
 * Provides:
 *  - A generic solve(SolverConfig) dispatch that constructs a BaseILSSolverConfig
 *    and forwards to the subclass's do_solve().
 *  - A static construct() helper (repair + per-vehicle minimize_makespan) shared
 *    by all ILS variants.
 *
 * Subclasses implement do_solve() with the full ILS loop.  Typed solve() overloads
 * on subclasses may call do_solve() directly (no cast needed for base-config fields).
 */
class BaseILSSolver : public Solver {
public:
    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override final;

protected:
    static void construct(
        local_search::BaseLSUtils& ls,
        const local_search::LSConfig& ls_cfg,
        int num_vehicles,
        model::Solution& sol,
        std::vector<bool>& vis,
        std::vector<local_search::RouteContext>& ctx);

    virtual model::Solution do_solve(const model::Problem& problem, const BaseILSSolverConfig& config) = 0;
};

} // namespace oplib::solver::metaheuristic
