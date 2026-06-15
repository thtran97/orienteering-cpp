#pragma once

#include "solver/metaheuristic/base_ils.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the ILS15 solver (Gunawan, Lau & Lu 2015).
 */
struct ILS15SolverConfig : public BaseILSSolverConfig {
    /// Iterations between consecutive cons increments (paper default: 2).
    int post_increment_period = 2;
};

/**
 * @brief ILS from Gunawan, Lau & Lu (2015) for the OPTW.
 *
 * Distinctive features vs ILS09:
 *  - Always-accept random walk (no strict-improvement gate on current solution).
 *  - Deterministic cons/post shake schedule: cons controls shake length; post
 *    controls starting position and advances by cons each iteration, wrapping
 *    modulo the smallest non-empty route size.
 *  - cons increments every post_increment_period iterations and wraps when it
 *    exceeds the maximum route size.
 *  - Replace local-search move after each construct phase.
 *  - Periodic restart to best after restart_threshold no-improvement iterations.
 */
class ILS15Solver : public BaseILSSolver {
public:
    std::string get_name() const override { return "ILS15"; }

    using BaseILSSolver::solve;
    model::Solution solve(const model::Problem& problem,
                          const ILS15SolverConfig& config);

protected:
    model::Solution do_solve(const model::Problem& problem,
                             const BaseILSSolverConfig& config) override;
};

} // namespace oplib::solver::metaheuristic
