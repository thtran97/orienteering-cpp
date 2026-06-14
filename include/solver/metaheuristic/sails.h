#pragma once

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the SA-ILS solver.
 */
struct SAILSSolverConfig : public SolverConfig {
    int    alpha            = oplib::constants::DEFAULT_ALPHA;    ///< RCL reward exponent
    int    rcl_size         = oplib::constants::DEFAULT_RCL_SIZE;
    double initial_temp     = 1000.0;  ///< SA starting temperature (T0)
    double cooling_rate     = 0.75;    ///< geometric cooling factor in (0,1)
    int    inner_loop       = 50;      ///< iterations per temperature level
    int    max_shake_length = 3;       ///< max customers removed per perturbation
};

/**
 * @brief Simulated-Annealing Iterated Local Search (SA-ILS).
 *
 * Iterated local search whose acceptance criterion is the Metropolis rule:
 * a perturbed-then-repaired candidate that is worse than the incumbent by
 * `delta` (< 0, since reward is maximised) is still accepted with probability
 * exp(delta / T). The temperature T starts at `initial_temp` and is cooled
 * geometrically (T <- cooling_rate * T) after each inner loop, and reheated when
 * it becomes negligible — giving repeated diversification followed by
 * intensification.
 *
 * SAILS existed only as a stub in toptwLib; this is a fresh implementation on
 * the shared BaseLSUtils primitives, matching the parameters declared there
 * (T0 = 1000, cooling = 0.75, inner loop = 50).
 */
class SAILSSolver : public Solver {
public:
    std::string get_name() const override { return "SAILS"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem&    problem,
                          const SAILSSolverConfig& config);
};

} // namespace oplib::solver::metaheuristic
