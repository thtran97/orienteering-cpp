#pragma once

#include <vector>

#include "solver/solver.h"

namespace oplib::solver::cpsat {

struct CPSATOPTWSolverConfig : public SolverConfig {
    int  num_workers  = 4;    // parallel CP-SAT workers; set to 1 for determinism
    bool greedy_hint  = true; // seed solver with a greedy OPTW solution before search
};

/**
 * @brief Exact CP-SAT solver dedicated to single-vehicle OPTW.
 *
 * Identical circuit-based formulation to CPSATSolver, with two OPTW-specific
 * improvements:
 *   1. Greedy warm-start: a time-window-aware nearest-feasible greedy builds
 *      a real solution in O(n²) and provides a complete CP-SAT hint, giving the
 *      solver a non-trivial lower bound from the first second instead of reward 0.
 *   2. Multi-worker parallel search (default: 4 workers) appropriate for the
 *      harder OPTW search space where the LP relaxation bound is loose.
 *
 * Returns an empty depot-to-depot route when no feasible solution is found.
 * Requires problem.has_time_windows() == true.
 */
class CPSATOPTWSolver : public Solver {
public:
    std::string get_name() const override { return "CPSAT_OPTW"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig& config) override;

    model::Solution solve(const model::Problem& problem,
                          const CPSATOPTWSolverConfig& config);

private:
    // Returns a time-window-feasible greedy route as a sequence of node indices
    // [source, c1, c2, ..., sink]. Used to seed the CP-SAT hint.
    std::vector<int> greedy_route(const model::Problem& problem,
                                  double tmax_d) const;
};

}  // namespace oplib::solver::cpsat
