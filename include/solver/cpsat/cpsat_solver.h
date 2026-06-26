#pragma once

#include "solver/solver.h"

namespace oplib::solver::cpsat {

struct CPSATSolverConfig : public SolverConfig {
    // Inherits: seed, max_cpu_time, max_iterations, verbose
    // max_cpu_time is forwarded to CP-SAT's time limit.
    // max_iterations is unused (CP-SAT manages its own search).
};

/**
 * @brief Exact CP-SAT solver for single-vehicle Orienteering Problems (OP / OPTW).
 *
 * Uses Google OR-Tools CP-SAT to solve the problem to proven optimality within
 * the time budget.  The formulation is circuit-based:
 *   - AddCircuit with optional self-loops for customer nodes
 *   - OnlyEnforceIf timing constraints (start-of-service propagation)
 *   - Maximize total reward of visited customers
 *
 * Handles both OP (time budget only) and OPTW (time budget + per-node windows).
 * Returns an empty depot-to-depot route when no feasible solution is found.
 */
class CPSATSolver : public Solver {
public:
    std::string get_name() const override { return "CPSAT"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig& config) override;

    model::Solution solve(const model::Problem& problem,
                          const CPSATSolverConfig& config);
};

}  // namespace oplib::solver::cpsat
