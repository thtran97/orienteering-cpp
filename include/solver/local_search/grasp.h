#pragma once

#include <vector>
#include <memory>

#include "solver/solver.h"

namespace oplib::solver::local_search {

/**
 * @brief Configuration for GRASP solver.
 * 
 * Extends SolverConfig with GRASP-specific parameters.
 */
struct GraspConfig : public SolverConfig {
    double alpha = 0.3;        // RCL parameter for randomized greedy (0=greedy, 1=random)
    int lns_iterations = 100;  // Number of LNS iterations per GRASP iteration
};

/**
 * @brief GRASP (Greedy Randomized Adaptive Search Procedure) Solver.
 * 
 * GRASP is a metaheuristic that combines randomized greedy construction with
 * local search improvement. Each iteration:
 * 1. Build a randomized greedy solution (RCL-based construction)
 * 2. Improve it with Large Neighborhood Search (LNS)
 * 3. Track the global best solution
 * 
 * The alpha parameter controls diversity in the construction phase:
 * - alpha=0: pure greedy (deterministic)
 * - alpha=1: fully random (high variance)
 * - alpha≈0.3: typical default (good balance)
 */
class GraspSolver : public Solver {
public:
    std::string get_name() const override { return "GRASP"; }

    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override;

    /**
     * @brief Solve with GRASP-specific configuration.
     * 
     * @param problem The problem instance
     * @param config GRASP configuration including alpha and lns_iterations
     * @return The best solution found across all iterations
     */
    model::Solution solve(const model::Problem& problem, const GraspConfig& config);

    // GRASP parameters
    double alpha = 0.3;         // RCL parameter
    int lns_iterations = 100;   // Inner LNS iterations per GRASP iteration
};

} // namespace oplib::solver::local_search
