#pragma once

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the GRASP + VNS solver.
 */
struct GraspVnsSolverConfig : public SolverConfig {
    int    alpha           = oplib::constants::DEFAULT_ALPHA;
    int    rcl_size        = oplib::constants::DEFAULT_RCL_SIZE;
    int    max_shake_length = 3; ///< VNS neighbourhood depth (max customers shaken per move)
};

/**
 * @brief GRASP + Variable Neighbourhood Search (VNS) metaheuristic.
 *
 * Algorithm per restart (bounded by max_cpu_time):
 *  1. init() — empty depot routes.
 *  2. construct() — repair() + minimize_makespan() per vehicle (GRASP construction).
 *  3. VNS loop: for shake_length in [1..max_shake_length]:
 *       for each vehicle: for each position in route:
 *         shake → repair → if improved: reset shake_length, minimize_makespan; else revert.
 *  4. Update global best if improved.
 *
 * Ported from toptwLib/lib/src/solver/local_search/GRASP_VNS.cpp.
 */
class GraspVnsSolver : public Solver {
public:
    std::string get_name() const override { return "GraspVns"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem&       problem,
                          const GraspVnsSolverConfig& config);
};

} // namespace oplib::solver::metaheuristic
