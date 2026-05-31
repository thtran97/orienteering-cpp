#include <iostream>

#include "solver/local_search/grasp.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/local_search/LNS.h"

namespace oplib::solver::local_search {

model::Solution GraspSolver::solve(const model::Problem& problem, const SolverConfig& config) {
    GraspConfig grasp_config;
    grasp_config.seed = config.seed;
    grasp_config.max_iterations = config.max_iterations;
    grasp_config.max_cpu_time = config.max_cpu_time;
    grasp_config.verbose = config.verbose;
    grasp_config.alpha = alpha;
    grasp_config.lns_iterations = lns_iterations;
    return solve(problem, grasp_config);
}

model::Solution GraspSolver::solve(const model::Problem& problem, const GraspConfig& config) {
    model::Solution best_solution(problem.get_num_vehicles());
    double best_reward = -1.0;

    constructive::RandomizedGreedySolver rg_solver;
    LNSSolver lns_solver;

    for (int i = 0; i < config.max_iterations; ++i) {
        // Construction phase: one randomized greedy construction
        constructive::RandomizedGreedyConfig rg_config;
        rg_config.seed = config.seed + i;
        rg_config.max_iterations = 1;  // single construction per GRASP iteration
        rg_config.verbose = false;
        rg_config.max_cpu_time = config.max_cpu_time;
        rg_config.alpha = config.alpha;
        
        model::Solution sol = rg_solver.solve(problem, rg_config);

        // Local search phase: LNS improvement
        SolverConfig lns_config;
        lns_config.seed = config.seed + i;
        lns_config.max_iterations = config.lns_iterations;
        lns_config.verbose = false;
        lns_config.max_cpu_time = config.max_cpu_time;
        
        sol = lns_solver.solve_from(problem, sol, lns_config);

        // Update global best
        if (sol.total_reward > best_reward) {
            best_reward = sol.total_reward;
            best_solution = sol;
            
            if (config.verbose) {
                std::cout << "GRASP iteration " << i << ": improved to reward " << best_reward << std::endl;
            }
        }
    }

    return best_solution;
}

} // namespace oplib::solver::local_search
