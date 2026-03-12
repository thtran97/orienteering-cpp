#include <algorithm>

#include "solver/local_search/LNS.h"
#include "solver/constructive/greedy.h"
#include "core/random.h"

namespace oplib::solver::local_search {

model::Solution LNSSolver::solve(const model::Problem& problem, const SolverConfig& config) {
    constructive::GreedySolver initial_solver;
    model::Solution initial = initial_solver.solve(problem, config);
    return solve_from(problem, initial, config);
}

model::Solution LNSSolver::solve_from(
    const model::Problem& problem,
    const model::Solution& initial_solution,
    const SolverConfig& config)
{
    model::Solution current_sol = initial_solution;
    model::Solution best_sol = current_sol;
    
    utils::Random rng(config.seed + 42);

    for (int i = 0; i < config.max_iterations; ++i) {
        model::Solution trial_sol = current_sol;
        
        int destroy_count = rng.next_int(min_destroy, max_destroy);
        destroy_random(problem, trial_sol, destroy_count, rng);
        repair_greedy(problem, trial_sol, config);

        if (trial_sol.total_reward > current_sol.total_reward) {
            current_sol = trial_sol;
            if (current_sol.total_reward > best_sol.total_reward) {
                best_sol = current_sol;
            }
        }
    }

    return best_sol;
}

void LNSSolver::destroy_random(
    const model::Problem& problem,
    model::Solution& solution,
    int count,
    utils::Random& rng)
{
    for (int i = 0; i < count; ++i) {
        int v = rng.next_int(0, solution.get_num_vehicles() - 1);
        auto& route = solution.get_route(v);
        
        // Must have more than just depots (size > 2)
        if (route.size() <= 2) continue;

        // Random starting position between first and last customer (indices 1 to size-2)
        int start_pos = rng.next_int(1, static_cast<int>(route.size()) - 2);
        
        // Random removal length
        int removal_len = rng.next_int(1, min_destroy + 1);
        
        // Perform contiguous removal with wraparound
        std::vector<NodeId> to_remove;
        for (int j = 0; j < removal_len; ++j) {
            int pos = (start_pos + j) % (route.size() - 1);
            if (pos == 0) pos = route.size() - 1;  // Skip last depot
            to_remove.push_back(route[pos]);
        }
        
        // Remove in reverse order to maintain indices
        std::sort(to_remove.begin(), to_remove.end());
        to_remove.erase(std::unique(to_remove.begin(), to_remove.end()), to_remove.end());
        
        for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
            NodeId node_to_remove = *it;
            // Find and erase this node from route
            auto pos_it = std::find(route.begin() + 1, route.end() - 1, node_to_remove);
            if (pos_it != route.end()) {
                solution.total_reward -= problem.get_reward(node_to_remove);
                route.erase(pos_it);
            }
        }
    }
}

void LNSSolver::repair_greedy(
    const model::Problem& problem,
    model::Solution& solution,
    const SolverConfig& config)
{
    constructive::GreedySolver greedy;
    constructive::GreedySolverConfig greedy_config;
    greedy_config.seed = config.seed;
    greedy_config.verbose = config.verbose;
    greedy_config.max_iterations = config.max_iterations;
    
    solution = greedy.solve_from_partial(problem, solution, greedy_config);
}

} // namespace oplib::solver::local_search
