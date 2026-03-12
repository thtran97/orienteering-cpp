#include <algorithm>

#include "solver/constructive/randomized_greedy.h"
#include "solver/constructive/greedy.h"
#include "core/random.h"
#include <limits>

namespace oplib::solver::constructive {

model::Solution RandomizedGreedySolver::solve(const model::Problem& problem, const SolverConfig& config) {
    model::Solution best_solution(problem.get_num_vehicles());
    double best_reward = -1.0;

    for (int i = 0; i < config.max_iterations; ++i) {
        utils::Random rng(config.seed + i);  // Different seed for each iteration
        model::Solution current_sol = construct_randomized_greedy(problem, config, rng, 0.3);  // Default alpha
        
        if (current_sol.total_reward > best_reward) {
            best_reward = current_sol.total_reward;
            best_solution = current_sol;
        }
    }

    return best_solution;
}

model::Solution RandomizedGreedySolver::solve(const model::Problem& problem, const RandomizedGreedyConfig& config) {
    // Create a modified config with alpha embedded in the solver context
    // We'll use the base config but handle alpha during construction
    SolverConfig base_config;
    base_config.seed = config.seed;
    base_config.max_iterations = config.max_iterations;
    base_config.max_cpu_time = config.max_cpu_time;
    base_config.verbose = config.verbose;
    
    model::Solution best_solution(problem.get_num_vehicles());
    double best_reward = -1.0;

    for (int i = 0; i < base_config.max_iterations; ++i) {
        utils::Random rng(base_config.seed + i);  // Different seed for each iteration
        model::Solution current_sol = construct_randomized_greedy(problem, base_config, rng, config.alpha);
        
        if (current_sol.total_reward > best_reward) {
            best_reward = current_sol.total_reward;
            best_solution = current_sol;
        }
    }

    return best_solution;
}

model::Solution RandomizedGreedySolver::construct_randomized_greedy(
    const model::Problem& problem,
    const SolverConfig& config,
    utils::Random& rng,
    double alpha)
{
    move_evaluator = create_move_evaluator(MoveEvaluatorType::REWARD_PER_TIME);
    
    int num_vehicles = problem.get_num_vehicles();
    model::Solution solution(num_vehicles);
    std::vector<bool> visited(problem.get_num_nodes(), false);
    
    // Mark depots as visited
    visited[problem.get_source_depot()] = true;
    visited[problem.get_sink_depot()] = true;
    
    // Initialize routes with depots and route contexts
    std::vector<RouteContext> route_contexts(num_vehicles);
    for (int v = 0; v < num_vehicles; ++v) {
        solution.get_route(v) = {problem.get_source_depot(), problem.get_sink_depot()};
        
        route_contexts[v].arrival_times = {0.0, 0.0};
        route_contexts[v].departure_times = {0.0, 0.0};
        route_contexts[v].max_shift = {std::numeric_limits<Time>::max(), std::numeric_limits<Time>::max()};
        route_contexts[v].cumulative_time = 0.0;
    }
    
    solution.total_reward = 0.0;
    solution.total_travel_time = 0.0;
    
    // Construction loop: find all feasible insertions, build RCL, randomly select
    while (true) {
        // Clear stale cache entries before each step: after an insertion at position P,
        // all cache keys (c, v, pos >= P) refer to shifted positions and are invalid.
        // Clearing entirely is simpler and correct since we re-evaluate everything anyway.
        infeasibility_cache.clear();

        // Get all feasible insertions via inherited GreedySolver method
        std::vector<InsertionMove> all_moves = find_all_feasible_insertions(
            problem, solution, visited, route_contexts);
        
        if (all_moves.empty()) break;
        
        // Build RCL: include moves whose score is within alpha of the best
        double max_score = all_moves[0].score;  // sorted descending
        double min_score = all_moves.back().score;
        double threshold = max_score - alpha * (max_score - min_score);
        
        std::vector<InsertionMove> rcl;
        for (const auto& m : all_moves) {
            if (m.score >= threshold) {
                rcl.push_back(m);
            }
        }
        
        if (rcl.empty()) break;
        
        // Randomly select from RCL
        const auto& selected = rcl[rng.next_int(0, static_cast<int>(rcl.size()) - 1)];
        
        // Apply insertion via inherited method (handles route, timing, max_shift, visited)
        apply_insertion(problem, solution, selected, route_contexts, visited);

        // Update solution-level metrics
        solution.total_reward += problem.get_reward(selected.customer);
        solution.total_travel_time += selected.time_shift;
    }
    
    return solution;
}

} // namespace oplib::solver::constructive
