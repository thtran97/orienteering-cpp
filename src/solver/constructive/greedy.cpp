#include <algorithm>
#include <limits>
#include <cmath>

#include "solver/constructive/greedy.h"
#include "core/random.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"

namespace oplib::solver::constructive {


model::Solution GreedySolver::solve(const model::Problem& problem, const SolverConfig& config) {
    // Convert to GreedySolverConfig with defaults
    GreedySolverConfig greedy_config;
    greedy_config.seed = config.seed;
    greedy_config.verbose = config.verbose;
    return solve(problem, greedy_config);
}

model::Solution GreedySolver::solve(const model::Problem& problem, const GreedySolverConfig& config) {
    // Select appropriate components if not provided at construction based on problem type and config
    feasibility_checker = select_feasibility_checker(problem);
    move_evaluator = create_move_evaluator(config.move_evaluator);

    // Initialize solution with empty routes and track visited customers
    int num_vehicles = problem.get_num_vehicles();
    model::Solution solution(num_vehicles);
    std::vector<bool> visited(problem.get_num_nodes(), false);

    // Mark depots as visited
    visited[problem.get_source_depot()] = true;
    visited[problem.get_sink_depot()] = true;

    // Initialize route contexts for incremental update 
    // (arrival/departure/shift/budget)
    std::vector<RouteContext> route_contexts(num_vehicles);
    
    for (int v = 0; v < num_vehicles; ++v) {
        solution.get_route(v) = {problem.get_source_depot(), problem.get_sink_depot()};
        
        // Initialize times: at source depot at time 0, at sink depot at time 0 (will be updated)
        route_contexts[v].arrival_times = {0.0, 0.0};
        route_contexts[v].departure_times = {0.0, 0.0};
        route_contexts[v].cumulative_time = 0.0;
    }
    
    // clear cache before starting construction
    infeasibility_cache.clear();

    // Track metrics
    solution.total_reward = 0.0;
    solution.total_travel_time = 0.0;

    // Greedy construction: iteratively insert best feasible customer
    while (true) {
        auto best_move = find_best_insertion(problem, solution, visited, route_contexts);
        if (!best_move.feasible) break;  // No more feasible insertions

        // Apply the best insertion move and update solution contexts
        apply_insertion(problem, solution, best_move, route_contexts, visited);

        // Update metrics incrementally
        solution.total_reward += problem.get_reward(best_move.customer);
        solution.total_travel_time += best_move.time_shift;
    }
    
    return solution;
}

std::unique_ptr<FeasibilityChecker> GreedySolver::select_feasibility_checker(
    const model::Problem& problem
) {
    if (problem.has_time_windows()) {
        if (problem.is_time_dependent()) {
            return std::make_unique<TimeDependentFeasibilityChecker>();
        } else {
            return std::make_unique<TimeWindowFeasibilityChecker>();
        }
    } else {
        return std::make_unique<DistanceFeasibilityChecker>();
    }
}

InsertionMove GreedySolver::find_best_insertion(
    const model::Problem& problem, 
    const model::Solution& solution, 
    const std::vector<bool>& visited,
    const std::vector<RouteContext>& route_contexts
) {
    InsertionMove best_move;
    best_move.feasible = false;
    int num_vehicles = problem.get_num_vehicles();
    
    for (int v = 0; v < num_vehicles; ++v) {
        for (NodeId c = 0; c < problem.get_num_nodes(); ++c) {
            if (visited[c]) continue;
            if (c == problem.get_source_depot() || c == problem.get_sink_depot()) continue;

            const auto& route = solution.get_route(v);
            for (int pos = 1; pos < static_cast<int>(route.size()); ++pos) {
                evaluate_insertion_candidate(best_move, c, v, pos, problem, solution, route_contexts);
            }
        }
    }

    return best_move;
}

void GreedySolver::evaluate_insertion_candidate(
    InsertionMove& best_move,
    NodeId c,
    int v,
    int pos,
    const model::Problem& problem,
    const model::Solution& solution,
    const std::vector<RouteContext>& route_contexts
) {
    // Validate position bounds
    const auto& route = solution.get_route(v);
    if (pos < 1 || pos > static_cast<int>(route.size())) {
        return;  // Invalid position
    }
    
    // Check cache before feasibility check
    if (infeasibility_cache.is_infeasible(c, v, pos)) {
        return;  // Skip cached infeasible triplet
    }
    
    const auto& context = route_contexts[v];
    
    // Check feasibility of this insertion
    // Pass the complete route context: arrival times, departure times, and cumulative time
    auto feasibility_result = feasibility_checker->check_feasibility(
        problem, solution, v, c, pos,
        context.arrival_times, context.departure_times,
        context.cumulative_time
    );
    
    if (!feasibility_result.feasible) {
        infeasibility_cache.mark_infeasible(c, v, pos);
        return;
    }
    
    // Compute insertion score based on feasibility results
    // The evaluator uses travel time cost and total time cost to compute the score
    double score = move_evaluator->evaluate(
        problem, c,
        feasibility_result.travel_time_cost
    );
    
    // Update best move if this is better
    if (score > best_move.score) {
        best_move.customer = c;
        best_move.vehicle = v;
        best_move.position = pos;
        best_move.score = score;
        best_move.feasible = true;
        best_move.arrival_time = feasibility_result.arrival_time_at_customer;
        best_move.departure_time = feasibility_result.departure_time_from_customer;
        best_move.time_shift = feasibility_result.travel_time_cost;  // Store travel time cost as time_shift
    }
}


void GreedySolver::apply_insertion(
    const model::Problem& problem,
    model::Solution& solution,
    const InsertionMove& move,
    std::vector<RouteContext>& route_contexts,
    std::vector<bool>& visited
) {
    // Insert customer into route
    auto& route = solution.get_route(move.vehicle);
    auto& context = route_contexts[move.vehicle];
    
    route.insert(route.begin() + move.position, move.customer);
    
    // Insert arrival and departure times at the insertion point
    context.arrival_times.insert(
        context.arrival_times.begin() + move.position,
        move.arrival_time
    );
    context.departure_times.insert(
        context.departure_times.begin() + move.position,
        move.departure_time
    );
    
    // Update contexts of all successor nodes incrementally
    // The departure time from the inserted customer propagates downstream
    for (int i = move.position + 1; i < static_cast<int>(route.size()); ++i) {
        NodeId prev_node = route[i - 1];
        NodeId curr_node = route[i];
        Time departure_from_prev = context.departure_times[i - 1];
        
        // Compute new arrival and departure times for successor node
        Time travel_time = problem.get_travel_time(prev_node, curr_node, departure_from_prev);
        Time arrival_time = departure_from_prev + travel_time;
        
        // Respect time window opening if this is a time-window problem
        const auto& tw = problem.get_time_window(curr_node);
        Time start_service = std::max(arrival_time, tw.opening);
        Time departure_time = start_service + problem.get_service_time(curr_node);
        
        // Update the context for this successor
        context.arrival_times[i] = arrival_time;
        context.departure_times[i] = departure_time;
    }
    
    // Update cumulative time
    context.cumulative_time += move.time_shift;
    
    // Mark as visited
    visited[move.customer] = true;
    
    // Invalidate cache entries for inserted customer
    infeasibility_cache.invalidate_customer(move.customer);
}

std::pair<Time, Time> GreedySolver::compute_arrival_departure_times(
    const model::Problem& problem,
    NodeId prev,
    NodeId customer,
    NodeId next,
    Time departure_from_prev
) {
    // Compute arrival times using feasibility checker's logic
    Time travel_time_prev_to_c = problem.get_travel_time(prev, customer, departure_from_prev);
    Time arrival_at_c = departure_from_prev + travel_time_prev_to_c;
    
    // Respect time window opening
    const auto& tw_c = problem.get_time_window(customer);
    Time start_service = std::max(arrival_at_c, tw_c.opening);
    Time departure_from_c = start_service + problem.get_service_time(customer);
    
    return {arrival_at_c, departure_from_c};
}

} // namespace oplib::solver::constructive
