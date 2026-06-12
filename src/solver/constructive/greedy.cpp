#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <tuple>

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
    // Initialize move evaluator based on config
    move_evaluator = create_move_evaluator(config.move_evaluator);

    // Initialize solution with empty routes and track visited customers
    int num_vehicles = problem.get_num_vehicles();
    model::Solution solution(num_vehicles);
    std::vector<bool> visited(problem.get_num_nodes(), false);

    // Mark depots as visited
    visited[problem.get_source_depot()] = true;
    visited[problem.get_sink_depot()] = true;

    // Initialize route contexts for incremental update
    // (arrival/departure/max_shift/budget)
    std::vector<RouteContext> route_contexts(num_vehicles);

    // Base route time is the direct source→sink distance; this seeds the budget
    // check so that cumulative_time always represents the TOTAL trip time, not
    // just the extra detour added on top of the direct leg.
    Time base_trip_time = problem.get_distance(problem.get_source_depot(),
                                               problem.get_sink_depot());
    for (int v = 0; v < num_vehicles; ++v) {
        solution.get_route(v) = {problem.get_source_depot(), problem.get_sink_depot()};

        route_contexts[v].arrival_times = {0.0, 0.0};
        route_contexts[v].departure_times = {0.0, 0.0};
        route_contexts[v].max_shift = {std::numeric_limits<Time>::max(), std::numeric_limits<Time>::max()};
        route_contexts[v].cumulative_time = base_trip_time;
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

std::vector<InsertionMove> GreedySolver::find_all_feasible_insertions(
    const model::Problem& problem,
    const model::Solution& solution,
    const std::vector<bool>& visited,
    const std::vector<RouteContext>& route_contexts
) {
    std::vector<InsertionMove> all_moves;
    int num_vehicles = problem.get_num_vehicles();
    
    for (int v = 0; v < num_vehicles; ++v) {
        for (NodeId c = 0; c < problem.get_num_nodes(); ++c) {
            if (visited[c]) continue;
            if (c == problem.get_source_depot() || c == problem.get_sink_depot()) continue;

            const auto& route = solution.get_route(v);
            for (int pos = 1; pos < static_cast<int>(route.size()); ++pos) {
                // Check cache
                if (infeasibility_cache.is_infeasible(c, v, pos)) {
                    continue;
                }
                
                // Check feasibility
                auto feasibility_result = check_insertion_feasible(problem, solution, v, c, pos, route_contexts);
                
                if (!feasibility_result.feasible) {
                    infeasibility_cache.mark_infeasible(c, v, pos);
                    continue;
                }
                
                // Compute score
                double score = move_evaluator->evaluate(
                    problem, c,
                    feasibility_result.travel_time_cost
                );
                
                InsertionMove move;
                move.customer = c;
                move.vehicle = v;
                move.position = pos;
                move.score = score;
                move.feasible = true;
                move.arrival_time = feasibility_result.arrival_time_at_customer;
                move.departure_time = feasibility_result.departure_time_from_customer;
                move.time_shift = feasibility_result.travel_time_cost;
                
                all_moves.push_back(move);
            }
        }
    }
    
    // Sort by score descending
    std::sort(all_moves.begin(), all_moves.end(),
        [](const InsertionMove& a, const InsertionMove& b) { return a.score > b.score; });
    
    return all_moves;
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
    
    // Check feasibility of this insertion using max_shift from route context
    auto feasibility_result = check_insertion_feasible(problem, solution, v, c, pos, route_contexts);
    
    if (!feasibility_result.feasible) {
        infeasibility_cache.mark_infeasible(c, v, pos);
        return;
    }
    
    // Compute insertion score based on feasibility results
    // The evaluator uses travel time cost to compute the score
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

FeasibilityResult GreedySolver::check_insertion_feasible(
    const model::Problem& problem,
    const model::Solution& solution,
    int vehicle,
    NodeId customer,
    int position,
    const std::vector<RouteContext>& route_contexts
) const {
    FeasibilityResult result;
    
    const auto& route = solution.get_route(vehicle);
    const auto& context = route_contexts[vehicle];
    
    // Validate position bounds
    if (position < 1 || position >= static_cast<int>(route.size())) {
        result.feasible = false;
        return result;
    }
    
    NodeId prev = route[position - 1];
    NodeId next = route[position];
    Time departure_from_prev = context.departure_times[position - 1];
    
    // Compute times for the inserted customer
    auto times = compute_insertion_times(problem, prev, customer, departure_from_prev);
    Time arrival_at_customer = std::get<0>(times);
    Time departure_from_customer = std::get<1>(times);
    
    // Check if the customer's time window is violated
    const auto& tw_customer = problem.get_time_window(customer);
    if (arrival_at_customer > tw_customer.closing) {
        result.feasible = false;
        result.travel_time_cost = 0.0;
        result.arrival_time_at_customer = arrival_at_customer;
        return result;
    }
    
    // Compute travel times for cost calculation
    Time old_travel_time = problem.get_travel_time(prev, next, departure_from_prev);
    Time new_travel_time_to_customer = problem.get_travel_time(prev, customer, departure_from_prev);
    Time new_travel_time_from_customer = problem.get_travel_time(customer, next, departure_from_customer);
    Time travel_time_cost = (new_travel_time_to_customer + new_travel_time_from_customer) - old_travel_time;
    
    // Check time budget constraint (if applicable)
    Time budget = problem.get_budget();
    if (context.cumulative_time + travel_time_cost > budget) {
        result.feasible = false;
        result.travel_time_cost = travel_time_cost;
        return result;
    }
    
    if (problem.has_time_windows()){
        // Check if next node can still be reached within its time window
        Time arrival_at_next = departure_from_customer + new_travel_time_from_customer;
        const auto& tw_next = problem.get_time_window(next);
        
        if (arrival_at_next > tw_next.closing) {
            result.feasible = false;
            result.travel_time_cost = travel_time_cost;
            result.arrival_time_at_customer = arrival_at_customer;
            result.departure_time_from_customer = departure_from_customer;
            return result;
        }
        
        // Optional: Check max_shift constraint if available
        // max_shift[position-1] indicates how much delay that node can tolerate
        if (position > 0 && context.max_shift[position - 1] < std::numeric_limits<Time>::max()) {
            Time time_shift = (departure_from_customer + new_travel_time_from_customer) - 
                            (departure_from_prev + old_travel_time);
            // Allow small numerical tolerance
            if (time_shift > context.max_shift[position - 1] + 1e-6) {
                result.feasible = false;
                result.travel_time_cost = travel_time_cost;
                result.arrival_time_at_customer = arrival_at_customer;
                result.departure_time_from_customer = departure_from_customer;
                return result;
            }
        }
    }
    
    // All constraints satisfied
    result.feasible = true;
    result.travel_time_cost = travel_time_cost;
    result.arrival_time_at_customer = arrival_at_customer;
    result.departure_time_from_customer = departure_from_customer;
    return result;
}

std::pair<Time, Time> GreedySolver::compute_insertion_times(
    const model::Problem& problem,
    NodeId prev,
    NodeId customer,
    Time departure_from_prev
) const {
    // Compute arrival times using the problem's travel time function
    Time travel_time = problem.get_travel_time(prev, customer, departure_from_prev);
    Time arrival_at_customer = departure_from_prev + travel_time;
    
    // Respect time window opening
    const auto& tw = problem.get_time_window(customer);
    Time start_service = std::max(arrival_at_customer, tw.opening);
    Time departure_from_customer = start_service + problem.get_service_time(customer);
    
    return {arrival_at_customer, departure_from_customer};
}


RouteContext GreedySolver::rebuild_route_context(
    const model::Problem& problem,
    const std::vector<NodeId>& route
) const {
    RouteContext ctx;
    ctx.arrival_times.clear();
    ctx.departure_times.clear();
    ctx.max_shift.clear();
    ctx.cumulative_time = 0.0;
    
    if (route.empty()) return ctx;
    
    // Forward pass: compute arrival/departure times for each node in route
    for (size_t i = 0; i < route.size(); ++i) {
        NodeId node = route[i];
        
        if (i == 0) {
            // Source depot: arrive at time 0
            ctx.arrival_times.push_back(0.0);
            ctx.departure_times.push_back(0.0);
        } else {
            // Compute arrival from predecessor
            NodeId prev_node = route[i - 1];
            Time departure_from_prev = ctx.departure_times[i - 1];
            Time travel_time = problem.get_travel_time(prev_node, node, departure_from_prev);
            Time arrival_at_node = departure_from_prev + travel_time;
            
            // Respect time window opening
            const auto& tw = problem.get_time_window(node);
            Time start_service = std::max(arrival_at_node, tw.opening);
            Time departure_from_node = start_service + problem.get_service_time(node);
            
            ctx.arrival_times.push_back(arrival_at_node);
            ctx.departure_times.push_back(departure_from_node);
        }
    }
    
    // Backward pass: compute max_shift for each position
    ctx.max_shift.assign(route.size(), std::numeric_limits<Time>::max());
    
    for (int i = static_cast<int>(route.size()) - 1; i >= 0; --i) {
        const auto& tw = problem.get_time_window(route[i]);
        Time slack = tw.closing - ctx.departure_times[i];
        
        if (i == static_cast<int>(route.size()) - 1) {
            // Last position: slack from its time window
            ctx.max_shift[i] = slack;
        } else {
            // Limited by successor's max_shift and own slack
            ctx.max_shift[i] = std::min(ctx.max_shift[i + 1], slack);
        }
    }
    
    // Cumulative travel time: sum of travel costs
    for (size_t i = 1; i < route.size(); ++i) {
        NodeId prev = route[i - 1];
        NodeId curr = route[i];
        Time departure_from_prev = ctx.departure_times[i - 1];
        ctx.cumulative_time += problem.get_travel_time(prev, curr, departure_from_prev);
    }
    
    return ctx;
}

model::Solution GreedySolver::solve_from_partial(
    const model::Problem& problem,
    model::Solution partial_solution,
    const GreedySolverConfig& config
) {
    move_evaluator = create_move_evaluator(config.move_evaluator);
    
    int num_vehicles = problem.get_num_vehicles();
    std::vector<bool> visited(problem.get_num_nodes(), false);
    
    // Mark depots as visited
    visited[problem.get_source_depot()] = true;
    visited[problem.get_sink_depot()] = true;
    
    // Reconstruct route contexts from partial solution
    std::vector<RouteContext> route_contexts(num_vehicles);
    for (int v = 0; v < num_vehicles; ++v) {
        const auto& route = partial_solution.get_route(v);
        route_contexts[v] = rebuild_route_context(problem, route);
        
        // Mark nodes in this route as visited
        for (int i = 1; i < static_cast<int>(route.size()) - 1; ++i) {
            visited[route[i]] = true;
        }
    }
    
    infeasibility_cache.clear();
    
    // Continue greedy insertion from partial solution
    while (true) {
        auto best_move = find_best_insertion(problem, partial_solution, visited, route_contexts);
        if (!best_move.feasible) break;
        
        apply_insertion(problem, partial_solution, best_move, route_contexts, visited);
        
        partial_solution.total_reward += problem.get_reward(best_move.customer);
        partial_solution.total_travel_time += best_move.time_shift;
    }
    
    return partial_solution;
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
    
    // Also insert time tracking vectors at insertion point
    context.arrival_times.insert(context.arrival_times.begin() + move.position, move.arrival_time);
    context.departure_times.insert(context.departure_times.begin() + move.position, move.departure_time);
    context.max_shift.insert(context.max_shift.begin() + move.position, 0.0);
    
    // Update successors and incrementally update their max_shift
    // Forward pass: propagate time downstream and compute max_shift until shift becomes 0
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
        
        // Update max_shift for successor:
        // max_shift[i] indicates how much slack is available before violating downstream constraints
        // This is the slack at the time window closing minus current departure time
        Time slack = tw.closing - departure_time;
        context.max_shift[i] = slack;
        
        // If slack becomes 0 or negative, successor nodes can't shift further
        if (slack <= 0.0) {
            break;
        }
    }
    
    // Backward pass: update max_shift for predecessors
    // Each predecessor's max_shift is limited by its successor's slack and its own time window
    for (int i = move.position - 1; i >= 0; --i) {
        const auto& tw_curr = problem.get_time_window(route[i]);
        // Max shift is limited by successor's max shift
        Time max_shift_from_successor = context.max_shift[i + 1];
        // And also limited by available slack in current node's time window
        Time own_slack = tw_curr.closing - context.departure_times[i];
        context.max_shift[i] = std::min(max_shift_from_successor, own_slack);
        
        // If max_shift becomes 0, predecessors have no flexibility
        if (context.max_shift[i] < 0.0) {
            std::cerr << "Warning: max_shift became negative at position " << i << " in vehicle " << move.vehicle << std::endl;
            throw std::runtime_error("Negative max_shift indicates an error in time window calculations");
        }
    }
    
    // Update cumulative time
    context.cumulative_time += move.time_shift;
    
    // Mark as visited
    visited[move.customer] = true;
}

} // namespace oplib::solver::constructive
