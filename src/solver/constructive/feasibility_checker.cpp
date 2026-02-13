#include <algorithm>
#include <cmath>
#include "solver/constructive/feasibility_checker.h"
#include "model/problem.h"
#include "model/solution.h"

namespace oplib::solver::constructive {

FeasibilityResult DistanceFeasibilityChecker::check_feasibility(
    const Problem& problem,
    const model::Solution& solution,
    int vehicle,
    NodeId customer,
    int position,
    const std::vector<Time>& arrival_times,
    const std::vector<Time>& departure_times,
    Time cumulative_time
) const {
    FeasibilityResult result;
    
    const auto& route = solution.get_route(vehicle);
    
    // Cannot insert at invalid positions
    if (position < 1 || position >= static_cast<int>(route.size())) {
        result.feasible = false;
        return result;
    }
    
    NodeId prev = route[position - 1];
    NodeId next = route[position];
    
    // Calculate travel time cost of insertion
    Time old_edge_time = problem.get_distance(prev, next);
    Time new_edges_time = problem.get_distance(prev, customer) + 
                          problem.get_distance(customer, next);
    Time added_time = new_edges_time - old_edge_time;
    
    // Check if new total time exceeds budget
    Time budget = problem.get_budget();
    Time new_total_time = cumulative_time + added_time;
    
    if (new_total_time > budget) {
        result.feasible = false;
        result.travel_time_cost = added_time;
        return result;
    }
    
    result.feasible = true;
    result.travel_time_cost = added_time;
    result.time_cost = 0.0; // No additional time tracking for pure distance variants
    
    return result;
}

FeasibilityResult TimeWindowFeasibilityChecker::check_feasibility(
    const Problem& problem,
    const model::Solution& solution,
    int vehicle,
    NodeId customer,
    int position,
    const std::vector<Time>& arrival_times,
    const std::vector<Time>& departure_times,
    Time cumulative_time
) const {
    FeasibilityResult result;
    
    const auto& route = solution.get_route(vehicle);
    
    // Cannot insert at invalid positions
    if (position < 1 || position >= static_cast<int>(route.size())) {
        result.feasible = false;
        return result;
    }
    
    NodeId prev = route[position - 1];
    NodeId next = route[position];
    
    // First, check time budget constraint
    Time old_edge_time = problem.get_distance(prev, next);
    Time new_edges_time = problem.get_distance(prev, customer) + 
                          problem.get_distance(customer, next);
    Time added_time = new_edges_time - old_edge_time;
    Time budget = problem.get_budget();
    Time new_total_time = cumulative_time + added_time;
    
    if (new_total_time > budget) {
        result.feasible = false;
        result.travel_time_cost = added_time;
        return result;
    }
    
    // Check time window constraints
    // Compute arrival and departure times at customer
    Time departure_from_prev = departure_times[position - 1];
    auto [arrival_at_customer, departure_from_customer] = 
        compute_times(problem, prev, customer, departure_from_prev);
    
    // Check if arrival at customer respects time window
    const auto& tw_customer = problem.get_time_window(customer);
    if (arrival_at_customer > tw_customer.closing) {
        result.feasible = false;
        result.travel_time_cost = added_time;
        result.arrival_time_at_customer = arrival_at_customer;
        return result;
    }
    
    // Ensure we start service within time window
    Time start_service = std::max(arrival_at_customer, tw_customer.opening);
    Time actual_departure = start_service + problem.get_service_time(customer);
    
    // Check if next node can still be reached within its time window
    auto [arrival_at_next, departure_from_next] = 
        compute_times(problem, customer, next, actual_departure);
    
    const auto& tw_next = problem.get_time_window(next);
    if (arrival_at_next > tw_next.closing) {
        result.feasible = false;
        result.travel_time_cost = added_time;
        result.arrival_time_at_customer = arrival_at_customer;
        result.departure_time_from_customer = actual_departure;
        return result;
    }
    
    // All constraints satisfied
    result.feasible = true;
    result.travel_time_cost = added_time;
    result.arrival_time_at_customer = arrival_at_customer;
    result.departure_time_from_customer = actual_departure;
    result.time_cost = actual_departure - departure_from_prev;  // Total time added to route
    
    return result;
}

std::pair<Time, Time> TimeWindowFeasibilityChecker::compute_times(
    const Problem& problem,
    NodeId prev_node,
    NodeId current_node,
    Time departure_from_prev
) const {
    Time travel_time = problem.get_travel_time(prev_node, current_node, departure_from_prev);
    Time arrival = departure_from_prev + travel_time;
    
    // Respect time window opening
    const auto& tw = problem.get_time_window(current_node);
    Time start_service = std::max(arrival, tw.opening);
    Time departure = start_service + problem.get_service_time(current_node);
    
    return {arrival, departure};
}

std::pair<Time, Time> TimeDependentFeasibilityChecker::compute_times(
    const Problem& problem,
    NodeId prev_node,
    NodeId current_node,
    Time departure_from_prev
) const {
    // For time-dependent problems, travel time depends on departure time
    Time travel_time = problem.get_travel_time(prev_node, current_node, departure_from_prev);
    Time arrival = departure_from_prev + travel_time;
    
    // Respect time window opening
    const auto& tw = problem.get_time_window(current_node);
    Time start_service = std::max(arrival, tw.opening);
    Time departure = start_service + problem.get_service_time(current_node);
    
    return {arrival, departure};
}

} // namespace oplib::solver::constructive
