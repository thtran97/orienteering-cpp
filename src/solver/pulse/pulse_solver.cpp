#include <algorithm>
#include <iostream>
#include <numeric>

#include "solver/pulse/pulse_solver.h"

namespace oplib::solver::pulse {

// ---------------------------------------------------------------------------
// Recursive pulse
// ---------------------------------------------------------------------------

void PulseSolver::pulse(NodeId node, SearchState& state) const
{
    const model::Problem& problem = *state.problem;
    const NodeId sink = problem.get_sink_depot();

    // ---- Bound 3: reward bound ----
    Reward potential = state.current_reward + (*state.ub)[node];
    if (potential <= state.best_reward) return;

    ++state.pulses_launched;
    if (state.max_labels > 0 && state.pulses_launched > state.max_labels) return;

    // ---- Reached sink ----
    if (node == sink) {
        if (state.current_reward > state.best_reward) {
            state.best_reward = state.current_reward;
            state.best_path   = state.current_path;
        }
        return;
    }

    const int nn = static_cast<int>(problem.get_num_nodes());

    // Explore successors ordered by reward (greedy ordering)
    std::vector<std::pair<Reward, NodeId>> nexts;
    nexts.reserve(nn);
    for (NodeId j = 1; j < nn; ++j) {
        if (state.visited[j]) continue;

        Time travel  = problem.get_travel_time(node, j, state.current_time);
        Time arrival = state.current_time + travel;
        const auto& tw_j = problem.get_time_window(j);

        // ---- Bound 1: time window ----
        if (arrival > tw_j.closing) continue;

        Time dep_j = std::max(arrival, tw_j.opening) + problem.get_service_time(j);

        // ---- Bound 2: budget (remaining) ----
        // Check we can still reach sink from j
        Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
        const auto& tw_sink = problem.get_time_window(sink);
        if (arr_sink > tw_sink.closing) continue;

        Time total = dep_j - problem.get_time_window(problem.get_source_depot()).opening
                     + problem.get_travel_time(j, sink, dep_j);
        if (total > problem.get_budget()) continue;

        nexts.emplace_back(problem.get_reward(j) + (*state.ub)[j], j);
    }

    // Sort descending by potential (greedy-first expansion)
    std::sort(nexts.begin(), nexts.end(),
              [](const auto& a, const auto& b){ return a.first > b.first; });

    for (auto [pot, j] : nexts) {
        Time travel  = problem.get_travel_time(node, j, state.current_time);
        Time arrival = state.current_time + travel;
        const auto& tw_j = problem.get_time_window(j);
        Time dep_j   = std::max(arrival, tw_j.opening) + problem.get_service_time(j);

        // Push state
        state.visited[j]   = true;
        state.current_path.push_back(j);
        Time saved_time    = state.current_time;
        Reward saved_reward = state.current_reward;

        state.current_time   = dep_j;
        state.current_reward += (j == sink ? 0.0 : problem.get_reward(j));

        pulse(j, state);

        // Pop state
        state.visited[j]     = false;
        state.current_path.pop_back();
        state.current_time   = saved_time;
        state.current_reward = saved_reward;

        if (state.max_labels > 0 && state.pulses_launched > state.max_labels) break;
    }
}

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

model::Solution PulseSolver::solve(const model::Problem& problem,
                                    const SolverConfig&   config)
{
    // Forward to the typed overload, preserving PulseSolverConfig defaults
    // (max_labels = 1 000 000).  Keeping it unlimited (0) here would make
    // every call via the base Solver interface potentially run forever on
    // large instances.
    PulseSolverConfig ps_cfg;
    ps_cfg.seed         = config.seed;
    ps_cfg.max_cpu_time = config.max_cpu_time;
    ps_cfg.verbose      = config.verbose;
    // ps_cfg.max_labels   retains its default (1 000 000)
    return solve(problem, ps_cfg);
}

model::Solution PulseSolver::solve(const model::Problem&    problem,
                                    const PulseSolverConfig& config)
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // Pre-compute backward bounds
    dp::BackwardDPSolver backward;
    std::vector<Reward> ub = backward.compute_bounds(problem);

    // Set up search state
    SearchState state;
    state.problem     = &problem;
    state.ub          = &ub;
    state.visited.assign(nn, false);
    state.visited[src]  = true;
    state.visited[sink] = false; // sink reachable but not "visited" until we arrive
    state.max_labels  = config.max_labels;

    // Source setup
    const auto& tw_src = problem.get_time_window(src);
    state.current_time   = tw_src.opening + problem.get_service_time(src);
    state.current_reward = 0.0;
    state.current_path   = {src};

    pulse(src, state);

    if (config.verbose)
        std::cout << "[Pulse] best=" << state.best_reward
                  << " pulses=" << state.pulses_launched << '\n';

    // Build solution from best path
    model::Solution sol(problem.get_num_vehicles());
    if (state.best_path.empty()) {
        sol.get_route(0) = {src, sink};
        return sol;
    }

    // Ensure path starts with src and ends with sink
    if (state.best_path.back() != sink)
        state.best_path.push_back(sink);

    sol.get_route(0) = state.best_path;
    sol.total_reward = state.best_reward;
    sol.total_travel_time = 0.0;
    const auto& route = sol.get_route(0);
    for (size_t i = 1; i < route.size(); ++i)
        sol.total_travel_time += problem.get_distance(route[i-1], route[i]);

    return sol;
}

} // namespace oplib::solver::pulse
