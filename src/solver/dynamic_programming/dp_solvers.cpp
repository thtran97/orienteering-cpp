#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>
#include <sstream>

#include "solver/dynamic_programming/dp_solvers.h"

namespace oplib::solver::dp {

// ---------------------------------------------------------------------------
// Label::to_string
// ---------------------------------------------------------------------------

std::string Label::to_string() const
{
    std::ostringstream ss;
    ss << "Label<n=" << nb_visits
       << " t=" << time_consumed
       << " p=" << profit_collected
       << " @" << last_visit << ">";
    return ss.str();
}

// ---------------------------------------------------------------------------
// ForwardDPSolver
// ---------------------------------------------------------------------------

/*static*/
model::Solution ForwardDPSolver::reconstruct(const Label* best,
                                              const model::Problem& problem)
{
    // Walk the parent chain to build the route
    std::vector<NodeId> rev_path;
    const Label* cur = best;
    while (cur != nullptr) {
        rev_path.push_back(cur->last_visit);
        cur = cur->parent;
    }
    std::reverse(rev_path.begin(), rev_path.end());

    model::Solution sol(problem.get_num_vehicles());
    sol.get_route(0) = rev_path;

    // Compute total reward and travel time
    sol.total_reward = 0.0;
    sol.total_travel_time = 0.0;
    const auto& route = sol.get_route(0);
    for (size_t i = 1; i < route.size(); ++i) {
        sol.total_travel_time += problem.get_distance(route[i-1], route[i]);
    }
    for (size_t i = 1; i + 1 < route.size(); ++i) { // skip depots
        sol.total_reward += problem.get_reward(route[i]);
    }
    return sol;
}

model::Solution ForwardDPSolver::solve(const model::Problem& problem,
                                        const SolverConfig&   config)
{
    DPSolverConfig dp_cfg;
    dp_cfg.seed        = config.seed;
    dp_cfg.max_cpu_time = config.max_cpu_time;
    dp_cfg.verbose     = config.verbose;
    return solve(problem, dp_cfg);
}

model::Solution ForwardDPSolver::solve(const model::Problem& problem,
                                        const DPSolverConfig& config)
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // Label pool (manage lifetime)
    std::vector<std::unique_ptr<Label>> label_pool;

    // Per-node label lists for dominance pruning
    std::vector<std::vector<Label*>> node_labels(nn);

    // Min-heap ordered by time_consumed
    auto cmp = [](Label* a, Label* b) { return a->time_consumed > b->time_consumed; };
    std::priority_queue<Label*, std::vector<Label*>, decltype(cmp)> pq(cmp);

    // Source label
    {
        std::vector<bool> vis(nn, false);
        vis[src]  = true;
        const auto& tw0 = problem.get_time_window(src);
        auto* lbl = new Label(src,
                              tw0.opening + problem.get_service_time(src),
                              0.0, nullptr, std::move(vis));
        label_pool.emplace_back(lbl);
        node_labels[src].push_back(lbl);
        pq.push(lbl);
    }

    Label* best_sink_label = nullptr;
    int    labels_explored = 0;

    while (!pq.empty()) {
        Label* li = pq.top(); pq.pop();
        if (li->dominated || li->extended) continue;
        li->extended = true;

        ++labels_explored;
        if (config.max_labels > 0 && labels_explored > config.max_labels) break;

        // Extend to all unvisited feasible nodes
        for (NodeId j = 1; j < nn; ++j) {
            if (li->is_visited[j]) continue;

            Time travel  = problem.get_travel_time(li->last_visit, j, li->time_consumed);
            Time arrival = li->time_consumed + travel;
            const auto& tw_j = problem.get_time_window(j);

            if (arrival > tw_j.closing) continue;

            Time dep_j = std::max(arrival, tw_j.opening) + problem.get_service_time(j);

            // Budget check (total time from source)
            Time total_time = dep_j - problem.get_time_window(src).opening
                              + problem.get_travel_time(j, sink, dep_j);
            if (total_time > problem.get_budget()) continue;

            // Check we can still reach sink from j
            Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
            if (arr_sink > problem.get_time_window(sink).closing) continue;

            Reward new_profit = li->profit_collected
                                + (j == sink ? 0.0 : problem.get_reward(j));

            std::vector<bool> new_vis = li->is_visited;
            new_vis[j] = true;

            auto* lj = new Label(j, dep_j, new_profit, li, std::move(new_vis));

            // Dominance check against existing labels at j
            bool dominated = false;
            for (Label* existing : node_labels[j]) {
                if (existing->dominated) continue;
                if (existing->dominates(*lj)) { dominated = true; break; }
                if (lj->dominates(*existing)) existing->dominated = true;
            }
            if (dominated) { delete lj; continue; }

            label_pool.emplace_back(lj);
            node_labels[j].push_back(lj);
            pq.push(lj);

            if (j == sink) {
                if (best_sink_label == nullptr
                    || lj->profit_collected > best_sink_label->profit_collected)
                    best_sink_label = lj;
            }
        }
    }

    if (best_sink_label == nullptr) {
        // Return empty feasible solution (just depots)
        model::Solution sol(problem.get_num_vehicles());
        sol.get_route(0) = {src, sink};
        return sol;
    }

    if (config.verbose)
        std::cout << "[ForwardDP] best=" << best_sink_label->profit_collected
                  << " labels=" << labels_explored << '\n';

    return reconstruct(best_sink_label, problem);
}

// ---------------------------------------------------------------------------
// BackwardDPSolver — compute upper bounds per node
// ---------------------------------------------------------------------------

std::vector<Reward> BackwardDPSolver::compute_bounds(const model::Problem& problem) const
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // ub[i] = a valid UPPER bound on the reward collectable on a path that
    // continues from node i to the sink (excluding i's own reward).
    //
    // Relaxation: sum the rewards of every customer j that is individually
    // still collectable after i, i.e.
    //   (1) i -> j is time-window feasible at the earliest departure from i, and
    //   (2) j -> sink is feasible within the sink's window and the time budget.
    // Summing all such j over-counts (a real route cannot visit them all), so
    // it is a sound upper bound. Using the earliest possible departure from i
    // is the most optimistic assumption, keeping the bound valid for any real
    // partial path that arrives at i later.
    //
    // Validity of the per-customer reachability test relies on the triangle
    // inequality of travel times (true for Euclidean OP/OPTW): if j is
    // reachable from i via an intermediate node it is also reachable directly,
    // so no collectable customer is ever excluded (no underestimate). The
    // previous path-recursion was invalid on the cyclic graph and produced
    // underestimates, which made Pulse/BidirectionalDP prune optimal paths.
    std::vector<Reward> ub(nn, 0.0);

    const Time budget       = problem.get_budget();
    const Time src_open      = problem.get_time_window(src).opening;
    const Time sink_closing  = problem.get_time_window(sink).closing;

    for (int i = 0; i < nn; ++i) {
        if (i == sink) { ub[i] = 0.0; continue; }

        const Time dep_i = problem.get_time_window(i).opening
                           + problem.get_service_time(i);
        Reward sum = 0.0;

        for (int j = 0; j < nn; ++j) {
            if (j == i || j == src || j == sink) continue;

            // (1) i -> j reachable before j closes (earliest departure from i).
            const Time arr_j = dep_i + problem.get_travel_time(i, j, dep_i);
            const auto& tw_j = problem.get_time_window(j);
            if (arr_j > tw_j.closing) continue;

            // (2) j -> sink feasible (sink window + total time budget).
            const Time dep_j = std::max(arr_j, tw_j.opening)
                               + problem.get_service_time(j);
            const Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
            if (arr_sink > sink_closing) continue;
            if (arr_sink - src_open > budget) continue;

            sum += problem.get_reward(j);
        }
        ub[i] = sum;
    }
    return ub;
}

model::Solution BackwardDPSolver::solve(const model::Problem& problem,
                                         const SolverConfig&   config)
{
    DPSolverConfig dp_cfg;
    dp_cfg.seed        = config.seed;
    dp_cfg.max_cpu_time = config.max_cpu_time;
    dp_cfg.verbose     = config.verbose;
    return solve(problem, dp_cfg);
}

model::Solution BackwardDPSolver::solve(const model::Problem& problem,
                                         const DPSolverConfig& config)
{
    // BackwardDP is primarily a bounding oracle.
    // For solve(), delegate to ForwardDP to return an actual solution.
    ForwardDPSolver fwd;
    DPSolverConfig fwd_cfg = config;
    return fwd.solve(problem, fwd_cfg);
}

// ---------------------------------------------------------------------------
// BidirectionalDPSolver
// ---------------------------------------------------------------------------

model::Solution BidirectionalDPSolver::solve(const model::Problem& problem,
                                              const SolverConfig&   config)
{
    DPSolverConfig dp_cfg;
    dp_cfg.seed        = config.seed;
    dp_cfg.max_cpu_time = config.max_cpu_time;
    dp_cfg.verbose     = config.verbose;
    return solve(problem, dp_cfg);
}

model::Solution BidirectionalDPSolver::solve(const model::Problem& problem,
                                              const DPSolverConfig& config)
{
    // Compute backward bounds first (used to prune forward labels)
    BackwardDPSolver backward;
    std::vector<Reward> ub = backward.compute_bounds(problem);

    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    std::vector<std::unique_ptr<Label>> label_pool;
    std::vector<std::vector<Label*>>    node_labels(nn);

    auto cmp = [](Label* a, Label* b) { return a->time_consumed > b->time_consumed; };
    std::priority_queue<Label*, std::vector<Label*>, decltype(cmp)> pq(cmp);

    {
        std::vector<bool> vis(nn, false);
        vis[src] = true;
        const auto& tw0 = problem.get_time_window(src);
        auto* lbl = new Label(src,
                              tw0.opening + problem.get_service_time(src),
                              0.0, nullptr, std::move(vis));
        label_pool.emplace_back(lbl);
        node_labels[src].push_back(lbl);
        pq.push(lbl);
    }

    Label* best_sink_label = nullptr;
    int    labels_explored = 0;

    while (!pq.empty()) {
        Label* li = pq.top(); pq.pop();
        if (li->dominated || li->extended) continue;
        li->extended = true;
        ++labels_explored;
        if (config.max_labels > 0 && labels_explored > config.max_labels) break;

        for (NodeId j = 1; j < nn; ++j) {
            if (li->is_visited[j]) continue;
            Time travel  = problem.get_travel_time(li->last_visit, j, li->time_consumed);
            Time arrival = li->time_consumed + travel;
            const auto& tw_j = problem.get_time_window(j);
            if (arrival > tw_j.closing) continue;
            Time dep_j = std::max(arrival, tw_j.opening) + problem.get_service_time(j);

            // Budget check
            Time total_time = dep_j - problem.get_time_window(src).opening
                              + problem.get_travel_time(j, sink, dep_j);
            if (total_time > problem.get_budget()) continue;

            // Backward bound pruning
            Reward potential = li->profit_collected
                               + (j == sink ? 0.0 : problem.get_reward(j))
                               + ub[j];
            if (best_sink_label && potential <= best_sink_label->profit_collected) continue;

            Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
            if (arr_sink > problem.get_time_window(sink).closing) continue;

            Reward new_profit = li->profit_collected
                                + (j == sink ? 0.0 : problem.get_reward(j));

            std::vector<bool> new_vis = li->is_visited;
            new_vis[j] = true;

            auto* lj = new Label(j, dep_j, new_profit, li, std::move(new_vis));

            bool dominated = false;
            for (Label* existing : node_labels[j]) {
                if (existing->dominated) continue;
                if (existing->dominates(*lj)) { dominated = true; break; }
                if (lj->dominates(*existing)) existing->dominated = true;
            }
            if (dominated) { delete lj; continue; }

            label_pool.emplace_back(lj);
            node_labels[j].push_back(lj);
            pq.push(lj);

            if (j == sink) {
                if (best_sink_label == nullptr
                    || lj->profit_collected > best_sink_label->profit_collected)
                    best_sink_label = lj;
            }
        }
    }

    if (config.verbose)
        std::cout << "[BidirectionalDP] best="
                  << (best_sink_label ? best_sink_label->profit_collected : 0.0)
                  << " labels=" << labels_explored << '\n';

    if (best_sink_label == nullptr) {
        model::Solution sol(problem.get_num_vehicles());
        sol.get_route(0) = {src, sink};
        return sol;
    }

    return ForwardDPSolver::reconstruct(best_sink_label, problem);
}

} // namespace oplib::solver::dp
