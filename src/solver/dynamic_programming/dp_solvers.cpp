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
    const NodeId sink = problem.get_sink_depot();

    // ub[i] = max reward collectable starting from node i and reaching sink.
    // Relaxation that ignores visited-node constraints (so it is an upper bound).
    //
    // Two improvements over a naive single-pass:
    //  1. Budget-awareness: arcs (i,j) whose minimum detour already exceeds
    //     the budget are excluded, keeping the bound tight and preventing
    //     it from including infeasible detours.
    //  2. Tie-breaking by distance to sink: when TW closing times are equal
    //     (e.g. in plain OP where all windows are [0,∞)), nodes closer to
    //     the sink are processed first.  This ensures their ub values are
    //     available when we compute ub for nodes farther from the sink,
    //     avoiding drastic underestimates that would cause incorrect pruning.
    std::vector<Reward> ub(nn, 0.0);

    const Time budget = problem.get_budget();

    std::vector<int> order(nn);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        double ca = problem.get_time_window(a).closing;
        double cb = problem.get_time_window(b).closing;
        if (std::abs(ca - cb) > 1e-9) return ca > cb;   // larger closing first
        // tie-break: closer to sink first (more "backward" in the route)
        return problem.get_distance(a, sink) < problem.get_distance(b, sink);
    });

    for (int i : order) {
        if (i == sink) { ub[i] = 0.0; continue; }
        Reward best = 0.0;
        Time dep_i = problem.get_time_window(i).opening
                     + problem.get_service_time(i);
        for (int j = 0; j < nn; ++j) {
            if (j == i) continue;
            Time arr_j = dep_i + problem.get_travel_time(i, j, dep_i);
            if (arr_j > problem.get_time_window(j).closing) continue;
            // Budget check: dep_i + travel(i→j) + service(j) + travel(j→sink)
            // must not exceed the total budget (using earliest possible departure
            // from i as the most optimistic assumption).
            Time dep_j = std::max(arr_j, problem.get_time_window(j).opening)
                         + problem.get_service_time(j);
            Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
            if (arr_sink > budget) continue;
            Reward cand = problem.get_reward(j) + ub[j];
            best = std::max(best, cand);
        }
        ub[i] = best;
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
