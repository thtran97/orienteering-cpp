#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>

#include "solver/dynamic_programming/dp_solvers.h"

namespace oplib::solver::dp {

// ---------------------------------------------------------------------------
// TrueBackwardDPSolver
// ---------------------------------------------------------------------------
//
// Convention: a backward label at node i stores
//   time_consumed = backward residual time τ_bw (0 at sink, grows toward source)
//
// Backward time window for customer i (i ≠ source, sink):
//   bw_open  = T − (b_i + s_i)
//   bw_close = T − (a_i + s_i)
// A backward label at i is feasible iff τ_bw ≤ bw_close.
//
// Backward extension from label at j to predecessor i:
//   τ' = max{ τ_bw + s_j + t_ij,  T − (b_i + s_i) }   (= bw_open_i at earliest)
// Feasible iff τ' ≤ T − (a_i + s_i)                    (= bw_close_i)
//
// The source depot has s_src = 0 and [a_src, b_src] = [0, T], so
//   bw_open_src  = T − T = 0
//   bw_close_src = T − 0 = T  (always feasible at source)

// ---------------------------------------------------------------------------

std::vector<std::vector<Label*>>
TrueBackwardDPSolver::compute_backward_labels(
    const model::Problem&                 problem,
    const DPSolverConfig&                 config,
    std::vector<std::unique_ptr<Label>>&  pool) const
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // T = effective time horizon: the tighter of the sink's closing window and
    // src_opening + budget.  OPProblem has no explicit time windows (default 1e18),
    // so we must cap with the budget to avoid treating all paths as feasible.
    const Time T = std::min(
        problem.get_time_window(sink).closing,
        problem.get_time_window(src).opening + problem.get_budget()
    );

    pool.clear();
    std::vector<std::vector<Label*>> node_labels(nn);

    // Min-heap on backward residual time (smallest residual = most time left)
    auto cmp = [](Label* a, Label* b) { return a->time_consumed > b->time_consumed; };
    std::priority_queue<Label*, std::vector<Label*>, decltype(cmp)> pq(cmp);

    // Seed: backward label at sink with τ_bw = 0, P = 0
    {
        std::vector<bool> vis(nn, false);
        vis[sink] = true;
        auto* lbl = new Label(sink, 0.0, 0.0, nullptr, std::move(vis));
        pool.emplace_back(lbl);
        node_labels[sink].push_back(lbl);
        pq.push(lbl);
    }

    int explored = 0;

    while (!pq.empty()) {
        Label* lj = pq.top(); pq.pop();
        if (lj->dominated || lj->extended) continue;
        lj->extended = true;
        ++explored;
        if (config.max_labels > 0 && explored > config.max_labels) break;

        const NodeId j      = lj->last_visit;
        const Time   tau_bw = lj->time_consumed;
        const Time   sj     = problem.get_service_time(j);

        // Extend backward to every unvisited node i (including source, skip sink)
        for (NodeId i = 0; i < nn; ++i) {
            if (i == sink || lj->is_visited[i]) continue;

            const Time   si  = problem.get_service_time(i);
            // t_ij is the forward arc i→j (in actual path we travel i→j to reach j)
            const Time   tij = problem.get_travel_time(i, j, 0.0);
            const auto&  twi = problem.get_time_window(i);

            // Backward time window for i
            const Time bw_open_i  = T - (twi.closing + si);
            const Time bw_close_i = T - (twi.opening + si);

            if (bw_open_i > bw_close_i) continue; // infeasible window (twi too tight)

            // Backward transition
            Time tau_prime = std::max(tau_bw + sj + tij, bw_open_i);
            if (tau_prime > bw_close_i) continue; // infeasible at i

            // Prize of i (source and sink have reward 0)
            Reward ri = (i == src) ? 0.0 : problem.get_reward(i);
            Reward new_profit = lj->profit_collected + ri;

            std::vector<bool> new_vis = lj->is_visited;
            new_vis[i] = true;

            auto* li = new Label(i, tau_prime, new_profit, lj, std::move(new_vis));

            // Dominance against existing backward labels at i
            bool dominated = false;
            for (Label* ex : node_labels[i]) {
                if (ex->dominated) continue;
                if (ex->dominates(*li)) { dominated = true; break; }
                if (li->dominates(*ex)) ex->dominated = true;
            }
            if (dominated) { delete li; continue; }

            pool.emplace_back(li);
            node_labels[i].push_back(li);
            pq.push(li);
        }
    }

    if (config.verbose)
        std::cout << "[TrueBackwardDP] backward_labels explored=" << explored << '\n';

    return node_labels;
}

// ---------------------------------------------------------------------------

/*static*/
model::Solution TrueBackwardDPSolver::reconstruct(const Label* best,
                                                    const model::Problem& problem)
{
    // The backward parent chain runs: source → ... → sink
    // (each parent points "toward the sink" in the backward sense)
    // Walking parent pointers from best gives path from source to sink directly.
    std::vector<NodeId> path;
    for (const Label* cur = best; cur != nullptr; cur = cur->parent)
        path.push_back(cur->last_visit);
    // path is already in source→sink order (backward labels extend toward source,
    // so the chain from a source-arriving label goes: source ← ... ← sink,
    // meaning iterating parent from "best at source" gives source first).
    // Verify by checking first element is source.
    // (No reversal needed — see convention in compute_backward_labels.)

    model::Solution sol(problem.get_num_vehicles());
    sol.get_route(0) = path;

    sol.total_reward = 0.0;
    sol.total_travel_time = 0.0;
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();
    for (size_t k = 1; k < path.size(); ++k)
        sol.total_travel_time += problem.get_distance(path[k-1], path[k]);
    for (size_t k = 1; k + 1 < path.size(); ++k) {
        NodeId n = path[k];
        if (n != src && n != sink)
            sol.total_reward += problem.get_reward(n);
    }
    return sol;
}

// ---------------------------------------------------------------------------

model::Solution TrueBackwardDPSolver::solve(const model::Problem& problem,
                                             const SolverConfig&   config)
{
    DPSolverConfig dp;
    dp.seed = config.seed;
    dp.max_cpu_time = config.max_cpu_time;
    dp.verbose = config.verbose;
    return solve(problem, dp);
}

model::Solution TrueBackwardDPSolver::solve(const model::Problem& problem,
                                             const DPSolverConfig& config)
{
    std::vector<std::unique_ptr<Label>> pool;
    auto node_labels = compute_backward_labels(problem, config, pool);

    const NodeId src = problem.get_source_depot();

    // Best backward label that reached the source
    Label* best = nullptr;
    for (Label* lbl : node_labels[src]) {
        if (lbl->dominated) continue;
        if (best == nullptr || lbl->profit_collected > best->profit_collected)
            best = lbl;
    }

    if (best == nullptr) {
        model::Solution sol(problem.get_num_vehicles());
        sol.get_route(0) = {src, problem.get_sink_depot()};
        return sol;
    }

    if (config.verbose)
        std::cout << "[TrueBackwardDP] best=" << best->profit_collected << '\n';

    return reconstruct(best, problem);
}

} // namespace oplib::solver::dp
