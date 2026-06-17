#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <queue>
#include <unordered_map>

#include "solver/dynamic_programming/dp_solvers.h"

namespace oplib::solver::dp {

// ---------------------------------------------------------------------------
// DSSRSolver — Decremental State Space Relaxation (Righini & Salani 2006 §4)
// ---------------------------------------------------------------------------
//
// Righini, G., & Salani, M. (2006). Dynamic programming for the orienteering problem with time windows.
//
// Outer loop: maintain a critical vertex set Θ.  Only Θ-nodes enforce
// elementarity in the inner DP; non-critical nodes may be revisited freely.
//
// Inner DP: structurally identical to ForwardDPSolver but with two changes:
//   1. Extension to j: skip the visited check when !critical_set[j]
//   2. Label creation: set new_vis[j] = true only when critical_set[j]
//
// As a result, is_visited only tracks Θ-nodes.  The existing dominates()
// (which checks S1 ⊆ S2) becomes a pure (time, profit) comparison for
// non-Θ positions, since all labels agree on those bits being false —
// exactly the DSSR relaxation: we don't distinguish which non-critical nodes
// were visited (we only care about time and total profit).
//
// Cycle detection: walk the parent chain of the best sink label; count how
// many times each node appears.  Any node with count > 1 is a cycle candidate.
//
// Safety valve: because non-critical nodes re-collect their reward on every
// revisit, a profitable zero/low-cost cycle among non-critical nodes can
// generate astronomically many mutually non-dominated labels (each lap is
// strictly more time *and* more profit than the last, so nothing dominates
// it) before the search exhausts the time budget. config.max_labels bounds
// this when explicitly set (>0); if the cap is hit, the caller falls back to
// a fully elementary run, which is bounded by the (finite) 2^|customers|
// elementary state space — exactly ForwardDPSolver's search space, already
// verified against brute force.
//
// config.max_labels == 0 means "unlimited" (same convention as every other
// DPSolverConfig-based solver): the explored-count cap is disabled and only
// config.max_cpu_time bounds the search. Do NOT substitute a small default
// cap here — callers that pass 0 expect exact/unbounded behavior (see
// test_exact_optimality.cpp, test_dp_improvements.cpp's unlimited_cfg()).

// ---------------------------------------------------------------------------

Label* DSSRSolver::run_inner_dp(
    const model::Problem&                 problem,
    const DSSRSolverConfig&               config,
    const std::vector<bool>&              critical_set,
    std::vector<std::unique_ptr<Label>>&  label_pool,
    std::vector<std::vector<Label*>>&     node_labels,
    bool&                                  hit_cap) const
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    hit_cap = false;

    label_pool.clear();
    for (auto& v : node_labels) v.clear();

    auto cmp = [](Label* a, Label* b) { return a->time_consumed > b->time_consumed; };
    std::priority_queue<Label*, std::vector<Label*>, decltype(cmp)> pq(cmp);

    {
        std::vector<bool> vis(nn, false);
        if (critical_set[src]) vis[src] = true;
        const auto& tw0 = problem.get_time_window(src);
        auto* lbl = new Label(src,
                              tw0.opening + problem.get_service_time(src),
                              0.0, nullptr, std::move(vis));
        label_pool.emplace_back(lbl);
        node_labels[src].push_back(lbl);
        pq.push(lbl);
    }

    Label* best_sink = nullptr;
    int    explored  = 0;
    auto t_start = std::chrono::steady_clock::now();

    while (!pq.empty()) {
        if (config.max_cpu_time > 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > config.max_cpu_time)
            break;
        Label* li = pq.top(); pq.pop();
        if (li->dominated || li->extended) continue;
        li->extended = true;
        ++explored;
        if (config.max_labels > 0 && explored > config.max_labels) { hit_cap = true; break; }

        // Sink is a true terminal: never extend past it. Without this guard,
        // since critical_set[sink] is always false, is_visited[sink] is
        // never tracked, so a label could "pass through" sink and keep
        // visiting more nodes — and because src/sink commonly share the
        // same coordinates (distance 0 between them), many such pass-through
        // variants are exact (time, profit, subset) ties with the direct
        // route, which the strict-dominance check (Phase 1) does not prune,
        // causing label-count blow-up.
        if (li->last_visit == sink) continue;

        for (NodeId j = 1; j < nn; ++j) {
            // A self-loop (j == current node) always costs exactly zero
            // travel time (distance(i,i) == 0 for any metric). For a
            // non-critical j the elementarity check below is skipped
            // entirely, so without this guard a label could re-extend to
            // itself for free, re-collecting its own reward every time —
            // an unbounded zero-cost profit injection, not a real cycle.
            if (j == li->last_visit) continue;

            // Elementarity: enforce only for critical nodes
            if (critical_set[j] && li->is_visited[j]) continue;
            // Non-critical nodes may be revisited (no check)

            Time travel  = problem.get_travel_time(li->last_visit, j, li->time_consumed);
            Time arrival = li->time_consumed + travel;
            const auto& tw_j = problem.get_time_window(j);
            if (arrival > tw_j.closing) continue;

            Time dep_j = std::max(arrival, tw_j.opening) + problem.get_service_time(j);

            // Budget check
            Time total_time = dep_j - problem.get_time_window(src).opening
                              + problem.get_travel_time(j, sink, dep_j);
            if (total_time > problem.get_budget()) continue;

            // Sink reachability
            Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
            if (arr_sink > problem.get_time_window(sink).closing) continue;

            Reward new_profit = li->profit_collected
                                + (j == sink ? 0.0 : problem.get_reward(j));

            std::vector<bool> new_vis = li->is_visited;
            if (critical_set[j]) new_vis[j] = true;  // track only critical nodes

            auto* lj = new Label(j, dep_j, new_profit, li, std::move(new_vis));

            bool dominated = false;
            for (Label* ex : node_labels[j]) {
                if (ex->dominated) continue;
                if (ex->dominates(*lj)) { dominated = true; break; }
                if (lj->dominates(*ex)) ex->dominated = true;
            }
            if (dominated) { delete lj; continue; }

            label_pool.emplace_back(lj);
            node_labels[j].push_back(lj);
            pq.push(lj);

            if (j == sink) {
                if (!best_sink || lj->profit_collected > best_sink->profit_collected)
                    best_sink = lj;
            }
        }
    }

    if (config.verbose)
            std::cerr << "[DSSR][inner] explored=" << explored << " hit_cap=" << hit_cap
                  << " best_sink=" << (best_sink ? best_sink->profit_collected : -1.0) << '\n';

    return best_sink;
}

// ---------------------------------------------------------------------------

std::vector<int> DSSRSolver::count_visits(const Label* sink_label, int nn) const
{
    std::vector<int> counts(nn, 0);
    for (const Label* c = sink_label; c != nullptr; c = c->parent) {
        if (c->last_visit >= 0 && c->last_visit < nn)
            ++counts[c->last_visit];
    }
    return counts;
}

// ---------------------------------------------------------------------------
// MS_m initialization (§5): cycling attractiveness heuristic
//
// f_ij = p_i / (s_i + t_ij + s_j + t_ji)
//
// Four orderings of customers (excluding depots):
//   HCA:  by max_j(f_ij)
//   TCA:  by Σ_j(f_ij)
//   WHCA: by max_j(f_ij * (b_i - a_i))
//   WTCA: by Σ_j(f_ij * (b_i - a_i))
//
// Θ_init = top-m(HCA) ∩ top-m(TCA) ∩ top-m(WHCA) ∩ top-m(WTCA).
// Falls back to empty set if intersection is empty.

std::vector<bool> DSSRSolver::compute_ms_m_init(const model::Problem& problem,
                                                  int m) const
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // Collect customers (non-depot nodes)
    std::vector<NodeId> customers;
    for (NodeId i = 0; i < nn; ++i)
        if (i != src && i != sink) customers.push_back(i);

    if (customers.empty() || m <= 0) return std::vector<bool>(nn, false);

    // Per-customer cycling attractiveness scores
    struct Scores { double hca, tca, whca, wtca; };
    std::vector<Scores> scores(nn, {0, 0, 0, 0});

    for (NodeId i : customers) {
        const auto& twi = problem.get_time_window(i);
        const double window_i = twi.closing - twi.opening;
        const double si = problem.get_service_time(i);
        const double pi = problem.get_reward(i);

        double best_fij = 0.0, total_fij = 0.0;
        double best_wfij = 0.0, total_wfij = 0.0;

        for (NodeId j : customers) {
            if (j == i) continue;
            const double sj  = problem.get_service_time(j);
            const double tij = problem.get_travel_time(i, j, 0.0);
            const double tji = problem.get_travel_time(j, i, 0.0);
            const double denom = si + tij + sj + tji;
            if (denom < 1e-12) continue;

            const double fij  = pi / denom;
            const double wfij = fij * window_i;

            best_fij   = std::max(best_fij,   fij);
            total_fij  += fij;
            best_wfij  = std::max(best_wfij,  wfij);
            total_wfij += wfij;
        }

        scores[i] = {best_fij, total_fij, best_wfij, total_wfij};
    }

    // Rank customers by each criterion (descending)
    int cap = std::min(m, static_cast<int>(customers.size()));

    auto top_m = [&](auto key_fn) -> std::vector<bool> {
        std::vector<NodeId> ranked = customers;
        std::sort(ranked.begin(), ranked.end(),
                  [&](NodeId a, NodeId b) { return key_fn(a) > key_fn(b); });
        std::vector<bool> in_top(nn, false);
        for (int k = 0; k < cap; ++k) in_top[ranked[k]] = true;
        return in_top;
    };

    auto hca  = top_m([&](NodeId i) { return scores[i].hca;  });
    auto tca  = top_m([&](NodeId i) { return scores[i].tca;  });
    auto whca = top_m([&](NodeId i) { return scores[i].whca; });
    auto wtca = top_m([&](NodeId i) { return scores[i].wtca; });

    // Intersection
    std::vector<bool> result(nn, false);
    bool any = false;
    for (NodeId i : customers) {
        if (hca[i] && tca[i] && whca[i] && wtca[i]) {
            result[i] = true;
            any = true;
        }
    }

    // If intersection is empty, return empty set (no harm to correctness)
    if (!any) return std::vector<bool>(nn, false);
    return result;
}

// ---------------------------------------------------------------------------

model::Solution DSSRSolver::solve(const model::Problem& problem,
                                   const SolverConfig&   config)
{
    DSSRSolverConfig c;
    c.seed        = config.seed;
    c.max_cpu_time = config.max_cpu_time;
    c.verbose     = config.verbose;
    return solve(problem, c);
}

model::Solution DSSRSolver::solve(const model::Problem&   problem,
                                   const DSSRSolverConfig& config)
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // Initialize critical set
    std::vector<bool> critical_set(nn, false);
    if (config.use_ms_init)
        critical_set = compute_ms_m_init(problem, config.ms_m);

    // Label pool and per-node lists reused across DSSR iterations
    std::vector<std::unique_ptr<Label>> label_pool;
    std::vector<std::vector<Label*>>    node_labels(nn);

    Label* best_sink = nullptr;
    auto t_start = std::chrono::steady_clock::now();

    for (int iter = 0; iter <= config.max_dssr_iters; ++iter) {
        if (config.max_cpu_time > 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > config.max_cpu_time) {
            if (config.verbose) std::cerr << "[DSSR] timeout at iter=" << iter << "\n";
            break;
        }
        bool hit_cap = false;
        best_sink = run_inner_dp(problem, config, critical_set, label_pool, node_labels, hit_cap);

        if (hit_cap) {
            // Relaxed inner DP didn't converge within the label cap — most
            // likely a cheap/zero-cost cycle among non-critical nodes is
            // re-collecting reward indefinitely. Fall back to a fully
            // elementary run (= ForwardDPSolver semantics) to guarantee a
            // correct result.
            if (config.verbose)
                std::cerr << "[DSSR] iter=" << iter
                          << " hit label cap — falling back to full elementarity\n";
            std::fill(critical_set.begin(), critical_set.end(), true);
            critical_set[src]  = false;
            critical_set[sink] = false;
            bool fallback_hit_cap = false;
            best_sink = run_inner_dp(problem, config, critical_set, label_pool, node_labels,
                                      fallback_hit_cap);
            break;
        }

        if (best_sink == nullptr) {
            // No feasible solution found
            if (config.verbose)
                std::cerr << "[DSSR] iter=" << iter << " no solution\n";
            break;
        }

        // Detect cycles via visit counts in parent chain
        std::vector<int> counts = count_visits(best_sink, nn);
        int max_count = *std::max_element(counts.begin(), counts.end());

        if (config.verbose)
            std::cerr << "[DSSR] iter=" << iter
                      << " profit=" << best_sink->profit_collected
                      << " max_visits=" << max_count
                      << " |Θ|=" << std::count(critical_set.begin(), critical_set.end(), true)
                      << '\n';

        if (max_count <= 1) break;  // elementary → optimal

        // Expand critical set according to strategy
        bool expanded = false;
        switch (config.strategy) {
            case DSSRExpansionStrategy::HMO: {
                // Add single node with highest visit count (skip depots)
                int best_count = 1;
                NodeId best_node = -1;
                for (NodeId k = 0; k < nn; ++k) {
                    if (k == src || k == sink) continue;
                    if (counts[k] > best_count) { best_count = counts[k]; best_node = k; }
                }
                if (best_node >= 0 && !critical_set[best_node]) {
                    critical_set[best_node] = true;
                    expanded = true;
                }
                break;
            }
            case DSSRExpansionStrategy::HMO_ALL: {
                // Add all nodes tied for the highest count
                int best_count = 1;
                for (NodeId k = 0; k < nn; ++k) {
                    if (k == src || k == sink) continue;
                    best_count = std::max(best_count, counts[k]);
                }
                for (NodeId k = 0; k < nn; ++k) {
                    if (k == src || k == sink) continue;
                    if (counts[k] == best_count && !critical_set[k]) {
                        critical_set[k] = true;
                        expanded = true;
                    }
                }
                break;
            }
            case DSSRExpansionStrategy::MO_ALL: {
                // Add all nodes visited more than once
                for (NodeId k = 0; k < nn; ++k) {
                    if (k == src || k == sink) continue;
                    if (counts[k] > 1 && !critical_set[k]) {
                        critical_set[k] = true;
                        expanded = true;
                    }
                }
                break;
            }
        }

        if (!expanded) {
            // Critical set couldn't grow further — path is as elementary as we can force
            break;
        }

        // Safety: if all nodes are critical we're running a full elementary DP
        // next iteration — just let it run (it is ForwardDPSolver semantics)
    }

    if (best_sink == nullptr) {
        model::Solution sol(problem.get_num_vehicles());
        sol.get_route(0) = {src, sink};
        return sol;
    }

    return ForwardDPSolver::reconstruct(best_sink, problem);
}

} // namespace oplib::solver::dp
