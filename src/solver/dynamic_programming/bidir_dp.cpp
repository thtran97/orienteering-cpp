#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>

#include "solver/dynamic_programming/dp_solvers.h"

namespace oplib::solver::dp {

namespace {

model::Solution best_sol_from_labels(const std::vector<Label*>& labels,
                                     NodeId src, NodeId sink,
                                     const model::Problem& problem) {
    const Label* best = nullptr;
    for (const Label* lbl : labels) {
        if (lbl->dominated) continue;
        if (best == nullptr || lbl->profit_collected > best->profit_collected)
            best = lbl;
    }
    if (best == nullptr) {
        model::Solution sol(problem.get_num_vehicles());
        sol.get_route(0) = {src, sink};
        return sol;
    }
    return ForwardDPSolver::reconstruct(best, problem);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// TrueBidirectionalDPSolver
// ---------------------------------------------------------------------------
//
// Algorithm (Righini & Salani 2006, §3):
//
//  1. Run a forward DP from the source but do NOT extend labels whose
//     time_consumed > T/2.  This builds the "forward frontier."
//
//  2. Run a backward DP from the sink (via TrueBackwardDPSolver) with the
//     same T/2 stop: backward labels with time_consumed > T/2 are not extended.
//
//  3. Match every forward label at node i with every backward label at
//     adjacent node j:
//       • visited sets are disjoint (no node in both paths)
//       • fw.time_consumed + s_i + t_ij + s_j + bw.time_consumed ≤ T
//       • total prize = fw.profit_collected + bw.profit_collected
//       (each label already includes the prize of its endpoint node)
//
//  4. Also allow labels that pass through the T/2 boundary to reach the
//     sink directly (as in plain ForwardDP), so solutions contained
//     entirely in one half are not missed.

// ---------------------------------------------------------------------------

void TrueBidirectionalDPSolver::try_match(
    const Label*                           fw,
    NodeId                                 j,
    const std::vector<std::vector<Label*>>& bw_labels,
    const model::Problem&                  problem,
    Reward&                                best_reward,
    model::Solution&                       best_sol) const
{
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const Time   T    = std::min(
        problem.get_time_window(sink).closing,
        problem.get_time_window(src).opening + problem.get_budget()
    );

    const NodeId i = fw->last_visit;
    const Time   sj = problem.get_service_time(j);
    const Time   tij = problem.get_travel_time(i, j, fw->time_consumed);

    for (Label* bw : bw_labels[j]) {
        if (bw->dominated) continue;

        // Time feasibility: total path time ≤ T
        // fw.time_consumed is departure time from i (includes s_i already).
        // bw.time_consumed is backward residual at j (accounts for s_j on the backward side).
        // The joining link i→j contributes s_i (already in fw.time_consumed) + t_ij + s_j.
        // But s_i is already included in fw.time_consumed; and the backward residual at j
        // already accounts for s_j from the sink side. The gap is just t_ij:
        //   total = fw.time_consumed + t_ij + bw.time_consumed + sj
        // We add sj because bw.time_consumed at j was computed with τ' = max{τ+s_j+t_kj, ...}
        // meaning s_j was subtracted from the forward timeline when building the backward label.
        // To reconstruct total forward time: fw_dep_i + t_ij + s_j + bw_residual_j = total?
        // Simpler: use the matching condition from the paper directly:
        //   τ_fw + s_i + t_ij + s_j + τ_bw ≤ T
        // where τ_fw is departure from i (= fw.time_consumed which already includes s_i),
        // so: fw.time_consumed + t_ij + s_j + bw.time_consumed ≤ T
        // (s_i is already baked into fw.time_consumed per forward DP convention)
        if (fw->time_consumed + tij + sj + bw->time_consumed > T + 1e-9) continue;

        // Disjoint visited sets
        bool overlap = false;
        for (int k = 0; k < nn && !overlap; ++k)
            if (fw->is_visited[k] && bw->is_visited[k]) overlap = true;
        // j must not already appear in the forward path (it belongs to the backward path)
        if (fw->is_visited[j]) overlap = true;
        if (overlap) continue;

        Reward combined = fw->profit_collected + bw->profit_collected;
        if (combined <= best_reward) continue;

        // Build the combined solution
        // Forward path: walk fw parent chain (gives src → ... → i)
        std::vector<NodeId> fwd_path;
        for (const Label* c = fw; c != nullptr; c = c->parent)
            fwd_path.push_back(c->last_visit);
        std::reverse(fwd_path.begin(), fwd_path.end());

        // Backward path: walk bw parent chain (gives j → ... → sink)
        std::vector<NodeId> bwd_path;
        for (const Label* c = bw; c != nullptr; c = c->parent)
            bwd_path.push_back(c->last_visit);

        // Combined: fwd_path + bwd_path (i is last of fwd, j is first of bwd)
        std::vector<NodeId> route = fwd_path;
        route.insert(route.end(), bwd_path.begin(), bwd_path.end());

        best_reward = combined;
        best_sol = model::Solution(problem.get_num_vehicles());
        best_sol.get_route(0) = route;
        best_sol.total_reward = combined;
        // Compute travel time
        best_sol.total_travel_time = 0.0;
        for (size_t k = 1; k < route.size(); ++k)
            best_sol.total_travel_time += problem.get_distance(route[k-1], route[k]);

        (void)src; (void)sink;
    }
}

// ---------------------------------------------------------------------------

model::Solution TrueBidirectionalDPSolver::solve(const model::Problem& problem,
                                                   const SolverConfig&   config)
{
    DPSolverConfig dp;
    dp.seed = config.seed;
    dp.max_cpu_time = config.max_cpu_time;
    dp.verbose = config.verbose;
    return solve(problem, dp);
}

model::Solution TrueBidirectionalDPSolver::solve(const model::Problem& problem,
                                                   const DPSolverConfig& config)
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();
    const Time   T    = std::min(
        problem.get_time_window(sink).closing,
        problem.get_time_window(src).opening + problem.get_budget()
    );
    const Time   half = T / 2.0;
    auto t_start = std::chrono::steady_clock::now();
    const double time_budget = config.max_cpu_time;

    // ------------------------------------------------------------------
    // Phase 1: Forward DP (half-resource stopping)
    // ------------------------------------------------------------------
    std::vector<std::unique_ptr<Label>> fw_pool;
    std::vector<std::vector<Label*>>    fw_labels(nn);

    {
        auto cmp = [](Label* a, Label* b) { return a->time_consumed > b->time_consumed; };
        std::priority_queue<Label*, std::vector<Label*>, decltype(cmp)> pq(cmp);

        {
            std::vector<bool> vis(nn, false);
            vis[src] = true;
            const auto& tw0 = problem.get_time_window(src);
            auto* lbl = new Label(src, tw0.opening + problem.get_service_time(src),
                                  0.0, nullptr, std::move(vis));
            fw_pool.emplace_back(lbl);
            fw_labels[src].push_back(lbl);
            pq.push(lbl);
        }

        int explored = 0;
        while (!pq.empty()) {
            if (time_budget > 0 &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > time_budget) {
                if (config.verbose) std::cerr << "[TrueBidir] forward timeout after " << explored << " labels\n";
                break;
            }
            Label* li = pq.top(); pq.pop();
            if (li->dominated || li->extended) continue;
            li->extended = true;
            ++explored;
            if (config.max_labels > 0 && explored > config.max_labels) break;

            // Past half: keep label in fw_labels for matching but don't extend
            if (li->time_consumed > half) continue;

            for (NodeId j = 1; j < nn; ++j) {
                if (li->is_visited[j]) continue;

                Time travel  = problem.get_travel_time(li->last_visit, j, li->time_consumed);
                Time arrival = li->time_consumed + travel;
                const auto& tw_j = problem.get_time_window(j);
                if (arrival > tw_j.closing) continue;

                Time dep_j = std::max(arrival, tw_j.opening) + problem.get_service_time(j);

                // Budget check (still required even in half-resource mode)
                Time arr_sink = dep_j + problem.get_travel_time(j, sink, dep_j);
                if (arr_sink > problem.get_time_window(sink).closing) continue;
                if (arr_sink - problem.get_time_window(src).opening > problem.get_budget()) continue;

                Reward new_profit = li->profit_collected
                                    + (j == sink ? 0.0 : problem.get_reward(j));

                std::vector<bool> new_vis = li->is_visited;
                new_vis[j] = true;

                auto* lj = new Label(j, dep_j, new_profit, li, std::move(new_vis));

                bool dominated = false;
                for (Label* ex : fw_labels[j]) {
                    if (ex->dominated) continue;
                    if (ex->dominates(*lj)) { dominated = true; break; }
                    if (lj->dominates(*ex)) ex->dominated = true;
                }
                if (dominated) { delete lj; continue; }

                fw_pool.emplace_back(lj);
                fw_labels[j].push_back(lj);
                pq.push(lj);
            }
        }

        if (config.verbose)
            std::cerr << "[TrueBidir] forward labels explored=" << explored << '\n';
    }

    // Check if forward phase timed out
    if (time_budget > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > time_budget) {
        if (config.verbose) std::cerr << "[TrueBidir] timeout before backward phase\n";
        return best_sol_from_labels(fw_labels[sink], src, sink, problem);
    }

    // ------------------------------------------------------------------
    // Phase 2: Backward DP (half-resource stopping)
    // ------------------------------------------------------------------
    // We run the full backward DP — the half-resource filter is applied
    // below during matching (bw labels with residual > half are still
    // stored but only matched with forward labels satisfying the total ≤ T check).
    std::vector<std::unique_ptr<Label>> bw_pool;
    TrueBackwardDPSolver bw_solver;
    std::vector<std::vector<Label*>> bw_labels =
        bw_solver.compute_backward_labels(problem, config, bw_pool);

    // ------------------------------------------------------------------
    // Phase 3: Collect any forward labels that already reached the sink
    // ------------------------------------------------------------------
    Reward       best_reward = 0.0;
    model::Solution best_sol(problem.get_num_vehicles());
    best_sol.get_route(0) = {src, sink};

    {
        auto candidate = best_sol_from_labels(fw_labels[sink], src, sink, problem);
        if (candidate.total_reward > best_reward) {
            best_reward = candidate.total_reward;
            best_sol = std::move(candidate);
        }
    }

    // ------------------------------------------------------------------
    // Phase 4: Match forward labels with backward labels
    // ------------------------------------------------------------------
    int match_count = 0;
    for (NodeId i = 0; i < nn; ++i) {
        if (i == sink) continue;
        for (Label* fw : fw_labels[i]) {
            if (fw->dominated) continue;

            if (++match_count % 1000 == 0 && time_budget > 0 &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > time_budget) {
                if (config.verbose) std::cerr << "[TrueBidir] match timeout\n";
                goto done;
            }

            // Try all adjacent j (backward label is at j; we join via arc i→j)
            for (NodeId j = 0; j < nn; ++j) {
                if (j == src || fw->is_visited[j]) continue;
                if (bw_labels[j].empty()) continue;

                try_match(fw, j, bw_labels, problem, best_reward, best_sol);
            }
        }
    }
done:

    if (config.verbose)
        std::cerr << "[TrueBidir] best=" << best_reward << '\n';

    return best_sol;
}

} // namespace oplib::solver::dp
