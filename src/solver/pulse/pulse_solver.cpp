#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>

#include "solver/pulse/pulse_solver.h"

namespace oplib::solver::pulse {

// ---------------------------------------------------------------------------
// TMAX reduction
// ---------------------------------------------------------------------------

Time PulseSolver::compute_raw_tmax(const model::Problem& problem) const
{
    return std::min(problem.get_time_window(problem.get_sink_depot()).closing,
                     problem.get_time_window(problem.get_source_depot()).opening
                         + problem.get_budget());
}

Time PulseSolver::compute_effective_tmax(const model::Problem& problem) const
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    const Time raw = compute_raw_tmax(problem);

    Time latest_ret = 0.0;
    for (NodeId k = 1; k < nn; ++k) {
        if (k == src || k == sink) continue;
        Time t = problem.get_time_window(k).closing
                 + problem.get_service_time(k)
                 + problem.get_distance(k, sink);
        if (t > latest_ret) latest_ret = t;
    }
    return (latest_ret > 0.0 && latest_ret < raw) ? latest_ret : raw;
}

// ---------------------------------------------------------------------------
// Pre-built adjacency lists
// ---------------------------------------------------------------------------

PulseSolver::AdjList PulseSolver::compute_adj_list(
    const model::Problem& problem,
    Time effective_tmax) const
{
    const int    nn      = static_cast<int>(problem.get_num_nodes());
    const NodeId src     = problem.get_source_depot();
    const NodeId sink    = problem.get_sink_depot();
    const auto&  tw_sink = problem.get_time_window(sink);

    // Cache per-node time-window data once (k-loop below is O(n) per arc).
    std::vector<Time> node_open(nn), node_close(nn), node_svc(nn);
    for (NodeId n = 0; n < nn; ++n) {
        const auto& tw  = problem.get_time_window(n);
        node_open[n]    = tw.opening;
        node_close[n]   = tw.closing;
        node_svc[n]     = problem.get_service_time(n);
    }

    AdjList adj(nn);
    for (NodeId i = 0; i < nn; ++i) {
        if (i == sink) continue;
        const auto& tw_i       = problem.get_time_window(i);
        Time earliest_dep_i    = tw_i.opening + problem.get_service_time(i);

        for (NodeId j = 1; j < nn; ++j) {
            if (j == i || j == src) continue;

            Time dist           = problem.get_distance(i, j);
            Time earliest_arr_j = earliest_dep_i + dist;
            const auto& tw_j    = problem.get_time_window(j);

            if (earliest_arr_j > tw_j.closing) continue;

            if (j != sink) {
                Time earliest_dep_j  = std::max(earliest_arr_j, tw_j.opening)
                                       + problem.get_service_time(j);
                Time earliest_return = earliest_dep_j + problem.get_distance(j, sink);
                if (earliest_return > effective_tmax) continue;
            } else {
                if (earliest_arr_j > tw_sink.closing) continue;
            }

            // Permanent arc removal: if departing i at its LATEST possible time still
            // allows detouring through some k and reaching j before j opens, then k is
            // *always* a valid detour — the direct arc i->j is never needed.
            bool is_pruned = false;
            for (NodeId k = 1; k < nn && !is_pruned; ++k) {
                if (k == i || k == j || k == src || k == sink) continue;
                Time arr_k = node_close[i] + node_svc[i] + problem.get_distance(i, k);
                if (arr_k > node_close[k]) continue;
                Time dep_k = std::max(arr_k, node_open[k]) + node_svc[k];
                if (dep_k + problem.get_distance(k, j) <= node_open[j])
                    is_pruned = true;
            }
            if (is_pruned) continue;

            // Runtime detour list: k is a valid detour for this arc as long as the actual
            // departure time from i is <= latest_dep (sorted desc so callers can early-exit).
            std::vector<DetourEntry> detours;
            for (NodeId k = 1; k < nn; ++k) {
                if (k == i || k == j || k == src || k == sink) continue;
                Time latest_arr_k = std::min(node_open[j] - node_svc[k]
                                                            - problem.get_distance(k, j),
                                             node_close[k]);
                if (latest_arr_k < node_open[k]) continue;
                Time ld = std::min(latest_arr_k - problem.get_distance(i, k), node_close[i] + node_svc[i]);
                if (ld < node_open[i] + node_svc[i]) continue;
                detours.push_back({k, ld});
            }
            std::sort(detours.begin(), detours.end(),
                      [](const auto& a, const auto& b) { return a.second > b.second; });

            Reward rwd = (j == sink) ? 0.0 : problem.get_reward(j);
            // Greedy density ordering: reward(j) / dist(i,j), matches toptwLib's
            // PNode::sort() (default alpha=1): score / (dist + epsilon).
            double sort_key = rwd / (dist + 1e-5);
            adj[i].push_back({j, sort_key, dist, rwd, std::move(detours)});
        }

        std::sort(adj[i].begin(), adj[i].end(),
                  [](const ArcInfo& a, const ArcInfo& b) { return a.sort_key > b.sort_key; });
    }
    return adj;
}

// ---------------------------------------------------------------------------
// Oracle bound computation (bounding phase)
// Uses pre-cached node data to avoid virtual dispatch in the inner loop.
// ---------------------------------------------------------------------------

void PulseSolver::pulse_bound(NodeId node, Time dep_time, Reward score,
                               BoundState& bs) const
{
    const NodeId sink = bs.sink;

    if (bs.max_cpu_time > 0.0) {
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - bs.start_time).count() > bs.max_cpu_time) {
            bs.timed_out = true;
            return;
        }
    }
    if (bs.timed_out) return;

    const auto& nc  = *bs.node_close;
    const auto& no  = *bs.node_open;
    const auto& nsvc= *bs.node_svc;
    const auto& ds  = *bs.dist_to_sink;

    if (node == sink) {
        if (dep_time <= nc[sink])
            if (score > bs.best_root) bs.best_root = score;
        return;
    }

    if (bs.visited[node]) return;

    const Time min_dep = no[node] + nsvc[node];
    if (dep_time < min_dep) dep_time = min_dep;

    // Oracle pruning: only use entries from strictly-prior (higher-threshold) iterations,
    // which are guaranteed to be fully computed.
    if (bs.oracle && dep_time >= bs.threshold + bs.bound_step) {
        int idx = static_cast<int>(dep_time / bs.bound_step);
        const auto& ob = (*bs.oracle);
        if (idx < (int)ob[node].size() && ob[node][idx] >= 0.0)
            if (score + ob[node][idx] < bs.best_root) return;
    }

    bs.visited[node] = true;
    bs.current_path.push_back(node);

    if (!check_two_opt_improvement_bound(dep_time, bs)) {
        const Time sink_close = nc[sink];

        // Iterate adj_list for O(degree) expansion with no virtual dispatch.
        // arc.dist and arc.reward are pre-cached; node data from BoundState caches.
        for (const auto& arc : (*bs.adj_list)[node]) {
            NodeId j = arc.j;
            if (bs.visited[j]) continue;

            // ---- Detour pruning (matches toptwLib's pulse_bound) ----
            bool has_detour = false;
            for (const auto& [k, latest_dep] : arc.detours) {
                if (dep_time > latest_dep) break;   // sorted desc -> early exit
                if (!bs.visited[k]) { has_detour = true; break; }
            }
            if (has_detour) continue;

            Time arr_j = dep_time + arc.dist;
            if (arr_j > nc[j]) continue;

            Time dep_j    = std::max(arr_j, no[j]) + nsvc[j];
            Time arr_sink = dep_j + ds[j];
            if (arr_sink > sink_close || arr_sink > bs.budget_limit) continue;

            pulse_bound(j, dep_j, score + arc.reward, bs);
            if (bs.timed_out) break;
        }
    }

    bs.current_path.pop_back();
    bs.visited[node] = false;
}

PulseSolver::OracleBound PulseSolver::compute_oracle_bounds(
    const model::Problem& problem,
    const PulseSolverConfig& config,
    int bound_step,
    const AdjList& adj_list,
    const std::vector<Time>& node_open,
    const std::vector<Time>& node_close,
    const std::vector<Time>& node_svc,
    const std::vector<Time>& dist_to_sink,
    const std::vector<std::vector<bool>>& arc_exists,
    const std::vector<std::vector<Time>>& dist_mat) const
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // Use the RAW (unrefined) TMAX here, matching toptwLib's run_pulse_bounding,
    // which seeds its threshold loop from problem->get_latest_arrival(0) (raw) —
    // NOT the tightened compute_effective_tmax() used only for arc generation.
    // Skipping the raw->refined gap would leave oracle[node][idx] uncomputed
    // (sentinel -1) for departure times in that gap, silently disabling the
    // oracle-pruning check deep inside every subtree whose path reaches that
    // time range — causing a large blow-up in explored nodes.
    const Time TMAX = compute_raw_tmax(problem);

    if (TMAX > 1e12) {
        if (config.verbose)
            std::cout << "[Pulse] TMAX=" << TMAX << " too large for oracle, skipping.\n";
        return OracleBound{};
    }
    if (TMAX < bound_step * 5) {
        if (config.verbose)
            std::cout << "[Pulse] TMAX=" << TMAX << " < 5*step=" << bound_step * 5
                      << "; oracle skipped (too few bins).\n";
        return OracleBound{};
    }

    int max_idx = static_cast<int>(std::ceil(double(TMAX) / bound_step)) + 2;
    OracleBound oracle(nn, std::vector<Reward>(max_idx + 1, -1.0));

    Time threshold_start = TMAX;
    Time threshold_end   = static_cast<Time>(config.threshold_rate * problem.get_budget());

    BoundState bs;
    bs.problem      = &problem;
    bs.oracle       = &oracle;
    bs.adj_list     = &adj_list;
    bs.bound_step   = bound_step;
    bs.budget_limit = node_open[problem.get_source_depot()] + problem.get_budget();
    bs.sink         = sink;
    bs.max_cpu_time = (config.max_cpu_time > 0.0) ? config.max_cpu_time * 0.5 : 60.0;
    bs.start_time   = std::chrono::high_resolution_clock::now();
    bs.visited.assign(nn, false);
    bs.node_open    = &node_open;
    bs.node_close   = &node_close;
    bs.node_svc     = &node_svc;
    bs.dist_to_sink = &dist_to_sink;
    bs.arc_exists   = &arc_exists;
    bs.dist_mat     = &dist_mat;
    bs.current_path.reserve(nn);

    if (config.verbose)
        std::cout << "[Pulse] Oracle bounds (step=" << bound_step
                  << " TMAX=" << TMAX << " threshold_end=" << threshold_end << ")...\n";

    std::vector<bool> need_bounding(nn, true);

    for (Time thresh = threshold_start; thresh >= threshold_end; thresh -= bound_step) {
        int idx = static_cast<int>(std::ceil(thresh / bound_step));
        bs.threshold = thresh;

        for (NodeId i = 1; i < sink; ++i) {
            if (!need_bounding[i]) continue;

            const Time min_dep_i = node_open[i] + node_svc[i];

            if (thresh < min_dep_i - bound_step) {
                Reward fill = (idx + 1 <= max_idx && oracle[i][idx + 1] >= 0.0)
                              ? oracle[i][idx + 1] : 0.0;
                for (int k = 0; k <= idx; ++k)
                    oracle[i][k] = fill;
                need_bounding[i] = false;
                continue;
            }

            // Warm-start with previous threshold's result: oracle[i][idx] can only be
            // >= oracle[i][idx+1] (earlier departure → more options). Starting the DFS
            // with this lower bound lets it prune branches that can't improve on it.
            bs.best_root = (idx + 1 <= max_idx && oracle[i][idx + 1] >= 0.0)
                           ? oracle[i][idx + 1] : 0.0;
            std::fill(bs.visited.begin(), bs.visited.end(), false);
            bs.visited[src] = true;
            pulse_bound(i, thresh, 0.0, bs);
            oracle[i][idx] = bs.best_root;
            if (bs.timed_out) break;
        }
        if (bs.timed_out) break;
    }

    if (config.verbose) {
        auto now = std::chrono::high_resolution_clock::now();
        double el = std::chrono::duration<double>(now - bs.start_time).count();
        std::cout << "[Pulse] Oracle done in " << el * 1000.0 << " ms"
                  << (bs.timed_out ? " (partial)" : "") << '\n';
    }
    return oracle;
}

// ---------------------------------------------------------------------------
// Soft dominance helpers
//
// check_feasibility uses:
//   1. Fast pre-check of the 4 arcs that change after swap(i,j) via arc_exists[].
//      Most invalid swaps are rejected in O(1) without any path traversal.
//   2. In-place swap + restore — no heap allocation.
//   3. dist_mat[] for O(1) distance lookup (no virtual dispatch).
//
// check_two_opt_improvement does ALL-PAIRS (same as toptwLib Java reference).
// The arc_exists pre-check makes invalid pairs O(1), so O(n²) pairs per pulse
// is far cheaper than it appears: only ~(d/n)^4 of pairs need a full traversal.
// ---------------------------------------------------------------------------

bool PulseSolver::check_feasibility(std::vector<NodeId>& path, int i, int j,
                                     Time old_time, const SearchState& state) const
{
    const int pLen = (int)path.size();
    const auto& ae = state.arc_exists;
    const auto& dm = state.dist_mat;

    const NodeId pi = path[i], pj = path[j];

    // Pre-check the arcs that change after swapping positions i and j.
    // After swap: path[i-1]→pj and pi→path[j+1] are new in all cases;
    // additionally path[j-1]→pi and pj→path[i+1] for non-adjacent pairs.
    if (!ae[path[i-1]][pj]) return false;
    if (j + 1 < pLen && !ae[pi][path[j+1]]) return false;

    if (j == i + 1) {
        // Adjacent swap: the middle arc becomes pj → pi
        if (!ae[pj][pi]) return false;
    } else {
        if (!ae[pj][path[i+1]]) return false;
        if (!ae[path[j-1]][pi]) return false;
    }

    // Full in-place traversal (from the source departure time)
    std::swap(path[i], path[j]);

    Time cur = state.src_departure_time;
    bool ok  = true;
    for (int k = 1; k < pLen && ok; ++k) {
        const Time d = dm[path[k-1]][path[k]];
        if (d < 0.0) { ok = false; break; }          // arc not in adj_list
        const Time arr = cur + d;
        if (arr > state.node_close[path[k]]) { ok = false; break; }
        cur = std::max(arr, state.node_open[path[k]]) + state.node_svc[path[k]];
    }

    std::swap(path[i], path[j]);   // restore original order
    return ok && cur < old_time;
}

bool PulseSolver::check_two_opt_improvement(SearchState& state) const
{
    auto& path = state.current_path;
    const int pLen = (int)path.size();
    if (pLen < 4) return false;

    // Check all pairs involving at least one node in the last K positions.
    // Wider window = more pruning power; narrower = cheaper per pulse.
    // K=pLen-1 is full all-pairs; K=2 is the original window-3.
    // We use full all-pairs — arc_exists pre-check makes most pairs O(1).
    for (int i = 1; i < pLen - 2; ++i)
        for (int j = i + 1; j < pLen - 1; ++j)
            if (check_feasibility(path, i, j, state.current_time, state))
                return true;

    return false;
}

// Restricted soft-dominance check used during bounding: only swaps each
// earlier node with the second-to-last path position (toptwLib's pulse_bound
// variant), rather than all O(n²) pairs — much cheaper, applied at every node.
bool PulseSolver::check_feasibility_bound(std::vector<NodeId>& path, int i, int j,
                                          Time starting_time, const BoundState& bs) const
{
    const int pLen = (int)path.size();
    const auto& ae = *bs.arc_exists;
    const auto& dm = *bs.dist_mat;

    const NodeId pi = path[i], pj = path[j];

    if (!ae[path[i-1]][pj]) return false;
    if (j + 1 < pLen && !ae[pi][path[j+1]]) return false;

    if (j == i + 1) {
        if (!ae[pj][pi]) return false;
    } else {
        if (!ae[pj][path[i+1]]) return false;
        if (!ae[path[j-1]][pi]) return false;
    }

    std::swap(path[i], path[j]);

    Time cur = std::max(bs.threshold, (*bs.node_open)[path[0]]);
    bool ok  = true;
    for (int k = 1; k < pLen && ok; ++k) {
        const Time d = dm[path[k-1]][path[k]];
        if (d < 0.0) { ok = false; break; }
        const Time arr = cur + d;
        if (arr > (*bs.node_close)[path[k]]) { ok = false; break; }
        cur = std::max(arr, (*bs.node_open)[path[k]]) + (*bs.node_svc)[path[k]];
    }

    std::swap(path[i], path[j]);
    return ok && cur < starting_time;
}

bool PulseSolver::check_two_opt_improvement_bound(Time starting_time, BoundState& bs) const
{
    auto& path = bs.current_path;
    const int pLen = (int)path.size();
    if (pLen < 4) return false;

    const int j = pLen - 2;
    for (int i = 1; i < pLen - 2; ++i)
        if (check_feasibility_bound(path, i, j, starting_time, bs))
            return true;

    return false;
}

// ---------------------------------------------------------------------------
// Recursive pulse (main phase)
// ---------------------------------------------------------------------------

void PulseSolver::pulse(NodeId node, SearchState& state) const
{
    const NodeId sink = state.sink;

    // ---- CPU time limit ----
    if (state.max_cpu_time > 0.0) {
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - state.start_time).count() > state.max_cpu_time) {
            state.timed_out = true;
            return;
        }
    }
    if (state.timed_out) return;

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

    // ---- Soft dominance (all-pairs 2-opt) ----
    if (check_two_opt_improvement(state)) return;

    // Use cached node data (avoids virtual dispatch in hot loop)
    const Time  sink_close = state.node_close[sink];

    // ---- Oracle bound ----
    if (state.oracle && state.current_time >= state.threshold) {
        int idx = static_cast<int>(state.current_time / state.bound_step);
        const auto& ob = (*state.oracle);
        if (idx < (int)ob[node].size() && ob[node][idx] >= 0.0)
            if (state.current_reward + ob[node][idx] <= state.best_reward) return;
    }

    // ---- Enumerate feasible successors via pre-built adj_list (sorted desc by sort_key) ----
    const auto& adj = (*state.adj_list)[node];

    for (const auto& arc : adj) {
        if (state.timed_out) break;
        if (state.max_labels > 0 && state.pulses_launched > state.max_labels) break;

        NodeId j = arc.j;
        if (state.visited[j]) continue;

        Time arr = state.current_time + arc.dist;
        if (arr > state.node_close[j]) continue;

        Time dep_j    = std::max(arr, state.node_open[j]) + state.node_svc[j];
        Time arr_sink = dep_j + state.dist_to_sink[j];
        if (arr_sink > sink_close || arr_sink > state.budget_limit) continue;

        // ---- Detour pruning ----
        bool has_detour = false;
        for (const auto& [k, latest_dep] : arc.detours) {
            if (state.current_time > latest_dep) break;   // sorted desc -> early exit
            if (!state.visited[k]) { has_detour = true; break; }
        }
        if (has_detour) continue;

        // Push state
        state.visited[j]      = true;
        state.current_path.push_back(j);
        Time   saved_time     = state.current_time;
        Reward saved_reward   = state.current_reward;

        state.current_time    = dep_j;
        state.current_reward += arc.reward;

        pulse(j, state);

        // Pop state
        state.visited[j]      = false;
        state.current_path.pop_back();
        state.current_time    = saved_time;
        state.current_reward  = saved_reward;
    }
}

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

model::Solution PulseSolver::solve(const model::Problem& problem,
                                    const SolverConfig&   config)
{
    PulseSolverConfig ps_cfg;
    ps_cfg.seed         = config.seed;
    ps_cfg.max_cpu_time = config.max_cpu_time;
    ps_cfg.verbose      = config.verbose;
    return solve(problem, ps_cfg);
}

model::Solution PulseSolver::solve(const model::Problem&    problem,
                                    const PulseSolverConfig& config)
{
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    // Build node data cache — eliminates virtual dispatch in hot loops
    std::vector<Time> node_open(nn), node_close(nn), node_svc(nn), dist_to_sink(nn);
    for (int i = 0; i < nn; ++i) {
        const auto& tw = problem.get_time_window(i);
        node_open[i]     = tw.opening;
        node_close[i]    = tw.closing;
        node_svc[i]      = problem.get_service_time(i);
        dist_to_sink[i]  = problem.get_distance(i, sink);
    }

    // Effective TMAX
    const Time effective_tmax = compute_effective_tmax(problem);

    // Pre-build sparse, sorted adjacency lists BEFORE oracle — so bounding phase
    // can iterate O(degree) arcs instead of O(n), matching toptwLib's performance.
    AdjList adj_list = compute_adj_list(problem, effective_tmax);

    // Build arc_exists matrix and dist_mat from adj_list (O(1) lookup in hot paths).
    // Built BEFORE the oracle so the bounding phase can also use the restricted
    // soft-dominance check (matches toptwLib's pulse_bound, a major bounding speedup).
    std::vector<std::vector<bool>> arc_exists(nn, std::vector<bool>(nn, false));
    std::vector<std::vector<Time>> dist_mat(nn, std::vector<Time>(nn, -1.0));
    for (int i = 0; i < nn; ++i) {
        for (const auto& arc : adj_list[i]) {
            arc_exists[i][arc.j] = true;
            dist_mat[i][arc.j]   = arc.dist;
        }
    }

    // Bounding phase: time-indexed oracle bounds
    OracleBound oracle;
    bool use_oracle = false;
    if (config.bound_step > 0) {
        oracle     = compute_oracle_bounds(problem, config, config.bound_step,
                                           adj_list,
                                           node_open, node_close, node_svc, dist_to_sink,
                                           arc_exists, dist_mat);
        use_oracle = !oracle.empty();
    }

    // Phase II: optional finer bounding pass
    int active_bound_step = config.bound_step;
    if (config.second_bound_step > 0 && use_oracle) {
        OracleBound oracle2 = compute_oracle_bounds(problem, config, config.second_bound_step,
                                                     adj_list,
                                                     node_open, node_close, node_svc, dist_to_sink,
                                                     arc_exists, dist_mat);
        if (!oracle2.empty()) {
            oracle            = std::move(oracle2);
            active_bound_step = config.second_bound_step;
        }
    }

    // Set up search state
    SearchState state;
    state.problem            = &problem;
    state.oracle             = use_oracle ? &oracle : nullptr;
    state.adj_list           = &adj_list;
    state.bound_step         = active_bound_step;
    state.threshold          = static_cast<Time>(config.threshold_rate * problem.get_budget());
    state.sink               = sink;
    state.visited.assign(nn, false);
    state.visited[src]       = true;
    state.max_labels         = config.max_labels;
    state.max_cpu_time       = config.max_cpu_time;
    // toptwLib's run_pulse_bounding and run_pulse each reset their own clock,
    // giving bounding up to 0.5*max_cpu_time and search a SEPARATE, full
    // max_cpu_time budget (total wall time up to 1.5x max_cpu_time). Match
    // that exactly rather than sharing a single combined clock.
    state.start_time         = std::chrono::high_resolution_clock::now();

    const auto& tw_src        = problem.get_time_window(src);
    state.current_time        = tw_src.opening + problem.get_service_time(src);
    state.src_departure_time  = state.current_time;
    state.budget_limit        = tw_src.opening + problem.get_budget();
    state.current_reward      = 0.0;
    state.current_path        = {src};

    // Move caches into state
    state.node_open    = std::move(node_open);
    state.node_close   = std::move(node_close);
    state.node_svc     = std::move(node_svc);
    state.dist_to_sink = std::move(dist_to_sink);
    state.arc_exists   = std::move(arc_exists);
    state.dist_mat     = std::move(dist_mat);

    pulse(src, state);

    if (config.verbose) {
        std::cout << "[Pulse] best=" << state.best_reward
                  << " pulses=" << state.pulses_launched;
        if (state.timed_out) std::cout << " (timed out)";
        std::cout << '\n';
    }

    // Build solution
    model::Solution sol(problem.get_num_vehicles());
    if (state.best_path.empty()) {
        sol.get_route(0) = {src, sink};
        return sol;
    }
    if (state.best_path.back() != sink)
        state.best_path.push_back(sink);

    sol.get_route(0)     = state.best_path;
    sol.total_reward     = state.best_reward;
    sol.total_travel_time = 0.0;
    const auto& route = sol.get_route(0);
    for (size_t i = 1; i < route.size(); ++i)
        sol.total_travel_time += problem.get_distance(route[i-1], route[i]);

    return sol;
}

} // namespace oplib::solver::pulse
