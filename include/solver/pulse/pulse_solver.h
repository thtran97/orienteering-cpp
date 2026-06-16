#pragma once

#include <chrono>
#include <vector>
#include <limits>

#include "solver/solver.h"

namespace oplib::solver::pulse {

/**
 * @brief Configuration for the Pulse solver.
 */
struct PulseSolverConfig : public SolverConfig {
    int    max_labels         = 1000000; ///< pulse expansion budget (0 = unlimited)
    int    bound_step         = 10;      ///< oracle time granularity (problem time units); finer = tighter bounds
    double threshold_rate     = 0.2;     ///< fraction of budget below which oracle is not used
    int    second_bound_step  = 0;       ///< if > 0, run a second finer bounding pass (Phase II)
};

/**
 * @brief Pulse algorithm for the Orienteering Problem with Time Windows.
 *
 * Branch-and-bound DFS pruned by time-window infeasibility, budget infeasibility,
 * time-indexed oracle upper bounds, dynamic reachability bound, 2-opt soft dominance,
 * and detour dominance. Arc lists are pre-built and pre-sorted at the start of solve()
 * to reduce per-pulse-call work from O(n) to O(out-degree).
 *
 * @warning Exact in theory; exponential worst-case.
 */
class PulseSolver : public Solver {
public:
    std::string get_name() const override { return "Pulse"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const PulseSolverConfig& config);

private:
    // Oracle bound table: oracle[node][idx] = best reward reachable from node departing at idx*step
    using OracleBound = std::vector<std::vector<Reward>>;

    // (k, latest_dep_from_i) — k is a valid detour for the arc this entry is attached to,
    // as long as the actual departure time from i is <= latest_dep_from_i.
    using DetourEntry = std::pair<NodeId, Time>;

    // Pre-built, pre-sorted adjacency list entry.
    // adj_list[i][k] describes the k-th arc leaving node i (sorted desc by sort_key).
    struct ArcInfo {
        NodeId j;          ///< destination node
        double sort_key;   ///< reward(j) / dist(i,j), matches toptwLib's PNode::sort() density heuristic
        Time   dist;       ///< travel time from i to j
        Reward reward;     ///< reward collected at j (0 for sink)
        std::vector<DetourEntry> detours; ///< runtime detour candidates, sorted desc by latest_dep
    };
    using AdjList = std::vector<std::vector<ArcInfo>>;

    // State shared across recursive calls
    struct SearchState {
        const model::Problem*   problem;
        const OracleBound*      oracle;         ///< time-indexed oracle bounds (may be nullptr)
        const AdjList*          adj_list     = nullptr; ///< pre-built adjacency lists
        int                     bound_step;
        Time                    threshold;
        NodeId                  sink         = -1;      ///< cached sink depot id
        Time                    src_departure_time = 0.0;
        std::vector<bool>       visited;
        std::vector<NodeId>     current_path;
        Reward                  current_reward  = 0.0;
        Time                    current_time    = 0.0;
        Reward                  best_reward     = 0.0;
        std::vector<NodeId>     best_path;
        int                     pulses_launched = 0;
        int                     max_labels;
        double                  max_cpu_time   = 0.0;
        std::chrono::high_resolution_clock::time_point start_time;
        bool                    timed_out      = false;

        // Cached node data — avoids virtual dispatch in hot loops
        std::vector<Time>              node_open;
        std::vector<Time>              node_close;
        std::vector<Time>              node_svc;
        std::vector<Time>              dist_to_sink;
        // Precomputed budget deadline at sink (= src.opening + budget).
        // For TOPTW this equals sink.closing; for OP it is the real budget limit.
        Time                           budget_limit = 0.0;
        // arc_exists[i][j] = true iff arc (i→j) is in adj_list (pre-filtered feasible arc)
        std::vector<std::vector<bool>> arc_exists;
        // dist_mat[i][j] = travel time if arc in adj_list; -1 otherwise
        std::vector<std::vector<Time>> dist_mat;
    };

    // Bounding phase state
    struct BoundState {
        const model::Problem*   problem;
        const OracleBound*      oracle;
        const AdjList*          adj_list     = nullptr; ///< pre-built adj list for O(degree) bounding
        int                     bound_step;
        Time                    threshold;
        Time                    budget_limit = 0.0;     ///< src_open + budget
        NodeId                  sink         = -1;      ///< cached sink depot id
        int                     root;
        Reward                  best_root    = 0.0;
        std::vector<bool>       visited;
        double                  max_cpu_time = 0.0;
        std::chrono::high_resolution_clock::time_point start_time;
        bool                    timed_out    = false;

        // Cached node data for pulse_bound hot loop (always set, no nullptrs)
        const std::vector<Time>* node_open   = nullptr;
        const std::vector<Time>* node_close  = nullptr;
        const std::vector<Time>* node_svc    = nullptr;
        const std::vector<Time>* dist_to_sink= nullptr;
        // arc_exists[i][j] / dist_mat[i][j] — shared with SearchState, used by the
        // restricted soft-dominance check during bounding.
        const std::vector<std::vector<bool>>* arc_exists = nullptr;
        const std::vector<std::vector<Time>>* dist_mat   = nullptr;
        std::vector<NodeId>     current_path;
    };

    void pulse(NodeId node, SearchState& state) const;
    void pulse_bound(NodeId node, Time dep_time, Reward score, BoundState& bs) const;

    OracleBound  compute_oracle_bounds(const model::Problem& problem,
                                       const PulseSolverConfig& config,
                                       int bound_step,
                                       const AdjList& adj_list,
                                       const std::vector<Time>& node_open,
                                       const std::vector<Time>& node_close,
                                       const std::vector<Time>& node_svc,
                                       const std::vector<Time>& dist_to_sink,
                                       const std::vector<std::vector<bool>>& arc_exists,
                                       const std::vector<std::vector<Time>>& dist_mat) const;
    AdjList      compute_adj_list(const model::Problem& problem,
                                  Time effective_tmax) const;
    Time         compute_effective_tmax(const model::Problem& problem) const;
    // Raw (unrefined) horizon — matches toptwLib's problem->get_latest_arrival(0),
    // used to seed the oracle bounding loop's threshold range.
    Time         compute_raw_tmax(const model::Problem& problem) const;

    bool check_two_opt_improvement(SearchState& state) const;
    bool check_feasibility(std::vector<NodeId>& path, int i, int j,
                           Time old_time, const SearchState& state) const;
    // Restricted variant used during bounding: only tries swapping each earlier
    // node with the second-to-last path position (matches toptwLib's pulse_bound
    // soft-dominance check).
    bool check_two_opt_improvement_bound(Time starting_time, BoundState& bs) const;
    bool check_feasibility_bound(std::vector<NodeId>& path, int i, int j,
                                 Time starting_time, const BoundState& bs) const;
};

} // namespace oplib::solver::pulse
