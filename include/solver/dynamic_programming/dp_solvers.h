#pragma once

#include <vector>
#include <queue>

#include "solver/solver.h"
#include "solver/dynamic_programming/label.h"

namespace oplib::solver::dp {

/**
 * @brief Configuration for DP-based exact solvers.
 *
 * @note max_nodes limits the number of labels explored, preventing memory blow-up
 *       on large instances.  Set to 0 for no limit (only feasible for n ≤ ~20).
 */
struct DPSolverConfig : public SolverConfig {
    int max_labels = 500000; ///< label budget (0 = unlimited)
};

/**
 * @brief Forward label-setting DP solver for the Orienteering Problem (single vehicle).
 *
 * Performs a Dijkstra-style forward pass: labels are ordered by time_consumed.
 * At each step the earliest-time label is extended to all unvisited feasible customers.
 * Dominance pruning removes labels that are strictly inferior on (time, reward, visited).
 *
 * Adapted from toptwLib/lib/include/solver/dynamic_programming/forward_DP.h.
 *
 * @warning Exact complexity is O(n × 2^n).  Practical only for n ≤ ~20.
 *          For larger instances use one of the heuristic solvers.
 */
class ForwardDPSolver : public Solver {
public:
    std::string get_name() const override { return "ForwardDP"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const DPSolverConfig& config);

    /// Build a Solution from the label chain ending at `best`.
    static model::Solution reconstruct(const Label* best,
                                       const model::Problem& problem);
};

// ---------------------------------------------------------------------------

/**
 * @brief Backward label-correcting DP for computing upper-bound rewards.
 *
 * Runs a backward pass from the sink depot, computing the maximum collectable
 * reward from each node to the sink.  Useful as a bounding oracle for the
 * Pulse algorithm (Phase 10).
 *
 * Adapted from toptwLib/lib/include/solver/dynamic_programming/backward_DP.h.
 */
class BackwardDPSolver : public Solver {
public:
    std::string get_name() const override { return "BackwardDP"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const DPSolverConfig& config);

    /**
     * @brief Compute the upper-bound reward reachable from each node to sink.
     *
     * The returned vector has one entry per node: ub[i] = max reward reachable
     * from node i (at its earliest departure time) to the sink.
     */
    std::vector<Reward> compute_bounds(const model::Problem& problem) const;
};

// ---------------------------------------------------------------------------

/**
 * @brief Bidirectional DP combining forward + backward passes.
 *
 * The forward pass explores from the source; the backward pass provides
 * tighter reward bounds used to prune the forward search.
 * Solutions are found when a forward label reaches the sink.
 *
 * Adapted from toptwLib/lib/include/solver/dynamic_programming/bidirectional_DP.h.
 */
class BidirectionalDPSolver : public Solver {
public:
    std::string get_name() const override { return "BidirectionalDP"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const DPSolverConfig& config);
};

} // namespace oplib::solver::dp
