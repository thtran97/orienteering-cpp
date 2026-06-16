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

// ---------------------------------------------------------------------------

/**
 * @brief True backward label-setting DP (paper §3, Righini & Salani 2006).
 *
 * Mirrors ForwardDPSolver but extends labels from the sink backward toward
 * the source using backward time windows:
 *   [a_i + s_i, b_i + s_i]  for intermediate customers.
 * Backward transition: τ' = max{ τ + s_j + t_ij, T − (b_i + s_i) }
 * where T is the time horizon.
 *
 * BackwardDPSolver::compute_bounds() (fast relaxed oracle) remains separate
 * and is still used by PulseSolver — this class is the exact complement.
 */
class TrueBackwardDPSolver : public Solver {
public:
    std::string get_name() const override { return "TrueBackwardDP"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const DPSolverConfig& config);

    /**
     * @brief Run the backward DP and return all non-dominated labels per node.
     *
     * The returned pool owns the label memory; node_labels[i] holds raw
     * pointers into pool for node i.  Used by TrueBidirectionalDPSolver.
     */
    std::vector<std::vector<Label*>> compute_backward_labels(
        const model::Problem&                  problem,
        const DPSolverConfig&                  config,
        std::vector<std::unique_ptr<Label>>&   pool) const;

    /// Reconstruct a Solution from a backward label chain ending at the source.
    static model::Solution reconstruct(const Label* best,
                                       const model::Problem& problem);
};

// ---------------------------------------------------------------------------

/**
 * @brief True bidirectional DP with half-resource stopping and label matching.
 *
 * Runs a forward pass (time_consumed ≤ T/2) and a backward pass
 * (backward residual ≤ T/2), then matches all compatible (fw, bw) label
 * pairs at adjacent nodes.  Supersedes BidirectionalDPSolver for exact work.
 *
 * Matching condition for forward label at i and backward label at j:
 *   • visited sets disjoint
 *   • fw.time_consumed + s_i + t_ij + s_j + bw.time_consumed ≤ T
 */
class TrueBidirectionalDPSolver : public Solver {
public:
    std::string get_name() const override { return "TrueBidirectionalDP"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const DPSolverConfig& config);

private:
    /// Try to combine forward label fw (at node i) with all backward labels
    /// at node j.  Updates best_reward / best_sol when a better match is found.
    void try_match(const Label*                           fw,
                   NodeId                                 j,
                   const std::vector<std::vector<Label*>>& bw_labels,
                   const model::Problem&                  problem,
                   Reward&                                best_reward,
                   model::Solution&                       best_sol) const;
};

// ---------------------------------------------------------------------------

/** Critical-vertex expansion strategy for DSSR. */
enum class DSSRExpansionStrategy {
    HMO,     ///< Add the single node with the highest visit count
    HMO_ALL, ///< Add all nodes tied for the highest visit count
    MO_ALL   ///< Add every node visited more than once
};

/**
 * @brief Configuration for the DSSR solver.
 *
 * max_labels (inherited) is applied per inner-DP iteration.
 */
struct DSSRSolverConfig : public DPSolverConfig {
    DSSRExpansionStrategy strategy      = DSSRExpansionStrategy::MO_ALL;
    int                   max_dssr_iters = 50;  ///< safety cap on outer loop
    bool                  use_ms_init   = true; ///< seed Θ via cycling attractiveness
    int                   ms_m          = 3;    ///< top-m intersection size for MS_m
};

/**
 * @brief DSSR exact solver (Righini & Salani 2006, Section 4).
 *
 * Outer loop: maintain a critical set Θ of vertices for which elementarity
 * is enforced.  Inner DP: forward label-setting where only Θ-nodes are
 * tracked in is_visited; non-critical nodes may be revisited.  After each
 * inner DP: detect cycles via parent-chain walk; add cycled nodes to Θ;
 * repeat until the optimal path is elementary.
 *
 * Optional MS_m initialization (Section 5): pre-seeds Θ with vertices that
 * have the highest "cycling attractiveness" across four orderings, reducing
 * the number of DSSR outer iterations.
 */
class DSSRSolver : public Solver {
public:
    std::string get_name() const override { return "DSSR"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const DSSRSolverConfig& config);

private:
    /// Forward DP inner loop parameterised by critical_set.
    /// Only nodes in critical_set are checked/set in is_visited.
    ///
    /// Non-critical nodes may be revisited and re-collect their reward each
    /// time (the literal DSSR relaxation) — for non-negative-reward problems
    /// with cheap/zero-cost cycles this can generate an enormous number of
    /// mutually non-dominated labels (more time AND more profit each lap).
    /// To guard against this, the search is capped at config.max_labels
    /// (or an internal default if unset); `hit_cap` is set to true if the
    /// cap was reached before the priority queue emptied naturally, signaling
    /// that the caller should fall back to a fully elementary (correct but
    /// possibly slower) run instead of trusting this (possibly non-optimal)
    /// result.
    Label* run_inner_dp(const model::Problem&                 problem,
                        const DSSRSolverConfig&               config,
                        const std::vector<bool>&              critical_set,
                        std::vector<std::unique_ptr<Label>>&  label_pool,
                        std::vector<std::vector<Label*>>&     node_labels,
                        bool&                                  hit_cap) const;

    /// Walk parent chain from sink label; return visit count per node.
    std::vector<int> count_visits(const Label* sink_label, int num_nodes) const;

    /// Compute MS_m initial critical set via cycling attractiveness (§5).
    std::vector<bool> compute_ms_m_init(const model::Problem& problem, int m) const;
};

} // namespace oplib::solver::dp
