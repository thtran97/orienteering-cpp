#pragma once

#include <vector>
#include <limits>

#include "solver/solver.h"
#include "solver/dynamic_programming/dp_solvers.h"

namespace oplib::solver::pulse {

/**
 * @brief Configuration for the Pulse solver.
 */
struct PulseSolverConfig : public SolverConfig {
    int max_labels = 1000000; ///< pulse expansion budget (0 = unlimited)
};

/**
 * @brief Pulse algorithm for the Orienteering Problem with Time Windows.
 *
 * The Pulse algorithm is a branch-and-bound approach over paths in the time-expanded
 * graph.  At each node a "pulse" is launched and pruned by:
 *   1. Time-window infeasibility.
 *   2. Budget infeasibility.
 *   3. Reward bound: current_reward + backward_bound[node] ≤ best_known.
 *
 * Backward bounds are pre-computed by BackwardDPSolver::compute_bounds().
 *
 * Ported from toptwLib/lib/src/solver/pulse_algo/pulse_graph.cpp.
 *
 * @warning Exact in theory; exponential worst-case.  Use only for small instances
 *          or with a label budget (max_labels) as a heuristic upper-bound.
 */
class PulseSolver : public Solver {
public:
    std::string get_name() const override { return "Pulse"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem& problem,
                          const PulseSolverConfig& config);

private:
    // State shared across recursive calls (avoids repeated argument passing)
    struct SearchState {
        const model::Problem*  problem;
        const std::vector<Reward>* ub;      ///< backward bounds per node
        std::vector<bool>      visited;
        std::vector<NodeId>    current_path;
        Reward                 current_reward  = 0.0;
        Time                   current_time    = 0.0;
        Reward                 best_reward     = 0.0;
        std::vector<NodeId>    best_path;
        int                    pulses_launched = 0;
        int                    max_labels;
    };

    void pulse(NodeId node, SearchState& state) const;
};

} // namespace oplib::solver::pulse
