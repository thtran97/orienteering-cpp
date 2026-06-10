#pragma once

#include <memory>
#include <vector>

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"
#include "solver/policy_learning/mcts_node.h"

namespace oplib::solver::policy_learning {

/**
 * @brief Configuration for the MCTS solver.
 */
struct MCTSSolverConfig : public SolverConfig {
    int    alpha    = oplib::constants::DEFAULT_ALPHA;
    int    rcl_size = oplib::constants::DEFAULT_RCL_SIZE;
    int    rollout_depth = 10; ///< max repair steps in the simulation phase
};

/**
 * @brief Monte Carlo Tree Search solver for the Orienteering Problem.
 *
 * Each MCTS iteration:
 *  1. Selection  — traverse the tree using UCB1 until a leaf is reached.
 *  2. Expansion  — add a new child for one unvisited feasible customer.
 *  3. Simulation — greedy rollout (repair) from the expanded state.
 *  4. Backpropagation — update nb_simulations and reward_estimated up to root.
 *
 * Adapted from toptwLib/lib/src/solver/policy_learning/mcts_solver.cpp.
 * Uses BaseLSUtils::repair() for the rollout phase.
 */
class MCTSSolver : public Solver {
public:
    std::string get_name() const override { return "MCTS"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem&   problem,
                          const MCTSSolverConfig& config);

private:
    // -----------------------------------------------------------------------
    // Tree management helpers
    // -----------------------------------------------------------------------
    MCTSNode* select(MCTSNode* root) const;
    MCTSNode* expand(MCTSNode* node,
                     const model::Problem& problem,
                     oplib::utils::Random& rng) const;
    double    simulate(MCTSNode* node,
                       const model::Problem& problem,
                       local_search::BaseLSUtils& ls,
                       const local_search::LSConfig& ls_cfg) const;
    void      backpropagate(MCTSNode* node, double reward) const;

    /// Extract the greedy policy solution from the tree (path with best avg value).
    model::Solution extract_best(MCTSNode* root,
                                  const model::Problem& problem,
                                  local_search::BaseLSUtils& ls,
                                  const local_search::LSConfig& ls_cfg) const;

    /// Recursively free all tree nodes.
    static void free_tree(MCTSNode* node);
};

} // namespace oplib::solver::policy_learning
