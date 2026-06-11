#pragma once

#include <unordered_map>
#include <vector>

#include "core/types.h"

namespace oplib::solver::policy_learning {

/**
 * @brief A node in the Monte Carlo Tree Search tree.
 *
 * Tracks the last-visited node, accumulated time/reward along the path,
 * visited-customer flags, and UCB statistics (nb_simulations, value).
 *
 * Adapted from toptwLib/lib/include/solver/policy_learning/mcts_node.h.
 * Uses std::vector<bool> instead of boost::dynamic_bitset.
 */
struct MCTSNode {
    NodeId                state;         ///< last visited node id
    double                reward_collected  = 0.0;
    Time                  time_consumed     = 0.0;
    int                   nb_simulations    = 0;
    double                reward_estimated  = 0.0; ///< rollout value estimate

    std::vector<bool>     visited;       ///< which nodes have been visited on this path
    MCTSNode*             parent         = nullptr;
    std::unordered_map<NodeId, MCTSNode*> children; ///< action → child

    MCTSNode(NodeId state_, MCTSNode* parent_, std::vector<bool> visited_)
        : state(state_), visited(std::move(visited_)), parent(parent_) {}

    ~MCTSNode() = default;

    void add_child(NodeId action, MCTSNode* child) { children[action] = child; }
    MCTSNode* get_child(NodeId action) {
        auto it = children.find(action);
        return (it != children.end()) ? it->second : nullptr;
    }

    /// UCB1 score used for selection.
    double ucb_score() const {
        constexpr double EPS = 1e-5;
        double avg = (reward_collected + reward_estimated) / (nb_simulations + EPS);
        double explore = std::sqrt(2.0 * std::log(parent->nb_simulations + 1)
                                   / (nb_simulations + EPS));
        return avg + explore;
    }
};

} // namespace oplib::solver::policy_learning
