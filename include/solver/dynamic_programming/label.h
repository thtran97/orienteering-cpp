#pragma once

#include <vector>
#include <string>

#include "core/types.h"

namespace oplib::solver::dp {

/**
 * @brief A label representing a partial path state in the DP algorithm.
 *
 * Tracks the current node, elapsed time, collected reward, and a visited-customer
 * bitfield.  Labels are used by the forward, backward and bidirectional DP solvers.
 *
 * Adapted from toptwLib/lib/include/solver/dynamic_programming/label.h.
 * Uses std::vector<bool> instead of boost::dynamic_bitset.
 */
struct Label {
    NodeId            last_visit        = 0;
    Time              time_consumed     = 0.0; ///< departure time from last_visit
    Reward            profit_collected  = 0.0;
    std::vector<bool> is_visited;              ///< one flag per node

    int     nb_visits = 0;
    Label*  parent    = nullptr;
    bool    extended  = false;
    bool    dominated = false;

    Label() = default;

    Label(NodeId node, Time time, Reward profit, Label* prev,
          std::vector<bool> visited)
        : last_visit(node)
        , time_consumed(time)
        , profit_collected(profit)
        , is_visited(std::move(visited))
        , parent(prev)
    {
        nb_visits = (prev != nullptr) ? prev->nb_visits + 1 : 0;
    }

    /// Dominance check: this label dominates `other` if it is at least as good
    /// on all criteria (time ≤, profit ≥, visited ⊆).
    bool dominates(const Label& other) const {
        if (time_consumed  > other.time_consumed)  return false;
        if (profit_collected < other.profit_collected) return false;
        // visited subset check
        for (size_t i = 0; i < is_visited.size(); ++i)
            if (is_visited[i] && !other.is_visited[i]) return false;
        return true;
    }

    std::string to_string() const;
};

} // namespace oplib::solver::dp
