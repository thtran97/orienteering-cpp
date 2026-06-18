#pragma once

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/types.h"
#include "model/problem.h"

namespace oplib::solver::local_search {

/**
 * A conflict over a set of customers is a minimal subset that cannot all be
 * visited in one route while respecting their time windows and the route budget.
 */
using TwConflict = std::vector<NodeId>;

/**
 * Held-Karp based time-window conflict extractor.
 *
 * Given a candidate client set, uses bottom-up subset enumeration to find
 * the smallest infeasible subsets (minimal TW conflicts). Stops as soon as
 * the first conflict size is found; can return all conflicts of that size.
 *
 * Uses uint64_t bitmasks (supports up to 63 clients per call; conflict_max_size
 * is typically 5, so this is always sufficient).
 *
 * Direct port of xplainer::HeldKarpSolver from kb_ls_cpp, adapted to use the
 * orienteering-cpp model::Problem API.
 */
class TWXplainer {
public:
    static constexpr Time LATE_ARRIVAL = 1e18;

    explicit TWXplainer(const model::Problem& problem);

    /**
     * Try to extract a minimal TW conflict from client_set.
     *
     * @param client_set      Candidate clients to examine (NodeId values).
     * @param conflict_list   Output: discovered minimal conflict(s).
     * @param multi_conflicts If true, return all conflicts of the minimum size;
     *                        if false, return after the first one found.
     * @return true if at least one conflict was found.
     */
    bool extract_conflict(const std::vector<NodeId>& client_set,
                          std::vector<TwConflict>&   conflict_list,
                          bool                       multi_conflicts = false);

private:
    const model::Problem& problem_;

    // DP cache: key -> (earliest_arrival_at_c, predecessor_index_in_client_set)
    std::unordered_map<size_t, std::pair<Time, int>> cache_;

    // Temporary result buffer (avoids repeated allocation)
    std::vector<std::pair<Time, int>> tmp_results_;

    static size_t make_key(uint64_t mask, int ic) {
        return (static_cast<size_t>(mask) << 8) | static_cast<uint32_t>(ic);
    }

    // Generate all combinations C(N, K) as index vectors (1-based).
    static std::vector<std::vector<int>> combinations(int N, int K);
};

} // namespace oplib::solver::local_search
