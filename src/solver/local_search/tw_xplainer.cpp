#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "solver/local_search/tw_xplainer.h"

namespace oplib::solver::local_search {

TWXplainer::TWXplainer(const model::Problem& problem)
    : problem_(problem)
{}

// All C(N, K) combinations as 1-based index vectors.
std::vector<std::vector<int>> TWXplainer::combinations(int N, int K) {
    std::string bitmask(K, 1);
    bitmask.resize(N, 0);
    std::vector<std::vector<int>> res;
    do {
        std::vector<int> c;
        c.reserve(K);
        for (int i = 0; i < N; ++i)
            if (bitmask[i]) c.push_back(i + 1);
        res.push_back(std::move(c));
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
    return res;
}

bool TWXplainer::extract_conflict(const std::vector<NodeId>& client_set,
                                   std::vector<TwConflict>&   conflict_list,
                                   bool                       multi_conflicts)
{
    conflict_list.clear();
    const int n = static_cast<int>(client_set.size());
    if (n == 0) return false;
    assert(n <= 63);

    const NodeId src    = problem_.get_source_depot();
    const NodeId sink   = problem_.get_sink_depot();
    const Time   budget = problem_.get_budget();

    cache_.clear();

    // Base case: singleton subsets — earliest arrival from depot
    for (int ic = 1; ic <= n; ++ic) {
        uint64_t mask = uint64_t(1) << ic;
        NodeId c = client_set[ic - 1];
        Time arrival = problem_.get_travel_time(src, c);
        if (arrival > problem_.get_time_window(c).closing)
            arrival = LATE_ARRIVAL;
        cache_[make_key(mask, ic)] = {arrival, 0};
    }

    // Bottom-up: enumerate subsets of size 2..n
    for (int subset_size = 2; subset_size <= n; ++subset_size) {
        auto all_subsets = combinations(n, subset_size);

        for (const auto& subset : all_subsets) {
            uint64_t S = 0;
            for (int ic : subset) S |= (uint64_t(1) << ic);

            int violation_hits = 0;

            for (int ik : subset) {
                uint64_t S_minus_k = S ^ (uint64_t(1) << ik);
                tmp_results_.clear();
                NodeId k = client_set[ik - 1];
                const auto& tw_k = problem_.get_time_window(k);

                // Best path to k through each predecessor m in S \ {k}
                for (int im : subset) {
                    if (im == ik) continue;
                    NodeId m = client_set[im - 1];
                    auto it = cache_.find(make_key(S_minus_k, im));
                    if (it == cache_.end()) continue;
                    Time arr_m = it->second.first;
                    if (arr_m == LATE_ARRIVAL) continue;

                    const auto& tw_m = problem_.get_time_window(m);
                    Time dep_m = std::max(arr_m, tw_m.opening);
                    Time arr_k = dep_m + problem_.get_travel_time(m, k, dep_m);
                    if (arr_k <= tw_k.closing)
                        tmp_results_.emplace_back(arr_k, im);
                }

                // Earliest arrival at k over all predecessors
                std::pair<Time, int> opt = {LATE_ARRIVAL, -1};
                if (!tmp_results_.empty()) {
                    opt = *std::min_element(tmp_results_.begin(), tmp_results_.end(),
                        [](const auto& a, const auto& b) { return a.first < b.first; });
                }
                cache_[make_key(S, ik)] = opt;

                // Check if k can be last and still reach the sink within budget
                if (opt.first == LATE_ARRIVAL) {
                    ++violation_hits;
                } else {
                    Time dep_k    = std::max(opt.first, tw_k.opening);
                    Time arr_sink = dep_k + problem_.get_travel_time(k, sink, dep_k);
                    if (arr_sink > budget)
                        ++violation_hits;
                }
            }

            // All clients fail → minimal conflict
            if (violation_hits == subset_size) {
                TwConflict cft;
                cft.reserve(subset_size);
                for (int ic : subset)
                    cft.push_back(client_set[ic - 1]);
                conflict_list.push_back(std::move(cft));
                if (!multi_conflicts)
                    return true;
            }
        }

        if (!conflict_list.empty())
            return true;
    }

    return false;
}

} // namespace oplib::solver::local_search
