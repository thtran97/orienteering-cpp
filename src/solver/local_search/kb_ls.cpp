#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "solver/local_search/kb_ls.h"

namespace oplib::solver::local_search {

// ---------------------------------------------------------------------------
// kb_repair
// ---------------------------------------------------------------------------

void kb_repair(BaseLSUtils&                   ls,
               oplib::utils::Random&          rng,
               const model::Problem&          problem,
               model::Solution&               solution,
               std::vector<bool>&             visited,
               std::vector<RouteContext>&     contexts,
               const LSConfig&                config,
               knowledge_base::ConflictStore& kb,
               std::vector<InfeasiblePair>&    infeasible_out,
               const std::vector<bool>&        backbone)
{
    const int    nv   = problem.get_num_vehicles();
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    using CandidateEntry = std::pair<double, std::array<int, 3>>;
    auto cmp = [](const CandidateEntry& a, const CandidateEntry& b) {
        return a.first > b.first; // min-heap
    };
    std::priority_queue<CandidateEntry,
                        std::vector<CandidateEntry>,
                        decltype(cmp)> rcl(cmp);

    constexpr double INF = BaseLSUtils::INF;

    while (true) {
        while (!rcl.empty()) rcl.pop();

        for (NodeId c = 0; c < nn; ++c) {
            if (visited[c]) continue;
            if (c == src || c == sink) continue;
            if (!backbone.empty() && backbone[c]) continue; // backbone infeasible

            double best_score = -1.0;
            int    best_v     = -1;
            int    best_pos   = -1;

            for (int v = 0; v < nv; ++v) {
                // KB pre-filter
                if (!kb.check_assign(static_cast<int>(c), v)) {
                    infeasible_out.emplace_back(c, v);
                    continue;
                }

                const auto& route = solution.get_route(v);
                const int rsz = static_cast<int>(route.size());

                double min_shift = INF;
                int    min_pos   = -1;

                for (int pos = 1; pos < rsz; ++pos) {
                    double shift = ls.check_insertion(solution, contexts, v, c, pos);
                    if (shift < min_shift) {
                        min_shift = shift;
                        min_pos   = pos;
                    }
                }

                if (min_pos == -1) {
                    // TW infeasible in this vehicle
                    infeasible_out.emplace_back(c, v);
                    continue;
                }

                // Heuristic score: reward^alpha / (shift + eps)
                const double eps   = 1e-6;
                double reward = problem.get_reward(c);
                double score  = std::pow(reward, config.alpha) / (min_shift + eps);

                if (score > best_score) {
                    best_score = score;
                    best_v     = v;
                    best_pos   = min_pos;
                }
            }

            if (best_v == -1) continue;

            if (static_cast<int>(rcl.size()) >= config.rcl_size) {
                if (best_score <= rcl.top().first) continue;
                rcl.pop();
            }
            rcl.emplace(best_score, std::array<int, 3>{best_v, static_cast<int>(c), best_pos});
        }

        if (rcl.empty()) break;

        // Collect for weighted roulette selection
        std::vector<CandidateEntry> candidates;
        double sum_scores = 0.0;
        while (!rcl.empty()) {
            sum_scores += rcl.top().first;
            candidates.push_back(rcl.top());
            rcl.pop();
        }

        // Weighted roulette (mirrors BaseLSUtils::repair)
        // candidates is worst→best; default to best in case of float shortfall
        double threshold = rng.next_double(0.0, 1.0);
        double accum = 0.0;
        const CandidateEntry* chosen = &candidates.back();
        for (const auto& cand : candidates) {
            accum += cand.first / sum_scores;
            if (accum >= threshold) {
                chosen = &cand;
                break;
            }
        }

        int chosen_v   = (*chosen).second[0];
        int chosen_c   = (*chosen).second[1];
        int chosen_pos = (*chosen).second[2];
        double shift = ls.check_insertion(solution, contexts, chosen_v, chosen_c, chosen_pos);
        if (shift >= INF) continue; // stale

        ls.apply_insertion_public(solution, contexts, visited,
                                  chosen_v, chosen_c, chosen_pos, shift);
        kb.assign(chosen_c, chosen_v);
    }
}

// ---------------------------------------------------------------------------
// kb_sync
// ---------------------------------------------------------------------------

void kb_sync(knowledge_base::ConflictStore& kb,
             const model::Problem&          problem,
             const model::Solution&         solution)
{
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();
    const int nv      = problem.get_num_vehicles();

    kb.reset();
    for (int v = 0; v < nv; ++v) {
        for (NodeId c : solution.get_route(v)) {
            if (c == src || c == sink) continue;
            kb.assign(static_cast<int>(c), v);
        }
    }
}

// ---------------------------------------------------------------------------
// guess_conflict_scope
// ---------------------------------------------------------------------------

std::vector<NodeId> guess_conflict_scope(
    NodeId                 c,
    int                    vehicle,
    const model::Solution& solution,
    const model::Problem&  problem,
    int                    max_scope_size,
    ScopeHeuristic         heuristic,
    oplib::utils::Random&  rng)
{
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();
    const auto&  route = solution.get_route(vehicle);

    std::vector<NodeId> scope;
    scope.push_back(c);

    // Collect route customers (excluding depots and c itself)
    std::vector<NodeId> route_clients;
    for (NodeId ci : route) {
        if (ci == src || ci == sink || ci == c) continue;
        route_clients.push_back(ci);
    }

    const int want = max_scope_size - 1; // how many route clients to add
    if (want <= 0 || route_clients.empty()) return scope;

    if (heuristic == ScopeHeuristic::Random) {
        rng.shuffle(route_clients);
        for (int i = 0; i < want && i < static_cast<int>(route_clients.size()); ++i)
            scope.push_back(route_clients[i]);
        return scope;
    }

    // For the other heuristics, score each route client and keep the best `want`.
    // We use a max-heap capped at `want` (pop when full, keep smallest distance).
    // "Smallest distance = most similar" → we want to keep the `want` most similar.
    using Scored = std::pair<double, NodeId>; // (score = dissimilarity, client)
    // Min-heap so we can keep the `want` smallest dissimilarities (most similar)
    // Strategy: use a max-heap of size `want`; pop the largest (most dissimilar)
    // when full, keeping the best `want`.
    auto max_cmp = [](const Scored& a, const Scored& b) { return a.first < b.first; };
    std::priority_queue<Scored, std::vector<Scored>, decltype(max_cmp)> pq(max_cmp);

    const auto& tw_c = problem.get_time_window(c);

    for (NodeId ci : route_clients) {
        double score = 0.0;
        const auto& tw_ci = problem.get_time_window(ci);
        switch (heuristic) {
            case ScopeHeuristic::NearestTW: {
                double mid_c  = tw_c.opening  + tw_c.closing;
                double mid_ci = tw_ci.opening + tw_ci.closing;
                score = std::abs(mid_ci - mid_c);
                break;
            }
            case ScopeHeuristic::NearestClosing:
                score = std::abs(tw_ci.closing - tw_c.closing);
                break;
            case ScopeHeuristic::ShortestTW:
                score = tw_ci.closing - tw_ci.opening;
                break;
            default:
                break;
        }
        if (static_cast<int>(pq.size()) < want) {
            pq.emplace(score, ci);
        } else if (score < pq.top().first) {
            pq.pop();
            pq.emplace(score, ci);
        }
    }

    while (!pq.empty()) {
        scope.push_back(pq.top().second);
        pq.pop();
    }
    return scope;
}

// ---------------------------------------------------------------------------
// compute_backbone
// ---------------------------------------------------------------------------

std::vector<bool> compute_backbone(const model::Problem& problem)
{
    const int    nn     = static_cast<int>(problem.get_num_nodes());
    const NodeId src    = problem.get_source_depot();
    const NodeId sink   = problem.get_sink_depot();
    const Time   budget = problem.get_budget();

    std::vector<bool> infeasible(nn, false);

    for (NodeId c = 0; c < nn; ++c) {
        if (c == src || c == sink) continue;

        const auto& tw      = problem.get_time_window(c);
        const Time  service = problem.get_service_time(c);

        // Earliest possible arrival at c from source
        Time arrive = problem.get_travel_time(src, c);

        // Can we reach c before its TW closes?
        if (arrive > tw.closing) { infeasible[c] = true; continue; }

        // Wait for TW opening if we arrive early, then apply service
        Time depart = std::max(arrive, tw.opening) + service;

        // Can we reach the sink before the budget expires?
        Time arrive_sink = depart + problem.get_travel_time(c, sink);
        if (arrive_sink > budget) { infeasible[c] = true; }
    }

    return infeasible;
}

} // namespace oplib::solver::local_search
