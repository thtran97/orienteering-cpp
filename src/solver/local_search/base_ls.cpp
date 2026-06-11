#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "solver/local_search/base_ls.h"

namespace oplib::solver::local_search {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

BaseLSUtils::BaseLSUtils(const model::Problem& problem, oplib::utils::Random& rng)
    : problem_(problem), rng_(rng)
{}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void BaseLSUtils::init(model::Solution& solution,
                       std::vector<bool>& visited,
                       std::vector<RouteContext>& contexts) const
{
    const NodeId src  = problem_.get_source_depot();
    const NodeId sink = problem_.get_sink_depot();
    const int nv      = problem_.get_num_vehicles();
    const int nn      = static_cast<int>(problem_.get_num_nodes());

    solution = model::Solution(nv);
    contexts.assign(nv, RouteContext{});
    visited.assign(nn, false);
    visited[src]  = true;
    visited[sink] = true;

    for (int v = 0; v < nv; ++v) {
        solution.get_route(v) = {src, sink};
        recompute_context(solution.get_route(v), contexts[v]);
    }
}

// ---------------------------------------------------------------------------
// Context recomputation
// ---------------------------------------------------------------------------

void BaseLSUtils::recompute_context(const std::vector<NodeId>& route,
                                    RouteContext& ctx) const
{
    const int n = static_cast<int>(route.size());
    ctx.arrival_times.assign(n, 0.0);
    ctx.departure_times.assign(n, 0.0);
    ctx.max_shift.assign(n, INF);
    ctx.cumulative_time = 0.0;

    // Forward pass: compute arrival and departure at each position
    // Source depot departs at its opening time (or 0 if no TW).
    {
        const auto& tw0 = problem_.get_time_window(route[0]);
        ctx.arrival_times[0]   = tw0.opening;
        ctx.departure_times[0] = tw0.opening + problem_.get_service_time(route[0]);
    }

    for (int i = 1; i < n; ++i) {
        NodeId prev = route[i - 1];
        NodeId curr = route[i];
        Time dep_prev  = ctx.departure_times[i - 1];
        Time travel    = problem_.get_travel_time(prev, curr, dep_prev);
        Time arrival   = dep_prev + travel;
        const auto& tw = problem_.get_time_window(curr);
        Time start_svc = std::max(arrival, tw.opening);
        ctx.arrival_times[i]   = arrival;
        ctx.departure_times[i] = start_svc + problem_.get_service_time(curr);
        ctx.cumulative_time += travel;
    }

    // Backward pass: compute max_shift at each position
    // max_shift[n-1] (sink depot) = tw.closing - departure
    {
        const auto& tw_sink  = problem_.get_time_window(route[n - 1]);
        ctx.max_shift[n - 1] = tw_sink.closing - ctx.departure_times[n - 1];
    }

    for (int i = n - 2; i >= 0; --i) {
        const auto& tw_i   = problem_.get_time_window(route[i]);
        Time own_slack     = tw_i.closing - ctx.departure_times[i];
        // Wait time at the next node absorbs some shift before propagating
        Time wait_next = std::max(0.0, problem_.get_time_window(route[i + 1]).opening
                                       - ctx.arrival_times[i + 1]);
        ctx.max_shift[i] = std::min(own_slack, wait_next + ctx.max_shift[i + 1]);
    }
}

// ---------------------------------------------------------------------------
// Feasibility checking
// ---------------------------------------------------------------------------

double BaseLSUtils::check_insertion(const model::Solution& solution,
                                    const std::vector<RouteContext>& contexts,
                                    int vehicle,
                                    NodeId customer,
                                    int position) const
{
    const auto& route = solution.get_route(vehicle);
    const auto& ctx   = contexts[vehicle];
    const int    rsz  = static_cast<int>(route.size());

    if (position < 1 || position >= rsz) return INF;

    NodeId prev = route[position - 1];
    NodeId next = route[position];
    Time dep_prev = ctx.departure_times[position - 1];

    // Travel times to and from customer
    Time t_prev_c = problem_.get_travel_time(prev, customer, dep_prev);
    Time arrival_c = dep_prev + t_prev_c;

    const auto& tw_c = problem_.get_time_window(customer);
    if (arrival_c > tw_c.closing) return INF;

    Time start_c = std::max(arrival_c, tw_c.opening);
    Time dep_c   = start_c + problem_.get_service_time(customer);
    Time t_c_next = problem_.get_travel_time(customer, next, dep_c);
    Time arrival_next_new = dep_c + t_c_next;

    const auto& tw_next = problem_.get_time_window(next);
    if (arrival_next_new > tw_next.closing) return INF;

    // Shift = change in arrival at next
    Time shift = arrival_next_new - ctx.arrival_times[position];

    // Wait at next absorbs shift before it propagates further
    Time wait_at_next = std::max(0.0, tw_next.opening - ctx.arrival_times[position]);
    if (shift > wait_at_next + ctx.max_shift[position]) return INF;

    // Budget constraint
    Time old_travel = problem_.get_travel_time(prev, next, dep_prev);
    Time extra_travel = t_prev_c + t_c_next - old_travel;
    if (ctx.cumulative_time + extra_travel > problem_.get_budget()) return INF;

    return extra_travel; // time_shift = additional travel time
}

// ---------------------------------------------------------------------------
// Public insertion wrapper
// ---------------------------------------------------------------------------

void BaseLSUtils::apply_insertion_public(model::Solution& solution,
                                          std::vector<RouteContext>& contexts,
                                          std::vector<bool>& visited,
                                          int vehicle,
                                          NodeId customer,
                                          int position,
                                          Time time_shift)
{
    apply_insertion(solution, contexts, visited, vehicle, customer, position, time_shift);
}

// ---------------------------------------------------------------------------
// Insertion application
// ---------------------------------------------------------------------------

void BaseLSUtils::apply_insertion(model::Solution& solution,
                                  std::vector<RouteContext>& contexts,
                                  std::vector<bool>& visited,
                                  int vehicle,
                                  NodeId customer,
                                  int position,
                                  Time time_shift)
{
    auto& route = solution.get_route(vehicle);
    auto& ctx   = contexts[vehicle];

    // Insert customer into route and extend context arrays
    route.insert(route.begin() + position, customer);

    const auto& tw_c = problem_.get_time_window(customer);
    NodeId prev = route[position - 1];
    Time dep_prev = ctx.departure_times[position - 1];
    Time t_prev_c = problem_.get_travel_time(prev, customer, dep_prev);
    Time arrival_c = dep_prev + t_prev_c;
    Time start_c   = std::max(arrival_c, tw_c.opening);
    Time dep_c     = start_c + problem_.get_service_time(customer);

    ctx.arrival_times.insert(ctx.arrival_times.begin() + position, arrival_c);
    ctx.departure_times.insert(ctx.departure_times.begin() + position, dep_c);
    ctx.max_shift.insert(ctx.max_shift.begin() + position, 0.0); // filled in backward pass

    // Forward pass: propagate shift through successors.
    // Break only when the departure time at node i is unchanged — that guarantees
    // all downstream nodes are unaffected. Checking arrival is wrong: a node that
    // absorbs the shift via its time-window wait leaves departure unchanged, but a
    // node with no wait and a decreased arrival also decreases its departure, which
    // must still propagate to its successors.
    const int rsz = static_cast<int>(route.size());
    for (int i = position + 1; i < rsz; ++i) {
        NodeId prv = route[i - 1];
        NodeId cur = route[i];
        Time dep_prv   = ctx.departure_times[i - 1];
        Time travel    = problem_.get_travel_time(prv, cur, dep_prv);
        Time arr       = dep_prv + travel;
        const auto& tw = problem_.get_time_window(cur);
        Time start_svc = std::max(arr, tw.opening);
        Time old_dep   = ctx.departure_times[i];
        ctx.arrival_times[i]   = arr;
        ctx.departure_times[i] = start_svc + problem_.get_service_time(cur);
        if (ctx.departure_times[i] == old_dep) break; // downstream unaffected
    }

    // Backward pass: recompute max_shift
    {
        const auto& tw_sink = problem_.get_time_window(route[rsz - 1]);
        ctx.max_shift[rsz - 1] = tw_sink.closing - ctx.departure_times[rsz - 1];
    }
    for (int i = rsz - 2; i >= 0; --i) {
        const auto& tw_i  = problem_.get_time_window(route[i]);
        Time own_slack    = tw_i.closing - ctx.departure_times[i];
        Time wait_next    = std::max(0.0, problem_.get_time_window(route[i + 1]).opening
                                          - ctx.arrival_times[i + 1]);
        ctx.max_shift[i] = std::min(own_slack, wait_next + ctx.max_shift[i + 1]);
    }

    ctx.cumulative_time += time_shift;
    solution.total_reward += problem_.get_reward(customer);
    visited[customer] = true;
}

// ---------------------------------------------------------------------------
// Customer removal
// ---------------------------------------------------------------------------

void BaseLSUtils::remove_customer_at(model::Solution& solution,
                                     std::vector<RouteContext>& contexts,
                                     std::vector<bool>& visited,
                                     int vehicle,
                                     int position)
{
    auto& route = solution.get_route(vehicle);
    auto& ctx   = contexts[vehicle];
    const int rsz = static_cast<int>(route.size());

    // position must refer to a customer (not a depot)
    if (position < 1 || position >= rsz - 1) return;

    NodeId removed = route[position];
    solution.total_reward -= problem_.get_reward(removed);
    visited[removed] = false;

    route.erase(route.begin() + position);
    ctx.arrival_times.erase(ctx.arrival_times.begin() + position);
    ctx.departure_times.erase(ctx.departure_times.begin() + position);
    ctx.max_shift.erase(ctx.max_shift.begin() + position);

    // Recompute context fully after removal (simpler and safe)
    recompute_context(route, ctx);
}

// ---------------------------------------------------------------------------
// Repair (RCL-based greedy construction)
// ---------------------------------------------------------------------------

double BaseLSUtils::heuristic_score(NodeId customer, Time time_shift, int alpha) const
{
    double r = static_cast<double>(problem_.get_reward(customer));
    return std::pow(r, alpha) / (time_shift + 1e-5);
}

void BaseLSUtils::repair(model::Solution& solution,
                         std::vector<bool>& visited,
                         std::vector<RouteContext>& contexts,
                         const LSConfig& config)
{
    const int nv  = problem_.get_num_vehicles();
    const int nn  = static_cast<int>(problem_.get_num_nodes());
    const NodeId src  = problem_.get_source_depot();
    const NodeId sink = problem_.get_sink_depot();

    // Min-heap: top = lowest score → popping top evicts the worst candidate,
    // keeping at most rcl_size best candidates.
    using CandidateEntry = std::pair<double, std::array<int, 3>>; // {score, {vehicle, customer, position}}
    auto cmp = [](const CandidateEntry& a, const CandidateEntry& b) {
        return a.first > b.first; // min-heap (lowest score on top)
    };
    std::priority_queue<CandidateEntry,
                        std::vector<CandidateEntry>,
                        decltype(cmp)> rcl(cmp);

    while (true) {
        // Clear the RCL for this round
        while (!rcl.empty()) rcl.pop();

        // For each unvisited customer, find the best (vehicle, position) by min time_shift
        for (NodeId c = 0; c < nn; ++c) {
            if (visited[c]) continue;
            if (c == src || c == sink) continue;

            double best_score = -1.0;
            int    best_v     = -1;
            int    best_pos   = -1;

            for (int v = 0; v < nv; ++v) {
                const auto& route = solution.get_route(v);
                const int rsz = static_cast<int>(route.size());

                double min_shift = INF;
                int    min_pos   = -1;

                for (int pos = 1; pos < rsz; ++pos) {
                    double shift = check_insertion(solution, contexts, v, c, pos);
                    if (shift < min_shift) {
                        min_shift = shift;
                        min_pos   = pos;
                    }
                }

                if (min_pos == -1) continue; // no feasible position in this vehicle

                double score = heuristic_score(c, min_shift, config.alpha);
                if (score > best_score) {
                    best_score = score;
                    best_v     = v;
                    best_pos   = min_pos;
                }
            }

            if (best_v == -1) continue; // not insertable anywhere

            // Maintain RCL as a capped min-heap
            if (static_cast<int>(rcl.size()) >= config.rcl_size) {
                if (best_score <= rcl.top().first) continue; // worse than all in RCL
                rcl.pop(); // evict worst
            }
            rcl.emplace(best_score, std::array<int, 3>{best_v, static_cast<int>(c), best_pos});
        }

        if (rcl.empty()) break;

        // Collect candidates for weighted random selection
        std::vector<CandidateEntry> candidates;
        double sum_scores = 0.0;
        while (!rcl.empty()) {
            sum_scores += rcl.top().first;
            candidates.push_back(rcl.top());
            rcl.pop();
        }

        // Weighted roulette selection.
        // candidates is ordered worst→best (min-heap pop order), so candidates.back()
        // is the highest-score candidate. Default to it so floating-point shortfall in
        // the accumulated sum still picks the best option, not the worst.
        double threshold = rng_.next_double(0.0, 1.0);
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
        double shift   = check_insertion(solution, contexts, chosen_v, chosen_c, chosen_pos);

        if (shift >= INF) continue; // stale (route changed) — try next round

        apply_insertion(solution, contexts, visited, chosen_v, chosen_c, chosen_pos, shift);
    }
}

// ---------------------------------------------------------------------------
// Destroy / Shake
// ---------------------------------------------------------------------------

void BaseLSUtils::shake(model::Solution& solution,
                        std::vector<bool>& visited,
                        std::vector<RouteContext>& contexts,
                        int vehicle,
                        int position,
                        int length)
{
    auto& route = solution.get_route(vehicle);
    const int rsz = static_cast<int>(route.size());
    const int num_customers = rsz - 2; // excluding depots

    if (num_customers <= 0 || length <= 0) return;

    // Clamp length to available customers
    int removal_len = std::min(length, num_customers);

    // Remove customers at positions [position, position+removal_len)
    // wrapping around the route if needed (circular shake).
    // We collect positions to remove and then erase back-to-front.
    std::vector<NodeId> to_remove;
    to_remove.reserve(removal_len);

    // Collect consecutive customer positions starting at `position`, wrapping
    // circularly around the customer segment [1, num_customers].
    for (int k = 0; k < removal_len; ++k) {
        int cur_pos = ((position - 1 + k) % num_customers) + 1;
        to_remove.push_back(route[cur_pos]);
    }

    // Remove by node ID (std::find) because earlier removals shift indices.
    for (NodeId removed : to_remove) {
        auto& rt = solution.get_route(vehicle);
        auto it  = std::find(rt.begin() + 1, rt.end() - 1, removed);
        if (it == rt.end() - 1) continue;
        int pos_in_rt = static_cast<int>(it - rt.begin());
        remove_customer_at(solution, contexts, visited, vehicle, pos_in_rt);
    }
}

void BaseLSUtils::destroy(model::Solution& solution,
                          std::vector<bool>& visited,
                          std::vector<RouteContext>& contexts,
                          int vehicle,
                          double removal_ratio)
{
    const auto& route     = solution.get_route(vehicle);
    const int num_customers = static_cast<int>(route.size()) - 2;
    if (num_customers <= 0) return;

    int max_removal = std::max(1, static_cast<int>(removal_ratio * num_customers));
    int removal_len = rng_.next_int(1, max_removal);
    int position    = rng_.next_int(1, num_customers); // 1-based customer position

    shake(solution, visited, contexts, vehicle, position, removal_len);
}

// ---------------------------------------------------------------------------
// Local search helpers
// ---------------------------------------------------------------------------

Time BaseLSUtils::propagate_through_subseq(Time departure_from_start,
                                           const std::vector<NodeId>& subseq,
                                           NodeId end_node) const
{
    Time t    = departure_from_start;
    NodeId prev = (subseq.empty() ? end_node : subseq[0]); // set below
    // We need the node before subseq[0]; caller provides departure_from_start
    // which already accounts for it.  So just propagate forward.
    for (int i = 0; i < static_cast<int>(subseq.size()); ++i) {
        NodeId cur = subseq[i];
        NodeId prv = (i == 0) ? end_node : subseq[i - 1]; // placeholder; see note
        // We can't reuse 'prv' from outside, so we need a different approach:
        // departure_from_start is departure of the node BEFORE subseq[0].
        // On first iteration we compute travel from that implicit prev node.
        // To handle this correctly, the caller must pass the prev node separately.
        // This method is called with departure_from_start already accounting for
        // travel from prev to subseq[0], so we just need subseq propagation.
        // Actually simpler: store prev node in the method signature.
        (void)prv; // unused — see overload below
        (void)cur;
        break;
    }
    // Use the correct propagation below (called via the overload with prev node)
    (void)t; (void)prev;
    return INF; // not directly called; see overloads used in check_swap/check_2opt
}

// Internal helper: propagate from 'departure after prev_node' through subseq to end_node.
static Time propagate(const model::Problem& problem,
                      NodeId prev_node,
                      Time dep_from_prev,
                      const std::vector<NodeId>& subseq,
                      NodeId end_node)
{
    Time t       = dep_from_prev;
    NodeId prv   = prev_node;
    for (NodeId cur : subseq) {
        t += problem.get_travel_time(prv, cur, t);
        const auto& tw = problem.get_time_window(cur);
        if (t > tw.closing) return BaseLSUtils::INF;
        t  = std::max(t, tw.opening) + problem.get_service_time(cur);
        prv = cur;
    }
    // Arrival at end_node
    t += problem.get_travel_time(prv, end_node, t);
    const auto& tw_end = problem.get_time_window(end_node);
    if (t > tw_end.closing) return BaseLSUtils::INF;
    return t; // arrival at end_node
}

Time BaseLSUtils::check_swap(const model::Solution& solution,
                             const std::vector<RouteContext>& contexts,
                             int vehicle,
                             int pos1,
                             int pos2) const
{
    const auto& route = solution.get_route(vehicle);
    const auto& ctx   = contexts[vehicle];
    const int rsz     = static_cast<int>(route.size());

    if (pos1 < 1 || pos2 >= rsz - 1 || pos1 >= pos2) return INF;

    NodeId prev_c1  = route[pos1 - 1];
    NodeId c1       = route[pos1];
    NodeId c2       = route[pos2];
    NodeId next_c2  = route[pos2 + 1];
    Time   dep_prev = ctx.departure_times[pos1 - 1];

    // New subsequence: [c2, mid..., c1]
    std::vector<NodeId> subseq;
    subseq.push_back(c2);
    for (int i = pos1 + 1; i < pos2; ++i) subseq.push_back(route[i]);
    subseq.push_back(c1);

    Time new_arrival_next = propagate(problem_, prev_c1, dep_prev, subseq, next_c2);
    if (new_arrival_next >= INF) return INF;

    Time shift = new_arrival_next - ctx.arrival_times[pos2 + 1];
    Time wait_next = std::max(0.0, problem_.get_time_window(next_c2).opening
                                   - ctx.arrival_times[pos2 + 1]);
    if (shift > wait_next + ctx.max_shift[pos2 + 1]) return INF;
    return shift;
}

Time BaseLSUtils::check_2opt(const model::Solution& solution,
                             const std::vector<RouteContext>& contexts,
                             int vehicle,
                             int pos1,
                             int pos2) const
{
    const auto& route = solution.get_route(vehicle);
    const auto& ctx   = contexts[vehicle];
    const int rsz     = static_cast<int>(route.size());

    if (pos1 < 1 || pos2 >= rsz - 1 || pos1 >= pos2) return INF;

    // Skip reversed segment if problem is time-dependent (reversal may be invalid)
    if (problem_.is_time_dependent()) return INF;

    NodeId prev_c1  = route[pos1 - 1];
    NodeId next_c2  = route[pos2 + 1];
    Time   dep_prev = ctx.departure_times[pos1 - 1];

    // New subsequence: reversed [pos1..pos2]
    std::vector<NodeId> subseq(route.begin() + pos1, route.begin() + pos2 + 1);
    std::reverse(subseq.begin(), subseq.end());

    Time new_arrival_next = propagate(problem_, prev_c1, dep_prev, subseq, next_c2);
    if (new_arrival_next >= INF) return INF;

    Time shift = new_arrival_next - ctx.arrival_times[pos2 + 1];
    Time wait_next = std::max(0.0, problem_.get_time_window(next_c2).opening
                                   - ctx.arrival_times[pos2 + 1]);
    if (shift > wait_next + ctx.max_shift[pos2 + 1]) return INF;
    return shift;
}

void BaseLSUtils::apply_swap(model::Solution& solution,
                             std::vector<RouteContext>& contexts,
                             int vehicle,
                             int pos1,
                             int pos2)
{
    auto& route = solution.get_route(vehicle);
    std::swap(route[pos1], route[pos2]);
    recompute_context(route, contexts[vehicle]);
}

void BaseLSUtils::apply_2opt(model::Solution& solution,
                             std::vector<RouteContext>& contexts,
                             int vehicle,
                             int pos1,
                             int pos2)
{
    auto& route = solution.get_route(vehicle);
    std::reverse(route.begin() + pos1, route.begin() + pos2 + 1);
    recompute_context(route, contexts[vehicle]);
}

// ---------------------------------------------------------------------------
// Minimize makespan
// ---------------------------------------------------------------------------

bool BaseLSUtils::minimize_makespan(model::Solution& solution,
                                    std::vector<RouteContext>& contexts,
                                    int vehicle)
{
    bool improved = false;
    const auto& route = solution.get_route(vehicle);
    const int rsz = static_cast<int>(route.size());

    // Swap phase: first-improvement, restarts on each improvement
    bool swap_improved = true;
    while (swap_improved) {
        swap_improved = false;
        for (int pi = 1; pi < rsz - 1 && !swap_improved; ++pi) {
            for (int pj = pi + 1; pj < rsz - 1 && !swap_improved; ++pj) {
                Time shift = check_swap(solution, contexts, vehicle, pi, pj);
                if (shift < -1e-9) { // shift reduces makespan
                    apply_swap(solution, contexts, vehicle, pi, pj);
                    improved      = true;
                    swap_improved = true; // restart outer loops
                }
            }
        }
    }

    // 2-opt phase
    bool twoopt_improved = true;
    while (twoopt_improved) {
        twoopt_improved = false;
        for (int pi = 1; pi < rsz - 1 && !twoopt_improved; ++pi) {
            for (int pj = pi + 1; pj < rsz - 1 && !twoopt_improved; ++pj) {
                Time shift = check_2opt(solution, contexts, vehicle, pi, pj);
                if (shift < -1e-9) {
                    apply_2opt(solution, contexts, vehicle, pi, pj);
                    improved        = true;
                    twoopt_improved = true;
                }
            }
        }
    }

    return improved;
}

} // namespace oplib::solver::local_search
