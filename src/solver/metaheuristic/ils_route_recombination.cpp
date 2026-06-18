#include <algorithm>
#include <chrono>
#include <iostream>
#include <unordered_set>

#include "solver/metaheuristic/ils_route_recombination.h"
#include "core/random.h"

namespace oplib::solver::metaheuristic {

// ---------------------------------------------------------------------------
// Helper: Hamming similarity (fraction of non-depot customers shared)
// ---------------------------------------------------------------------------

double ILSRouteRecombinationSolver::hamming_similarity(const model::Solution& a,
                                                        const model::Solution& b,
                                                        int num_nodes) const
{
    std::vector<bool> in_a(num_nodes, false), in_b(num_nodes, false);
    for (int v = 0; v < a.get_num_vehicles(); ++v)
        for (NodeId n : a.get_route(v)) in_a[n] = true;
    for (int v = 0; v < b.get_num_vehicles(); ++v)
        for (NodeId n : b.get_route(v)) in_b[n] = true;

    int shared = 0, total = 0;
    for (int i = 0; i < num_nodes; ++i) {
        if (in_a[i] || in_b[i]) ++total;
        if (in_a[i] && in_b[i]) ++shared;
    }
    return total == 0 ? 1.0 : static_cast<double>(shared) / total;
}

// ---------------------------------------------------------------------------
// Helper: add to elite pool with diversity filter
// ---------------------------------------------------------------------------

void ILSRouteRecombinationSolver::add_to_pool(std::vector<model::Solution>& pool,
                                               const model::Solution&         sol,
                                               int                            pool_size,
                                               double                         similarity_threshold) const
{
    for (const auto& member : pool) {
        std::unordered_set<NodeId> member_customers;
        for (int v = 0; v < member.get_num_vehicles(); ++v) {
            const auto& rb = member.get_route(v);
            for (size_t i = 1; i + 1 < rb.size(); ++i)
                member_customers.insert(rb[i]);
        }

        int shared = 0, total_cust = 0;
        for (int v = 0; v < sol.get_num_vehicles(); ++v) {
            const auto& ra = sol.get_route(v);
            for (size_t i = 1; i + 1 < ra.size(); ++i) {
                ++total_cust;
                if (member_customers.count(ra[i])) ++shared;
            }
        }
        double sim = (total_cust == 0) ? 1.0 : static_cast<double>(shared) / total_cust;
        if (sim > similarity_threshold) return;
    }

    if (static_cast<int>(pool.size()) < pool_size) {
        pool.push_back(sol);
    } else {
        auto worst = std::min_element(pool.begin(), pool.end(),
            [](const model::Solution& x, const model::Solution& y){
                return x.total_reward < y.total_reward;
            });
        if (sol.total_reward > worst->total_reward)
            *worst = sol;
    }
}

// ---------------------------------------------------------------------------
// Helper: recompute visited flags + per-vehicle RouteContext for a solution
// ---------------------------------------------------------------------------

void ILSRouteRecombinationSolver::rebuild_bookkeeping(
    local_search::BaseLSUtils&                ls,
    const model::Problem&                     problem,
    const model::Solution&                    sol,
    std::vector<bool>&                        visited,
    std::vector<local_search::RouteContext>&  ctx)
{
    const int nn = static_cast<int>(problem.get_num_nodes());
    const int nv = problem.get_num_vehicles();
    visited.assign(nn, false);
    ctx.assign(nv, local_search::RouteContext{});
    for (int v = 0; v < nv; ++v) {
        for (NodeId n : sol.get_route(v)) visited[n] = true;
        ls.recompute_context(sol.get_route(v), ctx[v]);
    }
}

// ---------------------------------------------------------------------------
// Route recombination: max-weight set-packing over the pool's routes
// ---------------------------------------------------------------------------

model::Solution ILSRouteRecombinationSolver::recombine_routes(
    const model::Problem&               problem,
    const std::vector<model::Solution>& pool,
    local_search::BaseLSUtils&          ls,
    const local_search::LSConfig&       ls_cfg,
    const RRConfig&                     rr_cfg)
{
    const int    nv   = problem.get_num_vehicles();
    const int    nn   = static_cast<int>(problem.get_num_nodes());
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();

    struct Candidate { Reward reward; double sort_key; std::vector<NodeId> customers; };
    std::vector<Candidate> candidates;
    for (const auto& sol : pool) {
        for (int v = 0; v < sol.get_num_vehicles(); ++v) {
            const auto& r = sol.get_route(v);
            if (r.size() <= 2) continue;
            std::vector<NodeId> customers(r.begin() + 1, r.end() - 1);
            Reward reward = 0.0;
            for (NodeId c : customers) reward += problem.get_reward(c);
            // E2: sort by reward density (reward per customer) instead of total reward
            double sort_key = rr_cfg.density_sort
                ? reward / static_cast<double>(customers.size())
                : reward;
            candidates.push_back({reward, sort_key, std::move(customers)});
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.sort_key > b.sort_key; });

    model::Solution   recombined(nv);
    std::vector<bool> used(nn, false);
    Reward            placed_reward = 0.0;
    int               assigned      = 0;

    for (const auto& cand : candidates) {
        if (assigned >= nv) break;
        bool conflict = false;
        for (NodeId c : cand.customers)
            if (used[c]) { conflict = true; break; }
        if (conflict) continue;

        auto& route = recombined.get_route(assigned);
        route.assign(1, src);
        for (NodeId c : cand.customers) { route.push_back(c); used[c] = true; }
        route.push_back(sink);
        placed_reward += cand.reward;
        ++assigned;
    }
    for (int v = assigned; v < nv; ++v)
        recombined.get_route(v) = {src, sink};

    std::vector<bool>                       visited;
    std::vector<local_search::RouteContext> ctx;
    rebuild_bookkeeping(ls, problem, recombined, visited, ctx);
    recombined.total_reward = placed_reward;

    // E5: re-optimise transplanted route order before repair so that repair
    // sees a better-arranged starting point.
    if (rr_cfg.makespan_before)
        for (int v = 0; v < assigned; ++v)
            ls.minimize_makespan(recombined, ctx, v);

    ls.repair(recombined, visited, ctx, ls_cfg);
    for (int v = 0; v < nv; ++v)
        ls.minimize_makespan(recombined, ctx, v);

    // E1: swap lower-reward scheduled customers for higher-reward unvisited ones.
    if (rr_cfg.replace_pass)
        for (int v = 0; v < nv; ++v)
            ls.replace(recombined, visited, ctx, v);

    return recombined;
}

// ---------------------------------------------------------------------------
// Typed solve — forwards directly to do_solve
// ---------------------------------------------------------------------------

model::Solution ILSRouteRecombinationSolver::solve(
    const model::Problem&                    problem,
    const ILSRouteRecombinationSolverConfig& config)
{
    return do_solve(problem, config);
}

// ---------------------------------------------------------------------------
// do_solve
// ---------------------------------------------------------------------------

model::Solution ILSRouteRecombinationSolver::do_solve(
    const model::Problem&      problem,
    const BaseILSSolverConfig& base_cfg)
{
    using Clock = std::chrono::high_resolution_clock;

    const auto* cfg_ptr = dynamic_cast<const ILSRouteRecombinationSolverConfig*>(&base_cfg);
    int    pool_size            = cfg_ptr ? cfg_ptr->pool_size            : 10;
    double similarity_threshold = cfg_ptr ? cfg_ptr->similarity_threshold : 0.5;

    last_rr_time_ms_ = 0.0;

    oplib::utils::Random      rng(static_cast<uint32_t>(base_cfg.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = base_cfg.alpha;
    ls_cfg.rcl_size = base_cfg.rcl_size;

    const int nv = problem.get_num_vehicles();

    model::Solution                         best;
    std::vector<bool>                       best_vis;
    std::vector<local_search::RouteContext> best_ctx;
    ls.init(best, best_vis, best_ctx);
    construct(ls, ls_cfg, nv, best, best_vis, best_ctx);

    std::vector<model::Solution> elite_pool;
    add_to_pool(elite_pool, best, pool_size, similarity_threshold);

    model::Solution                         current  = best;
    std::vector<bool>                       cur_vis  = best_vis;
    std::vector<local_search::RouteContext> cur_ctx  = best_ctx;

    int shake_length = 1;
    int no_impr      = 0;

    auto t_start = Clock::now();

    for (int iter = 0;
         base_cfg.max_iterations <= 0 || iter < base_cfg.max_iterations;
         ++iter)
    {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= base_cfg.max_cpu_time) break;

        // Perturb
        for (int v = 0; v < nv; ++v) {
            const auto& route = current.get_route(v);
            const int   nc    = static_cast<int>(route.size()) - 2;
            if (nc <= 0) continue;
            int pos = rng.next_int(1, nc);
            ls.shake(current, cur_vis, cur_ctx, v, pos, shake_length);
        }
        construct(ls, ls_cfg, nv, current, cur_vis, cur_ctx);

        // Acceptance
        if (current.total_reward > best.total_reward) {
            best         = current;
            best_vis     = cur_vis;
            best_ctx     = cur_ctx;
            shake_length = 1;
            no_impr      = 0;
            add_to_pool(elite_pool, best, pool_size, similarity_threshold);
            if (base_cfg.verbose)
                std::cout << "[ILS+RR] iter=" << iter
                          << " reward=" << best.total_reward << '\n';
        } else {
            ++no_impr;
        }

        // Route recombination when stagnating
        if (no_impr >= base_cfg.restart_threshold && !elite_pool.empty()) {
            RRConfig prod_cfg;
            prod_cfg.replace_pass = true;   // E1 always on in production
            auto rr_t0 = Clock::now();
            model::Solution recombined = recombine_routes(problem, elite_pool, ls, ls_cfg, prod_cfg);
            last_rr_time_ms_ += std::chrono::duration<double, std::milli>(Clock::now() - rr_t0).count();

            if (recombined.total_reward > best.total_reward) {
                best = recombined;
                rebuild_bookkeeping(ls, problem, best, best_vis, best_ctx);
                add_to_pool(elite_pool, best, pool_size, similarity_threshold);
                shake_length = 1;
                if (base_cfg.verbose)
                    std::cout << "[ILS+RR] recombination -> "
                              << best.total_reward << '\n';
            } else {
                ++shake_length;
            }

            current = best;
            cur_vis = best_vis;
            cur_ctx = best_ctx;
            no_impr = 0;
        }
    }

    last_pool_ = elite_pool;
    return best;
}

} // namespace oplib::solver::metaheuristic
