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
    // Check similarity against existing pool members.
    // Build a flat set of each member's customers (all vehicles combined) for O(1)
    // lookup, reducing the per-member check from O(V·n²) to O(V·n).
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
        if (sim > similarity_threshold) return; // too similar — don't add
    }

    if (static_cast<int>(pool.size()) < pool_size) {
        pool.push_back(sol);
    } else {
        // Replace the worst pool member
        auto worst = std::min_element(pool.begin(), pool.end(),
            [](const model::Solution& x, const model::Solution& y){
                return x.total_reward < y.total_reward;
            });
        if (sol.total_reward > worst->total_reward)
            *worst = sol;
    }
}

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

model::Solution ILSRouteRecombinationSolver::solve(const model::Problem& problem,
                                                    const SolverConfig&   config)
{
    ILSRouteRecombinationSolverConfig cfg;
    cfg.seed              = config.seed;
    cfg.max_cpu_time      = config.max_cpu_time;
    cfg.max_iterations    = config.max_iterations;
    cfg.verbose           = config.verbose;
    return solve(problem, cfg);
}

model::Solution ILSRouteRecombinationSolver::solve(
    const model::Problem&                    problem,
    const ILSRouteRecombinationSolverConfig& config)
{
    using Clock = std::chrono::high_resolution_clock;

    oplib::utils::Random      rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    const int nv = problem.get_num_vehicles();
    const int nn = static_cast<int>(problem.get_num_nodes());

    auto construct = [&](model::Solution&                         sol,
                         std::vector<bool>&                       vis,
                         std::vector<local_search::RouteContext>& ctx)
    {
        ls.repair(sol, vis, ctx, ls_cfg);
        for (int v = 0; v < nv; ++v)
            ls.minimize_makespan(sol, ctx, v);
    };

    // Initial solution
    model::Solution                         best;
    std::vector<bool>                       best_vis;
    std::vector<local_search::RouteContext> best_ctx;
    ls.init(best, best_vis, best_ctx);
    construct(best, best_vis, best_ctx);

    std::vector<model::Solution> elite_pool;
    add_to_pool(elite_pool, best, config.pool_size, config.similarity_threshold);

    model::Solution                         current  = best;
    std::vector<bool>                       cur_vis  = best_vis;
    std::vector<local_search::RouteContext> cur_ctx  = best_ctx;

    int shake_length = 1;
    int no_impr      = 0;

    auto t_start = Clock::now();

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;

        // Perturb
        for (int v = 0; v < nv; ++v) {
            const auto& route = current.get_route(v);
            const int   nc    = static_cast<int>(route.size()) - 2;
            if (nc <= 0) continue;
            int pos = rng.next_int(1, nc);
            ls.shake(current, cur_vis, cur_ctx, v, pos, shake_length);
        }
        construct(current, cur_vis, cur_ctx);

        // Acceptance
        if (current.total_reward > best.total_reward) {
            best         = current;
            best_vis     = cur_vis;
            best_ctx     = cur_ctx;
            shake_length = 1;
            no_impr      = 0;
            add_to_pool(elite_pool, best, config.pool_size, config.similarity_threshold);
            if (config.verbose)
                std::cout << "[ILS+RR] iter=" << iter << " reward=" << best.total_reward << '\n';
        } else {
            ++no_impr;
        }

        // Route recombination when stagnating
        if (no_impr >= config.restart_threshold && !elite_pool.empty()) {
            // Pick a guiding solution from the pool
            const auto& guide = elite_pool[rng.next_int(0, static_cast<int>(elite_pool.size()) - 1)];

            // Collect customers in guide but not in current
            std::vector<bool> in_current(nn, false);
            for (int v = 0; v < nv; ++v)
                for (NodeId n : current.get_route(v)) in_current[n] = true;

            // Try inserting guide customers into current greedily
            for (int v = 0; v < guide.get_num_vehicles(); ++v) {
                const auto& gr = guide.get_route(v);
                for (size_t i = 1; i + 1 < gr.size(); ++i) {
                    NodeId c = gr[i];
                    if (in_current[c]) continue;

                    // Find best insertion across all vehicles
                    double best_shift = local_search::BaseLSUtils::INF;
                    int    best_veh   = -1, best_pos = -1;
                    for (int tv = 0; tv < nv; ++tv) {
                        const auto& tr = current.get_route(tv);
                        for (int p = 1; p < static_cast<int>(tr.size()); ++p) {
                            double shift = ls.check_insertion(current, cur_ctx, tv, c, p);
                            if (shift < best_shift) { best_shift = shift; best_veh = tv; best_pos = p; }
                        }
                    }
                    if (best_veh >= 0 && best_shift < local_search::BaseLSUtils::INF) {
                        ls.apply_insertion_public(current, cur_ctx, cur_vis, best_veh, c, best_pos, best_shift);
                        in_current[c] = true;
                    }
                }
            }

            // Accept recombined if better
            if (current.total_reward > best.total_reward) {
                best         = current;
                best_vis     = cur_vis;
                best_ctx     = cur_ctx;
                shake_length = 1;
                add_to_pool(elite_pool, best, config.pool_size, config.similarity_threshold);
            } else {
                current  = best;
                cur_vis  = best_vis;
                cur_ctx  = best_ctx;
                ++shake_length;
            }
            no_impr = 0;
        }
    }

    return best;
}

} // namespace oplib::solver::metaheuristic
