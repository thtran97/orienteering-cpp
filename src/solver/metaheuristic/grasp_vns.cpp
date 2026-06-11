#include <chrono>
#include <iostream>

#include "solver/metaheuristic/grasp_vns.h"
#include "core/random.h"

namespace oplib::solver::metaheuristic {

model::Solution GraspVnsSolver::solve(const model::Problem& problem,
                                       const SolverConfig&   config)
{
    GraspVnsSolverConfig cfg;
    cfg.seed            = config.seed;
    cfg.max_cpu_time    = config.max_cpu_time;
    cfg.max_iterations  = config.max_iterations;
    cfg.verbose         = config.verbose;
    return solve(problem, cfg);
}

model::Solution GraspVnsSolver::solve(const model::Problem&       problem,
                                       const GraspVnsSolverConfig& config)
{
    using Clock = std::chrono::high_resolution_clock;

    oplib::utils::Random         rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils    ls(problem, rng);
    local_search::LSConfig       ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    const int nv = problem.get_num_vehicles();

    model::Solution best;
    std::vector<bool>                       best_visited;
    std::vector<local_search::RouteContext> best_ctx;

    auto t_start = Clock::now();

    for (int iter = 0; ; ++iter) {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;
        if (iter >= config.max_iterations)  break;

        // ---- 1. Init ----
        model::Solution                         current;
        std::vector<bool>                       visited;
        std::vector<local_search::RouteContext> ctx;
        ls.init(current, visited, ctx);

        // ---- 2. GRASP construction (repair + makespan) ----
        ls.repair(current, visited, ctx, ls_cfg);
        for (int v = 0; v < nv; ++v)
            ls.minimize_makespan(current, ctx, v);

        // ---- 3. VNS ----
        // All position attempts within one shake_len level start from the same
        // state, so we save once per shake_len rather than once per (v, pos).
        for (int shake_len = 1; shake_len <= config.max_shake_length; ) {
            bool any_improved = false;

            model::Solution                         saved_sol = current;
            std::vector<bool>                       saved_vis = visited;
            std::vector<local_search::RouteContext> saved_ctx = ctx;

            for (int v = 0; v < nv && !any_improved; ++v) {
                const int rsz = static_cast<int>(saved_sol.get_route(v).size());
                for (int pos = 1; pos < rsz - 1 && !any_improved; ++pos) {
                    // Restore checkpoint before each attempt
                    current = saved_sol;
                    visited = saved_vis;
                    ctx     = saved_ctx;

                    ls.shake(current, visited, ctx, v, pos, shake_len);
                    ls.repair(current, visited, ctx, ls_cfg);

                    if (current.total_reward > saved_sol.total_reward) {
                        ls.minimize_makespan(current, ctx, v);
                        shake_len    = 1;
                        any_improved = true;
                    }
                }
            }

            if (!any_improved) {
                current = saved_sol; // last attempt left current in a shaken state
                visited = saved_vis;
                ctx     = saved_ctx;
                ++shake_len;
            }
        }

        // ---- 4. Update global best ----
        if (current.total_reward > best.total_reward) {
            best         = current;
            best_visited = visited;
            best_ctx     = ctx;
            if (config.verbose) {
                std::cout << "[GRASP+VNS] iter=" << iter
                          << " reward=" << best.total_reward << '\n';
            }
        }
    }

    return best;
}

} // namespace oplib::solver::metaheuristic
