#include <chrono>
#include <iostream>

#include "solver/metaheuristic/lns.h"
#include "core/random.h"

namespace oplib::solver::metaheuristic {

model::Solution LNSSolver::solve(const model::Problem& problem,
                                  const SolverConfig&   config)
{
    LNSSolverConfig cfg;
    cfg.seed           = config.seed;
    cfg.max_cpu_time   = config.max_cpu_time;
    cfg.max_iterations = config.max_iterations;
    cfg.verbose        = config.verbose;
    return solve(problem, cfg);
}

model::Solution LNSSolver::solve(const model::Problem& problem,
                                  const LNSSolverConfig& config)
{
    using Clock = std::chrono::high_resolution_clock;

    oplib::utils::Random      rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha         = config.alpha;
    ls_cfg.rcl_size      = config.rcl_size;
    ls_cfg.removal_ratio = config.removal_ratio;

    const int nv = problem.get_num_vehicles();

    // Initial solution
    model::Solution                         best;
    std::vector<bool>                       visited;
    std::vector<local_search::RouteContext> ctx;
    ls.init(best, visited, ctx);
    ls.repair(best, visited, ctx, ls_cfg);

    auto t_start = Clock::now();

    for (int iter = 0; config.max_iterations <= 0 || iter < config.max_iterations; ++iter) {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;

        // Work on a copy of the best solution
        model::Solution                         current  = best;
        std::vector<bool>                       cur_vis  = visited;
        std::vector<local_search::RouteContext> cur_ctx  = ctx;

        // Destroy: remove a fraction of customers from each vehicle
        for (int v = 0; v < nv; ++v)
            ls.destroy(current, cur_vis, cur_ctx, v, config.removal_ratio);

        // Repair: reinsert unvisited customers
        ls.repair(current, cur_vis, cur_ctx, ls_cfg);

        if (current.total_reward > best.total_reward) {
            best    = current;
            visited = cur_vis;
            ctx     = cur_ctx;
            if (config.verbose) {
                std::cout << "[LNS] iter=" << iter
                          << " reward=" << best.total_reward << '\n';
            }
        }
    }

    return best;
}

} // namespace oplib::solver::metaheuristic
