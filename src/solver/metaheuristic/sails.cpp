#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include "solver/metaheuristic/sails.h"
#include "core/random.h"

namespace oplib::solver::metaheuristic {

model::Solution SAILSSolver::solve(const model::Problem& problem,
                                   const SolverConfig&   config)
{
    SAILSSolverConfig cfg;
    cfg.seed           = config.seed;
    cfg.max_cpu_time   = config.max_cpu_time;
    cfg.max_iterations = config.max_iterations;
    cfg.verbose        = config.verbose;
    return solve(problem, cfg);
}

model::Solution SAILSSolver::solve(const model::Problem&    problem,
                                   const SAILSSolverConfig& config)
{
    using Clock = std::chrono::high_resolution_clock;

    oplib::utils::Random      rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    const int nv = problem.get_num_vehicles();

    auto construct = [&](model::Solution&                         sol,
                         std::vector<bool>&                       vis,
                         std::vector<local_search::RouteContext>& ctx)
    {
        ls.repair(sol, vis, ctx, ls_cfg);
        for (int v = 0; v < nv; ++v)
            ls.minimize_makespan(sol, ctx, v);
    };

    // Incumbent (current) and global best.
    model::Solution                         best;
    std::vector<bool>                       best_vis;
    std::vector<local_search::RouteContext> best_ctx;
    ls.init(best, best_vis, best_ctx);
    construct(best, best_vis, best_ctx);

    model::Solution                         current = best;
    std::vector<bool>                       cur_vis = best_vis;
    std::vector<local_search::RouteContext> cur_ctx = best_ctx;

    double       T       = std::max(config.initial_temp, 1e-6);
    const double cooling = (config.cooling_rate > 0.0 && config.cooling_rate < 1.0)
                           ? config.cooling_rate : 0.75;
    int          shake_length = 1;

    const auto t_start = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t_start).count(); };

    // max_iterations <= 0 means "no iteration cap": bound purely by max_cpu_time.
    int iter = 0;
    auto iter_ok = [&] { return config.max_iterations <= 0 || iter < config.max_iterations; };

    while (iter_ok() && elapsed() < config.max_cpu_time) {
        for (int inner = 0;
             inner < config.inner_loop && iter_ok() && elapsed() < config.max_cpu_time;
             ++inner, ++iter)
        {
            // Save the incumbent so a rejected move can be undone.
            model::Solution                         saved     = current;
            std::vector<bool>                       saved_vis = cur_vis;
            std::vector<local_search::RouteContext> saved_ctx = cur_ctx;

            // Perturb every vehicle, then repair + local search.
            for (int v = 0; v < nv; ++v) {
                const int nc = static_cast<int>(current.get_route(v).size()) - 2;
                if (nc <= 0) continue;
                int pos = rng.next_int(1, nc);
                ls.shake(current, cur_vis, cur_ctx, v, pos, shake_length);
            }
            construct(current, cur_vis, cur_ctx);

            // Metropolis acceptance (reward is maximised, so worse => delta < 0).
            const double delta  = current.total_reward - saved.total_reward;
            const bool   accept = (delta > 0.0) || (rng.next_double() < std::exp(delta / T));

            if (accept) {
                if (current.total_reward > best.total_reward) {
                    best = current; best_vis = cur_vis; best_ctx = cur_ctx;
                    shake_length = 1;
                } else if (delta <= 0.0) {
                    // Accepted a sideways/worse move: widen the perturbation.
                    shake_length = std::min(shake_length + 1, config.max_shake_length);
                }
            } else {
                current = saved; cur_vis = saved_vis; cur_ctx = saved_ctx;
            }
        }

        T *= cooling;
        if (T < 1e-3) { T = config.initial_temp; shake_length = 1; }  // reheat
    }

    if (config.verbose)
        std::cout << "[SAILS] best=" << best.total_reward << '\n';
    return best;
}

} // namespace oplib::solver::metaheuristic
