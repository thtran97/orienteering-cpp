/*
ILS solver implementation for TOPTW

Reference: "Vansteenwegen, P., Souffriau, W., Vanden Berghe, G., & Van Oudheusden, D. (2009). Iterated local search for the team orienteering problem with time windows. Computers & Operations Research, 36(12), 3281-3290."
*/

#include <chrono>
#include <iostream>

#include "solver/metaheuristic/ils09.h"
#include "core/random.h"

namespace oplib::solver::metaheuristic {

model::Solution ILS09Solver::solve(
    const model::Problem& problem,
    const ILS09SolverConfig& config)
{
    return do_solve(problem, config);
}

model::Solution ILS09Solver::do_solve(
    const model::Problem& problem,
    const BaseILSSolverConfig& config)
{
    using Clock = std::chrono::high_resolution_clock;

    oplib::utils::Random      rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    const int nv = problem.get_num_vehicles();

    // Build initial solution
    model::Solution                         best;
    std::vector<bool>                       best_vis;
    std::vector<local_search::RouteContext> best_ctx;
    ls.init(best, best_vis, best_ctx);
    construct(ls, ls_cfg, nv, best, best_vis, best_ctx);

    model::Solution                         current  = best;
    std::vector<bool>                       cur_vis  = best_vis;
    std::vector<local_search::RouteContext> cur_ctx  = best_ctx;

    int shake_length = 1;
    int no_impr      = 0;

    auto t_start = Clock::now();

    for (int iter = 0; config.max_iterations <= 0 || iter < config.max_iterations; ++iter) {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;

        // Perturb: shake each vehicle at a random position
        for (int v = 0; v < nv; ++v) {
            const auto& route = current.get_route(v);
            const int   nc    = static_cast<int>(route.size()) - 2;
            if (nc <= 0) continue;
            int pos = rng.next_int(1, nc);
            ls.shake(current, cur_vis, cur_ctx, v, pos, shake_length);
        }

        // Construct from perturbed solution
        construct(ls, ls_cfg, nv, current, cur_vis, cur_ctx);

        // Acceptance
        if (current.total_reward > best.total_reward) {
            best         = current;
            best_vis     = cur_vis;
            best_ctx     = cur_ctx;
            shake_length = 1;
            no_impr      = 0;
            if (config.verbose)
                std::cout << "[ILS09] iter=" << iter << " reward=" << best.total_reward << '\n';
        } else {
            ++no_impr;
            if (no_impr >= config.restart_threshold) {
                // Restart from best with longer shake
                current  = best;
                cur_vis  = best_vis;
                cur_ctx  = best_ctx;
                ++shake_length;
                no_impr = 0;
            }
        }
    }

    return best;
}

} // namespace oplib::solver::metaheuristic
