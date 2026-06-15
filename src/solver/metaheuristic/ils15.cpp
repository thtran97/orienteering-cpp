#include <algorithm>
#include <chrono>
#include <iostream>

#include "solver/metaheuristic/ils15.h"
#include "core/random.h"

namespace oplib::solver::metaheuristic {

model::Solution ILS15Solver::solve(const model::Problem& problem,
                                    const ILS15SolverConfig& config)
{
    return do_solve(problem, config);
}

model::Solution ILS15Solver::do_solve(const model::Problem&      problem,
                                       const BaseILSSolverConfig& base_cfg)
{
    using Clock = std::chrono::high_resolution_clock;

    const auto* cfg_ptr = dynamic_cast<const ILS15SolverConfig*>(&base_cfg);
    int post_increment_period = cfg_ptr ? cfg_ptr->post_increment_period : 2;

    oplib::utils::Random      rng(static_cast<uint32_t>(base_cfg.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = base_cfg.alpha;
    ls_cfg.rcl_size = base_cfg.rcl_size;

    const int nv = problem.get_num_vehicles();

    // Build initial solution
    model::Solution                         best;
    std::vector<bool>                       best_vis;
    std::vector<local_search::RouteContext> best_ctx;
    ls.init(best, best_vis, best_ctx);
    construct(ls, ls_cfg, nv, best, best_vis, best_ctx);
    for (int v = 0; v < nv; ++v)
        ls.replace(best, best_vis, best_ctx, v);

    model::Solution                         current  = best;
    std::vector<bool>                       cur_vis  = best_vis;
    std::vector<local_search::RouteContext> cur_ctx  = best_ctx;

    // Deterministic shake parameters
    int cons          = 1;  // shake length
    int post          = 1;  // shake start position (global, clamped per vehicle)
    int shake_counter = 0;
    int no_impr       = 0;

    auto t_start = Clock::now();

    for (int iter = 0;
         base_cfg.max_iterations <= 0 || iter < base_cfg.max_iterations;
         ++iter)
    {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= base_cfg.max_cpu_time) break;

        // Compute smallest and largest non-empty route customer counts for wrap logic.
        int smallest_nc = std::numeric_limits<int>::max();
        int largest_nc  = 0;
        for (int v = 0; v < nv; ++v) {
            int nc = std::max(0, static_cast<int>(current.get_route(v).size()) - 2);
            if (nc > 0 && nc < smallest_nc) smallest_nc = nc;
            if (nc > largest_nc)             largest_nc  = nc;
        }
        if (smallest_nc == std::numeric_limits<int>::max()) smallest_nc = 1;
        if (largest_nc  == 0)                               largest_nc  = 1;

        // Shake each vehicle using per-vehicle clamped position.
        for (int v = 0; v < nv; ++v) {
            const auto& route = current.get_route(v);
            const int   nc    = std::max(0, static_cast<int>(route.size()) - 2);
            if (nc <= 0) continue;
            // Clamp post to [1, nc] circularly.
            int effective_pos = ((post - 1) % nc) + 1;
            ls.shake(current, cur_vis, cur_ctx, v, effective_pos, cons);
        }

        // Update post and shake_counter.
        post += cons;
        if (post > smallest_nc) post -= smallest_nc;
        ++shake_counter;

        // Construct + replace local-search move.
        construct(ls, ls_cfg, nv, current, cur_vis, cur_ctx);
        for (int v = 0; v < nv; ++v)
            ls.replace(current, cur_vis, cur_ctx, v);

        // Always accept (random walk).
        if (current.total_reward > best.total_reward) {
            best      = current;
            best_vis  = cur_vis;
            best_ctx  = cur_ctx;
            cons          = 1;
            post          = 1;
            shake_counter = 0;
            no_impr       = 0;
            if (base_cfg.verbose)
                std::cout << "[ILS15] iter=" << iter
                          << " reward=" << best.total_reward << '\n';
        } else {
            ++no_impr;
        }

        // Evolve cons every post_increment_period shakes.
        if (shake_counter > 0 && shake_counter % post_increment_period == 0) {
            ++cons;
            if (cons > largest_nc) cons = 1;
        }

        // Periodic restart to best.
        if (no_impr >= base_cfg.restart_threshold) {
            current = best;
            cur_vis = best_vis;
            cur_ctx = best_ctx;
            no_impr = 0;
        }
    }

    return best;
}

} // namespace oplib::solver::metaheuristic
