#include <chrono>
#include <iostream>

#include "solver/constructive/randomized_greedy.h"
#include "core/random.h"

namespace oplib::solver::constructive {

model::Solution RandomizedGreedySolver::solve(const model::Problem& problem,
                                               const SolverConfig&   config)
{
    RandomizedGreedySolverConfig rg_config;
    rg_config.seed           = config.seed;
    rg_config.max_cpu_time   = config.max_cpu_time;
    rg_config.max_iterations = config.max_iterations;
    rg_config.verbose        = config.verbose;
    return solve(problem, rg_config);
}

model::Solution RandomizedGreedySolver::solve(const model::Problem&              problem,
                                               const RandomizedGreedySolverConfig& config)
{
    oplib::utils::Random rng(static_cast<uint32_t>(config.seed));

    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    model::Solution best;
    std::vector<bool>                     visited;
    std::vector<local_search::RouteContext> contexts;

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int iter = 0; config.max_iterations <= 0 || iter < config.max_iterations; ++iter) {
        // Wall-clock timeout
        auto elapsed = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;

        // Fresh solution for this restart
        model::Solution current;
        std::vector<local_search::RouteContext> cur_ctx;
        ls.init(current, visited, cur_ctx);

        ls.repair(current, visited, cur_ctx, ls_cfg);

        if (current.total_reward > best.total_reward) {
            best = current;
            if (config.verbose) {
                std::cout << "[RG] iter=" << iter
                          << " reward=" << best.total_reward << '\n';
            }
        }
    }

    return best;
}

} // namespace oplib::solver::constructive
