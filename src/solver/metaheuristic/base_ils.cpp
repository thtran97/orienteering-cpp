#include "solver/metaheuristic/base_ils.h"

namespace oplib::solver::metaheuristic {

model::Solution BaseILSSolver::solve(const model::Problem& problem,
                                     const SolverConfig& config)
{
    BaseILSSolverConfig cfg;
    cfg.seed           = config.seed;
    cfg.max_cpu_time   = config.max_cpu_time;
    cfg.max_iterations = config.max_iterations;
    cfg.verbose        = config.verbose;
    return do_solve(problem, cfg);
}

void BaseILSSolver::construct(local_search::BaseLSUtils& ls,
                              const local_search::LSConfig& ls_cfg,
                              int num_vehicles,
                              model::Solution& sol,
                              std::vector<bool>& vis,
                              std::vector<local_search::RouteContext>& ctx)
{
    ls.repair(sol, vis, ctx, ls_cfg);
    for (int v = 0; v < num_vehicles; ++v)
        ls.minimize_makespan(sol, ctx, v);
}

} // namespace oplib::solver::metaheuristic
