#include <chrono>
#include <iostream>
#include <vector>

#include "solver/metaheuristic/kb_ils.h"

namespace oplib::solver::metaheuristic {

using Clock = std::chrono::high_resolution_clock;

model::Solution KBILSSolver::solve(const model::Problem&    problem,
                                    const KBILSSolverConfig& config)
{
    return do_solve(problem, config);
}

model::Solution KBILSSolver::do_solve(const model::Problem&         problem,
                                       const BaseILSSolverConfig&    base_cfg)
{
    const auto& config = static_cast<const KBILSSolverConfig&>(base_cfg);

    oplib::utils::Random rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils ls(problem, rng);

    local_search::LSConfig ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    const int nv        = problem.get_num_vehicles();
    const int nb_clients = static_cast<int>(problem.get_num_nodes()) - 2; // excl. depots

    knowledge_base::ConflictStore kb(nb_clients > 0 ? nb_clients : 1, nv);
    local_search::TWXplainer      xplainer(problem);

    // ---- Initial solution ----
    model::Solution                         best;
    std::vector<bool>                       best_vis;
    std::vector<local_search::RouteContext> best_ctx;
    ls.init(best, best_vis, best_ctx);

    std::vector<local_search::InfeasiblePair> infeasible;
    local_search::kb_repair(ls, rng, problem, best, best_vis, best_ctx,
                            ls_cfg, kb, infeasible);
    for (int v = 0; v < nv; ++v)
        ls.minimize_makespan(best, best_ctx, v);

    learn_tw_conflicts(kb, xplainer, infeasible, best, problem,
                       config.conflict_max_size, config.scope_heuristic,
                       config.xplain_ms, rng);

    if (config.verbose)
        std::cout << "[KBILS] init reward=" << best.total_reward
                  << " kb=" << kb.size() << '\n';

    model::Solution                         current  = best;
    std::vector<bool>                       cur_vis  = best_vis;
    std::vector<local_search::RouteContext> cur_ctx  = best_ctx;

    int shake_length = 1;
    int no_impr      = 0;

    auto t_start = Clock::now();

    for (int iter = 0;
         config.max_iterations <= 0 || iter < config.max_iterations;
         ++iter)
    {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;

        // Perturb: shake each vehicle; track removed customers for KB unassign
        std::vector<bool> vis_before = cur_vis;
        for (int v = 0; v < nv; ++v) {
            const auto& route = current.get_route(v);
            const int nc = static_cast<int>(route.size()) - 2;
            if (nc <= 0) continue;
            int pos = rng.next_int(1, nc);
            ls.shake(current, cur_vis, cur_ctx, v, pos, shake_length);
        }

        // Unassign customers removed by shake
        const int nn   = static_cast<int>(problem.get_num_nodes());
        const NodeId src  = problem.get_source_depot();
        const NodeId sink = problem.get_sink_depot();
        for (int i = 0; i < nn; ++i) {
            if (i == src || i == sink) continue;
            if (vis_before[i] && !cur_vis[i])
                kb.unassign(i);
        }

        // Reconstruct with KB
        infeasible.clear();
        local_search::kb_repair(ls, rng, problem, current, cur_vis, cur_ctx,
                                ls_cfg, kb, infeasible);
        for (int v = 0; v < nv; ++v)
            ls.minimize_makespan(current, cur_ctx, v);

        // Conflict learning (first learn_iterations only)
        if (iter < config.learn_iterations) {
            learn_tw_conflicts(kb, xplainer, infeasible, current, problem,
                               config.conflict_max_size, config.scope_heuristic,
                               config.xplain_ms, rng);
        }

        // Acceptance: strict improvement
        if (current.total_reward > best.total_reward) {
            best         = current;
            best_vis     = cur_vis;
            best_ctx     = cur_ctx;
            shake_length = 1;
            no_impr      = 0;
            if (config.verbose)
                std::cout << "[KBILS] iter=" << iter
                          << " reward=" << best.total_reward
                          << " kb=" << kb.size() << '\n';
        } else {
            ++no_impr;
            if (no_impr >= config.restart_threshold) {
                current  = best;
                cur_vis  = best_vis;
                cur_ctx  = best_ctx;
                ++shake_length;
                no_impr = 0;
                local_search::kb_sync(kb, problem, best);
            }
        }
    }

    return best;
}

// ---------------------------------------------------------------------------
// learn_tw_conflicts
// ---------------------------------------------------------------------------

void KBILSSolver::learn_tw_conflicts(
    knowledge_base::ConflictStore&                   kb,
    local_search::TWXplainer&                        xplainer,
    const std::vector<local_search::InfeasiblePair>& pairs,
    const model::Solution&                           solution,
    const model::Problem&                            problem,
    int                                              max_scope_size,
    local_search::ScopeHeuristic                     heuristic,
    float                                            budget_ms,
    oplib::utils::Random&                            rng)
{
    if (budget_ms <= 0.f || pairs.empty()) return;

    auto t0 = Clock::now();

    for (const auto& [c, v] : pairs) {
        double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        if (ms >= budget_ms) break;

        std::vector<NodeId> scope =
            local_search::guess_conflict_scope(c, v, solution, problem,
                                               max_scope_size, heuristic, rng);
        if (scope.size() < 2) continue;

        std::vector<local_search::TwConflict> conflicts;
        if (xplainer.extract_conflict(scope, conflicts, /*multi=*/true)) {
            for (auto& cft : conflicts) {
                std::vector<int> int_scope(cft.begin(), cft.end());
                kb.add_conflict(int_scope);
            }
        }
    }
}

} // namespace oplib::solver::metaheuristic
