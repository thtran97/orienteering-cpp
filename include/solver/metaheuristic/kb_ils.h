#pragma once

#include "solver/local_search/kb_ls.h"
#include "solver/local_search/tw_xplainer.h"
#include "solver/metaheuristic/base_ils.h"
#include "knowledge_base/conflict_store.h"

namespace oplib::solver::metaheuristic {

/**
 * @brief Configuration for the KB-guided ILS solver.
 *
 * Inherits alpha, rcl_size, and restart_threshold from BaseILSSolverConfig.
 */
struct KBILSSolverConfig : public BaseILSSolverConfig {
    int    learn_iterations  = 100;
    int    conflict_max_size = 5;
    float  xplain_ms         = 500.f;
    local_search::ScopeHeuristic scope_heuristic =
        local_search::ScopeHeuristic::NearestTW;

    mutable int kb_size_out = 0; ///< set by do_solve after the run
};

/**
 * @brief Knowledge-Base Iterated Local Search (KBILS).
 *
 * Port of KBLNS17::solve_v1() from kb_ls_cpp.  The KB (ConflictStore with
 * 2-watched literals) accumulates time-window conflict clauses during the
 * first `learn_iterations` iterations.  A Held-Karp xplainer extracts
 * minimal infeasible client subsets and feeds them to the KB.  The KB
 * pre-filters candidate insertions: if the KB knows (c, v) is infeasible,
 * the expensive TW check is skipped.
 *
 * Structure matches ILS09Solver (shake + repair + strict-improvement
 * acceptance + restart), adding the KB layer on top.
 */
class KBILSSolver : public BaseILSSolver {
public:
    std::string get_name() const override { return "KBILS"; }

    using BaseILSSolver::solve;
    model::Solution solve(const model::Problem& problem,
                          const KBILSSolverConfig& config);

protected:
    model::Solution do_solve(const model::Problem& problem,
                             const BaseILSSolverConfig& config) override;

private:
    void learn_tw_conflicts(
        knowledge_base::ConflictStore&              kb,
        local_search::TWXplainer&                   xplainer,
        const std::vector<local_search::InfeasiblePair>& pairs,
        const model::Solution&                      solution,
        const model::Problem&                       problem,
        int                                         max_scope_size,
        local_search::ScopeHeuristic                heuristic,
        float                                       budget_ms,
        oplib::utils::Random&                       rng);
};

} // namespace oplib::solver::metaheuristic
