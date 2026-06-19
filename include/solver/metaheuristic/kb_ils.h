#pragma once

#include "solver/local_search/kb_ls.h"
#include "solver/local_search/tw_xplainer.h"
#include "solver/metaheuristic/base_ils.h"
#include "knowledge_base/conflict_store.h"

namespace oplib::solver::metaheuristic {

struct KBILSSolverConfig : public BaseILSSolverConfig {
    int    learn_iterations  = 100;
    int    conflict_max_size = 5;
    float  xplain_ms         = 500.f;
    local_search::ScopeHeuristic scope_heuristic =
        local_search::ScopeHeuristic::NearestTW;

    // Clause forgetting: every forget_interval iterations remove clauses whose
    // activity counter is below forget_min_activity. 0 = disabled.
    int    forget_interval     = 0;
    int    forget_min_activity = 1;

    mutable int kb_size_out      = 0; ///< set by do_solve after the run
    mutable int kb_removed_out   = 0; ///< total clauses removed by compaction
};

/**
 * KB-guided ILS: port of KBLNS17::solve_v1() from kb_ls_cpp.
 * The KB (ConflictStore, 2-watched literals) accumulates TW conflict clauses
 * during the first `learn_iterations` iterations.  A Held-Karp xplainer
 * extracts minimal infeasible client subsets and feeds them to the KB.
 * The KB pre-filters candidate insertions before the expensive TW check.
 *
 * Structure mirrors ILS09Solver (shake + repair + strict-improvement
 * acceptance + restart with kb_sync).
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
        knowledge_base::ConflictStore&                   kb,
        local_search::TWXplainer&                        xplainer,
        const std::vector<local_search::InfeasiblePair>& pairs,
        const model::Solution&                           solution,
        const model::Problem&                            problem,
        int                                              max_scope_size,
        local_search::ScopeHeuristic                     heuristic,
        float                                            budget_ms,
        oplib::utils::Random&                            rng);
};

} // namespace oplib::solver::metaheuristic
