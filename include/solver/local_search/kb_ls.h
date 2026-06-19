#pragma once

#include <utility>
#include <vector>

#include "core/random.h"
#include "core/types.h"
#include "knowledge_base/conflict_store.h"
#include "model/problem.h"
#include "model/solution.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::local_search {

// (customer, vehicle) pair blocked or rejected during repair
using InfeasiblePair = std::pair<NodeId, int>;

/**
 * Scope-guessing heuristic: which route clients to include when asking the
 * xplainer to explain why client c could not go in vehicle v.
 */
enum class ScopeHeuristic {
    NearestTW,      // clients whose TW midpoint is closest to c's TW midpoint
    ShortestTW,     // clients with the shortest TW duration
    NearestClosing, // clients whose closing time is closest to c's closing time
    Random          // random sample
};

/**
 * KB-aware construction sweep: mirrors BaseLSUtils::repair() but with a
 * ConflictStore pre-filter.  For each (c, v) pair the KB says is infeasible,
 * the pair is recorded in infeasible_out (for later xplain); pairs that pass
 * the KB check but fail the TW check are also recorded.  Pairs that pass both
 * are inserted normally and the KB is updated with kb.assign(c, v).
 *
 * backbone: optional per-client infeasibility mask (true = skip the client).
 * Pass an empty vector to disable backbone filtering.
 */
void kb_repair(BaseLSUtils&                        ls,
               oplib::utils::Random&               rng,
               const model::Problem&               problem,
               model::Solution&                    solution,
               std::vector<bool>&                  visited,
               std::vector<RouteContext>&           contexts,
               const LSConfig&                     config,
               knowledge_base::ConflictStore&      kb,
               std::vector<InfeasiblePair>&         infeasible_out,
               const std::vector<bool>&             backbone = {});

/**
 * Pre-compute backbone: for each client, check whether it can ever be visited
 * in a single-client tour [src → c → sink] within the time-window and budget
 * constraints.  Returns a per-NodeId mask where true = definitively infeasible.
 */
std::vector<bool> compute_backbone(const model::Problem& problem);

/**
 * Synchronise the KB to an existing solution: reset all assignments, then
 * call kb.assign(c, v) for every customer c in vehicle v's route.
 * Used after a restart to re-sync the KB to the restored best solution.
 */
void kb_sync(knowledge_base::ConflictStore& kb,
             const model::Problem&          problem,
             const model::Solution&         solution);

/**
 * Build a conflict scope: include c plus up to (max_scope_size - 1) clients
 * from vehicle v's route according to the chosen heuristic.
 */
std::vector<NodeId> guess_conflict_scope(
    NodeId                     c,
    int                        vehicle,
    const model::Solution&     solution,
    const model::Problem&      problem,
    int                        max_scope_size,
    ScopeHeuristic             heuristic,
    oplib::utils::Random&      rng);

} // namespace oplib::solver::local_search
