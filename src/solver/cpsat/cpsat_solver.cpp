#include "solver/cpsat/cpsat_solver.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/sat_parameters.pb.h"

namespace oplib::solver::cpsat {

using operations_research::sat::BoolVar;
using operations_research::sat::CpModelBuilder;
using operations_research::sat::CpSolverResponse;
using operations_research::sat::CpSolverStatus;
using operations_research::sat::IntVar;
using operations_research::sat::LinearExpr;
using operations_research::sat::SatParameters;
using operations_research::sat::SolutionBooleanValue;
using operations_research::sat::SolveWithParameters;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int64_t scale_time(double t, int64_t scale, int64_t cap) {
    if (t >= 1e15) return cap;
    int64_t v = static_cast<int64_t>(std::floor(t * static_cast<double>(scale)));
    return std::min(v, cap);
}

// ---------------------------------------------------------------------------
// Dispatch: base SolverConfig -> typed config
// ---------------------------------------------------------------------------

model::Solution CPSATSolver::solve(const model::Problem& problem,
                                   const SolverConfig& config) {
    CPSATSolverConfig cfg;
    cfg.seed           = config.seed;
    cfg.max_cpu_time   = config.max_cpu_time;
    cfg.max_iterations = config.max_iterations;
    cfg.verbose        = config.verbose;
    return solve(problem, cfg);
}

// ---------------------------------------------------------------------------
// Core CP-SAT implementation (single-vehicle OPTW/OP)
// ---------------------------------------------------------------------------

model::Solution CPSATSolver::solve(const model::Problem& problem,
                                   const CPSATSolverConfig& config) {
    const int n      = static_cast<int>(problem.get_num_nodes());
    const int source = static_cast<int>(problem.get_source_depot());
    const int sink   = static_cast<int>(problem.get_sink_depot());

    // Fallback: empty route source -> sink
    model::Solution empty(1);
    empty.get_route(0)    = {static_cast<NodeId>(source), static_cast<NodeId>(sink)};
    empty.total_reward      = 0.0;
    empty.total_travel_time = problem.get_distance(source, sink);

    if (n <= 2) return empty;

    // ------------------------------------------------------------------
    // Time scaling: CP-SAT requires integer variables.
    // Use the problem's own scale when it already stores integer times;
    // otherwise multiply by 1000 (millisecond precision).
    // ------------------------------------------------------------------
    const int64_t TIME_SCALE = problem.is_scaled() ? 1LL : 1000LL;

    // Effective time budget.
    double tmax_d = std::min(problem.get_budget(),
                              problem.get_time_window(sink).closing);
    if (tmax_d >= 1e15) tmax_d = problem.get_budget();
    const int64_t TMAX =
        static_cast<int64_t>(std::ceil(tmax_d * static_cast<double>(TIME_SCALE)));

    if (TMAX <= 0) return empty;

    // ------------------------------------------------------------------
    // Pre-compute scaled arc costs.
    // Use floor so scaled integer costs never exceed the real cost:
    // any route feasible in the continuous domain is also integer-feasible.
    // ------------------------------------------------------------------
    auto scaled_dist = [&](int i, int j) -> int64_t {
        return static_cast<int64_t>(
            std::floor(problem.get_distance(i, j) * static_cast<double>(TIME_SCALE)));
    };
    auto scaled_svc = [&](int i) -> int64_t {
        return static_cast<int64_t>(
            std::floor(problem.get_service_time(i) * static_cast<double>(TIME_SCALE)));
    };

    // ------------------------------------------------------------------
    // Arc feasibility: only create variables for arcs that can possibly
    // appear in a valid route. Filters based on the minimum path time
    // source→i→j→sink (a valid lower bound by the triangle inequality).
    // ------------------------------------------------------------------
    auto arc_feasible = [&](int i, int j) -> bool {
        if (i == j)      return false;
        if (j == source) return false;  // mid-route return to source forbidden
        if (i == sink)   return false;  // sink has no real outgoing arcs

        const double tw_i_open  = problem.get_time_window(i).opening;
        const double tw_i_close = problem.get_time_window(i).closing;
        const double tw_j_close = problem.get_time_window(j).closing;
        const double tw_j_open  = problem.get_time_window(j).opening;
        const double svc_i      = problem.get_service_time(i);

        // Earliest possible arrival at i from source.
        const double arr_i = problem.get_time_window(source).opening
                             + problem.get_distance(source, i);
        if (arr_i > tw_i_close + 1e-9) return false;

        // Earliest departure from i.
        const double dep_i = std::max(arr_i, tw_i_open) + svc_i;

        // Earliest arrival at j.
        const double arr_j = dep_i + problem.get_distance(i, j);
        if (arr_j > tw_j_close + 1e-9) return false;

        // Can we still return to sink in time?
        // Use 1e-9 tolerance to match SolutionChecker and avoid spurious
        // pruning from floating-point rounding in Euclidean distance sums.
        const double dep_j    = std::max(arr_j, tw_j_open) + problem.get_service_time(j);
        const double arr_sink = dep_j + problem.get_distance(j, sink);
        if (arr_sink > tmax_d + 1e-9) return false;

        return true;
    };

    // ------------------------------------------------------------------
    // Build CP-SAT model
    // ------------------------------------------------------------------
    CpModelBuilder cp_model;

    // Start-of-service time variables.
    std::vector<IntVar> start(n);
    for (int i = 0; i < n; ++i) {
        int64_t lo = scale_time(problem.get_time_window(i).opening, TIME_SCALE, TMAX);
        int64_t hi = (i == sink)
                         ? TMAX
                         : scale_time(problem.get_time_window(i).closing, TIME_SCALE, TMAX);
        if (lo > hi) lo = hi;
        start[i] = cp_model.NewIntVar({lo, hi});
    }

    // Fix source to its opening time (depart at time 0 in the standard setup).
    cp_model.FixVariable(
        start[source],
        scale_time(problem.get_time_window(source).opening, TIME_SCALE, TMAX));

    // ------------------------------------------------------------------
    // Circuit constraint
    // ------------------------------------------------------------------
    auto circuit = cp_model.AddCircuitConstraint();

    // Self-loops for customer nodes (allow skipping them).
    std::vector<BoolVar> skip(n);
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        skip[i] = cp_model.NewBoolVar();
        circuit.AddArc(i, i, skip[i]);
    }

    // Virtual closing arc: sink -> source (always selected, closes the path).
    circuit.AddArc(sink, source, cp_model.TrueVar());

    // Real arcs with timing propagation.
    std::vector<std::vector<BoolVar>> arc_var(n, std::vector<BoolVar>(n));
    std::vector<std::vector<bool>>    has_arc(n, std::vector<bool>(n, false));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!arc_feasible(i, j)) continue;

            BoolVar lit      = cp_model.NewBoolVar();
            arc_var[i][j]    = lit;
            has_arc[i][j]    = true;
            circuit.AddArc(i, j, lit);

            // start[j] >= start[i] + svc[i] + dist(i,j)   when lit = 1
            const int64_t gap = scaled_svc(i) + scaled_dist(i, j);
            cp_model.AddGreaterOrEqual(LinearExpr(start[j]),
                                       LinearExpr(start[i]) + gap)
                    .OnlyEnforceIf(lit);
        }
    }

    // ------------------------------------------------------------------
    // Objective: maximise reward of visited customers.
    // Rewards scaled by 1000 to handle fractional prizes while staying int64.
    // ------------------------------------------------------------------
    LinearExpr objective;
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        const double r     = problem.get_reward(i);
        const int64_t r_int = static_cast<int64_t>(std::round(r * 1000.0));
        if (r_int <= 0) continue;
        objective += LinearExpr::Term(skip[i].Not(), r_int);
    }
    cp_model.Maximize(objective);

    // ------------------------------------------------------------------
    // Solution hint: seed CP-SAT with the trivially-feasible empty route
    // (source → sink directly, all customers skipped). This ensures the
    // solver finds a feasible solution immediately rather than spending all
    // time in the LP relaxation on instances with tight time windows.
    // ------------------------------------------------------------------
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        cp_model.AddHint(skip[i], 1LL);  // skip all customers initially
    }
    // Hint real arc literals: only source→sink is active.
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!has_arc[i][j]) continue;
            cp_model.AddHint(arc_var[i][j], (i == source && j == sink) ? 1LL : 0LL);
        }
    }
    // Hint start times: source at opening, sink at source+dist(source,sink),
    // all customers at their opening time (they're skipped so value doesn't matter).
    {
        const int64_t src_open =
            scale_time(problem.get_time_window(source).opening, TIME_SCALE, TMAX);
        cp_model.AddHint(start[source], src_open);
        const int64_t sink_hint =
            src_open + scaled_dist(source, sink) + scaled_svc(source);
        cp_model.AddHint(start[sink],
                         std::min(sink_hint, TMAX));
        for (int i = 0; i < n; ++i) {
            if (i == source || i == sink) continue;
            cp_model.AddHint(start[i],
                             scale_time(problem.get_time_window(i).opening, TIME_SCALE, TMAX));
        }
    }

    // ------------------------------------------------------------------
    // Solver parameters
    // ------------------------------------------------------------------
    SatParameters params;
    params.set_max_time_in_seconds(config.max_cpu_time);
    params.set_log_search_progress(config.verbose);
    params.set_num_workers(1);  // single-threaded for determinism

    // ------------------------------------------------------------------
    // Solve
    // ------------------------------------------------------------------
    const CpSolverResponse response =
        SolveWithParameters(cp_model.Build(), params);

    const auto status = response.status();
    if (status != CpSolverStatus::OPTIMAL && status != CpSolverStatus::FEASIBLE) {
        if (config.verbose) {
            std::cout << "[CPSAT] No solution found (status="
                      << static_cast<int>(status) << ")\n";
        }
        return empty;
    }

    // ------------------------------------------------------------------
    // Extract solution: build successor map, follow path source -> sink.
    // ------------------------------------------------------------------
    std::vector<int> successor(n, -1);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!has_arc[i][j]) continue;
            if (SolutionBooleanValue(response, arc_var[i][j])) {
                successor[i] = j;
                break;
            }
        }
    }

    std::vector<NodeId> route;
    route.push_back(static_cast<NodeId>(source));
    int cur = source;
    for (int step = 0; step < n; ++step) {
        const int nxt = successor[cur];
        if (nxt < 0 || nxt == source) break;
        route.push_back(static_cast<NodeId>(nxt));
        if (nxt == sink) break;
        cur = nxt;
    }
    if (route.empty() || route.back() != static_cast<NodeId>(sink))
        route.push_back(static_cast<NodeId>(sink));

    // Compute metrics from the extracted route.
    Reward total_reward = 0.0;
    Time   total_time   = 0.0;
    for (size_t k = 1; k < route.size(); ++k) {
        const NodeId prev = route[k - 1];
        const NodeId curr = route[k];
        total_time += problem.get_distance(prev, curr);
        if (curr != static_cast<NodeId>(sink))
            total_reward += problem.get_reward(curr);
    }

    model::Solution sol(1);
    sol.get_route(0)      = std::move(route);
    sol.total_reward      = total_reward;
    sol.total_travel_time = total_time;
    return sol;
}

}  // namespace oplib::solver::cpsat
