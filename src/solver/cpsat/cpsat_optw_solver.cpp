#include "solver/cpsat/cpsat_optw_solver.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
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

static int64_t optw_scale_time(double t, int64_t scale, int64_t cap) {
    if (t >= 1e15) return cap;
    int64_t v = static_cast<int64_t>(std::floor(t * static_cast<double>(scale)));
    return std::min(v, cap);
}

// ---------------------------------------------------------------------------
// Greedy warm-start: time-window-aware nearest-feasible greedy
// ---------------------------------------------------------------------------

std::vector<int> CPSATOPTWSolver::greedy_route(const model::Problem& problem,
                                                double tmax_d) const {
    const int n      = static_cast<int>(problem.get_num_nodes());
    const int source = static_cast<int>(problem.get_source_depot());
    const int sink   = static_cast<int>(problem.get_sink_depot());

    std::vector<bool> visited(n, false);
    visited[source] = true;
    visited[sink]   = true;

    std::vector<int> route;
    route.reserve(n);
    route.push_back(source);

    int    cur  = source;
    double time = problem.get_time_window(source).opening;

    for (;;) {
        int    best     = -1;
        double best_val = -1.0;

        for (int j = 0; j < n; ++j) {
            if (visited[j] || j == source || j == sink) continue;

            const double dist_cj  = problem.get_distance(cur, j);
            const double arr_j    = time + dist_cj;
            const double tw_close = problem.get_time_window(j).closing;
            const double tw_open  = problem.get_time_window(j).opening;
            const double svc_j    = problem.get_service_time(j);

            if (arr_j > tw_close + 1e-9) continue;  // misses window

            const double dep_j    = std::max(arr_j, tw_open) + svc_j;
            const double arr_sink = dep_j + problem.get_distance(j, sink);
            if (arr_sink > tmax_d + 1e-9) continue;  // can't return in time

            // Score: reward per unit of "extra time" spent going to j
            const double extra  = std::max(arr_j, tw_open) + svc_j - time;
            const double reward = problem.get_reward(j);
            const double val    = reward / (extra + 1e-9);

            if (val > best_val) {
                best_val = val;
                best     = j;
            }
        }

        if (best < 0) break;

        const double arr_best = time + problem.get_distance(cur, best);
        time = std::max(arr_best, problem.get_time_window(best).opening)
               + problem.get_service_time(best);

        visited[best] = true;
        route.push_back(best);
        cur = best;
    }

    route.push_back(sink);
    return route;
}

// ---------------------------------------------------------------------------
// Dispatch: base SolverConfig -> typed config
// ---------------------------------------------------------------------------

model::Solution CPSATOPTWSolver::solve(const model::Problem& problem,
                                       const SolverConfig& config) {
    CPSATOPTWSolverConfig cfg;
    cfg.seed           = config.seed;
    cfg.max_cpu_time   = config.max_cpu_time;
    cfg.max_iterations = config.max_iterations;
    cfg.verbose        = config.verbose;
    return solve(problem, cfg);
}

// ---------------------------------------------------------------------------
// Core CP-SAT implementation (single-vehicle OPTW)
// ---------------------------------------------------------------------------

model::Solution CPSATOPTWSolver::solve(const model::Problem& problem,
                                       const CPSATOPTWSolverConfig& config) {
    const int n      = static_cast<int>(problem.get_num_nodes());
    const int source = static_cast<int>(problem.get_source_depot());
    const int sink   = static_cast<int>(problem.get_sink_depot());

    // Fallback: empty route source -> sink
    model::Solution empty(1);
    empty.get_route(0)      = {static_cast<NodeId>(source), static_cast<NodeId>(sink)};
    empty.total_reward      = 0.0;
    empty.total_travel_time = problem.get_distance(source, sink);

    if (n <= 2) return empty;

    if (!problem.has_time_windows()) {
        if (config.verbose)
            std::cout << "[CPSAT_OPTW] Problem has no time windows; use CPSATSolver instead.\n";
        return empty;
    }

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
    // Pre-compute scaled arc costs (floor to keep integer cost <= real cost).
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
    // Arc feasibility: filter based on source→i→j→sink path lower bound.
    // ------------------------------------------------------------------
    auto arc_feasible = [&](int i, int j) -> bool {
        if (i == j)      return false;
        if (j == source) return false;
        if (i == sink)   return false;

        const double tw_i_open  = problem.get_time_window(i).opening;
        const double tw_i_close = problem.get_time_window(i).closing;
        const double tw_j_close = problem.get_time_window(j).closing;
        const double tw_j_open  = problem.get_time_window(j).opening;
        const double svc_i      = problem.get_service_time(i);

        const double arr_i = problem.get_time_window(source).opening
                             + problem.get_distance(source, i);
        if (arr_i > tw_i_close + 1e-9) return false;

        const double dep_i = std::max(arr_i, tw_i_open) + svc_i;
        const double arr_j = dep_i + problem.get_distance(i, j);
        if (arr_j > tw_j_close + 1e-9) return false;

        const double dep_j    = std::max(arr_j, tw_j_open) + problem.get_service_time(j);
        const double arr_sink = dep_j + problem.get_distance(j, sink);
        if (arr_sink > tmax_d + 1e-9) return false;

        return true;
    };

    // ------------------------------------------------------------------
    // Build CP-SAT model
    // ------------------------------------------------------------------
    CpModelBuilder cp_model;

    // Start-of-service time variables with time window domains.
    std::vector<IntVar> start(n);
    for (int i = 0; i < n; ++i) {
        int64_t lo = optw_scale_time(problem.get_time_window(i).opening, TIME_SCALE, TMAX);
        int64_t hi = (i == sink)
                         ? TMAX
                         : optw_scale_time(problem.get_time_window(i).closing, TIME_SCALE, TMAX);
        if (lo > hi) lo = hi;
        start[i] = cp_model.NewIntVar({lo, hi});
    }

    cp_model.FixVariable(
        start[source],
        optw_scale_time(problem.get_time_window(source).opening, TIME_SCALE, TMAX));

    // ------------------------------------------------------------------
    // Circuit constraint
    // ------------------------------------------------------------------
    auto circuit = cp_model.AddCircuitConstraint();

    std::vector<BoolVar> skip(n);
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        skip[i] = cp_model.NewBoolVar();
        circuit.AddArc(i, i, skip[i]);
    }

    circuit.AddArc(sink, source, cp_model.TrueVar());

    std::vector<std::vector<BoolVar>> arc_var(n, std::vector<BoolVar>(n));
    std::vector<std::vector<bool>>    has_arc(n, std::vector<bool>(n, false));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!arc_feasible(i, j)) continue;

            BoolVar lit   = cp_model.NewBoolVar();
            arc_var[i][j] = lit;
            has_arc[i][j] = true;
            circuit.AddArc(i, j, lit);

            const int64_t gap = scaled_svc(i) + scaled_dist(i, j);
            cp_model.AddGreaterOrEqual(LinearExpr(start[j]),
                                       LinearExpr(start[i]) + gap)
                    .OnlyEnforceIf(lit);
        }
    }

    // ------------------------------------------------------------------
    // Objective: maximise reward of visited customers.
    // ------------------------------------------------------------------
    LinearExpr objective;
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        const int64_t r_int = static_cast<int64_t>(std::round(problem.get_reward(i) * 1000.0));
        if (r_int <= 0) continue;
        objective += LinearExpr::Term(skip[i].Not(), r_int);
    }
    cp_model.Maximize(objective);

    // ------------------------------------------------------------------
    // Solution hint: greedy warm-start (or empty route as fallback).
    // A complete hint lets presolve confirm feasibility immediately,
    // giving CP-SAT a real lower bound to work from.
    // ------------------------------------------------------------------
    std::vector<int> hint_route;
    if (config.greedy_hint)
        hint_route = greedy_route(problem, tmax_d);

    // If greedy produced only source→sink (no customers), fall back to empty.
    if (hint_route.size() <= 2) {
        hint_route = {source, sink};
    }

    // Mark which nodes are visited in the hint route.
    std::vector<bool> hint_visited(n, false);
    for (int node : hint_route) hint_visited[node] = true;

    // Compute hint start times by simulating the greedy route.
    std::vector<int64_t> hint_start(n);
    {
        double t = problem.get_time_window(source).opening;
        hint_start[source] = optw_scale_time(t, TIME_SCALE, TMAX);
        for (int k = 1; k < static_cast<int>(hint_route.size()); ++k) {
            const int prev = hint_route[k - 1];
            const int cur  = hint_route[k];
            t += problem.get_distance(prev, cur);
            t = std::max(t, problem.get_time_window(cur).opening);
            hint_start[cur] = optw_scale_time(t, TIME_SCALE, TMAX);
            if (cur != sink) t += problem.get_service_time(cur);
        }
        // Unvisited customers: hint at their window opening.
        for (int i = 0; i < n; ++i) {
            if (!hint_visited[i])
                hint_start[i] = optw_scale_time(
                    problem.get_time_window(i).opening, TIME_SCALE, TMAX);
        }
    }

    // Set skip hints.
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        cp_model.AddHint(skip[i], hint_visited[i] ? 0LL : 1LL);
    }

    // Set arc hints: mark route arcs as 1, all others as 0 (single pass).
    std::vector<std::vector<bool>> hint_arc(n, std::vector<bool>(n, false));
    for (int k = 0; k + 1 < static_cast<int>(hint_route.size()); ++k)
        hint_arc[hint_route[k]][hint_route[k + 1]] = true;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (!has_arc[i][j]) continue;
            cp_model.AddHint(arc_var[i][j], hint_arc[i][j] ? 1LL : 0LL);
        }
    }

    // Set start time hints.
    for (int i = 0; i < n; ++i)
        cp_model.AddHint(start[i], hint_start[i]);

    // ------------------------------------------------------------------
    // Solver parameters
    // ------------------------------------------------------------------
    SatParameters params;
    params.set_max_time_in_seconds(config.max_cpu_time);
    params.set_log_search_progress(config.verbose);
    params.set_num_workers(std::max(1, config.num_workers));

    // ------------------------------------------------------------------
    // Solve
    // ------------------------------------------------------------------
    const CpSolverResponse response =
        SolveWithParameters(cp_model.Build(), params);

    const auto status = response.status();
    if (status != CpSolverStatus::OPTIMAL && status != CpSolverStatus::FEASIBLE) {
        if (config.verbose) {
            std::cout << "[CPSAT_OPTW] No solution found (status="
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
