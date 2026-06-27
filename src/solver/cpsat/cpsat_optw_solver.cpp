#include "solver/cpsat/cpsat_optw_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
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

// Departure time after serving route[k]:
//   max(prev_dep + dist, tw_open[route[k]]) + svc[route[k]]
// Sink nodes are treated as having svc=0.
static std::vector<double> compute_dep(const model::Problem& problem,
                                       const std::vector<int>& route) {
    const int sink = static_cast<int>(problem.get_sink_depot());
    std::vector<double> dep(route.size());
    double t = problem.get_time_window(route[0]).opening
               + problem.get_service_time(route[0]);
    dep[0] = t;
    for (int k = 1; k < (int)route.size(); ++k) {
        t  = dep[k-1] + problem.get_distance(route[k-1], route[k]);
        t  = std::max(t, problem.get_time_window(route[k]).opening);
        dep[k] = t;
        if (route[k] != sink) dep[k] += problem.get_service_time(route[k]);
    }
    return dep;
}

// ---------------------------------------------------------------------------
// Greedy best-insertion improvement.
//
// Starting from 'route', repeatedly find the highest-reward unvisited customer
// and the minimum-extra-travel-time feasible insertion position, then insert.
// O(n^3) per call, negligible for n<=100.
// ---------------------------------------------------------------------------
static std::vector<int> improve_by_insertion(const model::Problem& problem,
                                              std::vector<int> route,
                                              double tmax_d) {
    const int n    = static_cast<int>(problem.get_num_nodes());
    const int sink = static_cast<int>(problem.get_sink_depot());

    std::vector<bool> in_route(n, false);
    for (int v : route) in_route[v] = true;

    std::vector<double> dep = compute_dep(problem, route);

    for (;;) {
        double best_rew  = 0.0;
        double best_cost = std::numeric_limits<double>::infinity();
        int best_j = -1, best_pos = -1;

        for (int j = 0; j < n; ++j) {
            if (in_route[j]) continue;
            const double rew_j   = problem.get_reward(j);
            if (rew_j < best_rew - 1e-9) continue;  // worse reward, skip
            const double tw_j_op = problem.get_time_window(j).opening;
            const double tw_j_cl = problem.get_time_window(j).closing;
            const double svc_j   = problem.get_service_time(j);

            for (int pos = 1; pos < (int)route.size(); ++pos) {
                // Departure from route[pos-1] → arrival at j
                const double arr_j = dep[pos-1] + problem.get_distance(route[pos-1], j);
                if (arr_j > tw_j_cl + 1e-9) continue;
                const double dep_j = std::max(arr_j, tw_j_op) + svc_j;

                // Propagate through the suffix (route[pos], ..., sink)
                bool ok = true;
                double t = dep_j;
                for (int k = pos; k < (int)route.size(); ++k) {
                    const int prev = (k == pos) ? j : route[k - 1];
                    t += problem.get_distance(prev, route[k]);
                    t  = std::max(t, problem.get_time_window(route[k]).opening);
                    if (t > problem.get_time_window(route[k]).closing + 1e-9) {
                        ok = false; break;
                    }
                    if (route[k] != sink) t += problem.get_service_time(route[k]);
                }
                if (!ok || t > tmax_d + 1e-9) continue;

                // Cost = extra travel time added
                const double cost = problem.get_distance(route[pos-1], j)
                                  + problem.get_distance(j, route[pos])
                                  - problem.get_distance(route[pos-1], route[pos]);

                // Prefer: (1) higher reward, (2) lower insertion cost
                if (rew_j > best_rew + 1e-9 ||
                    (rew_j >= best_rew - 1e-9 && cost < best_cost - 1e-9)) {
                    best_rew  = rew_j;
                    best_j    = j;
                    best_pos  = pos;
                    best_cost = cost;
                }
            }
        }

        if (best_j < 0) break;

        route.insert(route.begin() + best_pos, best_j);
        in_route[best_j] = true;
        dep = compute_dep(problem, route);
    }

    return route;
}

// ---------------------------------------------------------------------------
// Greedy warm-start: four scoring strategies + best-insertion improvement.
// ---------------------------------------------------------------------------

std::vector<int> CPSATOPTWSolver::greedy_route(const model::Problem& problem,
                                                double tmax_d) const {
    const int n      = static_cast<int>(problem.get_num_nodes());
    const int source = static_cast<int>(problem.get_source_depot());
    const int sink   = static_cast<int>(problem.get_sink_depot());

    using ScoreFn = std::function<double(int j, int cur, double time,
                                         double arr_j, double dep_j)>;

    auto run = [&](ScoreFn score) -> std::vector<int> {
        std::vector<bool> visited(n, false);
        visited[source] = visited[sink] = true;
        std::vector<int> route{source};
        int cur = source;
        double time = problem.get_time_window(source).opening;

        for (;;) {
            int    best     = -1;
            double best_val = -std::numeric_limits<double>::infinity();
            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                const double arr_j   = time + problem.get_distance(cur, j);
                const double tw_j_cl = problem.get_time_window(j).closing;
                const double tw_j_op = problem.get_time_window(j).opening;
                const double svc_j   = problem.get_service_time(j);
                if (arr_j > tw_j_cl + 1e-9) continue;
                const double dep_j   = std::max(arr_j, tw_j_op) + svc_j;
                if (dep_j + problem.get_distance(j, sink) > tmax_d + 1e-9) continue;
                const double val = score(j, cur, time, arr_j, dep_j);
                if (val > best_val) { best_val = val; best = j; }
            }
            if (best < 0) break;
            const double arr = time + problem.get_distance(cur, best);
            time = std::max(arr, problem.get_time_window(best).opening)
                   + problem.get_service_time(best);
            visited[best] = true;
            route.push_back(best);
            cur = best;
        }
        route.push_back(sink);
        return route;
    };

    // Strategy 1: best reward / extra-time ratio
    auto s1 = run([&](int j, int, double time, double, double dep_j) {
        return problem.get_reward(j) / (dep_j - time + 1e-9);
    });
    // Strategy 2: earliest deadline first (avoids missing tight windows)
    auto s2 = run([&](int j, int, double, double, double) {
        return -problem.get_time_window(j).closing;
    });
    // Strategy 3: maximum raw reward
    auto s3 = run([&](int j, int, double, double, double) {
        return static_cast<double>(problem.get_reward(j));
    });
    // Strategy 4: nearest feasible customer
    auto s4 = run([&](int j, int cur, double, double, double) {
        return -problem.get_distance(cur, j);
    });

    // Pick the greedy strategy with the highest reward as the starting point.
    auto reward_of = [&](const std::vector<int>& r) {
        double tot = 0.0;
        for (int k = 1; k + 1 < (int)r.size(); ++k)
            if (r[k] != sink) tot += problem.get_reward(r[k]);
        return tot;
    };
    std::vector<std::vector<int>> cands = {s1, s2, s3, s4};
    auto& best_greedy = *std::max_element(cands.begin(), cands.end(),
        [&](const auto& a, const auto& b) {
            return reward_of(a) < reward_of(b);
        });

    return improve_by_insertion(problem, best_greedy, tmax_d);
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

    model::Solution empty(1);
    empty.get_route(0)      = {static_cast<NodeId>(source), static_cast<NodeId>(sink)};
    empty.total_reward      = 0.0;
    empty.total_travel_time = problem.get_distance(source, sink);

    if (n <= 2) return empty;

    if (!problem.has_time_windows()) {
        if (config.verbose)
            std::cout << "[CPSAT_OPTW] Problem has no time windows; use CPSATSolver.\n";
        return empty;
    }

    // ------------------------------------------------------------------
    // Time scaling
    // ------------------------------------------------------------------
    const int64_t TIME_SCALE = problem.is_scaled() ? 1LL : 1000LL;

    double tmax_d = std::min(problem.get_budget(),
                              problem.get_time_window(sink).closing);
    if (tmax_d >= 1e15) tmax_d = problem.get_budget();
    const int64_t TMAX =
        static_cast<int64_t>(std::ceil(tmax_d * static_cast<double>(TIME_SCALE)));

    if (TMAX <= 0) return empty;

    auto scaled_dist = [&](int i, int j) -> int64_t {
        return static_cast<int64_t>(
            std::floor(problem.get_distance(i, j) * static_cast<double>(TIME_SCALE)));
    };
    auto scaled_svc = [&](int i) -> int64_t {
        return static_cast<int64_t>(
            std::floor(problem.get_service_time(i) * static_cast<double>(TIME_SCALE)));
    };

    // ------------------------------------------------------------------
    // Arc feasibility
    // ------------------------------------------------------------------
    auto arc_feasible = [&](int i, int j) -> bool {
        if (i == j || j == source || i == sink) return false;
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
    // Objective
    // ------------------------------------------------------------------
    LinearExpr objective;
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        const int64_t r_int = static_cast<int64_t>(
            std::round(problem.get_reward(i) * 1000.0));
        if (r_int <= 0) continue;
        objective += LinearExpr::Term(skip[i].Not(), r_int);
    }
    cp_model.Maximize(objective);

    // ------------------------------------------------------------------
    // Greedy warm-start hint: seed CP-SAT with the best greedy solution.
    // ------------------------------------------------------------------
    std::vector<int> hint_route;
    if (config.greedy_hint)
        hint_route = greedy_route(problem, tmax_d);
    if (hint_route.size() <= 2)
        hint_route = {source, sink};

    if (config.verbose) {
        double hint_rew = 0.0;
        for (int k = 1; k + 1 < (int)hint_route.size(); ++k)
            hint_rew += problem.get_reward(hint_route[k]);
        std::cout << "[CPSAT_OPTW] Hint reward: " << hint_rew
                  << " (" << hint_route.size() - 2 << " customers)\n";
    }

    if (config.greedy_hint) {
        std::vector<bool>    hv(n, false);
        std::vector<int64_t> hs(n, 0);
        for (int v : hint_route) hv[v] = true;
        double t = problem.get_time_window(source).opening;
        hs[source] = optw_scale_time(t, TIME_SCALE, TMAX);
        for (int k = 1; k < (int)hint_route.size(); ++k) {
            const int prev = hint_route[k - 1], cur = hint_route[k];
            t += problem.get_distance(prev, cur);
            t  = std::max(t, problem.get_time_window(cur).opening);
            hs[cur] = optw_scale_time(t, TIME_SCALE, TMAX);
            if (cur != sink) t += problem.get_service_time(cur);
        }
        for (int i = 0; i < n; ++i)
            if (!hv[i])
                hs[i] = optw_scale_time(
                    problem.get_time_window(i).opening, TIME_SCALE, TMAX);

        for (int i = 0; i < n; ++i) {
            if (i == source || i == sink) continue;
            cp_model.AddHint(skip[i], hv[i] ? 0LL : 1LL);
        }
        std::vector<std::vector<bool>> ha(n, std::vector<bool>(n, false));
        for (int k = 0; k + 1 < (int)hint_route.size(); ++k)
            ha[hint_route[k]][hint_route[k + 1]] = true;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (has_arc[i][j])
                    cp_model.AddHint(arc_var[i][j], ha[i][j] ? 1LL : 0LL);
        for (int i = 0; i < n; ++i)
            cp_model.AddHint(start[i], hs[i]);
    }

    // ------------------------------------------------------------------
    // Solve: single call with full time budget and all workers.
    // diversify_lns_params gives each parallel worker a distinct LNS
    // configuration (neighborhood size, operator selection, difficulty)
    // so the workers explore different basins simultaneously.
    // ------------------------------------------------------------------
    SatParameters params;
    params.set_log_search_progress(config.verbose);
    params.set_num_workers(std::max(1, config.num_workers));
    params.set_max_time_in_seconds(config.max_cpu_time);
    params.set_diversify_lns_params(true);

    const CpSolverResponse response =
        SolveWithParameters(cp_model.Build(), params);

    const auto status = response.status();
    if (status != CpSolverStatus::OPTIMAL && status != CpSolverStatus::FEASIBLE) {
        if (config.verbose)
            std::cout << "[CPSAT_OPTW] No solution found (status="
                      << static_cast<int>(status) << ")\n";
        return empty;
    }

    // ------------------------------------------------------------------
    // Extract solution
    // ------------------------------------------------------------------
    std::vector<int> successor(n, -1);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (has_arc[i][j] && SolutionBooleanValue(response, arc_var[i][j]))
                { successor[i] = j; break; }

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
