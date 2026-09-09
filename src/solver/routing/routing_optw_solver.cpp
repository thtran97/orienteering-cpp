#include "solver/routing/routing_optw_solver.h"

#include <cmath>
#include <vector>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace oplib::solver::routing {

using operations_research::DefaultRoutingSearchParameters;
using operations_research::FirstSolutionStrategy;
using operations_research::LocalSearchMetaheuristic_Value;
using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingSearchParameters;
using RoutingNodeIndex = RoutingIndexManager::NodeIndex;

model::Solution RoutingOPTWSolver::solve(const model::Problem& problem,
                                         const SolverConfig& config) {
    RoutingOPTWSolverConfig cfg;
    cfg.seed         = config.seed;
    cfg.max_cpu_time = config.max_cpu_time;
    cfg.verbose      = config.verbose;
    return solve(problem, cfg);
}

model::Solution RoutingOPTWSolver::solve(const model::Problem& problem,
                                         const RoutingOPTWSolverConfig& config) {
    const int n      = static_cast<int>(problem.get_num_nodes());
    const int source = static_cast<int>(problem.get_source_depot());
    const int sink   = static_cast<int>(problem.get_sink_depot());

    model::Solution empty(1);
    empty.get_route(0)      = {static_cast<NodeId>(source), static_cast<NodeId>(sink)};
    empty.total_reward      = 0.0;
    empty.total_travel_time = problem.get_distance(source, sink);

    if (n <= 2) return empty;
    if (!problem.has_time_windows()) return empty;

    const int64_t SCALE = problem.is_scaled() ? 1 : 1000;

    double tmax_d = std::min(problem.get_budget(), problem.get_time_window(sink).closing);
    if (tmax_d >= 1e15) tmax_d = problem.get_budget();
    const int64_t TMAX = static_cast<int64_t>(std::ceil(tmax_d * static_cast<double>(SCALE)));
    if (TMAX <= 0) return empty;

    RoutingIndexManager manager(n, 1,
        std::vector<RoutingNodeIndex>{RoutingNodeIndex(source)},
        std::vector<RoutingNodeIndex>{RoutingNodeIndex(sink)});
    RoutingModel routing(manager);

    // "Time" dimension: transit(i,j) = travel distance + service time at i.
    // This is the standard OR-Tools pattern for folding service time into
    // a cumulative time dimension with native time-window propagation.
    const int transit_cb = routing.RegisterTransitCallback(
        [&](int64_t from_index, int64_t to_index) -> int64_t {
            const int from = manager.IndexToNode(from_index).value();
            const int to   = manager.IndexToNode(to_index).value();
            const int64_t dist = static_cast<int64_t>(
                std::floor(problem.get_distance(from, to) * static_cast<double>(SCALE)));
            const int64_t svc = static_cast<int64_t>(
                std::floor(problem.get_service_time(from) * static_cast<double>(SCALE)));
            return dist + svc;
        });
    routing.AddDimension(transit_cb, /*slack_max=*/TMAX, /*capacity=*/TMAX,
                          /*fix_start_cumul_to_zero=*/false, "Time");
    auto* time_dim = routing.GetMutableDimension("Time");

    for (int i = 0; i < n; ++i) {
        int64_t idx;
        if (i == source)      idx = manager.GetStartIndex(0);
        else if (i == sink)   idx = manager.GetEndIndex(0);
        else                  idx = manager.NodeToIndex(RoutingNodeIndex(i));
        const auto tw = problem.get_time_window(i);
        int64_t lo = static_cast<int64_t>(std::floor(tw.opening * static_cast<double>(SCALE)));
        int64_t hi = (tw.closing >= 1e15) ? TMAX
                     : static_cast<int64_t>(std::ceil(tw.closing * static_cast<double>(SCALE)));
        hi = std::min(hi, TMAX);
        if (lo > hi) lo = hi;
        time_dim->CumulVar(idx)->SetRange(lo, hi);
    }

    // Arc cost = real travel distance. Kept small relative to disjunction
    // penalties (reward*1000) so reward-maximization dominates, but nonzero
    // so TabuSearch/GuidedLocalSearch's cost-based escape mechanisms have a
    // real gradient (an all-zero arc cost was empirically much weaker:
    // reward 228-251 vs 304-308 with real distances).
    routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitCallback(
        [&](int64_t from_index, int64_t to_index) -> int64_t {
            const int from = manager.IndexToNode(from_index).value();
            const int to   = manager.IndexToNode(to_index).value();
            return static_cast<int64_t>(
                std::floor(problem.get_distance(from, to) * static_cast<double>(SCALE)));
        }));

    // Optional visits: each customer may be dropped at cost = its reward
    // (the standard prize-collecting / orienteering pattern in RoutingModel).
    std::vector<int64_t> customer_index(n, -1);
    for (int i = 0; i < n; ++i) {
        if (i == source || i == sink) continue;
        const int64_t r = static_cast<int64_t>(std::round(problem.get_reward(i) * 1000.0));
        if (r <= 0) continue;
        const int64_t idx = manager.NodeToIndex(RoutingNodeIndex(i));
        customer_index[i] = idx;
        routing.AddDisjunction({idx}, r);
    }

    RoutingSearchParameters params = DefaultRoutingSearchParameters();
    params.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
    params.set_local_search_metaheuristic(
        static_cast<LocalSearchMetaheuristic_Value>(config.local_search_metaheuristic));
    params.mutable_time_limit()->set_seconds(static_cast<int64_t>(config.max_cpu_time));
    params.set_log_search(config.verbose);

    const auto* solution = routing.SolveWithParameters(params);
    if (!solution) return empty;

    // Extract route by walking the vehicle's chain from start to end.
    std::vector<NodeId> route;
    int64_t idx = manager.GetStartIndex(0);
    route.push_back(static_cast<NodeId>(manager.IndexToNode(idx).value()));
    while (!routing.IsEnd(idx)) {
        idx = solution->Value(routing.NextVar(idx));
        route.push_back(static_cast<NodeId>(manager.IndexToNode(idx).value()));
    }

    Reward total_reward = 0.0;
    Time   total_time   = 0.0;
    for (size_t k = 1; k < route.size(); ++k) {
        total_time += problem.get_distance(route[k - 1], route[k]);
        if (route[k] != static_cast<NodeId>(sink))
            total_reward += problem.get_reward(route[k]);
    }

    model::Solution sol(1);
    sol.get_route(0)      = std::move(route);
    sol.total_reward      = total_reward;
    sol.total_travel_time = total_time;
    return sol;
}

}  // namespace oplib::solver::routing
