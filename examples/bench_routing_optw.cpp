// Prototype: OPTW via OR-Tools RoutingModel (constraint_solver), using its
// native time-dimension propagation + AddDisjunction for optional visits.
// Not yet integrated as a project Solver — standalone validation only.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "bench_utils.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingSearchParameters;
using operations_research::DefaultRoutingSearchParameters;
using operations_research::FirstSolutionStrategy;
using operations_research::LocalSearchMetaheuristic;
using RoutingNodeIndex = RoutingIndexManager::NodeIndex;

int main(int argc, char** argv) {
    auto opts = bench::parse_cli(argc, argv, "routing_optw");
    auto instances = bench::discover_instances(opts.instance_path, opts.variants);
    if (instances.empty()) { std::cerr << "No instances found.\n"; return 1; }
    if (opts.quick > 0) instances = bench::quick_filter(std::move(instances), opts.quick);

    for (const auto& spec : instances) {
        auto problem = bench::parse_instance(spec);
        if (!problem) continue;
        bench::apply_overrides(opts, *problem);

        const int n      = static_cast<int>(problem->get_num_nodes());
        const int source  = static_cast<int>(problem->get_source_depot());
        const int sink    = static_cast<int>(problem->get_sink_depot());
        const int64_t SCALE = problem->is_scaled() ? 1 : 1000;

        double tmax_d = std::min(problem->get_budget(), problem->get_time_window(sink).closing);
        if (tmax_d >= 1e15) tmax_d = problem->get_budget();
        const int64_t TMAX = static_cast<int64_t>(std::ceil(tmax_d * (double)SCALE));

        RoutingIndexManager manager(n, 1,
            std::vector<RoutingNodeIndex>{RoutingNodeIndex(source)},
            std::vector<RoutingNodeIndex>{RoutingNodeIndex(sink)});
        RoutingModel routing(manager);

        // Transit = travel time + service time at the "from" node (standard
        // OR-Tools pattern for folding service time into a time dimension).
        const int transit_cb = routing.RegisterTransitCallback(
            [&](int64_t from_index, int64_t to_index) -> int64_t {
                const int from = manager.IndexToNode(from_index).value();
                const int to   = manager.IndexToNode(to_index).value();
                const int64_t dist = static_cast<int64_t>(
                    std::floor(problem->get_distance(from, to) * (double)SCALE));
                const int64_t svc = static_cast<int64_t>(
                    std::floor(problem->get_service_time(from) * (double)SCALE));
                return dist + svc;
            });
        // Arc cost = travel distance. Disjunction penalties (reward*1000)
        // dominate this by 1-2 orders of magnitude, so the search still
        // prioritizes maximizing collected reward; the small distance
        // signal gives GuidedLocalSearch's arc-penalization mechanism
        // something to actually work with (zero-cost arcs give it nothing
        // to escape local optima with).
        routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitCallback(
            [&](int64_t from_index, int64_t to_index) -> int64_t {
                const int from = manager.IndexToNode(from_index).value();
                const int to   = manager.IndexToNode(to_index).value();
                return static_cast<int64_t>(
                    std::floor(problem->get_distance(from, to) * (double)SCALE));
            }));

        routing.AddDimension(transit_cb, /*slack_max=*/TMAX, /*capacity=*/TMAX,
                              /*fix_start_cumul_to_zero=*/false, "Time");
        auto* time_dim = routing.GetMutableDimension("Time");

        for (int i = 0; i < n; ++i) {
            int64_t idx;
            if (i == source)      idx = manager.GetStartIndex(0);
            else if (i == sink)   idx = manager.GetEndIndex(0);
            else                  idx = manager.NodeToIndex(RoutingNodeIndex(i));
            const auto tw = problem->get_time_window(i);
            int64_t lo = static_cast<int64_t>(std::floor(tw.opening * (double)SCALE));
            int64_t hi = (tw.closing >= 1e15) ? TMAX
                         : static_cast<int64_t>(std::ceil(tw.closing * (double)SCALE));
            hi = std::min(hi, TMAX);
            if (lo > hi) lo = hi;
            time_dim->CumulVar(idx)->SetRange(lo, hi);
        }

        // Optional visits: customers may be dropped at a cost = their reward
        // (scaled to integers, dominating arc-cost distance by 1-2 orders of
        // magnitude); source/sink are mandatory (not added).
        std::vector<int64_t> customer_index(n, -1);
        for (int i = 0; i < n; ++i) {
            if (i == source || i == sink) continue;
            const int64_t r = static_cast<int64_t>(std::round(problem->get_reward(i) * 1000.0));
            if (r <= 0) continue;
            const int64_t idx = manager.NodeToIndex(RoutingNodeIndex(i));
            customer_index[i] = idx;
            routing.AddDisjunction({idx}, r);
        }

        RoutingSearchParameters params = DefaultRoutingSearchParameters();
        params.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
        params.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        params.mutable_time_limit()->set_seconds(static_cast<int64_t>(opts.timeout));
        params.set_log_search(opts.verbose);

        const auto* solution = routing.SolveWithParameters(params);
        if (!solution) {
            std::cout << spec.filepath << ": NO SOLUTION (status="
                      << routing.status() << ")\n";
            continue;
        }
        double reward = 0.0;
        for (int i = 0; i < n; ++i) {
            if (customer_index[i] < 0) continue;
            if (solution->Value(routing.ActiveVar(customer_index[i])) == 1)
                reward += problem->get_reward(i);
        }
        std::cout << spec.filepath << ": reward=" << reward
                   << " objective=" << solution->ObjectiveValue()
                   << " (status=" << routing.status() << ")\n";
    }
    return 0;
}
