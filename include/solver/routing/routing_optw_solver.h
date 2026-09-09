#pragma once

#include "solver/solver.h"

namespace oplib::solver::routing {

struct RoutingOPTWSolverConfig : public SolverConfig {
    // OR-Tools RoutingSearchParameters::LocalSearchMetaheuristic_Value.
    // Default is TABU_SEARCH (value 4): empirically escapes the local
    // optimum that GUIDED_LOCAL_SEARCH gets stuck in on tight-TW OPTW
    // instances (e.g. proves reward=308 on Cordeau pr01 in <5s, reliably,
    // vs. GUIDED_LOCAL_SEARCH's stuck-at-304 plateau even at 300s).
    int local_search_metaheuristic = 4;
};

/**
 * @brief Single-vehicle OPTW solver using OR-Tools RoutingModel.
 *
 * Unlike CPSATOPTWSolver (raw CP-SAT circuit constraint), this uses
 * OR-Tools' dedicated VRP/routing library (constraint_solver::RoutingModel),
 * purpose-built for exactly this problem class:
 *   - A "Time" dimension gives native time-window cumulative propagation
 *     (transit = travel distance + service time at the "from" node).
 *   - Each customer is modelled as an optional visit via AddDisjunction
 *     with penalty = reward (the standard prize-collecting / orienteering
 *     pattern) — dropping is allowed, penalty is the cost of dropping.
 *   - Arc cost = real travel distance, kept small relative to reward
 *     penalties so reward-maximization dominates, but nonzero so
 *     TabuSearch/GuidedLocalSearch's cost-based escape mechanisms have a
 *     real gradient to work with.
 *
 * Empirically much stronger than CPSATOPTWSolver on the Cordeau OPTW
 * benchmark set (1-3% gap vs. Pulse's proven optima, vs. 12-34% for the
 * circuit-based CP-SAT model), because RoutingModel's specialized
 * time-dimension propagator captures precedence + travel-time reasoning
 * more tightly than CP-SAT's generic reified linear timing constraints.
 *
 * Returns an empty depot-to-depot route when no feasible solution is found.
 * Requires problem.has_time_windows() == true.
 */
class RoutingOPTWSolver : public Solver {
public:
    std::string get_name() const override { return "ROUTING_OPTW"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig& config) override;

    model::Solution solve(const model::Problem& problem,
                          const RoutingOPTWSolverConfig& config);
};

}  // namespace oplib::solver::routing
