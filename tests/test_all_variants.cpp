// ============================================================================
// test_all_variants.cpp
//
// Smoke + correctness coverage ensuring every solver produces a *valid*
// solution for at least one instance of every problem variant.
//
// Variants covered: OP, OPTW, TOP, TOPTW, TDOP, TDOPTW, MCTOPMTW, SingleSat,
// TTDP.
//
// Solvers exercised:
//   - constructive:   GreedySolver, RandomizedGreedySolver
//   - metaheuristic:  LNSSolver, GraspVnsSolver, ILS09Solver,
//                     ILSRouteRecombinationSolver
//   - policy_learning: MCTSSolver
//   - exact (single-vehicle only): ForwardDPSolver, BidirectionalDPSolver,
//                     PulseSolver
//
// Instances are built programmatically with loose constraints so that a
// positive-reward solution always exists; this keeps the assertions strict
// without being flaky.
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <vector>

#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"
#include "model/variants/tdop.h"
#include "model/variants/tdoptw.h"
#include "model/variants/mctopmtw.h"
#include "model/variants/singlesat.h"
#include "model/variants/ttdp.h"

#include "solver/constructive/greedy.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/metaheuristic/lns.h"
#include "solver/metaheuristic/grasp_vns.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "solver/policy_learning/mcts_solver.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/pulse/pulse_solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;

namespace {

// ---------------------------------------------------------------------------
// Shared customer layout (depot at 0, sink at last index).
// Small, all customers reachable within a generous budget.
// ---------------------------------------------------------------------------

/// Returns the canonical node list: depot, 4 customers, sink.
/// `with_tw` decides whether time windows are loose (feasible for all).
std::vector<Node> canonical_nodes(bool with_tw) {
    std::vector<Node> nodes = {
        Node{0,  0.0,  0.0,  0.0, 0.0, {0.0, 1e6}},   // source depot
        Node{1, 10.0,  0.0, 10.0, 1.0, {0.0, 1e6}},
        Node{2, 20.0,  0.0, 20.0, 1.0, {0.0, 1e6}},
        Node{3, 20.0, 10.0, 15.0, 1.0, {0.0, 1e6}},
        Node{4, 10.0, 10.0, 25.0, 1.0, {0.0, 1e6}},
        Node{5,  0.0,  0.0,  0.0, 0.0, {0.0, 1e6}},   // sink depot
    };
    if (!with_tw) {
        for (auto& n : nodes) { n.tw = {0.0, 1e6}; n.service_time = 0.0; }
    }
    return nodes;
}

template <class ProblemT>
void add_nodes(ProblemT& p, bool with_tw) {
    for (const auto& n : canonical_nodes(with_tw)) p.add_node(n);
    p.finalize();
    p.preprocessing();
}

double euclid(const Node& a, const Node& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// ---------------------------------------------------------------------------
// Time-dependent setup: configure TD data so travel_time(i,j,t) == euclidean
// distance for all t.  This exercises the TD code paths while keeping the
// instance equivalent to its static counterpart (so positive reward is
// guaranteed and exact solvers stay correct).
// ---------------------------------------------------------------------------

void setup_td_identity(TDOPProblem& p, int n) {
    // Single arc category (0), single speed slot, speed 1.0 => time == distance.
    p.set_arc_categories(std::vector<std::vector<int>>(n, std::vector<int>(n, 0)));
    p.set_speed_matrix({{1.0}});
    p.set_time_slots({});  // empty boundaries => slot 0 for any departure time
}

void setup_td_identity(TDOPTWProblem& p, const std::vector<Node>& nodes) {
    const int n = static_cast<int>(nodes.size());
    std::vector<std::vector<std::vector<Time>>> tm(
        n, std::vector<std::vector<Time>>(n, std::vector<Time>(1, 0.0)));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            tm[i][j][0] = euclid(nodes[i], nodes[j]);
    p.set_transition_matrix(std::move(tm));
    p.set_start_time(0.0);
    p.set_slot_duration(1e9);  // one effective slot
}

// ---------------------------------------------------------------------------
// Validation: structure + no double visits + reward consistency.
// ---------------------------------------------------------------------------

void validate_solution(const model::Solution& sol,
                       const model::Problem&  problem,
                       bool expect_positive,
                       const char* label) {
    SCOPED_TRACE(label);
    ASSERT_GE(sol.get_num_vehicles(), 1);

    std::set<NodeId> visited_customers;
    Reward reward_from_routes = 0.0;

    for (int v = 0; v < sol.get_num_vehicles(); ++v) {
        const auto& route = sol.get_route(v);
        ASSERT_GE(route.size(), 2u) << "route must contain at least the two depots";
        EXPECT_EQ(route.front(), problem.get_source_depot());
        EXPECT_EQ(route.back(),  problem.get_sink_depot());

        for (size_t i = 1; i + 1 < route.size(); ++i) {
            NodeId c = route[i];
            EXPECT_TRUE(visited_customers.insert(c).second)
                << "customer " << c << " visited by more than one vehicle/position";
            reward_from_routes += problem.get_reward(c);
        }
    }

    // total_reward must equal the sum of collected node rewards.
    EXPECT_NEAR(sol.total_reward, reward_from_routes, 1e-6)
        << "total_reward inconsistent with the rewards of visited customers";

    if (expect_positive) EXPECT_GT(sol.total_reward, 0.0);
    else                 EXPECT_GE(sol.total_reward, 0.0);
}

// ---------------------------------------------------------------------------
// Config helpers + generic runner.
// ---------------------------------------------------------------------------

template <class C>
C base_cfg(int iterations) {
    C c;
    c.seed           = 42;
    c.max_iterations = iterations;
    c.max_cpu_time   = 2.0;
    c.verbose        = false;
    return c;
}

template <class SolverT, class ConfigT>
void run_ok(const model::Problem& p, const ConfigT& cfg,
            bool expect_positive, const char* label) {
    SolverT solver;
    auto sol = solver.solve(p, cfg);
    validate_solution(sol, p, expect_positive, label);
}

/// Runs every solver applicable to *any* variant (single- or multi-vehicle).
void run_common_solvers(const model::Problem& p) {
    using namespace oplib::solver::constructive;
    using namespace oplib::solver::metaheuristic;
    using namespace oplib::solver::policy_learning;

    run_ok<GreedySolver>(p, base_cfg<GreedySolverConfig>(1), true, "Greedy");
    run_ok<RandomizedGreedySolver>(p, base_cfg<RandomizedGreedySolverConfig>(20), true, "RandomizedGreedy");
    run_ok<LNSSolver>(p, base_cfg<LNSSolverConfig>(20), true, "LNS");
    run_ok<GraspVnsSolver>(p, base_cfg<GraspVnsSolverConfig>(5), true, "GraspVns");
    run_ok<ILS09Solver>(p, base_cfg<ILS09SolverConfig>(20), true, "ILS09");
    run_ok<ILSRouteRecombinationSolver>(p, base_cfg<ILSRouteRecombinationSolverConfig>(20), true, "ILS-RR");
    // MCTS is stochastic; assert validity + non-negativity (not strict positivity).
    run_ok<MCTSSolver>(p, base_cfg<MCTSSolverConfig>(50), false, "MCTS");
}

/// Runs the single-vehicle exact / bounding solvers.
void run_exact_solvers(const model::Problem& p) {
    solver::dp::DPSolverConfig dp;
    dp.seed = 42; dp.max_labels = 200000; dp.verbose = false;
    run_ok<solver::dp::ForwardDPSolver>(p, dp, true, "ForwardDP");
    run_ok<solver::dp::BidirectionalDPSolver>(p, dp, true, "BidirectionalDP");

    solver::pulse::PulseSolverConfig pl;
    pl.seed = 42; pl.max_labels = 200000; pl.verbose = false;
    run_ok<solver::pulse::PulseSolver>(p, pl, true, "Pulse");
}

}  // namespace

// ============================================================================
// Single-vehicle variants: common + exact solvers.
// ============================================================================

TEST(AllVariants, OP) {
    OPProblem p("op", 500.0);
    add_nodes(p, /*with_tw=*/false);
    run_common_solvers(p);
    run_exact_solvers(p);
}

TEST(AllVariants, OPTW) {
    OPTWProblem p("optw", 500.0);
    add_nodes(p, /*with_tw=*/true);
    run_common_solvers(p);
    run_exact_solvers(p);
}

TEST(AllVariants, SingleSat) {
    SingleSatProblem p("singlesat", 500.0);
    add_nodes(p, /*with_tw=*/true);
    run_common_solvers(p);
    run_exact_solvers(p);
}

TEST(AllVariants, TDOP) {
    TDOPProblem p("tdop", 500.0);
    add_nodes(p, /*with_tw=*/false);
    setup_td_identity(p, p.get_num_nodes());
    run_common_solvers(p);
    run_exact_solvers(p);
}

TEST(AllVariants, TDOPTW) {
    TDOPTWProblem p("tdoptw", 500.0);
    add_nodes(p, /*with_tw=*/true);
    setup_td_identity(p, canonical_nodes(/*with_tw=*/true));
    run_common_solvers(p);
    run_exact_solvers(p);
}

// ============================================================================
// Multi-vehicle variants: common solvers only (exact solvers are single-route).
// ============================================================================

TEST(AllVariants, TOP) {
    TOPProblem p("top", /*num_vehicles=*/2, 200.0);
    add_nodes(p, /*with_tw=*/false);
    run_common_solvers(p);
}

TEST(AllVariants, TOPTW) {
    TOPTWProblem p("toptw", /*num_vehicles=*/2, 300.0);
    add_nodes(p, /*with_tw=*/true);
    run_common_solvers(p);
}

TEST(AllVariants, MCTOPMTW) {
    MCTOPMTWProblem p("mctopmtw", /*num_vehicles=*/2, 300.0);
    add_nodes(p, /*with_tw=*/true);
    // Exercise the MCTOPMTW-specific API with one loose knapsack constraint.
    p.add_knapsack_constraint(1e6, {0.0, 1.0, 1.0, 1.0, 1.0, 0.0});
    run_common_solvers(p);
}

TEST(AllVariants, TTDP) {
    TTDPProblem p("ttdp", /*num_days=*/2, 200.0);
    add_nodes(p, /*with_tw=*/true);
    run_common_solvers(p);
}
