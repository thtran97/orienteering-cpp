#ifdef ENABLE_CPSAT_SOLVER

#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "solver/checker/solution_checker.h"
#include "solver/cpsat/cpsat_solver.h"
#include "solver/dynamic_programming/dp_solvers.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;
using namespace oplib::solver::cpsat;

namespace {

// Canonical 6-node layout (source, 4 customers, sink) from test_all_variants.cpp.
std::vector<Node> canonical_nodes(bool with_tw) {
    std::vector<Node> nodes = {
        Node{0,  0.0,  0.0,  0.0, 0.0, {0.0, 1e6}},
        Node{1, 10.0,  0.0, 10.0, 1.0, {0.0, 1e6}},
        Node{2, 20.0,  0.0, 20.0, 1.0, {0.0, 1e6}},
        Node{3, 20.0, 10.0, 15.0, 1.0, {0.0, 1e6}},
        Node{4, 10.0, 10.0, 25.0, 1.0, {0.0, 1e6}},
        Node{5,  0.0,  0.0,  0.0, 0.0, {0.0, 1e6}},
    };
    if (!with_tw) {
        for (auto& nd : nodes) { nd.tw = {0.0, 1e6}; nd.service_time = 0.0; }
    }
    return nodes;
}

template <class P>
void add_nodes(P& p, bool with_tw) {
    for (const auto& nd : canonical_nodes(with_tw)) p.add_node(nd);
    p.finalize();
    p.preprocessing();
}

void validate(const Solution& sol, const Problem& prob, const char* label) {
    SCOPED_TRACE(label);
    ASSERT_EQ(sol.get_num_vehicles(), 1);
    const auto& route = sol.get_route(0);
    ASSERT_GE(route.size(), 2u);
    EXPECT_EQ(route.front(), prob.get_source_depot());
    EXPECT_EQ(route.back(),  prob.get_sink_depot());

    // No duplicate customers.
    std::set<NodeId> seen;
    Reward r_check = 0.0;
    for (size_t i = 1; i + 1 < route.size(); ++i) {
        EXPECT_TRUE(seen.insert(route[i]).second) << "duplicate node " << route[i];
        r_check += prob.get_reward(route[i]);
    }
    EXPECT_NEAR(sol.total_reward, r_check, 1e-6);
    EXPECT_GT(sol.total_reward, 0.0);
}

CPSATSolverConfig make_cfg(double timeout = 10.0) {
    CPSATSolverConfig c;
    c.seed         = 42;
    c.max_cpu_time = timeout;
    c.verbose      = false;
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// OP: no time windows, budget only
// ---------------------------------------------------------------------------
TEST(CPSATSolver, OP_FindsPositiveReward) {
    OPProblem p("op_cpsat", 500.0);
    add_nodes(p, /*with_tw=*/false);

    CPSATSolver solver;
    auto sol = solver.solve(p, make_cfg());
    validate(sol, p, "OP");
}

// ---------------------------------------------------------------------------
// OPTW: time windows
// ---------------------------------------------------------------------------
TEST(CPSATSolver, OPTW_FindsPositiveReward) {
    OPTWProblem p("optw_cpsat", 500.0);
    add_nodes(p, /*with_tw=*/true);

    CPSATSolver solver;
    auto sol = solver.solve(p, make_cfg());
    validate(sol, p, "OPTW");
}

// ---------------------------------------------------------------------------
// OPTW: tight time windows that forbid visiting every customer
// ---------------------------------------------------------------------------
TEST(CPSATSolver, OPTW_TightWindows) {
    OPTWProblem p("optw_tight", 20.0);
    // Source and sink at origin; one reachable customer close by,
    // one reachable customer far away (time window forces a choice).
    std::vector<Node> nodes = {
        Node{0,  0.0, 0.0,  0.0, 0.0, {0.0, 20.0}},  // source
        Node{1,  1.0, 0.0, 10.0, 0.0, {0.0, 20.0}},  // close, always reachable
        Node{2, 50.0, 0.0, 20.0, 0.0, {0.0, 20.0}},  // far, unreachable
        Node{3,  0.0, 0.0,  0.0, 0.0, {0.0, 20.0}},  // sink
    };
    for (const auto& nd : nodes) p.add_node(nd);
    p.finalize();
    p.preprocessing();

    CPSATSolver solver;
    auto sol = solver.solve(p, make_cfg());

    ASSERT_EQ(sol.get_num_vehicles(), 1);
    const auto& route = sol.get_route(0);
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(),  3);
    EXPECT_NEAR(sol.total_reward, 10.0, 1e-6);  // only node 1 is reachable
    EXPECT_FALSE(std::find(route.begin(), route.end(), NodeId{2}) != route.end())
        << "unreachable node 2 must not appear in the route";
}

// ---------------------------------------------------------------------------
// Optimality check: compare CPSAT vs ForwardDP on a small instance.
// ---------------------------------------------------------------------------
TEST(CPSATSolver, OP_MatchesForwardDP) {
    OPProblem p("op_opt", 500.0);
    add_nodes(p, /*with_tw=*/false);

    dp::DPSolverConfig dp_cfg;
    dp_cfg.seed       = 42;
    dp_cfg.max_labels = 500000;
    dp::ForwardDPSolver dp_solver;
    const Reward dp_reward = dp_solver.solve(p, dp_cfg).total_reward;

    CPSATSolver cpsat_solver;
    const Reward cpsat_reward = cpsat_solver.solve(p, make_cfg(30.0)).total_reward;

    EXPECT_NEAR(cpsat_reward, dp_reward, 1e-4)
        << "CPSAT reward=" << cpsat_reward << " DP reward=" << dp_reward;
}

TEST(CPSATSolver, OPTW_MatchesForwardDP) {
    OPTWProblem p("optw_opt", 500.0);
    add_nodes(p, /*with_tw=*/true);

    dp::DPSolverConfig dp_cfg;
    dp_cfg.seed       = 42;
    dp_cfg.max_labels = 500000;
    dp::ForwardDPSolver dp_solver;
    const Reward dp_reward = dp_solver.solve(p, dp_cfg).total_reward;

    CPSATSolver cpsat_solver;
    const Reward cpsat_reward = cpsat_solver.solve(p, make_cfg(30.0)).total_reward;

    EXPECT_NEAR(cpsat_reward, dp_reward, 1e-4)
        << "CPSAT reward=" << cpsat_reward << " DP reward=" << dp_reward;
}

// ---------------------------------------------------------------------------
// SolutionChecker: CPSAT solution must pass the checker.
// ---------------------------------------------------------------------------
TEST(CPSATSolver, OPTW_PassesSolutionChecker) {
    OPTWProblem p("optw_check", 500.0);
    add_nodes(p, /*with_tw=*/true);

    CPSATSolver solver;
    auto sol = solver.solve(p, make_cfg());

    auto checker = solver::create_checker(p);
    ASSERT_NE(checker, nullptr);
    auto result = checker->check(p, sol);
    EXPECT_TRUE(result.valid) << (result.violations.empty() ? "" : result.violations[0]);
}

#endif  // ENABLE_CPSAT_SOLVER
