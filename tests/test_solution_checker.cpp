// ============================================================================
// test_solution_checker.cpp
//
// Unit tests for SolutionChecker and MCTOPMTWChecker.
// Each test builds a small problem instance manually and verifies that the
// checker accepts valid solutions and rejects invalid ones with the correct
// violation message.
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>

#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"
#include "model/variants/mctopmtw.h"
#include "model/solution.h"
#include "solver/checker/solution_checker.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;

namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// 6-node layout: source=0, customers=1..4, sink=5
// Placed on a line at x=0,10,20,30,40,50 so distances are exact integers.
//
//   Euclidean distances (simplified as |x_i - x_j| since y=0 for all):
//     0->1 = 10,  1->2 = 10,  ...,  4->5 = 10
//     0->5 = 50  (direct)
//
// With budget=200, all 4 customers fit easily.

template <class ProblemT>
ProblemT make_line_problem(Time budget, bool with_tw = false) {
    ProblemT p("test", budget);
    std::vector<Node> nodes = {
        Node{0, 0.0,  0.0,  0.0, 0.0, {0.0, 1e9}},  // source depot
        Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 1e9}},
        Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 1e9}},
        Node{3, 30.0, 0.0, 15.0, 0.0, {0.0, 1e9}},
        Node{4, 40.0, 0.0, 25.0, 0.0, {0.0, 1e9}},
        Node{5, 50.0, 0.0,  0.0, 0.0, {0.0, 1e9}},  // sink depot
    };
    if (with_tw) {
        // Tight but feasible time windows for a greedy left-to-right traversal:
        // 0->1 (travel 10): [10, 30]
        // 1->2 (travel 10): [20, 50]
        // sink at 50:       [50, 200]
        nodes[1].tw = {10.0, 30.0};
        nodes[2].tw = {20.0, 50.0};
        nodes[3].tw = {30.0, 200.0};
        nodes[4].tw = {40.0, 200.0};
        nodes[5].tw = {0.0,  200.0};
    }
    for (const auto& n : nodes) p.add_node(n);
    p.finalize();
    p.preprocessing();
    return p;
}

// Build a valid single-vehicle solution visiting all 4 customers in order.
Solution make_full_solution(int num_vehicles = 1) {
    Solution sol(num_vehicles);
    sol.get_route(0) = {0, 1, 2, 3, 4, 5};
    sol.total_reward = 70.0;  // 10+20+15+25
    sol.total_travel_time = 50.0;
    return sol;
}

// ---------------------------------------------------------------------------
// OP tests
// ---------------------------------------------------------------------------

TEST(SolutionChecker, OP_ValidSolution) {
    auto p = make_line_problem<OPProblem>(200.0);
    auto sol = make_full_solution();
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_TRUE(res.valid) << res.violations[0];
}

TEST(SolutionChecker, OP_BudgetViolation) {
    // Budget of 30 is far too tight for 0->1->2->3->4->5 (travel=50)
    auto p = make_line_problem<OPProblem>(30.0);
    auto sol = make_full_solution();
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("budget") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a budget violation message";
}

TEST(SolutionChecker, OP_DuplicateNode) {
    auto p = make_line_problem<OPProblem>(200.0);
    Solution sol(1);
    sol.get_route(0) = {0, 1, 2, 1, 5};  // node 1 appears twice
    sol.total_reward = 40.0;
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("more than once") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a duplicate-node violation";
}

TEST(SolutionChecker, OP_WrongStartDepot) {
    auto p = make_line_problem<OPProblem>(200.0);
    Solution sol(1);
    sol.get_route(0) = {1, 2, 3, 5};  // starts at customer 1 instead of depot 0
    sol.total_reward = 35.0;
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("source depot") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a wrong-start-depot violation";
}

TEST(SolutionChecker, OP_WrongEndDepot) {
    auto p = make_line_problem<OPProblem>(200.0);
    Solution sol(1);
    sol.get_route(0) = {0, 1, 2, 4};  // ends at customer 4 instead of sink 5
    sol.total_reward = 30.0;
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("sink depot") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a wrong-end-depot violation";
}

TEST(SolutionChecker, OP_RewardInconsistency) {
    auto p = make_line_problem<OPProblem>(200.0);
    auto sol = make_full_solution();
    sol.total_reward = 999.0;  // wrong
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("total_reward") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a reward inconsistency violation";
}

TEST(SolutionChecker, OP_EmptyRoute) {
    auto p = make_line_problem<OPProblem>(200.0);
    Solution sol(1);
    sol.get_route(0) = {0, 5};  // only depots – valid (zero reward)
    sol.total_reward = 0.0;
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_TRUE(res.valid) << res.violations[0];
}

// ---------------------------------------------------------------------------
// OPTW tests
// ---------------------------------------------------------------------------

TEST(SolutionChecker, OPTW_ValidSolution) {
    auto p = make_line_problem<OPTWProblem>(200.0, /*with_tw=*/true);
    // Route visits 1,2 which open at 10,20 respectively
    // Arrival at 1: t=10 (exactly at opening)
    // Arrival at 2: t=20 (exactly at opening)
    // Arrival at 5: t=30
    Solution sol(1);
    sol.get_route(0) = {0, 1, 2, 5};
    sol.total_reward = 30.0;  // 10 + 20
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_TRUE(res.valid) << res.violations[0];
}

TEST(SolutionChecker, OPTW_TimeWindowViolation) {
    auto p = make_line_problem<OPTWProblem>(200.0, /*with_tw=*/true);
    // Node 1 has window [10, 30]. If we somehow arrive too late:
    // Override node 1's window to [0, 5] so arrival at t=10 is too late.
    // We can't modify the problem after finalize, so we build a tighter problem:
    OPTWProblem p2("test_tw", 200.0);
    std::vector<Node> nodes = {
        Node{0,  0.0, 0.0,  0.0, 0.0, {0.0, 200.0}},
        Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 5.0}},   // closes at 5, but arrival is 10
        Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 200.0}},
        Node{3, 50.0, 0.0,  0.0, 0.0, {0.0, 200.0}},  // sink
    };
    for (const auto& n : nodes) p2.add_node(n);
    p2.finalize();
    p2.preprocessing();

    Solution sol(1);
    sol.get_route(0) = {0, 1, 3};  // arrival at 1 = t=10 > closing=5
    sol.total_reward = 10.0;
    SolutionChecker checker;
    auto res = checker.check(p2, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("closing time") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a time window violation";
}

// ---------------------------------------------------------------------------
// TOP (multi-vehicle) tests
// ---------------------------------------------------------------------------

template <class ProblemT>
ProblemT make_line_problem_multi(int num_vehicles, Time budget) {
    ProblemT p("test", num_vehicles, budget);
    std::vector<Node> nodes = {
        Node{0, 0.0,  0.0,  0.0, 0.0, {0.0, 1e9}},
        Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 1e9}},
        Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 1e9}},
        Node{3, 30.0, 0.0, 15.0, 0.0, {0.0, 1e9}},
        Node{4, 40.0, 0.0, 25.0, 0.0, {0.0, 1e9}},
        Node{5, 50.0, 0.0,  0.0, 0.0, {0.0, 1e9}},
    };
    for (const auto& n : nodes) p.add_node(n);
    p.finalize();
    p.preprocessing();
    return p;
}

TEST(SolutionChecker, TOP_ValidSolution) {
    auto p = make_line_problem_multi<TOPProblem>(2, 200.0);
    Solution sol(2);
    sol.get_route(0) = {0, 1, 2, 5};
    sol.get_route(1) = {0, 3, 4, 5};
    sol.total_reward = 70.0;  // 10+20+15+25
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_TRUE(res.valid) << res.violations[0];
}

TEST(SolutionChecker, TOP_DuplicateAcrossVehicles) {
    auto p = make_line_problem_multi<TOPProblem>(2, 200.0);
    Solution sol(2);
    sol.get_route(0) = {0, 1, 2, 5};
    sol.get_route(1) = {0, 1, 3, 5};  // node 1 visited by both vehicles
    sol.total_reward = 55.0;
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("more than once") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a cross-vehicle duplicate violation";
}

TEST(SolutionChecker, TOP_VehicleCountMismatch) {
    auto p = make_line_problem_multi<TOPProblem>(2, 200.0);
    Solution sol(1);  // problem expects 2 vehicles
    sol.get_route(0) = {0, 1, 2, 5};
    sol.total_reward = 30.0;
    SolutionChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("Vehicle count") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a vehicle-count violation";
}

// ---------------------------------------------------------------------------
// MCTOPMTW knapsack tests
// ---------------------------------------------------------------------------

TEST(SolutionChecker, MCTOPMTW_KnapsackSatisfied) {
    MCTOPMTWProblem p("test_mc", 2, 200.0);
    std::vector<Node> nodes = {
        Node{0, 0.0,  0.0,  0.0, 0.0, {0.0, 1e9}},
        Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 1e9}},
        Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 1e9}},
        Node{3, 30.0, 0.0, 15.0, 0.0, {0.0, 1e9}},
        Node{4, 40.0, 0.0, 25.0, 0.0, {0.0, 1e9}},
        Node{5, 50.0, 0.0,  0.0, 0.0, {0.0, 1e9}},
    };
    for (const auto& n : nodes) p.add_node(n);
    // Knapsack: sum of coefficients for visited nodes <= 3.0
    // coefficients: node 0=0, 1=1, 2=1, 3=1, 4=1, 5=0
    p.add_knapsack_constraint(3.0, {0.0, 1.0, 1.0, 1.0, 1.0, 0.0});
    p.finalize();
    p.preprocessing();

    Solution sol(2);
    sol.get_route(0) = {0, 1, 2, 5};  // uses 2 knapsack units
    sol.get_route(1) = {0, 5};         // uses 0
    sol.total_reward = 30.0;

    MCTOPMTWChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_TRUE(res.valid) << res.violations[0];
}

TEST(SolutionChecker, MCTOPMTW_KnapsackViolation) {
    MCTOPMTWProblem p("test_mc", 2, 200.0);
    std::vector<Node> nodes = {
        Node{0, 0.0,  0.0,  0.0, 0.0, {0.0, 1e9}},
        Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 1e9}},
        Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 1e9}},
        Node{3, 30.0, 0.0, 15.0, 0.0, {0.0, 1e9}},
        Node{4, 40.0, 0.0, 25.0, 0.0, {0.0, 1e9}},
        Node{5, 50.0, 0.0,  0.0, 0.0, {0.0, 1e9}},
    };
    for (const auto& n : nodes) p.add_node(n);
    // Knapsack: sum of coefficients <= 1.0, but we'll visit 3 customers each costing 1
    p.add_knapsack_constraint(1.0, {0.0, 1.0, 1.0, 1.0, 1.0, 0.0});
    p.finalize();
    p.preprocessing();

    Solution sol(2);
    sol.get_route(0) = {0, 1, 2, 5};  // 2 units
    sol.get_route(1) = {0, 3, 5};     // 1 unit → total 3 > rhs=1
    sol.total_reward = 45.0;

    MCTOPMTWChecker checker;
    auto res = checker.check(p, sol);
    EXPECT_FALSE(res.valid);
    bool found = false;
    for (const auto& v : res.violations)
        if (v.find("Knapsack constraint") != std::string::npos) { found = true; break; }
    EXPECT_TRUE(found) << "Expected a knapsack violation";
}

// ---------------------------------------------------------------------------
// create_checker() factory test
// ---------------------------------------------------------------------------

TEST(SolutionChecker, FactoryReturnsMCTOPMTWCheckerForMCProblem) {
    MCTOPMTWProblem p("mc", 1, 100.0);
    Node n0{0, 0.0, 0.0, 0.0, 0.0, {0.0, 1e9}};
    Node n1{1, 0.0, 0.0, 0.0, 0.0, {0.0, 1e9}};
    p.add_node(n0);
    p.add_node(n1);
    p.finalize();
    p.preprocessing();

    auto checker = create_checker(p);
    EXPECT_NE(dynamic_cast<MCTOPMTWChecker*>(checker.get()), nullptr);
}

TEST(SolutionChecker, FactoryReturnsBaseCheckerForOP) {
    OPProblem p("op", 100.0);
    Node n0{0, 0.0, 0.0, 0.0, 0.0, {0.0, 1e9}};
    Node n1{1, 0.0, 0.0, 0.0, 0.0, {0.0, 1e9}};
    p.add_node(n0);
    p.add_node(n1);
    p.finalize();
    p.preprocessing();

    auto checker = create_checker(p);
    // Should be base SolutionChecker, not MCTOPMTWChecker
    EXPECT_EQ(dynamic_cast<MCTOPMTWChecker*>(checker.get()), nullptr);
}

}  // namespace
