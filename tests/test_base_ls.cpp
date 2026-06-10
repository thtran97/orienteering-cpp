#include <gtest/gtest.h>

#include "solver/local_search/base_ls.h"
#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"
#include "core/random.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver::local_search;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

class BaseLSUtilsTest : public ::testing::Test {
protected:
    oplib::utils::Random rng{42};

    // Simple linear OP: source(0) -- c1(1) -- c2(2) -- c3(3) -- sink(4)
    // Budget = 100, rewards = 10, 20, 15
    OPProblem make_linear_op() {
        OPProblem p("linear_op", 100.0);
        p.add_node(Node{0, 0.0,  0.0, 0.0, 0.0});   // source
        p.add_node(Node{1, 10.0, 0.0, 10.0, 0.0});  // c1
        p.add_node(Node{2, 20.0, 0.0, 20.0, 0.0});  // c2
        p.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});  // c3
        p.add_node(Node{4, 40.0, 0.0, 0.0,  0.0});  // sink
        p.finalize();
        p.preprocessing();
        return p;
    }

    // OPTW with tight time windows to test TW feasibility
    OPTWProblem make_tight_optw() {
        OPTWProblem p("tight_optw", 100.0);
        // Nodes: source at 0, two customers, sink at 30
        // Travel: Euclidean distance
        p.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 1000.0}});   // source
        p.add_node(Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 20.0}});  // c1: TW closes at 20
        p.add_node(Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 30.0}});  // c2: TW closes at 30
        p.add_node(Node{3, 30.0, 0.0, 0.0,  0.0, {0.0, 1000.0}}); // sink
        p.finalize();
        p.preprocessing();
        return p;
    }
};

// ---------------------------------------------------------------------------
// init() tests
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, Init_EmptyRoutesAllVehicles) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;

    utils.init(sol, visited, ctxs);

    EXPECT_EQ(sol.get_num_vehicles(), 1);
    const auto& route = sol.get_route(0);
    EXPECT_EQ(route.size(), 2u);
    EXPECT_EQ(route.front(), problem.get_source_depot());
    EXPECT_EQ(route.back(), problem.get_sink_depot());

    EXPECT_EQ(static_cast<int>(visited.size()), problem.get_num_nodes());
    EXPECT_TRUE(visited[problem.get_source_depot()]);
    EXPECT_TRUE(visited[problem.get_sink_depot()]);
    EXPECT_FALSE(visited[1]);
    EXPECT_FALSE(visited[2]);
    EXPECT_FALSE(visited[3]);

    EXPECT_EQ(static_cast<int>(ctxs.size()), 1);
    EXPECT_EQ(static_cast<int>(ctxs[0].arrival_times.size()), 2);
    // Empty route {src, sink}: cumulative_time = direct depot-to-depot distance
    EXPECT_DOUBLE_EQ(ctxs[0].cumulative_time, problem.get_distance(
        problem.get_source_depot(), problem.get_sink_depot()));
}

// ---------------------------------------------------------------------------
// check_insertion() tests
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, CheckInsertion_FeasibleOnEmptyRoute) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    // Insert c1 (node 1) at position 1 in vehicle 0 — should be feasible
    double shift = utils.check_insertion(sol, ctxs, 0, 1, 1);
    EXPECT_LT(shift, BaseLSUtils::INF);
    EXPECT_GE(shift, 0.0);
}

TEST_F(BaseLSUtilsTest, CheckInsertion_InvalidPosition) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    // Position 0 is the depot — invalid insertion position
    EXPECT_GE(utils.check_insertion(sol, ctxs, 0, 1, 0), BaseLSUtils::INF);
    // Position >= route.size() — out of bounds
    EXPECT_GE(utils.check_insertion(sol, ctxs, 0, 1, 2), BaseLSUtils::INF);
}

TEST_F(BaseLSUtilsTest, CheckInsertion_BudgetViolation) {
    // Very tight budget — no customer can be inserted
    OPProblem problem("tight_budget", 1.0); // budget = 1 unit
    problem.add_node(Node{0, 0.0,  0.0, 0.0, 0.0});
    problem.add_node(Node{1, 50.0, 0.0, 10.0, 0.0});  // far away
    problem.add_node(Node{2, 100.0, 0.0, 0.0, 0.0});
    problem.finalize();
    problem.preprocessing();

    BaseLSUtils utils(problem, rng);
    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    double shift = utils.check_insertion(sol, ctxs, 0, 1, 1);
    EXPECT_GE(shift, BaseLSUtils::INF);
}

// ---------------------------------------------------------------------------
// repair() tests
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, Repair_ProducesFeasibleSolution) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    const auto& route = sol.get_route(0);
    EXPECT_GE(route.size(), 3u); // at least one customer inserted
    EXPECT_EQ(route.front(), problem.get_source_depot());
    EXPECT_EQ(route.back(), problem.get_sink_depot());
    EXPECT_GT(sol.total_reward, 0.0);

    // No duplicate customers
    std::set<NodeId> unique(route.begin(), route.end());
    EXPECT_EQ(unique.size(), route.size());
}

TEST_F(BaseLSUtilsTest, Repair_VisitedFlagSet) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    const auto& route = sol.get_route(0);
    for (NodeId n : route) {
        EXPECT_TRUE(visited[n]) << "Node " << n << " in route but not marked visited";
    }
}

TEST_F(BaseLSUtilsTest, Repair_TightOptw_RespectsTimeWindows) {
    auto problem = make_tight_optw();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    // Check that all visited customers are within their time windows
    const auto& route = sol.get_route(0);
    const auto& ctx   = ctxs[0];
    for (int i = 1; i < static_cast<int>(route.size()) - 1; ++i) {
        NodeId c = route[i];
        const auto& tw = problem.get_time_window(c);
        EXPECT_LE(ctx.arrival_times[i], tw.closing)
            << "Arrival at customer " << c << " violates time window";
    }
}

// ---------------------------------------------------------------------------
// destroy() / shake() tests
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, Destroy_ReducesRouteSize) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    int before = static_cast<int>(sol.get_route(0).size());
    utils.destroy(sol, visited, ctxs, 0, 0.5);
    int after = static_cast<int>(sol.get_route(0).size());

    EXPECT_LE(after, before);
}

TEST_F(BaseLSUtilsTest, Destroy_RemovedCustomersUnmarked) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    utils.destroy(sol, visited, ctxs, 0, 0.9); // remove most customers

    const auto& route = sol.get_route(0);
    for (int n = 0; n < problem.get_num_nodes(); ++n) {
        if (n == problem.get_source_depot() || n == problem.get_sink_depot()) continue;
        bool in_route = std::find(route.begin(), route.end(), n) != route.end();
        EXPECT_EQ(visited[n], in_route)
            << "Visited flag mismatch for node " << n;
    }
}

TEST_F(BaseLSUtilsTest, Shake_EmptyRoute_NoOp) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    // Shake on an empty route (only depots) — should not crash
    EXPECT_NO_THROW(utils.shake(sol, visited, ctxs, 0, 1, 2));
    EXPECT_EQ(sol.get_route(0).size(), 2u); // depots only unchanged
}

// ---------------------------------------------------------------------------
// recompute_context() tests
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, RecomputeContext_ConsistentWithManualInsertion) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    // Manually recompute context and compare
    RouteContext recomputed;
    utils.recompute_context(sol.get_route(0), recomputed);

    const auto& ctx = ctxs[0];
    ASSERT_EQ(ctx.arrival_times.size(), recomputed.arrival_times.size());
    for (int i = 0; i < static_cast<int>(ctx.arrival_times.size()); ++i) {
        EXPECT_NEAR(ctx.arrival_times[i], recomputed.arrival_times[i], 1e-6)
            << "Arrival time mismatch at position " << i;
        EXPECT_NEAR(ctx.departure_times[i], recomputed.departure_times[i], 1e-6)
            << "Departure time mismatch at position " << i;
    }
}

// ---------------------------------------------------------------------------
// minimize_makespan() tests
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, MinimizeMakespan_DoesNotIncreaseReward) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    double reward_before = sol.total_reward;
    utils.minimize_makespan(sol, ctxs, 0);
    double reward_after = sol.total_reward;

    // Makespan minimization must not change reward (no insertions/removals)
    EXPECT_NEAR(reward_before, reward_after, 1e-6);
}

TEST_F(BaseLSUtilsTest, MinimizeMakespan_RouteRemainsValid) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;
    utils.repair(sol, visited, ctxs, config);

    utils.minimize_makespan(sol, ctxs, 0);

    const auto& route = sol.get_route(0);
    EXPECT_EQ(route.front(), problem.get_source_depot());
    EXPECT_EQ(route.back(), problem.get_sink_depot());

    // No duplicates
    std::set<NodeId> unique(route.begin(), route.end());
    EXPECT_EQ(unique.size(), route.size());
}

// ---------------------------------------------------------------------------
// Round-trip: repair → destroy → repair
// ---------------------------------------------------------------------------

TEST_F(BaseLSUtilsTest, RepairDestroyRepair_ProducesFeasibleSolution) {
    auto problem = make_linear_op();
    BaseLSUtils utils(problem, rng);

    model::Solution sol;
    std::vector<bool> visited;
    std::vector<RouteContext> ctxs;
    utils.init(sol, visited, ctxs);

    LSConfig config;

    utils.repair(sol, visited, ctxs, config);
    utils.destroy(sol, visited, ctxs, 0, 0.5);
    utils.repair(sol, visited, ctxs, config);

    const auto& route = sol.get_route(0);
    EXPECT_GE(route.size(), 2u); // at least depots
    EXPECT_EQ(route.front(), problem.get_source_depot());
    EXPECT_EQ(route.back(), problem.get_sink_depot());

    // visited consistent with route
    for (int n = 0; n < problem.get_num_nodes(); ++n) {
        if (n == problem.get_source_depot() || n == problem.get_sink_depot()) continue;
        bool in_route = std::find(route.begin(), route.end(), n) != route.end();
        EXPECT_EQ(visited[n], in_route)
            << "Visited flag mismatch after round-trip for node " << n;
    }
}
