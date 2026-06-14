// ============================================================================
// test_route_recombination.cpp
//
// Phase 2 (route recombination): verify the set-packing route-recombination
// operator ILSRouteRecombinationSolver::recombine_routes(). It must:
//   - combine the best customer-disjoint routes from the pool,
//   - never return less reward than the best individual pool member,
//   - always produce a structurally valid, double-visit-free solution.
// ============================================================================

#include <gtest/gtest.h>

#include <set>

#include "model/variants/op.h"
#include "model/variants/top.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "solver/local_search/base_ls.h"
#include "core/random.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using oplib::solver::metaheuristic::ILSRouteRecombinationSolver;

namespace {

// Reward implied by the routes (sum of visited customer rewards).
Reward route_reward(const model::Solution& s, const model::Problem& p) {
    Reward r = 0.0;
    for (int v = 0; v < s.get_num_vehicles(); ++v) {
        const auto& route = s.get_route(v);
        for (size_t i = 1; i + 1 < route.size(); ++i) r += p.get_reward(route[i]);
    }
    return r;
}

void validate(const model::Solution& s, const model::Problem& p) {
    std::set<NodeId> seen;
    for (int v = 0; v < s.get_num_vehicles(); ++v) {
        const auto& route = s.get_route(v);
        ASSERT_GE(route.size(), 2u);
        EXPECT_EQ(route.front(), p.get_source_depot());
        EXPECT_EQ(route.back(), p.get_sink_depot());
        for (size_t i = 1; i + 1 < route.size(); ++i)
            EXPECT_TRUE(seen.insert(route[i]).second) << "double visit: " << route[i];
    }
    EXPECT_NEAR(s.total_reward, route_reward(s, p), 1e-6);
}

}  // namespace

// Two pool solutions each cover one cluster with a single vehicle; recombination
// must merge both clusters into the two available vehicles.
TEST(RouteRecombination, CombinesDisjointRoutes_TOP) {
    TOPProblem p("rr_top", /*vehicles=*/2, /*budget=*/100.0);
    p.add_node(Node{0,   0.0, 0.0,  0.0, 0.0});
    p.add_node(Node{1,  10.0, 0.0, 10.0, 0.0});
    p.add_node(Node{2,  20.0, 0.0, 10.0, 0.0});
    p.add_node(Node{3, -10.0, 0.0, 10.0, 0.0});
    p.add_node(Node{4, -20.0, 0.0, 10.0, 0.0});
    p.add_node(Node{5,   0.0, 0.0,  0.0, 0.0});
    p.finalize();
    p.preprocessing();

    model::Solution a(2);
    a.get_route(0) = {0, 1, 2, 5};   // right cluster, reward 20
    a.get_route(1) = {0, 5};
    model::Solution b(2);
    b.get_route(0) = {0, 3, 4, 5};   // left cluster, reward 20
    b.get_route(1) = {0, 5};

    oplib::utils::Random rng(1);
    solver::local_search::BaseLSUtils ls(p, rng);
    solver::local_search::LSConfig ls_cfg;

    auto sol = ILSRouteRecombinationSolver::recombine_routes(p, {a, b}, ls, ls_cfg);

    validate(sol, p);
    // Both disjoint clusters fit in the two vehicles -> all reward collected.
    EXPECT_DOUBLE_EQ(sol.total_reward, 40.0);
    // ...and it is at least as good as the best individual pool member.
    EXPECT_GE(sol.total_reward, std::max(route_reward(a, p), route_reward(b, p)));
}

// Single-vehicle: recombination keeps the best route and repairs leftovers; the
// result must never be worse than the best individual pool member.
TEST(RouteRecombination, NeverWorseThanBestMember_OP) {
    OPProblem p("rr_op", /*budget=*/100.0);
    p.add_node(Node{0,  0.0, 0.0,  0.0, 0.0});
    p.add_node(Node{1, 10.0, 0.0, 10.0, 0.0});
    p.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    p.add_node(Node{3,  0.0, 0.0,  0.0, 0.0});
    p.finalize();
    p.preprocessing();

    model::Solution a(1);
    a.get_route(0) = {0, 1, 3};   // reward 10
    model::Solution b(1);
    b.get_route(0) = {0, 2, 3};   // reward 30

    oplib::utils::Random rng(7);
    solver::local_search::BaseLSUtils ls(p, rng);
    solver::local_search::LSConfig ls_cfg;

    auto sol = ILSRouteRecombinationSolver::recombine_routes(p, {a, b}, ls, ls_cfg);

    validate(sol, p);
    EXPECT_GE(sol.total_reward, std::max(route_reward(a, p), route_reward(b, p)));
}

// An empty pool must still yield a structurally valid (empty) solution.
TEST(RouteRecombination, EmptyPoolYieldsValidSolution) {
    TOPProblem p("rr_empty", 2, 100.0);
    p.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    p.add_node(Node{1, 5.0, 0.0, 10.0, 0.0});
    p.add_node(Node{2, 0.0, 0.0, 0.0, 0.0});
    p.finalize();
    p.preprocessing();

    oplib::utils::Random rng(1);
    solver::local_search::BaseLSUtils ls(p, rng);
    solver::local_search::LSConfig ls_cfg;

    auto sol = ILSRouteRecombinationSolver::recombine_routes(p, {}, ls, ls_cfg);
    validate(sol, p);
    EXPECT_GE(sol.total_reward, 0.0);
}
