#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "model/variants/toptw.h"
#include "solver/local_search/tw_xplainer.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver::local_search;
using Node = oplib::model::Node;

namespace {

/**
 * Build a tiny TOPTW instance with 4 clients.
 *
 * Layout (all on the x-axis, unit distance = 1 per unit):
 *   depot(0) at 0  -- c1(1) at 1  -- c2(2) at 2  -- c3(3) at 3
 *                                                    -- sink(4) at 4
 *
 * Budget = 100 (not binding for small instances here).
 * Service time = 0 everywhere.
 */
TOPTWProblem make_xplainer_problem() {
    // Using TOPTWProblem with budget=100
    TOPTWProblem p("xplainer_test", 1, 100.0);
    using N = Node;
    p.add_node(N{0, 0.0, 0.0, 0.0, 0.0, {0.0, 1000.0}}); // source depot
    p.add_node(N{1, 1.0, 0.0, 1.0, 0.0, {0.0, 1000.0}}); // c1: wide TW
    p.add_node(N{2, 2.0, 0.0, 1.0, 0.0, {0.0, 1000.0}}); // c2: wide TW
    p.add_node(N{3, 3.0, 0.0, 1.0, 0.0, {0.0, 1000.0}}); // c3: wide TW
    p.add_node(N{4, 4.0, 0.0, 0.0, 0.0, {0.0, 1000.0}}); // sink depot
    p.finalize();
    p.preprocessing();
    return p;
}

/**
 * Mutually-exclusive pair: c1 closes before we can arrive after visiting c2.
 *
 * Layout: depot(0) -- c1(1) -- c2(2) -- sink(3)
 * Distances: 0->1 = 10, 0->2 = 20, 0->3 = 30, 1->2 = 10, 1->3 = 20, 2->3 = 10
 * TW: c1 = [0, 15],  c2 = [0, 35], budget = 100
 *
 * If we visit c2 first: arrive c1 = 20 + 10 = 30 > closing(15) → infeasible
 * If we visit c1 first: arrive c1 = 10 ≤ 15 ✓; depart c1 = 10;
 *                        arrive c2 = 10 + 10 = 20 ≤ 35 ✓ → feasible
 * So {c1, c2} is NOT a conflict (there exists a feasible ordering).
 *
 * Now make c1 = [0, 5]: arrive c1 = 10 > 5 regardless → c1 alone infeasible
 * (arrival from depot > closing).  Use that for singleton detection.
 */
TOPTWProblem make_tight_pair() {
    TOPTWProblem p("tight_pair", 1, 100.0);
    using N = Node;
    p.add_node(N{0,  0.0, 0.0, 0.0, 0.0, {0.0, 1000.0}});  // source depot
    p.add_node(N{1, 10.0, 0.0, 1.0, 0.0, {0.0, 15.0}});    // c1: closes at 15
    p.add_node(N{2, 20.0, 0.0, 1.0, 0.0, {0.0, 35.0}});    // c2: closes at 35
    p.add_node(N{3, 30.0, 0.0, 0.0, 0.0, {0.0, 1000.0}});  // sink depot
    p.finalize();
    p.preprocessing();
    return p;
}

/**
 * Mutual conflict: c1 and c2 are mutually infeasible.
 *
 * Layout: depot(0) -- c1(1) -- c2(2) -- sink(3)
 * c1 TW = [0, 12]:  can arrive from depot (dist=10) at 10 ≤ 12 ✓
 *                   but if c2 is before c1: arrive c1 = 20+10=30 > 12 ✗
 * c2 TW = [0, 12]:  can arrive from depot (dist=20) at 20 > 12 ✗ alone already!
 *
 * Actually let's use a cleaner setup. We want both orderings to fail:
 * c1 at x=5: dist(depot,c1)=5, dist(c2,c1)=15
 * c2 at x=20: dist(depot,c2)=20, dist(c1,c2)=15
 * c1 TW=[0,8], c2 TW=[0,8]:
 *   visit c1 first: arr_c1=5 ≤ 8 ✓; dep_c1=5; arr_c2=5+15=20 > 8 ✗
 *   visit c2 first: arr_c2=20 > 8 ✗ → also infeasible
 * → {c1, c2} IS a conflict.
 */
TOPTWProblem make_mutual_conflict() {
    TOPTWProblem p("mutual_conflict", 1, 100.0);
    using N = Node;
    p.add_node(N{0,  0.0, 0.0, 0.0, 0.0, {0.0, 1000.0}}); // source depot
    p.add_node(N{1,  5.0, 0.0, 1.0, 0.0, {0.0, 8.0}});    // c1 TW closes at 8
    p.add_node(N{2, 20.0, 0.0, 1.0, 0.0, {0.0, 8.0}});    // c2 TW closes at 8
    p.add_node(N{3, 25.0, 0.0, 0.0, 0.0, {0.0, 1000.0}}); // sink depot
    p.finalize();
    p.preprocessing();
    return p;
}

} // anonymous namespace

// Feasible set → no conflict reported.
TEST(TWXplainer, FeasibleSetNoConflict) {
    auto problem = make_xplainer_problem();
    TWXplainer xp(problem);

    std::vector<NodeId>      clients{1, 2, 3};
    std::vector<TwConflict>  conflicts;
    bool found = xp.extract_conflict(clients, conflicts);
    EXPECT_FALSE(found);
    EXPECT_TRUE(conflicts.empty());
}

// Empty set → no conflict.
TEST(TWXplainer, EmptySetNoConflict) {
    auto problem = make_xplainer_problem();
    TWXplainer xp(problem);

    std::vector<NodeId>     clients;
    std::vector<TwConflict> conflicts;
    EXPECT_FALSE(xp.extract_conflict(clients, conflicts));
}

// {c1, c2} is a genuine mutual conflict (both orderings violate TW).
TEST(TWXplainer, DetectsMinimalConflict) {
    auto problem = make_mutual_conflict();
    TWXplainer xp(problem);

    std::vector<NodeId>     clients{1, 2};
    std::vector<TwConflict> conflicts;
    bool found = xp.extract_conflict(clients, conflicts);
    EXPECT_TRUE(found);
    ASSERT_FALSE(conflicts.empty());

    // The conflict scope must contain both clients
    const auto& cft = conflicts[0];
    EXPECT_EQ(cft.size(), 2u);
    EXPECT_NE(std::find(cft.begin(), cft.end(), NodeId(1)), cft.end());
    EXPECT_NE(std::find(cft.begin(), cft.end(), NodeId(2)), cft.end());
}

// multi_conflicts=false stops after the first conflict.
TEST(TWXplainer, SingleConflictModeStopsEarly) {
    auto problem = make_mutual_conflict();
    TWXplainer xp(problem);

    std::vector<NodeId>     clients{1, 2};
    std::vector<TwConflict> conflicts;
    bool found = xp.extract_conflict(clients, conflicts, /*multi=*/false);
    EXPECT_TRUE(found);
    EXPECT_EQ(conflicts.size(), 1u);
}

// Feasible pair (c1 can be served before c2) → no conflict.
TEST(TWXplainer, FeasiblePairNoConflict) {
    auto problem = make_tight_pair();
    TWXplainer xp(problem);

    // c1 TW=[0,15]: arrival from depot = 10 ≤ 15 ✓
    // c2 TW=[0,35]: arrival from depot = 20 ≤ 35 ✓
    // visiting c1 then c2: arr_c2 = 10+10 = 20 ≤ 35 ✓ → feasible ordering exists
    std::vector<NodeId>     clients{1, 2};
    std::vector<TwConflict> conflicts;
    bool found = xp.extract_conflict(clients, conflicts);
    EXPECT_FALSE(found);
}
