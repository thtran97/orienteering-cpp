#include <gtest/gtest.h>

#include "model/variants/toptw.h"
#include "solver/metaheuristic/kb_ils.h"
#include "solver/metaheuristic/ils09.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver::metaheuristic;
using Node = oplib::model::Node;

namespace {

/**
 * Small TOPTW instance with 4 customers and 2 vehicles, budget=50.
 * Layout on x-axis, Euclidean distance. Wide time windows so that the
 * planner can serve all customers; tests solver correctness not performance.
 */
TOPTWProblem make_small_toptw() {
    TOPTWProblem p("small_toptw_kb", 2, 50.0);
    using N = Node;
    p.add_node(N{0,  0.0, 0.0,  0.0, 0.0, {0.0, 1000.0}}); // source
    p.add_node(N{1, 10.0, 0.0, 10.0, 0.0, {0.0, 50.0}});   // c1
    p.add_node(N{2, 15.0, 0.0, 15.0, 0.0, {0.0, 50.0}});   // c2
    p.add_node(N{3,  5.0, 5.0, 20.0, 0.0, {0.0, 50.0}});   // c3
    p.add_node(N{4, 12.0, 8.0, 12.0, 0.0, {0.0, 50.0}});   // c4
    p.add_node(N{5,  0.0, 0.0,  0.0, 0.0, {0.0, 1000.0}}); // sink
    p.finalize();
    p.preprocessing();
    return p;
}

} // anonymous namespace

// KBILSSolver produces a non-empty solution.
TEST(KBILSSolver, ProducesNonEmptySolution) {
    auto problem = make_small_toptw();
    KBILSSolver solver;
    KBILSSolverConfig cfg;
    cfg.max_iterations   = 50;
    cfg.max_cpu_time     = 5.0;
    cfg.seed             = 42;
    cfg.learn_iterations = 10;
    cfg.conflict_max_size = 3;

    auto sol = solver.solve(problem, cfg);
    EXPECT_GT(sol.total_reward, 0.0);
    EXPECT_EQ(sol.get_num_vehicles(), 2);
    EXPECT_EQ(sol.get_route(0).front(), problem.get_source_depot());
    EXPECT_EQ(sol.get_route(0).back(),  problem.get_sink_depot());
}

// KBILSSolver reward >= ILS09 reward on the same problem/seed (KB should not hurt).
TEST(KBILSSolver, RewardNotWorseThanILS09) {
    auto problem = make_small_toptw();

    ILS09SolverConfig ils_cfg;
    ils_cfg.max_iterations = 200;
    ils_cfg.max_cpu_time   = 10.0;
    ils_cfg.seed           = 42;
    ILS09Solver ils;
    auto ils_sol = ils.solve(problem, ils_cfg);

    KBILSSolverConfig kb_cfg;
    kb_cfg.max_iterations    = 200;
    kb_cfg.max_cpu_time      = 10.0;
    kb_cfg.seed              = 42;
    kb_cfg.learn_iterations  = 50;
    kb_cfg.conflict_max_size = 3;
    KBILSSolver kbsolver;
    auto kb_sol = kbsolver.solve(problem, kb_cfg);

    // KB solver should reach at least the ILS09 reward (same iterations, same seed).
    EXPECT_GE(kb_sol.total_reward, ils_sol.total_reward - 1e-6)
        << "KBILS reward=" << kb_sol.total_reward
        << " ILS09 reward=" << ils_sol.total_reward;
}

// KBILSSolver via the base SolverConfig dispatch.
TEST(KBILSSolver, BaseConfigDispatch) {
    auto problem = make_small_toptw();
    KBILSSolver solver;
    oplib::solver::SolverConfig cfg;
    cfg.max_iterations = 20;
    cfg.max_cpu_time   = 2.0;
    cfg.seed           = 7;

    auto sol = solver.solve(problem, cfg);
    EXPECT_GE(sol.total_reward, 0.0);
}

// Routes start at source and end at sink for every vehicle.
TEST(KBILSSolver, RoutesWellFormed) {
    auto problem = make_small_toptw();
    KBILSSolver solver;
    KBILSSolverConfig cfg;
    cfg.max_iterations = 30;
    cfg.seed = 123;

    auto sol = solver.solve(problem, cfg);
    for (int v = 0; v < sol.get_num_vehicles(); ++v) {
        const auto& route = sol.get_route(v);
        ASSERT_GE(route.size(), 2u);
        EXPECT_EQ(route.front(), problem.get_source_depot());
        EXPECT_EQ(route.back(),  problem.get_sink_depot());
    }
}
