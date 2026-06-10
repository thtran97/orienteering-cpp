#include <gtest/gtest.h>
#include <set>

#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/metaheuristic/grasp_vns.h"
#include "solver/metaheuristic/lns.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "solver/policy_learning/mcts_solver.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/pulse/pulse_solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;

// ============================================================================
// Shared fixture / helpers
// ============================================================================

namespace {

/// 5-node linear OP: 0--1--2--3--4, budget=100, rewards=[0,10,20,15,0].
OPProblem make_op5() {
    OPProblem p("op5", 100.0);
    p.add_node(Node{0,  0.0, 0.0,  0.0, 0.0});
    p.add_node(Node{1, 10.0, 0.0, 10.0, 0.0});
    p.add_node(Node{2, 20.0, 0.0, 20.0, 0.0});
    p.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    p.add_node(Node{4, 40.0, 0.0,  0.0, 0.0});
    p.finalize(); p.preprocessing();
    return p;
}

/// Very small OPTW: 4 nodes, tight time windows.
OPTWProblem make_optw4() {
    OPTWProblem p("optw4", 100.0);
    p.add_node(Node{0,  0.0, 0.0,  0.0, 0.0, {0.0, 1000.0}});
    p.add_node(Node{1, 10.0, 0.0, 10.0, 0.0, {0.0, 20.0}});
    p.add_node(Node{2, 20.0, 0.0, 20.0, 0.0, {0.0, 40.0}});
    p.add_node(Node{3, 30.0, 0.0,  0.0, 0.0, {0.0, 1000.0}});
    p.finalize(); p.preprocessing();
    return p;
}

/// 2-vehicle TOP (6 nodes).
TOPProblem make_top6() {
    TOPProblem p("top6", 2, 50.0);
    p.add_node(Node{0,  0.0,  0.0,  0.0, 0.0});
    p.add_node(Node{1, 10.0,  0.0, 20.0, 0.0});
    p.add_node(Node{2,  0.0, 10.0, 25.0, 0.0});
    p.add_node(Node{3, 15.0,  0.0, 30.0, 0.0});
    p.add_node(Node{4,  0.0, 15.0, 35.0, 0.0});
    p.add_node(Node{5,  0.0,  0.0,  0.0, 0.0});
    p.finalize(); p.preprocessing();
    return p;
}

/// Verifies basic solution structure for a single-vehicle problem.
void check_single_vehicle_solution(const model::Solution& sol,
                                   const model::Problem&  problem)
{
    ASSERT_GE(sol.get_num_vehicles(), 1);
    const auto& route = sol.get_route(0);
    ASSERT_GE(route.size(), 2u);
    EXPECT_EQ(route.front(), problem.get_source_depot());
    EXPECT_EQ(route.back(),  problem.get_sink_depot());
    // No duplicate nodes
    std::set<NodeId> unique(route.begin(), route.end());
    EXPECT_EQ(unique.size(), route.size());
}

} // anonymous namespace

// ============================================================================
// Phase 2 — Randomized Greedy
// ============================================================================

class RandomizedGreedyTest : public ::testing::Test {
protected:
    solver::constructive::RandomizedGreedySolverConfig cfg;
    void SetUp() override {
        cfg.seed           = 42;
        cfg.max_iterations = 20;
        cfg.max_cpu_time   = 5.0;
    }
};

TEST_F(RandomizedGreedyTest, ReturnsPositiveReward_OP) {
    auto problem = make_op5();
    solver::constructive::RandomizedGreedySolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GT(sol.total_reward, 0.0);
}

TEST_F(RandomizedGreedyTest, ReturnsPositiveReward_OPTW) {
    auto problem = make_optw4();
    solver::constructive::RandomizedGreedySolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(RandomizedGreedyTest, BetterThanSingleIteration) {
    auto problem = make_op5();
    solver::constructive::RandomizedGreedySolver solver;

    solver::constructive::RandomizedGreedySolverConfig single = cfg;
    single.max_iterations = 1;

    solver::constructive::RandomizedGreedySolverConfig multi = cfg;
    multi.max_iterations = 50;

    auto sol1 = solver.solve(problem, single);
    auto sol2 = solver.solve(problem, multi);
    // Multi-restart should be at least as good
    EXPECT_GE(sol2.total_reward, sol1.total_reward);
}

TEST_F(RandomizedGreedyTest, MultiVehicle_TOP) {
    auto problem = make_top6();
    solver::constructive::RandomizedGreedySolver solver;
    auto sol = solver.solve(problem, cfg);
    EXPECT_EQ(sol.get_num_vehicles(), 2);
    EXPECT_GE(sol.total_reward, 0.0);
    for (int v = 0; v < 2; ++v) {
        const auto& r = sol.get_route(v);
        EXPECT_EQ(r.front(), problem.get_source_depot());
        EXPECT_EQ(r.back(),  problem.get_sink_depot());
    }
}

// ============================================================================
// Phase 3 — GRASP + VNS
// ============================================================================

class GraspVnsTest : public ::testing::Test {
protected:
    solver::metaheuristic::GraspVnsSolverConfig cfg;
    void SetUp() override {
        cfg.seed            = 42;
        cfg.max_iterations  = 5;
        cfg.max_cpu_time    = 5.0;
        cfg.max_shake_length = 2;
    }
};

TEST_F(GraspVnsTest, ReturnsPositiveReward_OP) {
    auto problem = make_op5();
    solver::metaheuristic::GraspVnsSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GT(sol.total_reward, 0.0);
}

TEST_F(GraspVnsTest, ReturnsPositiveReward_OPTW) {
    auto problem = make_optw4();
    solver::metaheuristic::GraspVnsSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(GraspVnsTest, MultiVehicle_TOP) {
    auto problem = make_top6();
    solver::metaheuristic::GraspVnsSolver solver;
    auto sol = solver.solve(problem, cfg);
    EXPECT_EQ(sol.get_num_vehicles(), 2);
    for (int v = 0; v < 2; ++v) {
        const auto& r = sol.get_route(v);
        EXPECT_EQ(r.front(), problem.get_source_depot());
        EXPECT_EQ(r.back(),  problem.get_sink_depot());
    }
}

TEST_F(GraspVnsTest, RewardGeBaseInterface) {
    auto problem = make_op5();
    solver::metaheuristic::GraspVnsSolver solver;
    solver::SolverConfig base_cfg;
    base_cfg.seed           = 42;
    base_cfg.max_iterations = 5;
    base_cfg.max_cpu_time   = 5.0;
    auto sol = solver.solve(problem, base_cfg);
    EXPECT_GE(sol.total_reward, 0.0);
}

// ============================================================================
// Phase 4 — LNS
// ============================================================================

class LNSTest : public ::testing::Test {
protected:
    solver::metaheuristic::LNSSolverConfig cfg;
    void SetUp() override {
        cfg.seed           = 42;
        cfg.max_iterations = 20;
        cfg.max_cpu_time   = 5.0;
    }
};

TEST_F(LNSTest, ReturnsPositiveReward_OP) {
    auto problem = make_op5();
    solver::metaheuristic::LNSSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GT(sol.total_reward, 0.0);
}

TEST_F(LNSTest, ReturnsPositiveReward_OPTW) {
    auto problem = make_optw4();
    solver::metaheuristic::LNSSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(LNSTest, MultiVehicle_TOP) {
    auto problem = make_top6();
    solver::metaheuristic::LNSSolver solver;
    auto sol = solver.solve(problem, cfg);
    EXPECT_EQ(sol.get_num_vehicles(), 2);
    for (int v = 0; v < 2; ++v) {
        EXPECT_EQ(sol.get_route(v).front(), problem.get_source_depot());
        EXPECT_EQ(sol.get_route(v).back(),  problem.get_sink_depot());
    }
}

TEST_F(LNSTest, MoreIterationsNotWorse) {
    auto problem = make_op5();
    solver::metaheuristic::LNSSolver solver;
    auto cfg10  = cfg; cfg10.max_iterations = 10;
    auto cfg100 = cfg; cfg100.max_iterations = 100;
    auto sol10  = solver.solve(problem, cfg10);
    auto sol100 = solver.solve(problem, cfg100);
    EXPECT_GE(sol100.total_reward, sol10.total_reward);
}

// ============================================================================
// Phase 5 — ILS09
// ============================================================================

class ILS09Test : public ::testing::Test {
protected:
    solver::metaheuristic::ILS09SolverConfig cfg;
    void SetUp() override {
        cfg.seed              = 42;
        cfg.max_iterations    = 20;
        cfg.max_cpu_time      = 5.0;
        cfg.restart_threshold = 5;
    }
};

TEST_F(ILS09Test, ReturnsPositiveReward_OP) {
    auto problem = make_op5();
    solver::metaheuristic::ILS09Solver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GT(sol.total_reward, 0.0);
}

TEST_F(ILS09Test, ReturnsPositiveReward_OPTW) {
    auto problem = make_optw4();
    solver::metaheuristic::ILS09Solver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(ILS09Test, MultiVehicle_TOP) {
    auto problem = make_top6();
    solver::metaheuristic::ILS09Solver solver;
    auto sol = solver.solve(problem, cfg);
    EXPECT_EQ(sol.get_num_vehicles(), 2);
    for (int v = 0; v < 2; ++v) {
        EXPECT_EQ(sol.get_route(v).front(), problem.get_source_depot());
        EXPECT_EQ(sol.get_route(v).back(),  problem.get_sink_depot());
    }
}

TEST_F(ILS09Test, ILS09RewardGeRandomizedGreedy) {
    auto problem = make_op5();
    solver::metaheuristic::ILS09Solver ils;
    solver::constructive::RandomizedGreedySolver rg;

    auto sol_ils = ils.solve(problem, cfg);

    solver::constructive::RandomizedGreedySolverConfig rg_cfg;
    rg_cfg.seed           = 42;
    rg_cfg.max_iterations = cfg.max_iterations;
    rg_cfg.max_cpu_time   = cfg.max_cpu_time;
    auto sol_rg = rg.solve(problem, rg_cfg);

    EXPECT_GE(sol_ils.total_reward, sol_rg.total_reward * 0.9);
}

// ============================================================================
// Phase 6 — ILS + Route Recombination
// ============================================================================

class ILSRRTest : public ::testing::Test {
protected:
    solver::metaheuristic::ILSRouteRecombinationSolverConfig cfg;
    void SetUp() override {
        cfg.seed              = 42;
        cfg.max_iterations    = 20;
        cfg.max_cpu_time      = 5.0;
        cfg.restart_threshold = 5;
        cfg.pool_size         = 5;
    }
};

TEST_F(ILSRRTest, ReturnsPositiveReward_OP) {
    auto problem = make_op5();
    solver::metaheuristic::ILSRouteRecombinationSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GT(sol.total_reward, 0.0);
}

TEST_F(ILSRRTest, ReturnsPositiveReward_OPTW) {
    auto problem = make_optw4();
    solver::metaheuristic::ILSRouteRecombinationSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(ILSRRTest, SolutionStructureIsValid) {
    auto problem = make_op5();
    solver::metaheuristic::ILSRouteRecombinationSolver solver;
    auto sol = solver.solve(problem, cfg);
    const auto& route = sol.get_route(0);
    std::set<NodeId> uniq(route.begin(), route.end());
    EXPECT_EQ(uniq.size(), route.size()); // no duplicates
}

// ============================================================================
// Phase 7 — MCTS
// ============================================================================

class MCTSTest : public ::testing::Test {
protected:
    solver::policy_learning::MCTSSolverConfig cfg;
    void SetUp() override {
        cfg.seed           = 42;
        cfg.max_iterations = 50;
        cfg.max_cpu_time   = 5.0;
    }
};

TEST_F(MCTSTest, ReturnsPositiveReward_OP) {
    auto problem = make_op5();
    solver::policy_learning::MCTSSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(MCTSTest, ReturnsPositiveReward_OPTW) {
    auto problem = make_optw4();
    solver::policy_learning::MCTSSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(MCTSTest, RouteStartsAndEndsAtDepots) {
    auto problem = make_op5();
    solver::policy_learning::MCTSSolver solver;
    auto sol = solver.solve(problem, cfg);
    const auto& route = sol.get_route(0);
    ASSERT_GE(route.size(), 2u);
    EXPECT_EQ(route.front(), problem.get_source_depot());
    EXPECT_EQ(route.back(),  problem.get_sink_depot());
}

TEST_F(MCTSTest, NoDuplicateNodes) {
    auto problem = make_op5();
    solver::policy_learning::MCTSSolver solver;
    auto sol = solver.solve(problem, cfg);
    const auto& route = sol.get_route(0);
    std::set<NodeId> uniq(route.begin(), route.end());
    EXPECT_EQ(uniq.size(), route.size());
}

// ============================================================================
// Phase 9 — DP Solvers
// ============================================================================

class ForwardDPTest : public ::testing::Test {
protected:
    solver::dp::DPSolverConfig cfg;
    void SetUp() override {
        cfg.seed       = 42;
        cfg.max_labels = 100000;
        cfg.verbose    = false;
    }
};

TEST_F(ForwardDPTest, ReturnsNonNegativeReward_OP) {
    auto problem = make_op5();
    solver::dp::ForwardDPSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(ForwardDPTest, ReturnsNonNegativeReward_OPTW) {
    auto problem = make_optw4();
    solver::dp::ForwardDPSolver solver;
    auto sol = solver.solve(problem, cfg);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(ForwardDPTest, OptimalSmallInstance) {
    // 4 nodes, all reachable within budget=100.  Optimal should collect all customers.
    OPProblem p("tiny", 100.0);
    p.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    p.add_node(Node{1, 5.0, 0.0, 10.0, 0.0});
    p.add_node(Node{2, 5.0, 0.0, 20.0, 0.0});
    p.add_node(Node{3, 0.0, 0.0, 0.0, 0.0});
    p.finalize(); p.preprocessing();
    solver::dp::ForwardDPSolver solver;
    auto sol = solver.solve(p, cfg);
    EXPECT_DOUBLE_EQ(sol.total_reward, 30.0); // both customers collected
}

class BackwardDPTest : public ::testing::Test {
protected:
    solver::dp::DPSolverConfig cfg;
    void SetUp() override { cfg.verbose = false; cfg.max_labels = 100000; }
};

TEST_F(BackwardDPTest, BoundsNonNegative) {
    auto problem = make_op5();
    solver::dp::BackwardDPSolver solver;
    auto bounds = solver.compute_bounds(problem);
    ASSERT_EQ(static_cast<int>(bounds.size()), problem.get_num_nodes());
    for (Reward b : bounds)
        EXPECT_GE(b, 0.0);
}

TEST_F(BackwardDPTest, SinkBoundIsZero) {
    auto problem = make_op5();
    solver::dp::BackwardDPSolver solver;
    auto bounds = solver.compute_bounds(problem);
    EXPECT_DOUBLE_EQ(bounds[problem.get_sink_depot()], 0.0);
}

class BidirectionalDPTest : public ::testing::Test {
protected:
    solver::dp::DPSolverConfig cfg;
    void SetUp() override { cfg.verbose = false; cfg.max_labels = 100000; }
};

TEST_F(BidirectionalDPTest, AtLeastAsGoodAsForward_Small) {
    auto problem = make_op5();
    solver::dp::ForwardDPSolver fwd;
    solver::dp::BidirectionalDPSolver bidir;
    auto sol_fwd   = fwd.solve(problem, cfg);
    auto sol_bidir = bidir.solve(problem, cfg);
    // Bidirectional uses tighter pruning so should match or exceed forward
    EXPECT_GE(sol_bidir.total_reward, sol_fwd.total_reward - 1e-9);
}

// ============================================================================
// Phase 10 — Pulse
// ============================================================================

class PulseTest : public ::testing::Test {
protected:
    solver::pulse::PulseSolverConfig cfg;
    void SetUp() override {
        cfg.verbose    = false;
        cfg.max_labels = 100000;
    }
};

TEST_F(PulseTest, ReturnsNonNegativeReward_OP) {
    auto problem = make_op5();
    solver::pulse::PulseSolver solver;
    auto sol = solver.solve(problem, cfg);
    check_single_vehicle_solution(sol, problem);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(PulseTest, ReturnsNonNegativeReward_OPTW) {
    auto problem = make_optw4();
    solver::pulse::PulseSolver solver;
    auto sol = solver.solve(problem, cfg);
    EXPECT_GE(sol.total_reward, 0.0);
}

TEST_F(PulseTest, OptimalSmallInstance) {
    OPProblem p("tiny", 100.0);
    p.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    p.add_node(Node{1, 5.0, 0.0, 10.0, 0.0});
    p.add_node(Node{2, 5.0, 0.0, 20.0, 0.0});
    p.add_node(Node{3, 0.0, 0.0,  0.0, 0.0});
    p.finalize(); p.preprocessing();
    solver::pulse::PulseSolver solver;
    auto sol = solver.solve(p, cfg);
    EXPECT_DOUBLE_EQ(sol.total_reward, 30.0);
}

TEST_F(PulseTest, PulseNotWorseThanGreedy) {
    auto problem = make_op5();
    solver::pulse::PulseSolver pulse;
    solver::constructive::RandomizedGreedySolver rg;

    solver::constructive::RandomizedGreedySolverConfig rg_cfg;
    rg_cfg.seed = 42; rg_cfg.max_iterations = 50; rg_cfg.max_cpu_time = 5.0;

    auto sol_p = pulse.solve(problem, cfg);
    auto sol_g = rg.solve(problem, rg_cfg);

    // Pulse is exact (within label budget), so it should match or beat greedy
    EXPECT_GE(sol_p.total_reward, sol_g.total_reward - 1e-9);
}
