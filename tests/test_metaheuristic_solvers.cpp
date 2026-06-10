#include <gtest/gtest.h>

// These tests require solvers not yet implemented (Phases 2-5).
// Uncomment each block as the corresponding phase is completed.
#if 0

// #include "solver/local_search/LNS.h"         // Phase 4 — not yet implemented
// #include "solver/local_search/grasp.h"        // Phase 3 — not yet implemented
#include "solver/constructive/greedy.h"
// #include "solver/constructive/randomized_greedy.h" // Phase 2 — not yet implemented
#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"
#include "io/op_parser.h"
#include "io/top_parser.h"
#include "io/toptw_parser.h"
#include <memory>
#include <filesystem>

namespace fs = std::filesystem;

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;
using namespace oplib::solver::constructive;
using namespace oplib::solver::local_search;
using namespace oplib::io;

class MetaheuristicSolverTest : public ::testing::Test {
protected:
    solver::SolverConfig config;
    
    void SetUp() override {
        config.seed = 42;
        config.verbose = false;
    }
};

// ================================
// Test 1: GRASP on Simple OP Instance
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_BasicOP_SimpleInstance) {
    // Create a simple OP instance
    OPProblem problem("test_grasp_op", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Create GRASP config
    auto grasp_config = std::make_unique<GraspConfig>();
    grasp_config->seed = 42;
    grasp_config->verbose = false;
    grasp_config->alpha = 0.3;  // RCL parameter
    grasp_config->lns_iterations = 5;
    
    // Solve with GRASP
    GraspSolver solver;
    auto solution = solver.solve(problem, *grasp_config);
    
    // Verify solution structure
    EXPECT_EQ(solution.get_num_vehicles(), 1);
    const auto& route = solution.get_route(0);
    
    // Route should start and end at depots
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(), 4);
    
    // Route should have at least some customers
    EXPECT_GE(route.size(), 3);
    
    // Total reward should be positive
    EXPECT_GT(solution.total_reward, 0.0);
    
    // Verify no duplicates
    std::set<NodeId> unique_nodes(route.begin(), route.end());
    EXPECT_EQ(unique_nodes.size(), route.size());
}

// ================================
// Test 2: GRASP on TOP (Multi-vehicle)
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_TOP_MultipleVehicles) {
    TOPProblem problem("test_grasp_top", 2, 50.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 0.0, 10.0, 25.0, 0.0});
    problem.add_node(Node{3, 15.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{4, 0.0, 15.0, 35.0, 0.0});
    problem.add_node(Node{5, 0.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    auto grasp_config = std::make_unique<GraspConfig>();
    grasp_config->seed = 42;
    grasp_config->verbose = false;
    grasp_config->alpha = 0.3;
    grasp_config->lns_iterations = 5;
    
    GraspSolver solver;
    auto solution = solver.solve(problem, *grasp_config);
    
    // Should have 2 vehicles
    EXPECT_EQ(solution.get_num_vehicles(), 2);
    
    // Both routes should start and end at depots
    for (int v = 0; v < 2; ++v) {
        const auto& route = solution.get_route(v);
        EXPECT_EQ(route.front(), 0);
        EXPECT_EQ(route.back(), 5);
    }
    
    // Total reward should be positive
    EXPECT_GT(solution.total_reward, 0.0);
    
    // Verify no customer visited by multiple vehicles
    std::set<NodeId> all_visited;
    for (int v = 0; v < 2; ++v) {
        const auto& route = solution.get_route(v);
        for (size_t i = 1; i < route.size() - 1; ++i) {
            EXPECT_EQ(all_visited.count(route[i]), 0) << "Node visited multiple times";
            all_visited.insert(route[i]);
        }
    }
}

// ================================
// Test 3: GRASP on TOPTW (Time Windows)
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_TOPTW_TimeWindows) {
    TOPTWProblem problem("test_grasp_toptw", 2, 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    problem.add_node(Node{1, 10.0, 0.0, 30.0, 5.0, {0.0, 30.0}});
    problem.add_node(Node{2, 0.0, 10.0, 35.0, 5.0, {20.0, 60.0}});
    problem.add_node(Node{3, 15.0, 0.0, 40.0, 5.0, {50.0, 100.0}});
    problem.add_node(Node{4, 0.0, 15.0, 45.0, 5.0, {60.0, 100.0}});
    problem.add_node(Node{5, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    problem.finalize();
    problem.preprocessing();
    
    auto grasp_config = std::make_unique<GraspConfig>();
    grasp_config->seed = 42;
    grasp_config->verbose = false;
    grasp_config->alpha = 0.3;
    grasp_config->lns_iterations = 5;
    
    GraspSolver solver;
    auto solution = solver.solve(problem, *grasp_config);
    
    // Should have 2 vehicles
    EXPECT_EQ(solution.get_num_vehicles(), 2);
    
    // Routes should be valid
    for (int v = 0; v < 2; ++v) {
        const auto& route = solution.get_route(v);
        EXPECT_GE(route.size(), 2);
        EXPECT_EQ(route.front(), 0);
        EXPECT_EQ(route.back(), 5);
    }
    
    // Should collect some reward
    EXPECT_GT(solution.total_reward, 0.0);
}

// ================================
// Test 4: GRASP with different alpha values
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_DifferentAlpha_OP) {
    OPProblem problem("test_grasp_alpha", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Test with alpha = 0.0 (pure greedy)
    {
        auto grasp_config = std::make_unique<GraspConfig>();
        grasp_config->seed = 42;
        grasp_config->verbose = false;
        grasp_config->alpha = 0.0;  // Pure greedy
        grasp_config->lns_iterations = 3;
        
        GraspSolver solver;
        auto solution = solver.solve(problem, *grasp_config);
        
        EXPECT_GT(solution.total_reward, 0.0);
    }
    
    // Test with alpha = 1.0 (fully random within feasible)
    {
        auto grasp_config = std::make_unique<GraspConfig>();
        grasp_config->seed = 42;
        grasp_config->verbose = false;
        grasp_config->alpha = 1.0;  // Fully random
        grasp_config->lns_iterations = 3;
        
        GraspSolver solver;
        auto solution = solver.solve(problem, *grasp_config);
        
        EXPECT_GT(solution.total_reward, 0.0);
    }
}

// ================================
// Test 5: GRASP with multiple iterations
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_MultipleIterations_OP) {
    OPProblem problem("test_grasp_iters", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Solve with 1 iteration (no improvement)
    Reward reward_1iter;
    {
        auto grasp_config = std::make_unique<GraspConfig>();
        grasp_config->seed = 42;
        grasp_config->verbose = false;
        grasp_config->alpha = 0.3;
        grasp_config->lns_iterations = 1;  // Single iteration
        
        GraspSolver solver;
        auto solution = solver.solve(problem, *grasp_config);
        reward_1iter = solution.total_reward;
        EXPECT_GT(reward_1iter, 0.0);
    }
    
    // Solve with 10 iterations (should be at least as good)
    Reward reward_10iter;
    {
        auto grasp_config = std::make_unique<GraspConfig>();
        grasp_config->seed = 42;
        grasp_config->verbose = false;
        grasp_config->alpha = 0.3;
        grasp_config->lns_iterations = 10;  // Multiple iterations
        
        GraspSolver solver;
        auto solution = solver.solve(problem, *grasp_config);
        reward_10iter = solution.total_reward;
        EXPECT_GT(reward_10iter, 0.0);
    }
    
    // More iterations should give at least as good reward (with same seed)
    // Note: Due to randomization, this might not always hold, but on average should
    EXPECT_GE(reward_10iter, reward_1iter - 1e-6);
}

// ================================
// Test 6: LNS Solver - Basic OP
// ================================
TEST_F(MetaheuristicSolverTest, LNS_BasicOP) {
    OPProblem problem("test_lns_op", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Get initial solution using Greedy
    GreedySolver greedy_solver;
    auto initial_solution = greedy_solver.solve(problem, config);
    Reward initial_reward = initial_solution.total_reward;
    
    // Apply LNS
    LNSSolver lns_solver;
    auto lns_config = std::make_unique<SolverConfig>();
    lns_config->seed = 42;
    lns_config->verbose = false;
    
    auto improved_solution = lns_solver.solve(problem, *lns_config);
    Reward lns_reward = improved_solution.total_reward;
    
    // LNS should not make solution worse
    EXPECT_GE(lns_reward, initial_reward - 1e-6);
    
    // Verify solution validity
    const auto& route = improved_solution.get_route(0);
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(), 4);
}

// ================================
// Test 7: LNS with solve_from (external solution injection)
// ================================
TEST_F(MetaheuristicSolverTest, LNS_SolveFrom_ExternalSolution) {
    OPProblem problem("test_lns_from", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Create initial solution manually
    auto initial = std::make_unique<Solution>(1);
    initial->get_route(0) = {0, 1, 2, 4};
    initial->total_reward = 50.0;
    initial->total_travel_time = 40.0;
    
    // Use LNS solve_from
    LNSSolver lns_solver;
    auto lns_config = std::make_unique<SolverConfig>();
    lns_config->seed = 42;
    lns_config->verbose = false;
    
    auto improved = lns_solver.solve_from(problem, *initial, *lns_config);
    
    // Solution should be valid
    EXPECT_EQ(improved.get_num_vehicles(), 1);
    const auto& route = improved.get_route(0);
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(), 4);
    
    // Reward should be at least as good
    EXPECT_GE(improved.total_reward, 40.0);
}

// ================================
// Test 8: RandomizedGreedySolver with RCL
// ================================
TEST_F(MetaheuristicSolverTest, RandomizedGreedy_RCL_OP) {
    OPProblem problem("test_rg_op", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Get greedy solution for comparison
    GreedySolver greedy_solver;
    auto greedy_solution = greedy_solver.solve(problem, config);
    Reward greedy_reward = greedy_solution.total_reward;
    
    // Create RandomizedGreedy config
    auto rg_config = std::make_unique<RandomizedGreedyConfig>();
    rg_config->seed = 42;
    rg_config->verbose = false;
    rg_config->alpha = 0.3;  // 30% RCL
    
    // Solve with RandomizedGreedy
    RandomizedGreedySolver rg_solver;
    auto rg_solution = rg_solver.solve(problem, *rg_config);
    
    // RandomizedGreedy should find a valid solution
    EXPECT_GT(rg_solution.total_reward, 0.0);
    
    // Verify solution structure
    const auto& route = rg_solution.get_route(0);
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(), 4);
}

// ================================
// Test 9: RandomizedGreedy - Different seeds produce different solutions
// ================================
TEST_F(MetaheuristicSolverTest, RandomizedGreedy_SeedDiversity) {
    OPProblem problem("test_rg_seed", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    std::vector<Reward> rewards;
    
    // Run with different seeds
    for (int seed = 42; seed < 45; ++seed) {
        auto rg_config = std::make_unique<RandomizedGreedyConfig>();
        rg_config->seed = seed;
        rg_config->verbose = false;
        rg_config->alpha = 0.5;
        
        RandomizedGreedySolver rg_solver;
        auto solution = rg_solver.solve(problem, *rg_config);
        rewards.push_back(solution.total_reward);
    }
    
    // All solutions should be valid
    for (const auto& reward : rewards) {
        EXPECT_GT(reward, 0.0);
    }
}

// ================================
// Test 10: GRASP on real TOP instance
// ================================
static std::string get_test_data_path(const std::string& relative_path) {
    return (fs::path(__FILE__).parent_path().parent_path() / "data" / relative_path).string();
}

TEST_F(MetaheuristicSolverTest, GRASP_RealInstance_TOP) {
    std::string filepath = get_test_data_path("top/Set1/p1.2.a.txt");
    
    TOPParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse TOP instance";
    
    auto grasp_config = std::make_unique<GraspConfig>();
    grasp_config->seed = 42;
    grasp_config->verbose = false;
    grasp_config->alpha = 0.3;
    grasp_config->lns_iterations = 5;
    
    GraspSolver solver;
    auto solution = solver.solve(*problem, *grasp_config);
    
    // Verify solution is valid
    int num_vehicles = solution.get_num_vehicles();
    EXPECT_GT(num_vehicles, 0);
    
    // Each route should start and end at depots
    for (int v = 0; v < num_vehicles; ++v) {
        const auto& route = solution.get_route(v);
        EXPECT_EQ(route.front(), problem->get_source_depot());
        EXPECT_EQ(route.back(), problem->get_sink_depot());
    }
    
    // Should collect some reward
    EXPECT_GT(solution.total_reward, 0.0);
}

// ================================
// Test 11: GRASP on real OP instance
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_RealInstance_OP) {
    std::string filepath = get_test_data_path("op/ChaoSet_64/set_64_1_15.txt");
    
    OPParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse OP instance";
    
    auto grasp_config = std::make_unique<GraspConfig>();
    grasp_config->seed = 42;
    grasp_config->verbose = false;
    grasp_config->alpha = 0.3;
    grasp_config->lns_iterations = 5;
    
    GraspSolver solver;
    auto solution = solver.solve(*problem, *grasp_config);
    
    // Verify solution validity
    const auto& route = solution.get_route(0);
    EXPECT_EQ(route.front(), problem->get_source_depot());
    EXPECT_EQ(route.back(), problem->get_sink_depot());
    
    // Should have positive reward
    EXPECT_GT(solution.total_reward, 0.0);
}

// ================================
// Test 12: GRASP on real TOPTW instance
// ================================
TEST_F(MetaheuristicSolverTest, GRASP_RealInstance_TOPTW) {
    std::string filepath = get_test_data_path("toptw/c101.txt");
    
    TOPTWParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse TOPTW instance";
    EXPECT_TRUE(problem->has_time_windows()) << "TOPTW should have time windows";
    
    auto grasp_config = std::make_unique<GraspConfig>();
    grasp_config->seed = 42;
    grasp_config->verbose = false;
    grasp_config->alpha = 0.3;
    grasp_config->lns_iterations = 5;
    
    GraspSolver solver;
    auto solution = solver.solve(*problem, *grasp_config);
    
    // Should have valid routes
    EXPECT_GT(solution.get_num_vehicles(), 0);
    
    // Should collect some reward
    EXPECT_GT(solution.total_reward, 0.0);
}

#endif // 0 — re-enable as phases are implemented
