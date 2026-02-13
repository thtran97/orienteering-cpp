#include <gtest/gtest.h>
#include "solver/constructive/greedy.h"
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
using namespace oplib::io;

class GreedySolverTest : public ::testing::Test {
protected:
    solver::SolverConfig config;
    
    void SetUp() override {
        config.seed = 42;
        config.verbose = false;
    }
};

// ================================
// Test 1: Basic OP (Orienteering Problem)
// ================================
TEST_F(GreedySolverTest, BasicOP_SimpleInstance) {
    // Create a simple OP instance with 4 nodes + 2 depots
    // Budget: 100
    OPProblem problem("test_op", 100.0);
    
    // Node 0: Source depot at (0, 0)
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    
    // Node 1: Customer at (10, 0) with reward 20
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    
    // Node 2: Customer at (20, 0) with reward 30
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    
    // Node 3: Customer at (30, 0) with reward 15
    problem.add_node(Node{3, 30.0, 0.0, 15.0, 0.0});
    
    // Node 4: Sink depot at (40, 0)
    problem.add_node(Node{4, 40.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    // Solve
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    // Verify solution structure
    EXPECT_EQ(solution.get_num_vehicles(), 1);
    const auto& route = solution.get_route(0);
    
    // Route should start and end at depots
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(), 4);
    
    // Route should have at least some customers
    EXPECT_GE(route.size(), 3);  // At least depots + 1 customer
    
    // Total reward should be positive
    EXPECT_GT(solution.total_reward, 0.0);
    
    // Verify all nodes in route are unique (no duplicates)
    std::set<NodeId> unique_nodes(route.begin(), route.end());
    EXPECT_EQ(unique_nodes.size(), route.size());
}

// ================================
// Test 2: OP with tight budget
// ================================
TEST_F(GreedySolverTest, OP_TightBudget) {
    // Create OP with very tight budget - can only visit one node
    OPProblem problem("test_op_tight", 25.0);  // Only enough for 0 -> 1 -> 2
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 100.0, 0.0});  // High reward, close
    problem.add_node(Node{2, 0.0, 0.0, 0.0, 0.0});     // Sink at origin
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    const auto& route = solution.get_route(0);
    
    // Should visit node 1
    EXPECT_EQ(route.size(), 3);  // 0 -> 1 -> 2
    EXPECT_EQ(solution.total_reward, 100.0);
}

// ================================
// Test 3: OPTW (OP with Time Windows)
// ================================
TEST_F(GreedySolverTest, OPTW_BasicTimeWindows) {
    // Create OPTW instance with time windows
    OPTWProblem problem("test_optw", 100.0);  // Budget = Tmax
    
    // Node 0: Source depot, TW [0, 100], service 0
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    // Node 1: Customer at (10, 0), TW [0, 50], reward 20, service 5
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 5.0, {0.0, 50.0}});
    
    // Node 2: Customer at (20, 0), TW [15, 60], reward 30, service 5
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 5.0, {15.0, 60.0}});
    
    // Node 3: Customer at (30, 0), TW [80, 100], reward 25, service 5
    // This one is hard to reach after visiting 1 and 2
    problem.add_node(Node{3, 30.0, 0.0, 25.0, 5.0, {80.0, 100.0}});
    
    // Node 4: Sink depot at origin, TW [0, 100]
    problem.add_node(Node{4, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    const auto& route = solution.get_route(0);
    
    // Should have valid route
    EXPECT_GE(route.size(), 3);
    EXPECT_EQ(route.front(), 0);
    EXPECT_EQ(route.back(), 4);
    
    // Should collect some reward
    EXPECT_GT(solution.total_reward, 0.0);
}

// ================================
// Test 4: OPTW with infeasible time windows
// ================================
TEST_F(GreedySolverTest, OPTW_InfeasibleTimeWindows) {
    // Create instance where no customers can be visited
    OPTWProblem problem("test_optw_infeas", 100.0);
    
    // Node 0: Source depot
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    // Node 1: Customer with impossible time window [0, 5]
    // Travel time from 0 to 1 is 50, so we arrive at time 50 > 5
    problem.add_node(Node{1, 50.0, 0.0, 100.0, 5.0, {0.0, 5.0}});
    
    // Node 2: Sink depot
    problem.add_node(Node{2, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    const auto& route = solution.get_route(0);
    
    // Should only have depots (no customers visited)
    EXPECT_EQ(route.size(), 2);  // Just 0 -> 2
    EXPECT_EQ(solution.total_reward, 0.0);
}

// ================================
// Test 5: TOP (Team Orienteering Problem)
// ================================
TEST_F(GreedySolverTest, TOP_MultipleVehicles) {
    // Create TOP with 2 vehicles
    TOPProblem problem("test_top", 2, 50.0);  // 2 vehicles, budget 50 each
    
    // Node 0: Source depot
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    
    // Node 1: Customer at (10, 0), reward 20
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    
    // Node 2: Customer at (0, 10), reward 25
    problem.add_node(Node{2, 0.0, 10.0, 25.0, 0.0});
    
    // Node 3: Customer at (15, 0), reward 30
    problem.add_node(Node{3, 15.0, 0.0, 30.0, 0.0});
    
    // Node 4: Customer at (0, 15), reward 35
    problem.add_node(Node{4, 0.0, 15.0, 35.0, 0.0});
    
    // Node 5: Sink depot
    problem.add_node(Node{5, 0.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    // Should have 2 vehicles
    EXPECT_EQ(solution.get_num_vehicles(), 2);
    
    // Both routes should start and end at depots
    for (int v = 0; v < 2; ++v) {
        const auto& route = solution.get_route(v);
        EXPECT_EQ(route.front(), 0);
        EXPECT_EQ(route.back(), 5);
    }
    
    // At least one vehicle should visit customers
    bool has_customers = false;
    for (int v = 0; v < 2; ++v) {
        if (solution.get_route(v).size() > 2) {
            has_customers = true;
        }
    }
    EXPECT_TRUE(has_customers);
    
    // Total reward should be positive
    EXPECT_GT(solution.total_reward, 0.0);
    
    // Verify no customer is visited by multiple vehicles
    std::set<NodeId> all_visited;
    for (int v = 0; v < 2; ++v) {
        const auto& route = solution.get_route(v);
        for (size_t i = 1; i < route.size() - 1; ++i) {  // Skip depots
            EXPECT_EQ(all_visited.count(route[i]), 0) << "Node " << route[i] << " visited multiple times";
            all_visited.insert(route[i]);
        }
    }
}

// ================================
// Test 6: TOPTW (Team OP with Time Windows)
// ================================
TEST_F(GreedySolverTest, TOPTW_MultipleVehiclesWithTimeWindows) {
    // Create TOPTW with 2 vehicles
    TOPTWProblem problem("test_toptw", 2, 100.0);  // 2 vehicles, Tmax 100
    
    // Node 0: Source depot
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    // Node 1: Early customer TW [0, 30]
    problem.add_node(Node{1, 10.0, 0.0, 30.0, 5.0, {0.0, 30.0}});
    
    // Node 2: Middle customer TW [20, 60]
    problem.add_node(Node{2, 0.0, 10.0, 35.0, 5.0, {20.0, 60.0}});
    
    // Node 3: Late customer TW [50, 100]
    problem.add_node(Node{3, 15.0, 0.0, 40.0, 5.0, {50.0, 100.0}});
    
    // Node 4: Another late customer TW [60, 100]
    problem.add_node(Node{4, 0.0, 15.0, 45.0, 5.0, {60.0, 100.0}});
    
    // Node 5: Sink depot
    problem.add_node(Node{5, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
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
// Test 7: TOP - Single vehicle case
// ================================
TEST_F(GreedySolverTest, TOP_SingleVehicle) {
    // TOP with 1 vehicle should behave like OP
    TOPProblem problem("test_top_single", 1, 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    problem.add_node(Node{2, 20.0, 0.0, 30.0, 0.0});
    problem.add_node(Node{3, 0.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    EXPECT_EQ(solution.get_num_vehicles(), 1);
    const auto& route = solution.get_route(0);
    EXPECT_GE(route.size(), 3);  // Should visit at least one customer
}

// ================================
// Test 8: Empty problem
// ================================
TEST_F(GreedySolverTest, EmptyProblem) {
    // Problem with only depots
    OPProblem problem("test_empty", 100.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});  // Source
    problem.add_node(Node{1, 0.0, 0.0, 0.0, 0.0});  // Sink
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    const auto& route = solution.get_route(0);
    EXPECT_EQ(route.size(), 2);  // Just depots
    EXPECT_EQ(solution.total_reward, 0.0);
}

// ================================
// Test 9: TOP - Verify vehicles used efficiently
// ================================
TEST_F(GreedySolverTest, TOP_VehicleDistribution) {
    // Create TOP where using both vehicles should yield better reward
    TOPProblem problem("test_top_distrib", 2, 30.0);  // Tight budget per vehicle
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    
    // Cluster 1: Left side
    problem.add_node(Node{1, -10.0, 0.0, 50.0, 0.0});
    problem.add_node(Node{2, -15.0, 0.0, 40.0, 0.0});
    
    // Cluster 2: Right side
    problem.add_node(Node{3, 10.0, 0.0, 50.0, 0.0});
    problem.add_node(Node{4, 15.0, 0.0, 40.0, 0.0});
    
    problem.add_node(Node{5, 0.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    // At least one vehicle should visit customers
    int vehicles_used = 0;
    for (int v = 0; v < 2; ++v) {
        if (solution.get_route(v).size() > 2) {
            vehicles_used++;
        }
    }
    EXPECT_GE(vehicles_used, 1);
}

// ================================
// Test 10: Greedy should prefer high reward/distance ratio
// ================================
TEST_F(GreedySolverTest, OP_GreedyBehavior) {
    // Create instance where greedy should pick efficient nodes
    OPProblem problem("test_greedy_behavior", 50.0);
    
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    
    // Node 1: Close, high reward (reward/distance = 100/5 = 20)
    problem.add_node(Node{1, 5.0, 0.0, 100.0, 0.0});
    
    // Node 2: Far, low reward (reward/distance = 10/40 = 0.25)
    problem.add_node(Node{2, 40.0, 0.0, 10.0, 0.0});
    
    problem.add_node(Node{3, 0.0, 0.0, 0.0, 0.0});
    
    problem.finalize();
    problem.preprocessing();
    
    GreedySolver solver;
    auto solution = solver.solve(problem, config);
    
    const auto& route = solution.get_route(0);
    
    // Should definitely visit node 1 (high efficiency)
    EXPECT_TRUE(std::find(route.begin(), route.end(), 1) != route.end());
    
    // Should collect at least node 1's reward
    EXPECT_GE(solution.total_reward, 100.0);
}

// ============================================================================
// Integration Tests with Real Instances from Data Folder
// ============================================================================

// Helper function to construct data file path (static to avoid conflicts)
static std::string get_greedy_test_data_path(const std::string& relative_path) {
    return (fs::path(__FILE__).parent_path().parent_path() / "data" / relative_path).string();
}

// ================================
// Test 11: Real OP instance - Tsiligirides Set
// ================================
TEST_F(GreedySolverTest, RealInstance_OP_Tsiligirides) {
    std::string filepath = get_greedy_test_data_path("op/Tsiligirides_1/tsiligirides_problem_1_budget_05.txt");
    
    OPParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse OP instance";
    EXPECT_GT(problem->get_num_nodes(), 2) << "Problem should have nodes beyond depots";
    
    GreedySolver solver;
    auto solution = solver.solve(*problem, config);
    
    // Verify solution structure
    EXPECT_EQ(solution.get_num_vehicles(), 1);
    const auto& route = solution.get_route(0);
    
    // Route should start and end at depots
    EXPECT_EQ(route.front(), problem->get_source_depot());
    EXPECT_EQ(route.back(), problem->get_sink_depot());
    
    // Should visit at least some customers (greedy should find something)
    EXPECT_GE(route.size(), 2);
    
    // Verify no duplicates in route
    std::set<NodeId> unique_nodes(route.begin(), route.end());
    EXPECT_EQ(unique_nodes.size(), route.size()) << "Route contains duplicate nodes";
    
    // Verify budget constraint if solution has customers
    if (route.size() > 2) {
        Time total_time = 0.0;
        for (size_t i = 0; i + 1 < route.size(); ++i) {
            total_time += problem->get_distance(route[i], route[i+1]);
        }
        
        auto* op_problem = dynamic_cast<OPProblem*>(problem.get());
        ASSERT_NE(op_problem, nullptr);
        EXPECT_LE(total_time, op_problem->get_budget() + 1e-6) << "Solution violates budget constraint";
        
        // Total reward should match sum of visited node rewards
        Reward expected_reward = 0.0;
        for (size_t i = 1; i < route.size() - 1; ++i) {  // Skip depots
            expected_reward += problem->get_reward(route[i]);
        }
        EXPECT_NEAR(solution.total_reward, expected_reward, 1e-6);
    }
}

// ================================
// Test 12: Real OP instance - Chao Set
// ================================
TEST_F(GreedySolverTest, RealInstance_OP_ChaoSet) {
    std::string filepath = get_greedy_test_data_path("op/ChaoSet_64/set_64_1_15.txt");
    
    OPParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse OP instance";
    
    GreedySolver solver;
    auto solution = solver.solve(*problem, config);
    
    const auto& route = solution.get_route(0);
    
    // Basic validity checks
    EXPECT_GE(route.size(), 2);
    EXPECT_EQ(route.front(), problem->get_source_depot());
    EXPECT_EQ(route.back(), problem->get_sink_depot());
    
    // Verify no node visited twice
    std::set<NodeId> visited;
    for (NodeId node : route) {
        EXPECT_EQ(visited.count(node), 0) << "Node " << node << " visited multiple times";
        visited.insert(node);
    }
}

// ================================
// Test 13: Real TOP instance - Multiple vehicles
// ================================
TEST_F(GreedySolverTest, RealInstance_TOP_Set1) {
    std::string filepath = get_greedy_test_data_path("top/Set1/p1.2.a.txt");
    
    TOPParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse TOP instance";
    
    auto* top_problem = dynamic_cast<TOPProblem*>(problem.get());
    ASSERT_NE(top_problem, nullptr);
    
    int num_vehicles = top_problem->get_num_vehicles();
    EXPECT_GT(num_vehicles, 1) << "TOP should have multiple vehicles";
    
    GreedySolver solver;
    auto solution = solver.solve(*problem, config);
    
    EXPECT_EQ(solution.get_num_vehicles(), num_vehicles);
    
    // Verify each route
    std::set<NodeId> all_visited;
    for (int v = 0; v < num_vehicles; ++v) {
        const auto& route = solution.get_route(v);
        
        // Each route should start and end at depots
        EXPECT_EQ(route.front(), problem->get_source_depot());
        EXPECT_EQ(route.back(), problem->get_sink_depot());
        
        // Check no customer visited by multiple vehicles
        for (size_t i = 1; i < route.size() - 1; ++i) {
            NodeId node = route[i];
            EXPECT_EQ(all_visited.count(node), 0) 
                << "Node " << node << " visited by multiple vehicles";
            all_visited.insert(node);
        }
        
        // Verify budget constraint for this vehicle
        if (route.size() > 2) {
            Time route_time = 0.0;
            for (size_t i = 0; i + 1 < route.size(); ++i) {
                route_time += problem->get_distance(route[i], route[i+1]);
            }
            EXPECT_LE(route_time, top_problem->get_budget() + 1e-6)
                << "Vehicle " << v << " violates budget constraint";
        }
    }
    
    // Verify total reward
    Reward expected_reward = 0.0;
    for (NodeId node : all_visited) {
        expected_reward += problem->get_reward(node);
    }
    EXPECT_NEAR(solution.total_reward, expected_reward, 1e-6);
}

// ================================
// Test 14: Real TOPTW instance - Solomon benchmark
// ================================
TEST_F(GreedySolverTest, RealInstance_TOPTW_Solomon) {
    std::string filepath = get_greedy_test_data_path("toptw/c101.txt");
    
    TOPTWParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse TOPTW instance";
    EXPECT_TRUE(problem->has_time_windows()) << "TOPTW should have time windows";
    
    auto* toptw_problem = dynamic_cast<TOPTWProblem*>(problem.get());
    ASSERT_NE(toptw_problem, nullptr);
    
    GreedySolver solver;
    auto solution = solver.solve(*problem, config);
    
    // Verify solution structure
    EXPECT_GT(solution.get_num_vehicles(), 0);
    
    // Verify time window feasibility for each route
    for (int v = 0; v < solution.get_num_vehicles(); ++v) {
        const auto& route = solution.get_route(v);
        
        if (route.size() <= 2) continue;  // Skip empty routes
        
        Time current_time = 0.0;
        for (size_t i = 0; i < route.size(); ++i) {
            NodeId node = route[i];
            const auto& tw = problem->get_time_window(node);
            
            // Arrival time at node
            if (i > 0) {
                NodeId prev = route[i-1];
                Time travel_time = problem->get_travel_time(prev, node, current_time);
                current_time += travel_time;
            }
            
            // Check time window feasibility
            EXPECT_LE(current_time, tw.closing + 1e-6)
                << "Vehicle " << v << " arrives too late at node " << node
                << " (arrival: " << current_time << ", closing: " << tw.closing << ")";
            
            // Wait if early, then serve
            current_time = std::max(current_time, tw.opening);
            current_time += problem->get_service_time(node);
        }
    }
    
    // Verify no customer visited by multiple vehicles
    std::set<NodeId> all_visited;
    for (int v = 0; v < solution.get_num_vehicles(); ++v) {
        const auto& route = solution.get_route(v);
        for (size_t i = 1; i < route.size() - 1; ++i) {
            EXPECT_EQ(all_visited.count(route[i]), 0);
            all_visited.insert(route[i]);
        }
    }
}

// ================================
// Test 15: Real TOPTW instance - Cordeau benchmark
// ================================
TEST_F(GreedySolverTest, RealInstance_TOPTW_Cordeau) {
    std::string filepath = get_greedy_test_data_path("toptw/pr01.txt");
    
    TOPTWParser parser;
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr) << "Failed to parse TOPTW instance";
    
    GreedySolver solver;
    auto solution = solver.solve(*problem, config);
    
    // Basic validity checks
    EXPECT_GT(solution.get_num_vehicles(), 0);
    
    for (int v = 0; v < solution.get_num_vehicles(); ++v) {
        const auto& route = solution.get_route(v);
        EXPECT_EQ(route.front(), problem->get_source_depot());
        EXPECT_EQ(route.back(), problem->get_sink_depot());
    }
    
    // If any customers are visited, reward should be positive
    bool has_customers = false;
    for (int v = 0; v < solution.get_num_vehicles(); ++v) {
        if (solution.get_route(v).size() > 2) {
            has_customers = true;
            break;
        }
    }
    
    if (has_customers) {
        EXPECT_GT(solution.total_reward, 0.0);
    }
}

