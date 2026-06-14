#!/bin/bash
echo "Testing TOPTW Parser Fix - pr01.txt with 1-4 vehicles"
echo "======================================================"

cd /home/user/orienteering-cpp

# Create a temporary benchmark that only tests one instance
cat > test_pr01.cpp << 'EOFCPP'
#include <iostream>
#include <memory>
#include <iomanip>
#include "io/toptw_parser.h"
#include "solver/constructive/greedy.h"

using namespace oplib;
using namespace oplib::model::variants;
using namespace oplib::solver::constructive;

int main() {
    io::TOPTWParser parser;
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Instance: pr01.txt\n";
    std::cout << "Solver: Greedy\n";
    std::cout << std::string(70, '=') << "\n";
    std::cout << std::left << std::setw(10) << "Vehicles" 
              << std::setw(15) << "Total Reward"
              << std::setw(15) << "Visited Cust."
              << std::setw(15) << "CPU Time (ms)"
              << "\n";
    std::cout << std::string(70, '=') << "\n";
    
    GreedySolver solver;
    SolverConfig cfg;
    cfg.seed = 42;
    cfg.max_cpu_time = 10.0;
    cfg.verbose = false;
    
    for (int num_vehicles = 1; num_vehicles <= 4; ++num_vehicles) {
        auto problem = parser.read("data/toptw/pr01.txt");
        if (!problem) {
            std::cerr << "Failed to parse pr01.txt\n";
            return 1;
        }
        
        // Create new problem with desired vehicle count
        auto toptw = std::make_unique<TOPTWProblem>("pr01_v" + std::to_string(num_vehicles), 
                                                     num_vehicles, problem->get_budget());
        
        // Copy nodes
        for (int i = 0; i < problem->get_num_nodes(); ++i) {
            const auto& node = dynamic_cast<OPProblem*>(problem.get())->nodes[i];
            toptw->add_node(node);
        }
        toptw->finalize();
        toptw->preprocessing();
        
        auto start = std::chrono::high_resolution_clock::now();
        auto solution = solver.solve(*toptw, cfg);
        auto end = std::chrono::high_resolution_clock::now();
        double cpu_ms = std::chrono::duration<double, std::milli>(end - start).count();
        
        int visited = 0;
        for (int v = 0; v < solution.get_num_vehicles(); ++v) {
            const auto& route = solution.get_route(v);
            visited += std::max(0, static_cast<int>(route.size()) - 2);
        }
        
        std::cout << std::left << std::setw(10) << num_vehicles
                  << std::setw(15) << solution.total_reward
                  << std::setw(15) << visited
                  << std::setw(15) << cpu_ms
                  << "\n";
    }
    
    return 0;
}
EOFCPP

# Try to compile and run
g++ -std=c++17 -I. -I./build/_deps/googletest-src/googletest/include \
    -o test_pr01 test_pr01.cpp ./build/src/liborienteeringLib.a -lm 2>&1 | head -20

if [ -f test_pr01 ]; then
    ./test_pr01
else
    echo "Compilation failed, trying simpler test..."
    # Run the fixed benchmark on just the first instance
    ./build/examples/benchmark_toptw_vehicles --timeout 10 --output /tmp/test_results 2>&1 | grep "pr01\|===\|Instance"
fi
