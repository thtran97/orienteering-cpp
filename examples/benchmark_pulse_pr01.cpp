/**
 * @file benchmark_pulse_pr01.cpp
 * @brief Minimal benchmark: run pulse algorithm on pr01 TOPTW instance with 1 vehicle.
 *
 * Usage:
 *   benchmark_pulse_pr01 [path_to_pr01.txt] [max_cpu_seconds] [max_labels]
 *
 * Expected optimal reward: 308
 *
 * Notes on the pulse solver:
 * - The backward DP bound is a relaxed upper bound (sum of all individually reachable
 *   rewards). This is valid but loose, causing the pulse to explore exponentially
 *   many states on instances with many overlapping time windows like pr01 (48 customers).
 * - Running with max_labels=0 (unlimited) is needed for exact optimality, but may
 *   require significant time on pr01.
 * - The --pulse-labels 0 flag in the main benchmark also achieves this for pr01.
 */
#include <chrono>
#include <iostream>
#include <string>

#include "io/toptw_parser.h"
#include "model/problem.h"
#include "model/solution.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/pulse/pulse_solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver;
using namespace oplib::solver::pulse;

int main(int argc, char** argv)
{
    std::string filepath = "data/toptw/pr01.txt";
    if (argc >= 2) filepath = argv[1];

    double max_cpu = 300.0; // seconds
    if (argc >= 3) max_cpu = std::stod(argv[2]);

    int max_labels = 0; // 0 = unlimited
    if (argc >= 4) max_labels = std::stoi(argv[3]);

    std::cout << "=== Pulse Benchmark: pr01 with 1 vehicle ===" << '\n';
    std::cout << "Instance   : " << filepath << '\n';
    std::cout << "Max CPU    : " << max_cpu << "s" << '\n';
    std::cout << "Max labels : " << (max_labels == 0 ? "unlimited" : std::to_string(max_labels)) << '\n';

    // Parse
    TOPTWParser parser;
    auto problem_ptr = parser.read(filepath);
    if (!problem_ptr) {
        std::cerr << "ERROR: failed to parse " << filepath << '\n';
        return 1;
    }
    Problem& problem = *problem_ptr;

    std::cout << "Nodes      : " << problem.get_num_nodes() << '\n';
    std::cout << "Vehicles   : " << problem.get_num_vehicles() << '\n';
    std::cout << "Budget     : " << problem.get_budget() << '\n';

    // Pre-compute bounds and show quality
    dp::BackwardDPSolver backward;
    auto ub = backward.compute_bounds(problem);
    int src = problem.get_source_depot();
    std::cout << "UB at src  : " << ub[src] << " (sum-of-reachable bound)" << '\n';

    // Run pulse with unlimited labels
    PulseSolverConfig cfg;
    cfg.max_labels   = max_labels; // 0 = unlimited
    cfg.max_cpu_time = max_cpu;
    cfg.verbose      = true;

    PulseSolver solver;

    auto t0  = std::chrono::high_resolution_clock::now();
    Solution sol = solver.solve(problem, cfg);
    auto t1  = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    int reward = static_cast<int>(sol.total_reward);
    std::cout << '\n';
    std::cout << "BEST REWARD FOUND : " << reward << '\n';
    std::cout << "CPU time (ms)     : " << ms << '\n';
    // Print the route
    std::cout << "Route             : ";
    for (auto n : sol.get_route(0)) std::cout << n << " ";
    std::cout << '\n';
    // Verify route feasibility
    double t = 0;
    double check_reward = 0;
    const auto& route = sol.get_route(0);
    for (size_t i = 1; i < route.size(); ++i) {
        double travel = problem.get_travel_time(route[i-1], route[i], t);
        t += travel;
        double open = problem.get_time_window(route[i]).opening;
        double close = problem.get_time_window(route[i]).closing;
        if (t > close) {
            std::cout << "  [INFEASIBLE] late arrival at node " << route[i]
                      << " t=" << t << " close=" << close << '\n';
        }
        if (t < open) t = open;
        t += problem.get_service_time(route[i]);
        if (i + 1 < route.size())  // not sink
            check_reward += problem.get_reward(route[i]);
    }
    std::cout << "Verified reward   : " << check_reward << '\n';
    std::cout << "Final time        : " << t << " (budget=" << problem.get_budget() << ")\n";

    if (reward >= 308) {
        std::cout << "RESULT: OPTIMAL (>= 308)" << '\n';
        return 0;
    } else {
        std::cout << "RESULT: SUBOPTIMAL (expected 308, got " << reward << ")" << '\n';
        return 1;
    }
}
