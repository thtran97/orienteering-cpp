#include <iostream>
#include <iomanip>
#include "io/toptw_parser.h"
#include "solver/constructive/greedy.h"

using namespace oplib;

int main() {
    io::TOPTWParser parser;
    auto problem = parser.read("data/toptw/c101.txt");

    if (!problem) {
        std::cout << "Failed to parse\n";
        return 1;
    }

    std::cout << "=== Problem Loaded ===\n";
    std::cout << "Name: " << problem->get_name() << "\n";
    std::cout << "Num nodes: " << problem->get_num_nodes() << "\n";
    std::cout << "Num vehicles: " << problem->get_num_vehicles() << "\n";
    std::cout << "Budget: " << problem->get_budget() << "\n";
    std::cout << "\n=== Node Details (first 5 customers) ===\n";

    // Print all nodes with time windows
    std::cout << "All nodes:\n";
    for (int i = 0; i < static_cast<int>(problem->get_num_nodes()); ++i) {
        auto tw = problem->get_time_window(i);
        std::cout << "Node " << i
                  << ": reward=" << std::fixed << std::setprecision(0) << problem->get_reward(i)
                  << " service_time=" << problem->get_service_time(i)
                  << " tw=[" << tw.opening << ", " << tw.closing << "]\n";
    }

    std::cout << "\n=== Testing Travel Times and Time Windows ===\n";

    // Test travel time from depot to first customer
    int depot = problem->get_source_depot();
    int customer1 = 1;

    double travel_0_to_1 = problem->get_travel_time(depot, customer1, 0.0);
    auto tw1 = problem->get_time_window(customer1);

    std::cout << "Travel time depot(0) -> customer(1) at time 0: " << travel_0_to_1 << "\n";
    std::cout << "Arrival at customer 1: " << travel_0_to_1 << "\n";
    std::cout << "Customer 1 time window: [" << tw1.opening << ", " << tw1.closing << "]\n";
    std::cout << "Feasible? " << (travel_0_to_1 <= tw1.closing ? "YES" : "NO") << "\n";

    std::cout << "\n=== Running Greedy Solver ===\n";
    solver::constructive::GreedySolver greedy_solver;
    solver::SolverConfig config;
    config.verbose = true;
    auto solution = greedy_solver.solve(*problem, config);

    std::cout << "\n=== Solution ===\n";
    std::cout << "Total reward: " << solution.total_reward << "\n";
    std::cout << "Total travel time: " << solution.total_travel_time << "\n";
    const auto& routes = solution.get_routes();
    std::cout << "Num routes: " << routes.size() << "\n";

    for (size_t v = 0; v < routes.size(); ++v) {
        std::cout << "Route " << v << ": ";
        for (auto node : routes[v]) {
            std::cout << node << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
