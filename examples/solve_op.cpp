#include <iostream>
#include <memory>
#include <string>
#include "io/op_parser.h"
#include "model/problem.h"
#include "model/solution.h"
#include "solver/solver.h"
#include "solver/local_search/LNS.h"

using namespace oplib;

/*
 * Main function for solving Orienteering Problem instances.
 * Usage: ./solve_op -file path/to/instance [-seed 0] [-timeout 60.0]
 */
int main(int argc, char** argv) {
    // Simple argument parsing (avoid dependency on missing ArgParser)
    std::string filename;
    int seed = 0;
    double timeout = 60.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-file" && i + 1 < argc) {
            filename = argv[++i];
        } else if (arg == "-seed" && i + 1 < argc) {
            seed = std::stoi(argv[++i]);
        } else if (arg == "-timeout" && i + 1 < argc) {
            timeout = std::stod(argv[++i]);
        }
    }

    if (filename.empty()) {
        std::cerr << "Usage: ./solve_op -file <instance_file> [-seed <seed>] [-timeout <seconds>]\n";
        return 1;
    }

    // Load instance using OPParser (supports Chao format for OP)
    io::OPParser reader;
    std::unique_ptr<model::Problem> problem = reader.read(filename);

    if (!problem) {
        std::cerr << "Error: Could not read instance file: " << filename << std::endl;
        return 1;
    }

    std::cout << "[INFO::main] Problem: " << problem->get_name() << "\n";
    std::cout << "[INFO::main] Nodes: " << problem->get_num_nodes() << "\n";

    // Configure solver: use existing LNS solver
    oplib::solver::local_search::LNSSolver lns_solver;
    oplib::solver::SolverConfig config;
    config.seed = seed;
    config.max_cpu_time = timeout;
    config.verbose = true;

    // Solve the problem
    model::Solution best_sol = lns_solver.solve(*problem, config);

    // Print results
    std::cout << "\n[INFO::main] Best Solution Found:\n";
    std::cout << "Total Reward: " << best_sol.total_reward << "\n";
    std::cout << "Total Travel Time: " << best_sol.total_travel_time << "\n";

    const auto& routes = best_sol.get_routes();
    for (size_t i = 0; i < routes.size(); ++i) {
        std::cout << "Route " << i << ": ";
        for (auto node_id : routes[i]) {
            std::cout << node_id << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
