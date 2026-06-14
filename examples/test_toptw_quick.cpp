#include <iostream>
#include <iomanip>
#include "io/toptw_parser.h"
#include "solver/constructive/greedy.h"

using namespace oplib;

int main() {
    io::TOPTWParser parser;

    // Test pr01.txt (has 5 vehicles)
    std::cout << "Testing pr01.txt:\n";
    auto pr01 = parser.read("data/toptw/pr01.txt");
    if (pr01) {
        std::cout << "  Loaded with " << pr01->get_num_vehicles() << " vehicles\n";
        solver::constructive::GreedySolver greedy_solver;
        solver::SolverConfig config;
        config.seed = 42;
        auto solution = greedy_solver.solve(*pr01, config);
        std::cout << "  Greedy reward: " << std::fixed << std::setprecision(2)
                  << solution.total_reward << "\n";
    }

    std::cout << "\nTesting c101.txt:\n";
    auto c101 = parser.read("data/toptw/c101.txt");
    if (c101) {
        std::cout << "  Loaded with " << c101->get_num_vehicles() << " vehicles\n";
        solver::constructive::GreedySolver greedy_solver;
        solver::SolverConfig config;
        config.seed = 42;
        auto solution = greedy_solver.solve(*c101, config);
        std::cout << "  Greedy reward: " << std::fixed << std::setprecision(2)
                  << solution.total_reward << "\n";
    }

    return 0;
}
