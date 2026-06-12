#include <iostream>
#include <iomanip>
#include "model/variants/toptw.h"
#include "io/toptw_parser.h"
#include "solver/constructive/greedy.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver::constructive;

int main() {
    io::TOPTWParser parser;
    auto problem = parser.read("data/toptw/pr01.txt");
    
    if (!problem) {
        std::cerr << "Failed to parse\n";
        return 1;
    }
    
    std::cout << "Problem loaded: " << problem->get_name() << "\n";
    std::cout << "Nodes: " << problem->get_num_nodes() << "\n";
    std::cout << "Budget: " << problem->get_budget() << "\n";
    std::cout << "Source depot: " << problem->get_source_depot() << "\n";
    std::cout << "Sink depot: " << problem->get_sink_depot() << "\n";
    
    std::cout << "\nFirst 5 node rewards:\n";
    for (int i = 0; i < std::min(5, problem->get_num_nodes()); ++i) {
        std::cout << "  Node " << i << ": reward=" << problem->get_reward(i) << "\n";
    }
    
    GreedySolver solver;
    SolverConfig cfg;
    cfg.seed = 42;
    cfg.max_cpu_time = 10.0;
    cfg.verbose = false;
    
    std::cout << "\nRunning greedy solver with 1 vehicle...\n";
    auto solution = solver.solve(*problem, cfg);
    
    std::cout << "Solution:\n";
    std::cout << "  Total reward: " << solution.total_reward << "\n";
    std::cout << "  Total travel time: " << solution.total_travel_time << "\n";
    std::cout << "  Num vehicles: " << solution.get_num_vehicles() << "\n";
    
    int total_visited = 0;
    for (int v = 0; v < solution.get_num_vehicles(); ++v) {
        const auto& route = solution.get_route(v);
        int visited = std::max(0, static_cast<int>(route.size()) - 2);
        total_visited += visited;
        std::cout << "  Vehicle " << v << ": route size=" << route.size() << " (visited " << visited << " customers)\n";
    }
    std::cout << "  Total customers visited: " << total_visited << "\n";
    
    return 0;
}
