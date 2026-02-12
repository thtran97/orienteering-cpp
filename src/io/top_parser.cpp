#include "../../include/io/top_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace oplib::io {

std::unique_ptr<model::Problem> TOPParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    std::string line, label;
    int n = 0, m = 0;  // n = number of nodes, m = number of vehicles
    double tmax = 0.0; // tmax = maximum allowed travel time/distance (budget)

    // Line 1: n <val>
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> label >> n;
    }
    // Line 2: m <val>
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> label >> m;
    }
    // Line 3: tmax <val>
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> label >> tmax;
    }

    auto problem = std::make_unique<model::variants::TOPProblem>(filepath, m, tmax);

    // read remaining lines as nodes; format: x y reward
    int node_id = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        node.id = node_id++;
        if (!(ss >> node.x >> node.y >> node.reward)) continue;
        problem->add_node(node);
    }

    problem->finalize();
    return problem;
}

} // namespace oplib::io
