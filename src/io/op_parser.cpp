#include "../../include/io/op_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace oplib::io {

std::unique_ptr<model::Problem> OPParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    double budget;  // budget is the limit on total travel distance/cost, read from first line
    int m;          // number of vehicles, read from first line (ignored for OP but may be present in format)
    std::string line;

    if (!(file >> budget >> m)) return nullptr;
    std::getline(file, line); // consume rest of line

    auto problem = std::make_unique<model::variants::OPProblem>(filepath, budget);
    
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
