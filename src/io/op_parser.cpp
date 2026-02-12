#include "../../include/io/op_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace oplib::io {

std::unique_ptr<model::Problem> OPParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    double budget_raw;  // budget is the limit on total travel distance/cost, read from first line
    int m;          // number of vehicles, read from first line (ignored for OP but may be present in format)
    std::string line;

    if (!(file >> budget_raw >> m)) return nullptr;
    std::getline(file, line); 

    double time_scale = 1.0; 
    auto problem = std::make_unique<model::variants::OPProblem>(filepath, budget_raw * time_scale);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, time_scale); // Set integer scaling mode for consistency with benchmarks
    
    // Read node data: x, y, reward (one node per line)
    std::vector<model::Node> parsed_nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        if (!(ss >> node.x >> node.y >> node.reward)) continue;
        parsed_nodes.push_back(node);
    }

    // Basic validation: need at least 2 nodes (source and sink depots)
    if (parsed_nodes.size() < 2) return nullptr; 

    // Legacy OP format often includes two depots at start, rotates second node to end
    // customers.begin()+1, customers.begin()+2, customers.end()
    if (parsed_nodes.size() > 2) {
        std::rotate(parsed_nodes.begin() + 1, parsed_nodes.begin() + 2, parsed_nodes.end());
    }

    // Reassign node IDs and add to problem
    for (size_t i = 0; i < parsed_nodes.size(); ++i) {
        parsed_nodes[i].id = static_cast<NodeId>(i);
        problem->add_node(parsed_nodes[i]);
    }

    // Finalize the problem by computing the distance matrix
    problem->finalize();
    return problem;
}

} // namespace oplib::io
