#include <fstream>
#include <sstream>
#include <iostream>

#include "io/toptw_parser.h"

namespace oplib::io {

std::unique_ptr<model::Problem> TOPTWParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filepath << std::endl;
        return nullptr;
    }

    std::string filename = filepath.substr(filepath.find_last_of("/\\") + 1);
    double time_scale = 100.0; // Default Cordeau
    if (filename.rfind("r", 0) == 0 || filename.rfind("c", 0) == 0) {
        time_scale = 10.0; // Solomon
    }

    std::string line;
    int num_vehicles = 0;
    double tmax_raw = 0.0;

    if (std::getline(file, line)) {} // skip line 1
    if (std::getline(file, line)) {} // skip line 2

    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> num_vehicles;
        double dummy;
        ss >> dummy;
        ss >> tmax_raw;
    }

    if (std::getline(file, line)) {} // skip line 4

    // Use budget as tmax for TW variants
    auto problem = std::make_unique<model::variants::TOPTWProblem>(filepath, num_vehicles, tmax_raw * time_scale);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, time_scale);

    // read remaining lines as nodes
    // format: id;reward;service_time;opening;closing
    std::vector<model::Node> parsed_nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        if (!(ss >> node.id >> node.x >> node.y >> node.service_time >> node.reward >> node.tw.opening >> node.tw.closing)) {
            continue;
        }
        // Scale values during parse to match legacy GenericOrienteeringProblem::parse_toptw_instance
        node.service_time *= time_scale;
        node.tw.opening *= time_scale;
        node.tw.closing *= time_scale;
        parsed_nodes.push_back(node);
    }

    if (parsed_nodes.empty()) return nullptr;

    // Add nodes in order
    for (const auto& node : parsed_nodes) {
        problem->add_node(node);
    }

    // Duplicate depot as virtual end depot (Legacy convention)
    model::Node sink_depot = parsed_nodes[0];
    sink_depot.id = static_cast<NodeId>(parsed_nodes.size());
    problem->add_node(sink_depot);

    problem->finalize();
    return problem;
}

} // namespace oplib::io
