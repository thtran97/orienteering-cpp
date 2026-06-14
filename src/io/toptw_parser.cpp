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
    double time_scale = 1.0; // No scaling - data is already in correct units
    // (Removed: time_scale = 100.0 for Cordeau, 10.0 for Solomon)

    std::string line;
    int num_vehicles = 0;
    double tmax_raw = 0.0;

    // Line 1: num_vehicles, ???, budget, ???
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> num_vehicles;
        double dummy1;
        ss >> dummy1;
        ss >> tmax_raw;  // 3rd column is the budget
        double dummy3;
        ss >> dummy3;
    }

    // Skip line 2
    if (std::getline(file, line)) {}

    // Line 3 onwards: node data starts here

    // Use budget as tmax for TW variants
    auto problem = std::make_unique<model::variants::TOPTWProblem>(filepath, num_vehicles, tmax_raw * time_scale);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, time_scale);

    // read remaining lines as nodes
    // Format has inconsistency: depot has 9 columns, customers have 10
    // Depot: id x y service_time reward [3 fields] tw_closing (no tw_opening)
    // Customers: id x y service_time reward [3 fields] tw_opening tw_closing
    std::vector<model::Node> parsed_nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        double dummy1, dummy2, dummy3;  // Skip 3 extra fields (may be floats)
        double first_tw;  // Could be tw_opening or just a single TW value

        // Try to read all possible values
        if (!(ss >> node.id >> node.x >> node.y >> node.service_time >> node.reward
              >> dummy1 >> dummy2 >> dummy3)) {
            continue;
        }

        // Now try to read time windows - could be 1 or 2 values
        if (!(ss >> first_tw)) {
            // No time windows at all (shouldn't happen)
            node.tw.opening = 0.0;
            node.tw.closing = 1e18;
        } else {
            // We read one value. Check if there's another
            double second_tw;
            if (ss >> second_tw) {
                // Two values: opening and closing
                node.tw.opening = first_tw;
                node.tw.closing = second_tw;
            } else {
                // One value: treat as closing time, opening defaults to 0
                node.tw.opening = 0.0;
                node.tw.closing = first_tw;
            }
        }

        // DO NOT scale service times or time windows - they are already in the correct scale
        // Only distances (computed in finalize()) are scaled
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
