#include <fstream>
#include <sstream>
#include <iostream>

#include "io/toptw_parser.h"

namespace oplib::io {

// Cordeau/Solomon TOPTW instances use the PVRPTW file format:
//
//   Line 1: K ? n Q
//     K = number of vehicles (col 0)
//     ? = other planning parameter (unused for TOPTW)
//     n = number of customers
//     Q = capacity (unused for orienteering)
//
//   Line 2: planning horizon parameters (skipped)
//
//   Node lines: id x y service_time reward freq num_combos [combo_1 ... combo_{num_combos}] tw_open tw_close
//     freq, num_combos, and the combo codes are PVRPTW scheduling fields; for
//     TOPTW (single-period) only tw_open and tw_close matter.
//
//   Budget: taken from the depot's (node 0) tw_close, consistent with how
//   OPGraph::build_graph() derives tmax = source_tw.closing.

std::unique_ptr<model::Problem> TOPTWParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filepath << std::endl;
        return nullptr;
    }

    // time_scale = 1.0: coordinates, service times, and time windows are all in
    // consistent units; distances are floored to integer travel times.
    const double time_scale = 1.0;

    std::string line;
    int num_vehicles = 0;

    // Line 1: K ? n Q — only the first column (K) is used.
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> num_vehicles;
    }

    // Line 2: planning parameters — not used for single-period TOPTW.
    if (std::getline(file, line)) {}

    // Read all node lines.
    // Format: id x y service_time reward freq num_combos [combo_codes * num_combos] tw_open tw_close
    std::vector<model::Node> parsed_nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        int freq, num_combos;
        if (!(ss >> node.id >> node.x >> node.y >> node.service_time >> node.reward
                 >> freq >> num_combos)) {
            continue;
        }
        // Skip the visit-combination codes (PVRPTW scheduling, not used for TOPTW).
        for (int k = 0; k < num_combos; ++k) {
            double combo;
            if (!(ss >> combo)) break;
        }
        if (!(ss >> node.tw.opening >> node.tw.closing)) {
            continue;
        }
        parsed_nodes.push_back(node);
    }

    if (parsed_nodes.empty()) return nullptr;

    // Budget = depot's closing time window, matching OPGraph::build_graph() which
    // also sets tmax = source_tw.closing.
    double tmax = parsed_nodes[0].tw.closing;

    auto problem = std::make_unique<model::variants::TOPTWProblem>(filepath, num_vehicles, tmax);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, time_scale);

    for (const auto& node : parsed_nodes) {
        problem->add_node(node);
    }

    // Duplicate depot as virtual end depot (legacy convention: source and sink
    // share the same location so direct source→sink travel time is 0).
    model::Node sink_depot = parsed_nodes[0];
    sink_depot.id = static_cast<NodeId>(parsed_nodes.size());
    problem->add_node(sink_depot);

    problem->finalize();
    return problem;
}

} // namespace oplib::io
