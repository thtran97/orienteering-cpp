#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>

#include "io/toptw_parser.h"

namespace fs = std::filesystem;

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

    // time_scale = 100.0 for Cordeau TOPTW instances (pr*.txt):
    // Distances are computed as floor(euclidean * 100) to match the centesimal
    // precision used in the Cordeau et al. benchmark.
    // For Solomon instances (c*.txt, r*.txt) use scale=10 (decimetric precision).
    // Default fallback: scale=1 (raw integer distances).
    //
    // Detect instance type from filename to set appropriate scale.
    const std::string fname = fs::path(filepath).filename().string();
    double time_scale;
    if (fname.rfind("pr", 0) == 0) {
        time_scale = 100.0;  // Cordeau pr* instances: centesimal precision
    } else if (fname.rfind("r", 0) == 0 || fname.rfind("c", 0) == 0) {
        time_scale = 10.0;   // Solomon r*/c* instances: decimetric precision
    } else {
        time_scale = 1.0;    // other instances: raw integer distances
    }

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
        double tw_open, tw_close;
        if (!(ss >> tw_open >> tw_close)) {
            continue;
        }
        // Scale time values to match the chosen time_scale.
        // All time quantities (service, TW, budget) must be in the same units as
        // the travel time matrix (which finalize() computes as floor(dist * time_scale)).
        node.service_time = static_cast<Time>(node.service_time * time_scale);
        node.tw.opening   = static_cast<Time>(tw_open  * time_scale);
        node.tw.closing   = static_cast<Time>(tw_close * time_scale);
        parsed_nodes.push_back(node);
    }

    if (parsed_nodes.empty()) return nullptr;

    // Budget = depot's closing time window (already scaled above).
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
