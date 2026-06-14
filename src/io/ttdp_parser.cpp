#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "io/ttdp_parser.h"

namespace oplib::io {

std::unique_ptr<model::Problem> TTDPParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    // Read header line
    int k = 0, M = 0, SD = 0, N = 0;
    if (!(file >> k >> M >> SD >> N)) return nullptr;

    // Read depot line
    int depot_i;
    double depot_x, depot_y;
    double depot_d, depot_S;
    if (!(file >> depot_i >> depot_x >> depot_y >> depot_d >> depot_S)) return nullptr;
    // consume remaining tokens of depot line if any
    file >> std::ws;
    // For header remark: Tmax is closing - opening of starting point
    // Next token on depot line may include 0 and C; attempt to read two more
    // We try to read two values; if not present, default tmax=0
    if (!(file >> std::ws)) {}

    // For robustness, rewind to start of nodes parsing by using getline for remaining
    std::string rest_of_line_from_depot;
    std::getline(file, rest_of_line_from_depot);

    double time_scale = 10.0; // Solomon default for TTDP
    double tmax_per_day_raw = 0.0;
    // Try to parse opening/closing from the depot's rest_of_line
    {
        std::stringstream ss(rest_of_line_from_depot);
        double maybe0, maybeC;
        if (ss >> maybe0 >> maybeC) {
            tmax_per_day_raw = maybeC - maybe0;
        }
    }

    auto problem = std::make_unique<model::variants::TTDPProblem>(filepath, 7, tmax_per_day_raw * time_scale);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, time_scale);

    // Read nodes
    file.clear();
    file.seekg(0);
    std::string line;
    std::getline(file, line); // header
    // std::getline(file, line); // depot (will be read again in loop)

    std::vector<model::Node> parsed_nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        int i;
        double x, y, d, S, t;
        if (!(ss >> i >> x >> y >> d >> S >> t)) continue;
        node.id = i;
        node.x = x;
        node.y = y;
        node.service_time = d * time_scale;
        node.reward = S;
        
        std::vector<double> opens(7), closes(7);
        for (int day = 0; day < 7; ++day) {
            if (!(ss >> opens[day] >> closes[day])) {
                opens[day] = 0.0;
                closes[day] = 0.0;
            }
        }
        int start_day = SD % 7;
        node.tw.opening = opens[start_day] * time_scale;
        node.tw.closing = closes[start_day] * time_scale;
        parsed_nodes.push_back(node);
    }

    for (const auto& n : parsed_nodes) {
        problem->add_node(n);
    }
    
    // Virtual sink duplication if format follows TOPTW
    model::Node sink = parsed_nodes[0];
    sink.id = static_cast<NodeId>(parsed_nodes.size());
    problem->add_node(sink);

    // Finalize the problem by computing the distance matrix
    problem->finalize();
    return problem;
}

} // namespace oplib::io
