#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "io/singlesat_parser.h"

namespace oplib::io {

std::unique_ptr<model::Problem> SingleSatParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    // Try to read header: N Tmax
    int N = 0;          // nber of nodes
    double tmax = 0.0;  // tmax = maximum allowed travel time (budget)
    if (!(file >> N >> tmax)) return nullptr;

    auto problem = std::make_unique<model::variants::SingleSatProblem>(filepath, tmax);

    // Read remaining lines; formats vary, so be permissive.
    std::string line;
    std::getline(file, line); // consume rest of first line
    int auto_id = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::vector<double> tokens;
        double v;
        while (ss >> v) tokens.push_back(v);
        model::Node node;
        if (tokens.size() >= 7) {
            node.id = static_cast<int>(tokens[0]);
            node.x = tokens[1];
            node.y = tokens[2];
            node.service_time = tokens[3];
            node.reward = tokens[4];
            node.tw.opening = tokens[5];
            node.tw.closing = tokens[6];
        } else if (tokens.size() == 4) {
            // best-effort mapping: id, service_time, x, y
            node.id = static_cast<int>(tokens[0]);
            node.service_time = tokens[1];
            node.x = tokens[2];
            node.y = tokens[3];
        } else if (tokens.size() == 3) {
            node.id = auto_id++;
            node.x = tokens[0];
            node.y = tokens[1];
            node.reward = tokens[2];
        } else {
            continue;
        }
        problem->add_node(node);
    }

    problem->finalize();
    return problem;
}

} // namespace oplib::io
