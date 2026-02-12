#include "../../include/io/toptw_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace oplib::io {

std::unique_ptr<model::Problem> TOPTWParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filepath << std::endl;
        return nullptr;
    }

    std::string line;
    int num_vehicles = 0;
    double tmax = 0.0;

    if (std::getline(file, line)) {} // skip line 1
    if (std::getline(file, line)) {} // skip line 2

    if (std::getline(file, line)) {
        std::stringstream ss(line);
        ss >> num_vehicles;
        double dummy;
        ss >> dummy;
        ss >> tmax;
    }

    if (std::getline(file, line)) {} // skip line 4

    auto problem = std::make_unique<model::variants::TOPTWProblem>(filepath, num_vehicles, tmax);

    // read remaining lines as nodes
    // format: id;reward;service_time;opening;closing
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        if (!(ss >> node.id >> node.x >> node.y >> node.service_time >> node.reward >> node.tw.opening >> node.tw.closing)) {
            continue;
        }
        problem->add_node(node);
    }

    problem->finalize();
    return problem;
}

} // namespace oplib::io
