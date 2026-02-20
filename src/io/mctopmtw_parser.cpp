#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "io/mctopmtw_parser.h"

namespace oplib::io {

std::unique_ptr<model::Problem> MCTOPMTWParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    // First line: M N B E1 E2 ... E10
    std::string line;
    if (!std::getline(file, line)) return nullptr;
    std::stringstream ss(line);
    int M = 0, N = 0;
    double B = 0.0;
    ss >> M >> N >> B;
    std::vector<double> E;
    double tmp;
    while (ss >> tmp) E.push_back(tmp);
    // ensure we have 10 constraint RHS values (pad with zeros)
    E.resize(10, 0.0);

    // Read depot line
    if (!std::getline(file, line)) return nullptr;
    std::stringstream ss2(line);
    int depot_i;
    double depot_x, depot_y, depot_d, depot_S;
    double depot_o, depot_c, depot_E;
    ss2 >> depot_i >> depot_x >> depot_y >> depot_d >> depot_S >> std::ws;
    // Try to read O C E fields if present
    ss2 >> depot_o >> depot_c >> depot_E;
    double tmax_raw = depot_c; 

    double time_scale = 1.0; // MCTOPMTW often 1.0 but uses integer logic
    auto problem = std::make_unique<model::variants::MCTOPMTWProblem>(filepath, M, tmax_raw * time_scale);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, time_scale);

    // prepare knapsack coeff storage: 10 constraints x N nodes
    std::vector<std::vector<double>> coeffs(10, std::vector<double>());

    std::vector<model::Node> parsed_nodes;
    // Read remaining nodes
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream s(line);
        model::Node node;
        int i;
        double x, y, d, S;
        if (!(s >> i >> x >> y >> d >> S)) continue;
        node.id = i;
        node.x = x;
        node.y = y;
        node.service_time = d * time_scale;
        node.reward = S;
        // read up to 4 openings and one closing
        double O1=0,O2=0,O3=0,O4=0,C4=0;
        s >> O1 >> O2 >> O3 >> O4 >> C4;
        node.tw.opening = O1 * time_scale;
        node.tw.closing = C4 * time_scale;
        // skip E, b
        double Eflag, b_val;
        s >> Eflag >> b_val;
        // read e1..e10
        for (int j=0;j<10;j++){
            double ej=0.0;
            if (s >> ej) coeffs[j].push_back(ej);
            else coeffs[j].push_back(0.0);
        }
        parsed_nodes.push_back(node);
    }

    for (const auto& n : parsed_nodes) {
        problem->add_node(n);
    }

    // Now add knapsack constraints to problem
    for (int j=0;j<10;j++) {
        if (!coeffs[j].empty()) {
            problem->add_knapsack_constraint(E[j], coeffs[j]);
        }
    }

    problem->finalize();
    return problem;
}

} // namespace oplib::io
