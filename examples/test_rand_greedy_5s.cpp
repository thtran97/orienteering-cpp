#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "model/variants/toptw.h"
#include "model/node.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;

// Build a TOPTW problem with a chosen vehicle count (correct Cordeau format:
// id x y service_time reward dummy1 dummy2 dummy3 tw_opening tw_closing)
static std::unique_ptr<TOPTWProblem> load_toptw(const std::string& filepath, int num_vehicles) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    std::string line;
    double tmax_raw = 0.0;

    // Header line: num_vehicles ??? budget ???
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        int dummy_vehicles; double dummy;
        ss >> dummy_vehicles >> dummy >> tmax_raw;
    }
    if (std::getline(file, line)) {}  // skip line 2

    auto problem = std::make_unique<TOPTWProblem>(filepath, num_vehicles, tmax_raw);
    problem->set_scaling(ScalingMode::SCALED_INTEGER, 1.0);

    std::vector<Node> nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        Node node;
        double d1, d2, d3, first_tw;
        if (!(ss >> node.id >> node.x >> node.y >> node.service_time >> node.reward
              >> d1 >> d2 >> d3)) continue;
        if (!(ss >> first_tw)) { node.tw.opening = 0.0; node.tw.closing = 1e18; }
        else {
            double second_tw;
            if (ss >> second_tw) { node.tw.opening = first_tw; node.tw.closing = second_tw; }
            else { node.tw.opening = 0.0; node.tw.closing = first_tw; }
        }
        nodes.push_back(node);
    }
    if (nodes.empty()) return nullptr;

    for (const auto& n : nodes) problem->add_node(n);
    Node sink = nodes[0];
    sink.id = static_cast<NodeId>(nodes.size());
    problem->add_node(sink);
    problem->finalize();
    return problem;
}

int main() {
    std::vector<std::string> instances = {"pr01", "c101"};

    std::cout << "=== RandomizedGreedy (5s limit) ===\n\n";
    std::cout << std::left
              << std::setw(12) << "Instance"
              << std::setw(10) << "Vehicles"
              << std::setw(12) << "Reward"
              << std::setw(10) << "Visited"
              << std::setw(12) << "CPU(ms)"
              << "\n";
    std::cout << std::string(56, '-') << "\n";

    for (const auto& inst : instances) {
        for (int v = 1; v <= 4; ++v) {
            auto problem = load_toptw("data/toptw/" + inst + ".txt", v);
            if (!problem) { std::cout << inst << ": load failed\n"; continue; }

            solver::constructive::RandomizedGreedySolver solver;
            solver::SolverConfig config;
            config.seed = 42;
            config.max_cpu_time = 5.0;

            auto t0 = std::chrono::high_resolution_clock::now();
            auto sol = solver.solve(*problem, config);
            auto t1 = std::chrono::high_resolution_clock::now();
            double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            int visited = 0;
            for (const auto& r : sol.get_routes())
                visited += std::max(0, static_cast<int>(r.size()) - 2);

            std::cout << std::left
                      << std::setw(12) << (inst + ".txt")
                      << std::setw(10) << v
                      << std::setw(12) << std::fixed << std::setprecision(1) << sol.total_reward
                      << std::setw(10) << visited
                      << std::setw(12) << std::fixed << std::setprecision(1) << cpu_ms
                      << "\n";
        }
    }
    return 0;
}
