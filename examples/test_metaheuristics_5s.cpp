#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

#include "model/variants/toptw.h"
#include "model/node.h"
#include "solver/solver.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/metaheuristic/grasp_vns.h"
#include "solver/metaheuristic/lns.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "solver/metaheuristic/sails.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;

// Build a TOPTW problem with a chosen vehicle count (correct Cordeau format:
// id x y service_time reward dummy1 dummy2 dummy3 tw_opening tw_closing)
static std::unique_ptr<TOPTWProblem> load_toptw(const std::string& filepath, int num_vehicles) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    std::string line;
    double tmax_raw = 0.0;
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        int dv; double d; ss >> dv >> d >> tmax_raw;
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

using Factory = std::function<std::unique_ptr<Solver>()>;

// Independent feasibility verifier: re-walk every route from scratch and confirm
// arrival <= tw.closing at each node and total travel <= budget. Returns "" if
// feasible, otherwise a short reason. This does NOT reuse the solver's cached
// context, so it cross-checks the solver's own feasibility bookkeeping.
static std::string verify_feasible(const Problem& problem, const Solution& sol) {
    const double budget = problem.get_budget();
    for (int v = 0; v < sol.get_num_vehicles(); ++v) {
        const auto& route = sol.get_route(v);
        if (route.size() <= 2) continue;
        const auto& tw0 = problem.get_time_window(route[0]);
        double dep = tw0.opening + problem.get_service_time(route[0]);
        double cumulative_travel = 0.0;
        for (size_t i = 1; i < route.size(); ++i) {
            NodeId prev = route[i - 1], cur = route[i];
            double travel = problem.get_travel_time(prev, cur, dep);
            cumulative_travel += travel;
            double arrival = dep + travel;
            const auto& tw = problem.get_time_window(cur);
            if (arrival > tw.closing + 1e-6)
                return "TW@v" + std::to_string(v) + " node " + std::to_string(cur)
                     + " arr=" + std::to_string(arrival) + ">" + std::to_string(tw.closing);
            dep = std::max(arrival, tw.opening) + problem.get_service_time(cur);
        }
        if (cumulative_travel > budget + 1e-6)
            return "BUDGET@v" + std::to_string(v) + " " + std::to_string(cumulative_travel)
                 + ">" + std::to_string(budget);
    }
    return "";
}

int main() {
    std::vector<std::pair<std::string, Factory>> solvers = {
        {"rand_greedy", []{ return std::make_unique<constructive::RandomizedGreedySolver>(); }},
        {"grasp_vns",   []{ return std::make_unique<metaheuristic::GraspVnsSolver>(); }},
        {"lns",         []{ return std::make_unique<metaheuristic::LNSSolver>(); }},
        {"ils09",       []{ return std::make_unique<metaheuristic::ILS09Solver>(); }},
        {"ils_rr",      []{ return std::make_unique<metaheuristic::ILSRouteRecombinationSolver>(); }},
        {"sails",       []{ return std::make_unique<metaheuristic::SAILSSolver>(); }},
    };
    std::vector<std::string> instances = {"pr01", "c101"};

    std::cout << "=== Metaheuristics (5s limit) ===\n\n";
    std::cout << std::left
              << std::setw(12) << "Instance"
              << std::setw(14) << "Solver"
              << std::setw(10) << "Vehicles"
              << std::setw(12) << "Reward"
              << std::setw(10) << "Visited"
              << std::setw(12) << "CPU(ms)"
              << std::setw(10) << "Feasible"
              << "\n";
    std::cout << std::string(80, '-') << "\n";

    for (const auto& inst : instances) {
        for (const auto& [name, factory] : solvers) {
            for (int v = 1; v <= 4; ++v) {
                auto problem = load_toptw("data/toptw/" + inst + ".txt", v);
                if (!problem) { std::cout << inst << ": load failed\n"; continue; }

                SolverConfig config;
                config.seed = 42;
                config.max_cpu_time = 5.0;

                auto solver = factory();
                auto t0 = std::chrono::high_resolution_clock::now();
                auto sol = solver->solve(*problem, config);
                auto t1 = std::chrono::high_resolution_clock::now();
                double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

                int visited = 0;
                for (const auto& r : sol.get_routes())
                    visited += std::max(0, static_cast<int>(r.size()) - 2);

                std::string reason = verify_feasible(*problem, sol);
                std::string feas = reason.empty() ? "OK" : ("BAD:" + reason);

                std::cout << std::left
                          << std::setw(12) << (inst + ".txt")
                          << std::setw(14) << name
                          << std::setw(10) << v
                          << std::setw(12) << std::fixed << std::setprecision(1) << sol.total_reward
                          << std::setw(10) << visited
                          << std::setw(12) << std::fixed << std::setprecision(1) << cpu_ms
                          << std::setw(10) << feas
                          << "\n";
            }
        }
        std::cout << "\n";
    }
    return 0;
}
