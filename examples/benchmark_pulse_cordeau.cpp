/**
 * @file benchmark_pulse_cordeau.cpp
 * @brief Run pulse solver on all Cordeau pr01-pr20 OPTW instances (1 vehicle) and print a table.
 *
 * Usage:
 *   benchmark_pulse_cordeau [data_dir] [max_cpu_seconds]
 */
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "io/toptw_parser.h"
#include "model/problem.h"
#include "model/solution.h"
#include "solver/pulse/pulse_solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver::pulse;

struct Row {
    std::string name;
    int         nodes;
    int         reward;
    double      cpu_ms;
    bool        timeout;
};

static Solution run_once(Problem& problem, double max_cpu, int bound_step = 1000)
{
    PulseSolverConfig cfg;
    cfg.max_labels   = 0;
    cfg.max_cpu_time = max_cpu;
    cfg.verbose      = false;
    cfg.bound_step   = bound_step;  // 1000 = 10 original units * time_scale(100)
    return PulseSolver{}.solve(problem, cfg);
}

int main(int argc, char** argv)
{
    std::string data_dir = "data/toptw";
    double      max_cpu  = 120.0;
    if (argc >= 2) data_dir = argv[1];
    if (argc >= 3) max_cpu  = std::stod(argv[2]);

    TOPTWParser parser;
    std::vector<Row> rows;

    const int N = 20;
    std::cout << "Cordeau pr01-pr" << N << " | 1 vehicle | max_cpu=" << max_cpu << "s\n\n";

    for (int i = 1; i <= N; ++i) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "pr%02d.txt", i);
        std::string filepath = data_dir + "/" + buf;
        std::string name     = std::string("pr") + (i < 10 ? "0" : "") + std::to_string(i);

        auto problem_ptr = parser.read(filepath);
        if (!problem_ptr) {
            std::cerr << "ERROR: cannot read " << filepath << "\n";
            continue;
        }
        Problem& problem = *problem_ptr;

        Row r;
        r.name  = name;
        r.nodes = problem.get_num_nodes();

        auto t0 = std::chrono::high_resolution_clock::now();
        Solution sol = run_once(problem, max_cpu);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::high_resolution_clock::now() - t0).count();
        r.reward  = static_cast<int>(sol.total_reward);
        r.cpu_ms  = ms;
        r.timeout = (ms >= max_cpu * 1000.0 * 0.99);

        rows.push_back(r);

        std::cerr << "  " << name << " done  reward=" << r.reward
                  << " (" << static_cast<int>(r.cpu_ms) << "ms"
                  << (r.timeout ? ",TO" : "") << ")\n";
    }

    // ---- Print table ----
    std::cout << '\n';
    std::cout << std::left
              << std::setw(8)  << "Instance"
              << std::setw(8)  << "Nodes"
              << std::setw(10) << "Reward"
              << std::setw(14) << "CPU (ms)"
              << "Status\n";
    std::cout << std::string(54, '-') << '\n';
    int total = 0;
    for (const auto& r : rows) {
        std::cout << std::left
                  << std::setw(8)  << r.name
                  << std::setw(8)  << r.nodes
                  << std::setw(10) << r.reward
                  << std::setw(14) << std::fixed << std::setprecision(0) << r.cpu_ms
                  << (r.timeout ? "TIMEOUT" : "OK") << '\n';
        total += r.reward;
    }
    std::cout << std::string(54, '-') << '\n';
    std::cout << std::left << std::setw(26) << "TOTAL" << std::setw(10) << total << '\n';

    return 0;
}
