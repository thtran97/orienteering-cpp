/**
 * benchmark_lns_sensitivity.cpp
 *
 * Study the impact of alpha and removal_ratio on LNS performance over the
 * Cordeau OPTW set (pr*.txt files in data_dir).
 *
 * For each (alpha, removal_ratio) combination, every available instance is
 * solved NUM_RUNS times.  The aggregate average reward across all instances
 * (and per-instance min/avg/max) is reported.
 *
 * Usage (from repo root):
 *   ./build/examples/benchmark_lns_sensitivity [data_dir] [timelimit_s] [nruns]
 *   Defaults: data/toptw  60  3
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <dirent.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "io/toptw_parser.h"
#include "model/variants/toptw.h"
#include "solver/metaheuristic/lns.h"
#include "solver/solver.h"

using namespace oplib;
using namespace oplib::model::variants;
using namespace oplib::solver;
using namespace oplib::solver::metaheuristic;

static const std::vector<int>    ALPHA_VALUES    = {1, 2, 3, 5};
static const std::vector<double> REMOVAL_RATIOS  = {0.2, 0.3, 0.4, 0.5};

static std::vector<std::string> list_txt_files(const std::string& dir) {
    std::vector<std::string> files;
    DIR* d = opendir(dir.c_str());
    if (!d) { std::cerr << "Cannot open directory: " << dir << "\n"; return files; }
    struct dirent* e;
    while ((e = readdir(d)) != nullptr) {
        std::string name = e->d_name;
        if (name.size() > 4 && name.substr(name.size() - 4) == ".txt")
            files.push_back(dir + "/" + name);
    }
    closedir(d);
    std::sort(files.begin(), files.end());
    return files;
}

int main(int argc, char* argv[]) {
    const std::string data_dir  = (argc > 1) ? argv[1] : "data/toptw";
    const double      timelimit = (argc > 2) ? std::atof(argv[2]) : 60.0;
    const int         num_runs  = (argc > 3) ? std::atoi(argv[3]) : 3;

    auto files = list_txt_files(data_dir);
    if (files.empty()) { std::cerr << "No .txt files in " << data_dir << "\n"; return 1; }

    // Pre-parse all instances once
    io::TOPTWParser parser;
    struct Instance {
        std::string name;
        std::unique_ptr<model::Problem> problem;
    };
    std::vector<Instance> instances;
    for (const auto& path : files) {
        std::string fname = path.substr(path.find_last_of("/\\") + 1);
        std::string iname = fname.substr(0, fname.size() - 4);
        auto pb = parser.read(path);
        if (!pb) { std::cerr << "  [SKIP] " << path << "\n"; continue; }
        auto* toptw = dynamic_cast<TOPTWProblem*>(pb.get());
        if (!toptw) { std::cerr << "  [SKIP] not TOPTWProblem: " << path << "\n"; continue; }
        toptw->set_num_vehicles(1);
        instances.push_back({iname, std::move(pb)});
    }
    if (instances.empty()) { std::cerr << "No valid instances.\n"; return 1; }

    std::cerr << "Sensitivity study: " << instances.size() << " instance(s), "
              << "timelimit=" << timelimit << "s, runs=" << num_runs
              << ", alpha in {";
    for (int a : ALPHA_VALUES) std::cerr << a << ",";
    std::cerr << "}, removal_ratio in {";
    for (double r : REMOVAL_RATIOS) std::cerr << r << ",";
    std::cerr << "}\n\n";

    LNSSolver lns;

    // -----------------------------------------------------------------------
    // Summary table: avg reward across all instances per (alpha, removal_ratio)
    // -----------------------------------------------------------------------
    std::cout << "=== Aggregate avg reward over all instances ===\n\n";

    // Header: removal_ratio columns
    std::cout << std::left << std::setw(8) << "alpha\\rr";
    for (double rr : REMOVAL_RATIOS)
        std::cout << std::right << std::setw(10) << rr;
    std::cout << "\n" << std::string(8 + 10 * REMOVAL_RATIOS.size(), '-') << "\n";

    for (int alpha : ALPHA_VALUES) {
        std::cout << std::left << std::setw(8) << alpha;
        for (double rr : REMOVAL_RATIOS) {
            double total_avg = 0.0;
            for (const auto& inst : instances) {
                double sum = 0.0;
                for (int run = 0; run < num_runs; ++run) {
                    LNSSolverConfig cfg;
                    cfg.seed           = 42 + run;
                    cfg.alpha          = alpha;
                    cfg.removal_ratio  = rr;
                    cfg.max_cpu_time   = timelimit;
                    cfg.max_iterations = std::numeric_limits<int>::max();
                    auto sol = lns.solve(*inst.problem, cfg);
                    sum += sol.total_reward;
                }
                total_avg += sum / num_runs;
            }
            std::cout << std::right << std::fixed << std::setprecision(1)
                      << std::setw(10) << total_avg / instances.size();
            std::cout.flush();
        }
        std::cout << "\n";
    }

    // -----------------------------------------------------------------------
    // Per-instance detail for each (alpha, removal_ratio)
    // -----------------------------------------------------------------------
    std::cout << "\n=== Per-instance detail ===\n";

    for (int alpha : ALPHA_VALUES) {
        for (double rr : REMOVAL_RATIOS) {
            std::cout << "\nalpha=" << alpha << "  removal_ratio=" << rr << "\n";
            std::cout << std::left  << std::setw(14) << "instance"
                      << std::right << std::setw(10) << "min"
                      << std::setw(10) << "avg"
                      << std::setw(10) << "max" << "\n"
                      << std::string(44, '-') << "\n";

            for (const auto& inst : instances) {
                std::vector<double> rewards;
                for (int run = 0; run < num_runs; ++run) {
                    LNSSolverConfig cfg;
                    cfg.seed           = 42 + run;
                    cfg.alpha          = alpha;
                    cfg.removal_ratio  = rr;
                    cfg.max_cpu_time   = timelimit;
                    cfg.max_iterations = std::numeric_limits<int>::max();
                    auto sol = lns.solve(*inst.problem, cfg);
                    rewards.push_back(sol.total_reward);
                }
                double r_min = *std::min_element(rewards.begin(), rewards.end());
                double r_max = *std::max_element(rewards.begin(), rewards.end());
                double r_avg = std::accumulate(rewards.begin(), rewards.end(), 0.0) / num_runs;
                std::cout << std::left  << std::setw(14) << inst.name
                          << std::right << std::fixed << std::setprecision(1)
                          << std::setw(10) << r_min
                          << std::setw(10) << r_avg
                          << std::setw(10) << r_max << "\n";
                std::cout.flush();
            }
        }
    }

    return 0;
}
