/**
 * benchmark_lns_cordeau.cpp
 *
 * Run LNS on all *.txt files in data/toptw/ with nb_vehicles=1.
 * Each instance is solved NUM_RUNS times (different seeds) and
 * min/avg/max reward are reported.
 *
 * Usage (from repo root):
 *   ./build/examples/benchmark_lns_cordeau [data_dir] [alpha] [removal_ratio] [timelimit_s] [nruns]
 *   Defaults: data/toptw  2  0.4  300  5
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
    const std::string data_dir      = (argc > 1) ? argv[1] : "data/toptw";
    const int         alpha         = (argc > 2) ? std::atoi(argv[2]) : 2;
    const double      removal_ratio = (argc > 3) ? std::atof(argv[3]) : 0.4;
    const double      timelimit     = (argc > 4) ? std::atof(argv[4]) : 300.0;
    const int         num_runs      = (argc > 5) ? std::atoi(argv[5]) : 5;

    auto files = list_txt_files(data_dir);
    if (files.empty()) {
        std::cerr << "No .txt files found in " << data_dir << "\n";
        return 1;
    }

    std::cerr << "alpha=" << alpha
              << "  removal_ratio=" << removal_ratio
              << "  timelimit=" << timelimit << "s"
              << "  runs=" << num_runs << "\n\n";

    io::TOPTWParser parser;
    LNSSolver       lns;

    std::cout << std::left
              << std::setw(14) << "instance"
              << std::right
              << std::setw(7)  << "nodes"
              << std::setw(8)  << "budget"
              << std::setw(10) << "min"
              << std::setw(10) << "avg"
              << std::setw(10) << "max"
              << std::setw(10) << "time(s)"
              << "\n"
              << std::string(69, '-') << "\n";

    for (const auto& path : files) {
        std::string fname = path.substr(path.find_last_of("/\\") + 1);
        std::string iname = fname.substr(0, fname.size() - 4);

        auto problem_base = parser.read(path);
        if (!problem_base) {
            std::cerr << "  [SKIP] failed to parse " << path << "\n";
            continue;
        }
        auto* toptw = dynamic_cast<TOPTWProblem*>(problem_base.get());
        if (!toptw) { std::cerr << "  [SKIP] not TOPTWProblem: " << path << "\n"; continue; }
        toptw->set_num_vehicles(1);

        std::vector<double> rewards;
        rewards.reserve(num_runs);
        double total_time = 0.0;

        for (int run = 0; run < num_runs; ++run) {
            LNSSolverConfig cfg;
            cfg.seed           = 42 + run;
            cfg.alpha          = alpha;
            cfg.removal_ratio  = removal_ratio;
            cfg.max_cpu_time   = timelimit;
            cfg.max_iterations = std::numeric_limits<int>::max();

            auto t0  = std::chrono::high_resolution_clock::now();
            auto sol = lns.solve(*problem_base, cfg);
            total_time += std::chrono::duration<double>(
                std::chrono::high_resolution_clock::now() - t0).count();
            rewards.push_back(sol.total_reward);
        }

        double r_min = *std::min_element(rewards.begin(), rewards.end());
        double r_max = *std::max_element(rewards.begin(), rewards.end());
        double r_avg = std::accumulate(rewards.begin(), rewards.end(), 0.0) / num_runs;

        std::cout << std::left  << std::setw(14) << iname
                  << std::right << std::setw(7)  << (problem_base->get_num_nodes() - 2)
                  << std::setw(8)  << std::fixed << std::setprecision(0) << problem_base->get_budget()
                  << std::setw(10) << std::fixed << std::setprecision(1) << r_min
                  << std::setw(10) << std::fixed << std::setprecision(1) << r_avg
                  << std::setw(10) << std::fixed << std::setprecision(1) << r_max
                  << std::setw(10) << std::fixed << std::setprecision(2) << total_time / num_runs
                  << "\n";
        std::cout.flush();
    }

    return 0;
}
