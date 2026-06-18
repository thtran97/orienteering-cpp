// Benchmark: ILS09 vs ILS+RR on the Cordeau TOPTW set
//
// For each instance in data/toptw/Cordeau/ and each vehicle count in {2,3,4}:
//   1. Run ILS09     for 60 s  -> reward_ils
//   2. Run ILS+RR   for 60 s  -> reward_rr, accumulated RR post-processing time
//
// Outputs:
//   results/toptw/benchmark_cordeau_comparison.csv  — per-row results
//   stdout                                           — formatted summary table

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "bench_utils.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/ils_route_recombination.h"

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::solver::metaheuristic;

// ============================================================================
// Configuration
// ============================================================================
static constexpr double TIME_LIMIT_S   = 60.0;
static constexpr int    SEED           = 42;
static constexpr int    ALPHA          = 3;
static constexpr int    RCL_SIZE       = 5;
static constexpr int    RESTART_THRESH = 10;
static constexpr int    POOL_SIZE      = 10;
static constexpr double SIM_THRESH     = 0.5;
static const std::vector<int> VEHICLE_COUNTS = {2, 3, 4};

// ============================================================================
// CSV row
// ============================================================================
struct CompRow {
    std::string instance;
    int         nodes       = 0;
    int         vehicles    = 0;
    double      ils_reward  = 0.0;
    double      ils_ms      = 0.0;
    double      rr_reward   = 0.0;
    double      rr_ms       = 0.0;
    double      rr_only_ms  = 0.0; // time inside recombine_routes() calls
};

static void write_csv_header(std::ofstream& f)
{
    f << "Instance,Nodes,Vehicles,"
         "ILS_Reward,ILS_CPU_ms,"
         "ILSRR_Reward,ILSRR_CPU_ms,ILSRR_RRonly_ms,"
         "Improvement_pct\n";
}

static void write_csv_row(std::ofstream& f, const CompRow& r)
{
    double impr = (r.ils_reward > 0.0)
                  ? 100.0 * (r.rr_reward - r.ils_reward) / r.ils_reward
                  : 0.0;
    f << std::fixed << std::setprecision(4)
      << r.instance     << ','
      << r.nodes        << ','
      << r.vehicles     << ','
      << r.ils_reward   << ','
      << r.ils_ms       << ','
      << r.rr_reward    << ','
      << r.rr_ms        << ','
      << r.rr_only_ms   << ','
      << impr           << '\n';
}

// ============================================================================
// main
// ============================================================================
int main()
{
    using Clock = std::chrono::high_resolution_clock;

    // ---- Discover Cordeau instances ----------------------------------------
    const std::string cordeau_dir = "data/toptw/Cordeau";
    if (!fs::exists(cordeau_dir)) {
        std::cerr << "[ERROR] Cordeau directory not found: " << cordeau_dir << '\n';
        return 1;
    }

    std::vector<bench::InstanceSpec> specs;
    for (const auto& e : fs::directory_iterator(cordeau_dir)) {
        if (!e.is_regular_file()) continue;
        specs.push_back({e.path().string(), "toptw"});
    }
    std::sort(specs.begin(), specs.end(),
              [](const bench::InstanceSpec& a, const bench::InstanceSpec& b){
                  return a.filepath < b.filepath; });

    if (specs.empty()) {
        std::cerr << "[ERROR] No instances found in " << cordeau_dir << '\n';
        return 1;
    }
    std::cout << "Found " << specs.size() << " Cordeau TOPTW instance(s).\n"
              << "Vehicle counts: 2, 3, 4  |  Time limit: " << TIME_LIMIT_S << "s\n\n";

    // ---- Open CSV output ---------------------------------------------------
    fs::create_directories("results/toptw");
    std::ofstream csv("results/toptw/benchmark_cordeau_comparison.csv",
                      std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "[ERROR] Cannot open output CSV.\n";
        return 1;
    }
    write_csv_header(csv);

    // ---- Solver configs ----------------------------------------------------
    ILS09SolverConfig ils_cfg;
    ils_cfg.seed              = SEED;
    ils_cfg.max_cpu_time      = TIME_LIMIT_S;
    ils_cfg.max_iterations    = 0;   // unlimited
    ils_cfg.alpha             = ALPHA;
    ils_cfg.rcl_size          = RCL_SIZE;
    ils_cfg.restart_threshold = RESTART_THRESH;

    ILSRouteRecombinationSolverConfig rr_cfg;
    rr_cfg.seed               = SEED;
    rr_cfg.max_cpu_time       = TIME_LIMIT_S;
    rr_cfg.max_iterations     = 0;
    rr_cfg.alpha              = ALPHA;
    rr_cfg.rcl_size           = RCL_SIZE;
    rr_cfg.restart_threshold  = RESTART_THRESH;
    rr_cfg.pool_size          = POOL_SIZE;
    rr_cfg.similarity_threshold = SIM_THRESH;

    ILS09Solver                 ils_solver;
    ILSRouteRecombinationSolver rr_solver;

    // ---- Table header (stdout) ---------------------------------------------
    const int W_INST = 14, W_N = 6, W_V = 4, W_R = 10, W_T = 12, W_P = 8;
    auto sep = [&]{
        std::cout
            << std::string(W_INST,'-') << '+'
            << std::string(W_N,'-')    << '+'
            << std::string(W_V,'-')    << '+'
            << std::string(W_R,'-')    << '+'
            << std::string(W_R,'-')    << '+'
            << std::string(W_P,'-')    << '+'
            << std::string(W_T,'-')    << '+'
            << std::string(W_T,'-')    << '\n';
    };
    sep();
    std::cout
        << std::left  << std::setw(W_INST) << "Instance"  << '|'
        << std::right << std::setw(W_N)    << "Nodes"     << '|'
        << std::right << std::setw(W_V)    << "Veh"       << '|'
        << std::right << std::setw(W_R)    << "ILS_Rew"   << '|'
        << std::right << std::setw(W_R)    << "RR_Rew"    << '|'
        << std::right << std::setw(W_P)    << "Impr%"     << '|'
        << std::right << std::setw(W_T)    << "ILS_ms"    << '|'
        << std::right << std::setw(W_T)    << "RRonly_ms" << '\n';
    sep();

    int total = 0, improved = 0;
    double sum_ils = 0.0, sum_rr = 0.0, sum_rr_only = 0.0;

    // ---- Main loop ---------------------------------------------------------
    for (const auto& spec : specs) {
        auto problem_base = bench::parse_instance(spec);
        if (!problem_base) { std::cerr << "[WARN] Parse failed: " << spec.filepath << '\n'; continue; }

        std::string inst_name = fs::path(spec.filepath).stem().string();

        for (int nv : VEHICLE_COUNTS) {
            // clone + override vehicles
            auto problem = bench::parse_instance(spec);
            if (!problem) continue;
            bench::Options veh_opts;
            veh_opts.vehicles = nv;
            bench::apply_overrides(veh_opts, *problem);

            CompRow row;
            row.instance = inst_name;
            row.nodes    = problem->get_num_nodes();
            row.vehicles = nv;

            // -- ILS09 -------------------------------------------------------
            {
                auto t0  = Clock::now();
                auto sol = ils_solver.solve(*problem, ils_cfg);
                row.ils_ms     = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                row.ils_reward = sol.total_reward;
            }

            // -- ILS+RR ------------------------------------------------------
            {
                auto t0  = Clock::now();
                auto sol = rr_solver.solve(*problem, rr_cfg);
                row.rr_ms      = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                row.rr_reward  = sol.total_reward;
                row.rr_only_ms = rr_solver.get_last_rr_time_ms();
            }

            double impr = (row.ils_reward > 0.0)
                          ? 100.0 * (row.rr_reward - row.ils_reward) / row.ils_reward
                          : 0.0;

            write_csv_row(csv, row);
            csv.flush();

            // stdout row
            std::cout
                << std::left  << std::setw(W_INST) << inst_name          << '|'
                << std::right << std::setw(W_N)    << row.nodes          << '|'
                << std::right << std::setw(W_V)    << nv                 << '|'
                << std::fixed << std::setprecision(2)
                << std::right << std::setw(W_R)    << row.ils_reward     << '|'
                << std::right << std::setw(W_R)    << row.rr_reward      << '|'
                << std::setprecision(1)
                << std::right << std::setw(W_P)    << impr               << '|'
                << std::setprecision(0)
                << std::right << std::setw(W_T)    << row.ils_ms         << '|'
                << std::right << std::setw(W_T)    << row.rr_only_ms     << '\n';
            std::cout.flush();

            sum_ils     += row.ils_reward;
            sum_rr      += row.rr_reward;
            sum_rr_only += row.rr_only_ms;
            if (row.rr_reward > row.ils_reward + 1e-6) ++improved;
            ++total;
        }
    }

    // ---- Summary -----------------------------------------------------------
    sep();
    double avg_impr = (sum_ils > 0.0) ? 100.0 * (sum_rr - sum_ils) / sum_ils : 0.0;
    std::cout << '\n'
              << "Total runs : " << total << '\n'
              << "RR improved: " << improved << " / " << total << '\n'
              << std::fixed << std::setprecision(2)
              << "Avg ILS reward   : " << sum_ils / total << '\n'
              << "Avg ILS+RR reward: " << sum_rr  / total << '\n'
              << "Avg improvement  : " << avg_impr << " %\n"
              << std::setprecision(1)
              << "Avg RR-only time : " << sum_rr_only / total << " ms\n\n"
              << "CSV: results/toptw/benchmark_cordeau_comparison.csv\n";

    csv.close();
    return 0;
}
