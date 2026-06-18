#pragma once

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/mctopmtw_parser.h"
#include "io/op_parser.h"
#include "io/singlesat_parser.h"
#include "io/tdop_parser.h"
#include "io/tdoptw_parser.h"
#include "io/top_parser.h"
#include "io/toptw_parser.h"
#include "io/ttdp_parser.h"
#include "model/problem.h"
#include "model/solution.h"
#include "solver/solver.h"

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver;

namespace bench {

extern const std::vector<std::string> ALL_VARIANTS;

// ============================================================================
// CLI options
// ============================================================================

struct Options {
    std::vector<std::string> variants;
    std::string instance_path = "data";
    double      timeout       = 60.0;
    int         iterations    = 1000;
    int         seed          = 42;
    int         runs          = 1;
    std::string output_root   = "results";
    bool        verbose       = false;
    bool        overwrite     = false;
    int         quick         = 0;
    int         pulse_labels  = 0;
    int         vehicles      = 0;  ///< override num_vehicles (0 = from file)
};

Options parse_cli(int argc, char** argv, const std::string& solver_name);

// ============================================================================
// Instance discovery and parsing
// ============================================================================

struct InstanceSpec {
    std::string filepath;
    std::string variant;
    std::string aux1; // speed_matrix (TDOP) | transition_matrix (TDOPTW)
    std::string aux2; // arc_category (TDOP)
};

std::vector<InstanceSpec> discover_instances(
    const std::string& root_path,
    const std::vector<std::string>& variants);

std::vector<InstanceSpec> quick_filter(
    std::vector<InstanceSpec> instances, int n);

std::unique_ptr<Problem> parse_instance(const InstanceSpec& spec);

// ============================================================================
// Result and CSV I/O
// ============================================================================

struct RunResult {
    std::string solver;
    std::string instance_name;
    std::string variant;
    int         num_nodes          = 0;
    int         num_vehicles       = 1;
    bool        has_time_windows   = false;
    bool        is_time_dependent  = false;
    double      budget             = 0.0;
    int         run_id             = 1;
    double      reward             = 0.0;
    double      travel_time        = 0.0;
    int         customers_visited  = 0;
    double      cpu_ms             = 0.0;
    std::string status             = "OK";
};

void write_header(std::ofstream& out);
void write_row(std::ofstream& out, const RunResult& r);

std::map<std::string, std::ofstream> open_csv_files(
    const std::string& output_root,
    const std::string& solver_name,
    const std::vector<std::string>& variants,
    bool overwrite);

void close_csv_files(std::map<std::string, std::ofstream>& files);

// ============================================================================
// Apply CLI overrides to a parsed problem (e.g. --vehicles)
// ============================================================================

void apply_overrides(const Options& opts, Problem& problem);

// ============================================================================
// Run a solver and record timing/result
// ============================================================================

template<typename Solver, typename Config>
RunResult run_and_record(
    Solver& solver, Config& cfg,
    const Problem& problem,
    const InstanceSpec& spec,
    const std::string& solver_name,
    int run_id)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    Solution sol = solver.solve(problem, cfg);
    auto t1 = std::chrono::high_resolution_clock::now();

    RunResult r;
    r.solver          = solver_name;
    r.instance_name   = fs::path(spec.filepath).filename().string();
    r.variant         = spec.variant;
    r.num_nodes       = problem.get_num_nodes();
    r.num_vehicles    = problem.get_num_vehicles();
    r.has_time_windows  = problem.has_time_windows();
    r.is_time_dependent = problem.is_time_dependent();
    r.budget          = problem.get_budget();
    r.run_id          = run_id;
    r.reward          = sol.total_reward;
    r.travel_time     = sol.total_travel_time;

    for (const auto& route : sol.get_routes())
        r.customers_visited += std::max(0, static_cast<int>(route.size()) - 2);

    r.cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.status = "OK";
    return r;
}

} // namespace bench
