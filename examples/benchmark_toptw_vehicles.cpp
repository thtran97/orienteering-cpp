/**
 * benchmark_toptw_vehicles.cpp
 *
 * Loads all TOPTW instances from data/toptw/ and runs all solvers with
 * varying vehicle counts (1-4), using a 300-second time limit per run.
 *
 * Usage: ./benchmark_toptw_vehicles [--output folder]
 *
 * Output: formatted table to stdout + results/toptw_benchmark_<date>.csv
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
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
#include <dirent.h>
#include <sys/stat.h>

#include <sstream>
#include <fstream>

#include "io/toptw_parser.h"
#include "model/variants/toptw.h"
#include "model/node.h"
#include "solver/constructive/greedy.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/metaheuristic/grasp_vns.h"
#include "solver/metaheuristic/lns.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "solver/metaheuristic/sails.h"
#include "solver/policy_learning/mcts_solver.h"
#include "solver/solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;

// ============================================================================
// Instance loader
// ============================================================================

struct InstanceFile {
    std::string filepath;
    std::string filename;
};

static std::vector<InstanceFile> load_instance_files(const std::string& dir) {
    std::vector<InstanceFile> files;
    DIR* d = opendir(dir.c_str());
    if (!d) {
        std::cerr << "[ERROR] Cannot open directory: " << dir << "\n";
        return files;
    }

    struct dirent* entry;
    while ((entry = readdir(d)) != nullptr) {
        std::string name = entry->d_name;
        if (name.size() > 4 && name.substr(name.size() - 4) == ".txt") {
            files.push_back({dir + "/" + name, name});
        }
    }
    closedir(d);
    std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b) { return a.filename < b.filename; });
    return files;
}

// ============================================================================
// Solver registry
// ============================================================================

using SolverFactory = std::function<std::unique_ptr<Solver>()>;

static const std::vector<std::pair<std::string, SolverFactory>> SOLVER_LIST = {
    {"greedy",      []{ return std::make_unique<constructive::GreedySolver>(); }},
    {"rand_greedy", []{ return std::make_unique<constructive::RandomizedGreedySolver>(); }},
    {"grasp_vns",   []{ return std::make_unique<metaheuristic::GraspVnsSolver>(); }},
    {"lns",         []{ return std::make_unique<metaheuristic::LNSSolver>(); }},
    {"ils09",       []{ return std::make_unique<metaheuristic::ILS09Solver>(); }},
    {"ils_rr",      []{ return std::make_unique<metaheuristic::ILSRouteRecombinationSolver>(); }},
    {"sails",       []{ return std::make_unique<metaheuristic::SAILSSolver>(); }},
    {"mcts",        []{ return std::make_unique<policy_learning::MCTSSolver>(); }},
};

// ============================================================================
// Result
// ============================================================================

struct RunResult {
    std::string solver;
    std::string instance;
    int         vehicles      = 1;
    double      reward        = -1.0;
    double      cpu_ms        = 0.0;
    int         customers_visited = 0;
    int         nodes         = 0;
    std::string status;  // OK | ERROR:...
};

// ============================================================================
// Run one solver on one problem
// ============================================================================

static RunResult run_solver(const std::string& solver_name,
                            const SolverFactory& factory,
                            const std::string& instance_name,
                            const Problem& problem,
                            int num_vehicles,
                            const SolverConfig& cfg)
{
    RunResult r;
    r.solver    = solver_name;
    r.instance  = instance_name;
    r.vehicles  = num_vehicles;
    r.nodes     = problem.get_num_nodes();

    try {
        auto solver = factory();
        auto t0 = std::chrono::high_resolution_clock::now();
        Solution sol = solver->solve(problem, cfg);
        auto t1 = std::chrono::high_resolution_clock::now();
        r.cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        r.reward = sol.total_reward;
        for (const auto& route : sol.get_routes())
            r.customers_visited += std::max(0, static_cast<int>(route.size()) - 2);
        r.status = "OK";
    } catch (const std::exception& e) {
        r.status = std::string("ERROR:") + e.what();
    }
    return r;
}

// ============================================================================
// Printing
// ============================================================================

static std::string today_str()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tm);
    return buf;
}

static void print_separator() {
    std::cout << std::string(120, '=') << "\n";
}

static void print_header() {
    print_separator();
    std::cout << std::left
              << std::setw(16) << "Instance"
              << std::setw(14) << "Solver"
              << std::setw(8)  << "Vehicles"
              << std::setw(12) << "Reward"
              << std::setw(10) << "Visited"
              << std::setw(10) << "CPU(ms)"
              << std::setw(25) << "Status"
              << "\n";
    print_separator();
}

static void print_row(const RunResult& r) {
    std::string status_short = r.status.length() > 20
                               ? r.status.substr(0, 20)
                               : r.status;
    std::cout << std::left
              << std::setw(16) << r.instance
              << std::setw(14) << r.solver
              << std::setw(8)  << r.vehicles
              << std::fixed << std::setprecision(2)
              << std::setw(12) << r.reward
              << std::setw(10) << r.customers_visited
              << std::setprecision(1)
              << std::setw(10) << r.cpu_ms
              << std::setw(25) << status_short
              << "\n";
}

// ============================================================================
// CSV
// ============================================================================

static void write_csv_header(std::ofstream& out) {
    out << "Instance,Solver,Vehicles,Nodes,Reward,CustomersVisited,CPU_ms,Status\n";
}

static void write_csv_row(std::ofstream& out, const RunResult& r) {
    out << r.instance << ','
        << r.solver   << ','
        << r.vehicles << ','
        << r.nodes    << ','
        << std::fixed << std::setprecision(4)
        << r.reward   << ','
        << r.customers_visited << ','
        << r.cpu_ms   << ','
        << r.status   << '\n';
}

// ============================================================================
// CLI
// ============================================================================

struct Opts {
    std::string output = "results";
    double timeout = 300.0;
};

static Opts parse_args(int argc, char** argv) {
    Opts o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + a);
            return argv[++i];
        };
        if      (a == "--output" || a == "-o") o.output = next();
        else if (a == "--timeout" || a == "-t") o.timeout = std::stod(next());
        else if (a == "--help" || a == "-h") {
            std::cout << "Usage: benchmark_toptw_vehicles [--output folder] [--timeout seconds]\n";
            std::exit(0);
        }
    }
    return o;
}

// ============================================================================
// Main
// ============================================================================

static std::unique_ptr<TOPTWProblem> create_toptw_with_vehicles(
    const std::string& filepath,
    int num_vehicles)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return nullptr;
    }

    std::string filename = filepath.substr(filepath.find_last_of("/\\") + 1);
    double time_scale = 100.0;
    if (filename.rfind("r", 0) == 0 || filename.rfind("c", 0) == 0) {
        time_scale = 10.0;
    }

    std::string line;
    double tmax_raw = 0.0;

    // Line 1: num_vehicles, ???, budget, ???
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        int dummy_vehicles;
        double dummy;
        ss >> dummy_vehicles >> dummy >> tmax_raw;
    }

    // Skip line 2
    if (std::getline(file, line)) {}

    auto problem = std::make_unique<TOPTWProblem>(filepath, num_vehicles,
                                                   tmax_raw * time_scale);

    std::vector<Node> parsed_nodes;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        Node node;
        if (!(ss >> node.id >> node.x >> node.y >> node.service_time
              >> node.reward >> node.tw.opening >> node.tw.closing)) {
            continue;
        }
        node.service_time *= time_scale;
        node.tw.opening *= time_scale;
        node.tw.closing *= time_scale;
        parsed_nodes.push_back(node);
    }

    if (parsed_nodes.empty()) return nullptr;

    for (const auto& node : parsed_nodes) {
        problem->add_node(node);
    }

    Node sink_depot = parsed_nodes[0];
    sink_depot.id = static_cast<NodeId>(parsed_nodes.size());
    problem->add_node(sink_depot);

    problem->finalize();
    return problem;
}

int main(int argc, char** argv) {
    Opts opts = parse_args(argc, argv);

    auto instance_files = load_instance_files("data/toptw");
    if (instance_files.empty()) {
        std::cerr << "[ERROR] No TOPTW instances found in data/toptw/\n";
        return 1;
    }

    std::cout << "=== TOPTW Benchmark (Vehicles 1-4) ===\n"
              << "Instances       : " << instance_files.size() << "\n"
              << "Vehicle counts  : 1, 2, 3, 4\n"
              << "Solvers         : " << SOLVER_LIST.size() << "\n"
              << "Timeout per run : " << opts.timeout << "s\n"
              << "Total runs      : " << (instance_files.size() * 4 * SOLVER_LIST.size()) << "\n\n";

    SolverConfig cfg;
    cfg.seed = 42;
    cfg.max_cpu_time = opts.timeout;
    cfg.max_iterations = 10000;
    cfg.verbose = false;

    // Prepare output folder
    mkdir(opts.output.c_str(), 0755);
    std::string csv_path = opts.output + "/toptw_benchmark_" + today_str() + ".csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "[WARN] Cannot open CSV: " << csv_path << "\n";
    } else {
        write_csv_header(csv);
    }

    print_header();

    int ok = 0, err = 0, done = 0;
    int total = instance_files.size() * 4 * SOLVER_LIST.size();

    for (const auto& ifile : instance_files) {
        // For each vehicle count (1-4)
        for (int num_vehicles = 1; num_vehicles <= 4; ++num_vehicles) {
            auto problem = create_toptw_with_vehicles(ifile.filepath, num_vehicles);
            if (!problem) {
                std::cerr << "[WARN] Failed to load " << ifile.filename
                          << " with " << num_vehicles << " vehicles\n";
                continue;
            }

            for (const auto& [sname, sfactory] : SOLVER_LIST) {
                ++done;

                // Progress indicator
                if (done % 20 == 0) {
                    std::cout.flush();
                }

                auto r = run_solver(sname, sfactory, ifile.filename, *problem,
                                  num_vehicles, cfg);

                print_row(r);
                if (csv.is_open()) write_csv_row(csv, r);

                if (r.status == "OK") {
                    ++ok;
                } else {
                    ++err;
                }
            }
        }
    }

    print_separator();
    if (csv.is_open()) {
        csv.close();
        std::cout << "\nCSV saved: " << csv_path << "\n";
    }

    std::cout << "\n=== Summary ===\n"
              << "Total runs  : " << done << "\n"
              << "OK          : " << ok << "\n"
              << "Errors      : " << err << "\n";

    return (err > 0) ? 1 : 0;
}
