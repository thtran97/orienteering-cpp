/**
 * benchmark_all.cpp
 *
 * Self-contained benchmark: builds synthetic instances for every supported
 * variant (OP, OPTW, TOP, TOPTW) at three sizes each and runs all 10 solvers.
 *
 * Exact solvers (forward_dp, bidir_dp, pulse) are only run on small instances
 * (≤12 nodes) to avoid exponential blow-up.
 *
 * Usage: ./benchmark_all [--timeout <sec>] [--iterations <n>] [--seed <n>]
 *
 * Output: formatted table to stdout + results/benchmark_all_<date>.csv
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

#include "model/node.h"
#include "model/solution.h"
#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/top.h"
#include "model/variants/toptw.h"
#include "solver/constructive/greedy.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/metaheuristic/grasp_vns.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "solver/metaheuristic/lns.h"
#include "solver/policy_learning/mcts_solver.h"
#include "solver/pulse/pulse_solver.h"
#include "solver/solver.h"

namespace fs_compat {
#include <sys/stat.h>
inline bool create_dir(const std::string& p) { return mkdir(p.c_str(), 0755) == 0; }
inline bool exists(const std::string& p) { struct stat s{}; return stat(p.c_str(), &s) == 0; }
} // namespace fs_compat

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;

// ============================================================================
// Instance generation helpers
// ============================================================================

static Node make_node(NodeId id, double x, double y, double reward,
                      double open = 0.0, double close = 1e18,
                      double service = 0.0)
{
    Node n;
    n.id           = id;
    n.x            = x;
    n.y            = y;
    n.reward       = reward;
    n.service_time = service;
    n.tw           = {open, close};
    return n;
}

// --- OP instances -----------------------------------------------------------

// 8 nodes (6 customers), linear-ish layout. Budget allows ~5 customers.
static OPProblem make_op_small()
{
    OPProblem p("op_s8", 55.0);
    p.add_node(make_node(0,  0.0,  0.0,  0.0));  // source
    p.add_node(make_node(1,  8.0,  4.0, 15.0));
    p.add_node(make_node(2, 15.0, -3.0, 20.0));
    p.add_node(make_node(3, 22.0,  6.0, 12.0));
    p.add_node(make_node(4, 28.0, -5.0, 18.0));
    p.add_node(make_node(5, 35.0,  4.0, 10.0));
    p.add_node(make_node(6, 42.0, -2.0, 22.0));
    p.add_node(make_node(7, 50.0,  0.0,  0.0));  // sink
    p.finalize(); p.preprocessing();
    return p;
}

// 12 nodes (10 customers), grid-like layout. Budget covers ~6-8 customers.
static OPProblem make_op_medium()
{
    OPProblem p("op_m12", 80.0);
    p.add_node(make_node(0,   0.0,  0.0,  0.0));  // source
    p.add_node(make_node(1,   8.0,  8.0, 12.0));
    p.add_node(make_node(2,  16.0, -4.0, 20.0));
    p.add_node(make_node(3,  20.0, 10.0,  8.0));
    p.add_node(make_node(4,  28.0,  2.0, 25.0));
    p.add_node(make_node(5,  32.0, -8.0, 15.0));
    p.add_node(make_node(6,  36.0, 12.0, 10.0));
    p.add_node(make_node(7,  40.0, -2.0, 18.0));
    p.add_node(make_node(8,  44.0,  7.0, 22.0));
    p.add_node(make_node(9,  48.0, -5.0, 14.0));
    p.add_node(make_node(10, 52.0,  3.0, 16.0));
    p.add_node(make_node(11, 60.0,  0.0,  0.0));  // sink
    p.finalize(); p.preprocessing();
    return p;
}

// 18 nodes (16 customers), scattered layout.
static OPProblem make_op_large()
{
    OPProblem p("op_l18", 100.0);
    p.add_node(make_node(0,   0.0,   0.0,  0.0));  // source
    p.add_node(make_node(1,   6.0,   7.0, 14.0));
    p.add_node(make_node(2,  12.0,  -5.0, 20.0));
    p.add_node(make_node(3,  18.0,  10.0, 10.0));
    p.add_node(make_node(4,  22.0,  -8.0, 18.0));
    p.add_node(make_node(5,  27.0,   4.0, 22.0));
    p.add_node(make_node(6,  30.0, -12.0, 16.0));
    p.add_node(make_node(7,  34.0,   9.0, 12.0));
    p.add_node(make_node(8,  38.0,  -3.0, 24.0));
    p.add_node(make_node(9,  42.0,  11.0,  8.0));
    p.add_node(make_node(10, 45.0,  -7.0, 19.0));
    p.add_node(make_node(11, 49.0,   5.0, 15.0));
    p.add_node(make_node(12, 53.0, -10.0, 21.0));
    p.add_node(make_node(13, 57.0,   3.0, 11.0));
    p.add_node(make_node(14, 61.0,  -4.0, 17.0));
    p.add_node(make_node(15, 65.0,   8.0, 13.0));
    p.add_node(make_node(16, 68.0,  -2.0, 20.0));
    p.add_node(make_node(17, 75.0,   0.0,  0.0));  // sink
    p.finalize(); p.preprocessing();
    return p;
}

// --- OPTW instances ---------------------------------------------------------

// 8 nodes (6 customers) with tight-ish time windows
static OPTWProblem make_optw_small()
{
    OPTWProblem p("optw_s8", 70.0);
    p.add_node(make_node(0,  0.0,  0.0,  0.0,  0.0, 1000.0, 0.0));  // source
    p.add_node(make_node(1,  8.0,  4.0, 15.0,  0.0,   25.0, 2.0));
    p.add_node(make_node(2, 15.0, -3.0, 20.0,  5.0,   35.0, 2.0));
    p.add_node(make_node(3, 22.0,  6.0, 12.0,  0.0,   40.0, 2.0));
    p.add_node(make_node(4, 28.0, -5.0, 18.0, 10.0,   50.0, 2.0));
    p.add_node(make_node(5, 35.0,  4.0, 10.0, 20.0,   55.0, 2.0));
    p.add_node(make_node(6, 42.0, -2.0, 22.0,  0.0,   60.0, 2.0));
    p.add_node(make_node(7, 50.0,  0.0,  0.0,  0.0, 1000.0, 0.0));  // sink
    p.finalize(); p.preprocessing();
    return p;
}

// 12 nodes (10 customers) with moderate time windows
static OPTWProblem make_optw_medium()
{
    OPTWProblem p("optw_m12", 90.0);
    p.add_node(make_node(0,   0.0,  0.0,  0.0,  0.0, 1000.0, 0.0));
    p.add_node(make_node(1,   8.0,  8.0, 12.0,  0.0,   30.0, 2.0));
    p.add_node(make_node(2,  16.0, -4.0, 20.0,  5.0,   40.0, 2.0));
    p.add_node(make_node(3,  20.0, 10.0,  8.0, 10.0,   45.0, 2.0));
    p.add_node(make_node(4,  28.0,  2.0, 25.0,  0.0,   50.0, 2.0));
    p.add_node(make_node(5,  32.0, -8.0, 15.0, 15.0,   55.0, 2.0));
    p.add_node(make_node(6,  36.0, 12.0, 10.0, 20.0,   60.0, 2.0));
    p.add_node(make_node(7,  40.0, -2.0, 18.0,  0.0,   65.0, 2.0));
    p.add_node(make_node(8,  44.0,  7.0, 22.0, 10.0,   70.0, 2.0));
    p.add_node(make_node(9,  48.0, -5.0, 14.0, 25.0,   75.0, 2.0));
    p.add_node(make_node(10, 52.0,  3.0, 16.0,  0.0,   80.0, 2.0));
    p.add_node(make_node(11, 60.0,  0.0,  0.0,  0.0, 1000.0, 0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// 18 nodes (16 customers) with moderate time windows
static OPTWProblem make_optw_large()
{
    OPTWProblem p("optw_l18", 120.0);
    p.add_node(make_node(0,   0.0,   0.0,  0.0,  0.0, 1000.0, 0.0));
    p.add_node(make_node(1,   6.0,   7.0, 14.0,  0.0,   35.0, 2.0));
    p.add_node(make_node(2,  12.0,  -5.0, 20.0,  5.0,   45.0, 2.0));
    p.add_node(make_node(3,  18.0,  10.0, 10.0, 10.0,   50.0, 2.0));
    p.add_node(make_node(4,  22.0,  -8.0, 18.0,  0.0,   55.0, 2.0));
    p.add_node(make_node(5,  27.0,   4.0, 22.0, 15.0,   60.0, 2.0));
    p.add_node(make_node(6,  30.0, -12.0, 16.0,  5.0,   65.0, 2.0));
    p.add_node(make_node(7,  34.0,   9.0, 12.0, 20.0,   70.0, 2.0));
    p.add_node(make_node(8,  38.0,  -3.0, 24.0,  0.0,   75.0, 2.0));
    p.add_node(make_node(9,  42.0,  11.0,  8.0, 25.0,   80.0, 2.0));
    p.add_node(make_node(10, 45.0,  -7.0, 19.0, 10.0,   85.0, 2.0));
    p.add_node(make_node(11, 49.0,   5.0, 15.0,  0.0,   90.0, 2.0));
    p.add_node(make_node(12, 53.0, -10.0, 21.0, 30.0,   95.0, 2.0));
    p.add_node(make_node(13, 57.0,   3.0, 11.0, 15.0,  100.0, 2.0));
    p.add_node(make_node(14, 61.0,  -4.0, 17.0,  5.0,  105.0, 2.0));
    p.add_node(make_node(15, 65.0,   8.0, 13.0, 35.0,  110.0, 2.0));
    p.add_node(make_node(16, 68.0,  -2.0, 20.0,  0.0,  115.0, 2.0));
    p.add_node(make_node(17, 75.0,   0.0,  0.0,  0.0, 1000.0, 0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// --- TOP instances ----------------------------------------------------------

// 8 nodes (6 customers), 2 vehicles, budget per vehicle = 60
// (source→sink = 50, so budget must be > 50 for any route to be feasible)
static TOPProblem make_top_small()
{
    TOPProblem p("top_s8_2v", 2, 60.0);
    p.add_node(make_node(0,  0.0,  0.0,  0.0));
    p.add_node(make_node(1,  8.0,  4.0, 15.0));
    p.add_node(make_node(2, 15.0, -3.0, 20.0));
    p.add_node(make_node(3, 22.0,  6.0, 12.0));
    p.add_node(make_node(4, 28.0, -5.0, 18.0));
    p.add_node(make_node(5, 35.0,  4.0, 10.0));
    p.add_node(make_node(6, 42.0, -2.0, 22.0));
    p.add_node(make_node(7, 50.0,  0.0,  0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// 12 nodes (10 customers), 3 vehicles, budget=75
// (source→sink = 60, so budget must be > 60 for any route to be feasible)
static TOPProblem make_top_medium()
{
    TOPProblem p("top_m12_3v", 3, 75.0);
    p.add_node(make_node(0,   0.0,  0.0,  0.0));
    p.add_node(make_node(1,   8.0,  8.0, 12.0));
    p.add_node(make_node(2,  16.0, -4.0, 20.0));
    p.add_node(make_node(3,  20.0, 10.0,  8.0));
    p.add_node(make_node(4,  28.0,  2.0, 25.0));
    p.add_node(make_node(5,  32.0, -8.0, 15.0));
    p.add_node(make_node(6,  36.0, 12.0, 10.0));
    p.add_node(make_node(7,  40.0, -2.0, 18.0));
    p.add_node(make_node(8,  44.0,  7.0, 22.0));
    p.add_node(make_node(9,  48.0, -5.0, 14.0));
    p.add_node(make_node(10, 52.0,  3.0, 16.0));
    p.add_node(make_node(11, 60.0,  0.0,  0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// 18 nodes (16 customers), 4 vehicles, budget=90
// (source→sink = 75, so budget must be > 75 for any route to be feasible)
static TOPProblem make_top_large()
{
    TOPProblem p("top_l18_4v", 4, 90.0);
    p.add_node(make_node(0,   0.0,   0.0,  0.0));
    p.add_node(make_node(1,   6.0,   7.0, 14.0));
    p.add_node(make_node(2,  12.0,  -5.0, 20.0));
    p.add_node(make_node(3,  18.0,  10.0, 10.0));
    p.add_node(make_node(4,  22.0,  -8.0, 18.0));
    p.add_node(make_node(5,  27.0,   4.0, 22.0));
    p.add_node(make_node(6,  30.0, -12.0, 16.0));
    p.add_node(make_node(7,  34.0,   9.0, 12.0));
    p.add_node(make_node(8,  38.0,  -3.0, 24.0));
    p.add_node(make_node(9,  42.0,  11.0,  8.0));
    p.add_node(make_node(10, 45.0,  -7.0, 19.0));
    p.add_node(make_node(11, 49.0,   5.0, 15.0));
    p.add_node(make_node(12, 53.0, -10.0, 21.0));
    p.add_node(make_node(13, 57.0,   3.0, 11.0));
    p.add_node(make_node(14, 61.0,  -4.0, 17.0));
    p.add_node(make_node(15, 65.0,   8.0, 13.0));
    p.add_node(make_node(16, 68.0,  -2.0, 20.0));
    p.add_node(make_node(17, 75.0,   0.0,  0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// --- TOPTW instances --------------------------------------------------------

// 8 nodes (6 customers), 2 vehicles, budget=70, time windows
static TOPTWProblem make_toptw_small()
{
    TOPTWProblem p("toptw_s8_2v", 2, 70.0);
    p.add_node(make_node(0,  0.0,  0.0,  0.0,  0.0, 1000.0, 0.0));
    p.add_node(make_node(1,  8.0,  4.0, 15.0,  0.0,   25.0, 2.0));
    p.add_node(make_node(2, 15.0, -3.0, 20.0,  5.0,   35.0, 2.0));
    p.add_node(make_node(3, 22.0,  6.0, 12.0,  0.0,   45.0, 2.0));
    p.add_node(make_node(4, 28.0, -5.0, 18.0, 10.0,   55.0, 2.0));
    p.add_node(make_node(5, 35.0,  4.0, 10.0, 20.0,   60.0, 2.0));
    p.add_node(make_node(6, 42.0, -2.0, 22.0,  0.0,   65.0, 2.0));
    p.add_node(make_node(7, 50.0,  0.0,  0.0,  0.0, 1000.0, 0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// 12 nodes (10 customers), 3 vehicles, budget=90, time windows
static TOPTWProblem make_toptw_medium()
{
    TOPTWProblem p("toptw_m12_3v", 3, 90.0);
    p.add_node(make_node(0,   0.0,  0.0,  0.0,  0.0, 1000.0, 0.0));
    p.add_node(make_node(1,   8.0,  8.0, 12.0,  0.0,   35.0, 2.0));
    p.add_node(make_node(2,  16.0, -4.0, 20.0,  5.0,   45.0, 2.0));
    p.add_node(make_node(3,  20.0, 10.0,  8.0, 10.0,   50.0, 2.0));
    p.add_node(make_node(4,  28.0,  2.0, 25.0,  0.0,   55.0, 2.0));
    p.add_node(make_node(5,  32.0, -8.0, 15.0, 15.0,   60.0, 2.0));
    p.add_node(make_node(6,  36.0, 12.0, 10.0, 20.0,   65.0, 2.0));
    p.add_node(make_node(7,  40.0, -2.0, 18.0,  0.0,   70.0, 2.0));
    p.add_node(make_node(8,  44.0,  7.0, 22.0, 10.0,   75.0, 2.0));
    p.add_node(make_node(9,  48.0, -5.0, 14.0, 25.0,   80.0, 2.0));
    p.add_node(make_node(10, 52.0,  3.0, 16.0,  0.0,   85.0, 2.0));
    p.add_node(make_node(11, 60.0,  0.0,  0.0,  0.0, 1000.0, 0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// 18 nodes (16 customers), 4 vehicles, budget=120, time windows
static TOPTWProblem make_toptw_large()
{
    TOPTWProblem p("toptw_l18_4v", 4, 120.0);
    p.add_node(make_node(0,   0.0,   0.0,  0.0,  0.0, 1000.0, 0.0));
    p.add_node(make_node(1,   6.0,   7.0, 14.0,  0.0,   40.0, 2.0));
    p.add_node(make_node(2,  12.0,  -5.0, 20.0,  5.0,   50.0, 2.0));
    p.add_node(make_node(3,  18.0,  10.0, 10.0, 10.0,   55.0, 2.0));
    p.add_node(make_node(4,  22.0,  -8.0, 18.0,  0.0,   60.0, 2.0));
    p.add_node(make_node(5,  27.0,   4.0, 22.0, 15.0,   65.0, 2.0));
    p.add_node(make_node(6,  30.0, -12.0, 16.0,  5.0,   70.0, 2.0));
    p.add_node(make_node(7,  34.0,   9.0, 12.0, 20.0,   75.0, 2.0));
    p.add_node(make_node(8,  38.0,  -3.0, 24.0,  0.0,   80.0, 2.0));
    p.add_node(make_node(9,  42.0,  11.0,  8.0, 25.0,   85.0, 2.0));
    p.add_node(make_node(10, 45.0,  -7.0, 19.0, 10.0,   90.0, 2.0));
    p.add_node(make_node(11, 49.0,   5.0, 15.0,  0.0,   95.0, 2.0));
    p.add_node(make_node(12, 53.0, -10.0, 21.0, 30.0,  100.0, 2.0));
    p.add_node(make_node(13, 57.0,   3.0, 11.0, 15.0,  105.0, 2.0));
    p.add_node(make_node(14, 61.0,  -4.0, 17.0,  5.0,  110.0, 2.0));
    p.add_node(make_node(15, 65.0,   8.0, 13.0, 35.0,  115.0, 2.0));
    p.add_node(make_node(16, 68.0,  -2.0, 20.0,  0.0,  115.0, 2.0));
    p.add_node(make_node(17, 75.0,   0.0,  0.0,  0.0, 1000.0, 0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// ============================================================================
// Instance descriptor
// ============================================================================

struct InstanceDesc {
    std::string name;
    std::string variant;
    int         num_nodes;
    int         num_vehicles;
    bool        has_tw;
    bool        exact_eligible; // small enough for exact solvers
    std::function<std::unique_ptr<Problem>()> make;
};

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
    {"mcts",        []{ return std::make_unique<policy_learning::MCTSSolver>(); }},
    {"forward_dp",  []{ return std::make_unique<dp::ForwardDPSolver>(); }},
    {"bidir_dp",    []{ return std::make_unique<dp::BidirectionalDPSolver>(); }},
    {"pulse",       []{ return std::make_unique<pulse::PulseSolver>(); }},
};

// ============================================================================
// Result
// ============================================================================

struct RunResult {
    std::string solver;
    std::string instance;
    std::string variant;
    int         num_nodes    = 0;
    int         num_vehicles = 1;
    bool        has_tw       = false;
    double      reward       = -1.0;
    double      cpu_ms       = 0.0;
    int         visited      = 0;
    std::string status;  // OK | SKIP | ERROR:...
};

// ============================================================================
// Run one solver on one problem
// ============================================================================

static RunResult run_solver(const std::string& solver_name,
                             const SolverFactory& factory,
                             const InstanceDesc& desc,
                             const Problem& problem,
                             const SolverConfig& cfg)
{
    RunResult r;
    r.solver       = solver_name;
    r.instance     = desc.name;
    r.variant      = desc.variant;
    r.num_nodes    = desc.num_nodes;
    r.num_vehicles = desc.num_vehicles;
    r.has_tw       = desc.has_tw;

    bool is_exact = (solver_name == "forward_dp" ||
                     solver_name == "bidir_dp"   ||
                     solver_name == "pulse");
    if (is_exact && !desc.exact_eligible) {
        r.status = "SKIP(too_large)";
        return r;
    }

    try {
        auto solver = factory();
        auto t0 = std::chrono::high_resolution_clock::now();
        Solution sol = solver->solve(problem, cfg);
        auto t1 = std::chrono::high_resolution_clock::now();
        r.cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        r.reward = sol.total_reward;
        for (const auto& route : sol.get_routes())
            r.visited += std::max(0, static_cast<int>(route.size()) - 2);
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
    std::strftime(buf, sizeof(buf), "%y%m%d", &tm);
    return buf;
}

static void print_separator(int w_solver, int w_inst)
{
    std::cout << '+' << std::string(w_solver + 2, '-')
              << '+' << std::string(w_inst + 2, '-')
              << '+' << std::string(8, '-')     // variant
              << '+' << std::string(6, '-')     // nodes
              << '+' << std::string(4, '-')     // veh
              << '+' << std::string(4, '-')     // tw
              << '+' << std::string(10, '-')    // reward
              << '+' << std::string(7, '-')     // visited
              << '+' << std::string(10, '-')    // cpu_ms
              << '+' << std::string(20, '-')    // status
              << "+\n";
}

static void print_header(int w_solver, int w_inst)
{
    print_separator(w_solver, w_inst);
    std::cout << "| " << std::left << std::setw(w_solver) << "Solver"
              << " | " << std::setw(w_inst)    << "Instance"
              << " | " << std::setw(6)         << "Variant"
              << " | " << std::setw(4)         << "N"
              << " | " << std::setw(2)         << "V"
              << " | " << std::setw(2)         << "TW"
              << " | " << std::right << std::setw(8) << "Reward"
              << " | " << std::setw(5)         << "Visit"
              << " | " << std::setw(8)         << "CPU(ms)"
              << " | " << std::left << std::setw(18)  << "Status"
              << " |\n";
    print_separator(w_solver, w_inst);
}

static void print_row(const RunResult& r, int w_solver, int w_inst)
{
    std::string status_short = r.status.substr(0, 18);
    std::cout << "| " << std::left  << std::setw(w_solver) << r.solver
              << " | " << std::setw(w_inst) << r.instance
              << " | " << std::setw(6)  << r.variant
              << " | " << std::right << std::setw(4) << r.num_nodes
              << " | " << std::setw(2) << r.num_vehicles
              << " | " << (r.has_tw ? " Y" : " N")
              << " | ";
    if (r.reward < 0.0)
        std::cout << std::setw(8) << "-";
    else
        std::cout << std::fixed << std::setprecision(2) << std::setw(8) << r.reward;
    std::cout << " | " << std::setw(5) << r.visited
              << " | ";
    if (r.cpu_ms > 0.0)
        std::cout << std::fixed << std::setprecision(1) << std::setw(8) << r.cpu_ms;
    else
        std::cout << std::setw(8) << "-";
    std::cout << " | " << std::left << std::setw(18) << status_short
              << " |\n";
}

// ============================================================================
// CSV
// ============================================================================

static void write_csv_header(std::ofstream& out)
{
    out << "Solver,Instance,Variant,Nodes,Vehicles,HasTW,"
           "Reward,CustomersVisited,CPU_ms,Status\n";
}

static void write_csv_row(std::ofstream& out, const RunResult& r)
{
    out << r.solver       << ','
        << r.instance     << ','
        << r.variant      << ','
        << r.num_nodes    << ','
        << r.num_vehicles << ','
        << (r.has_tw ? "yes" : "no") << ','
        << std::fixed << std::setprecision(4)
        << r.reward       << ','
        << r.visited      << ','
        << r.cpu_ms       << ','
        << r.status       << '\n';
}

// ============================================================================
// CLI
// ============================================================================

struct Opts {
    double timeout    = 10.0;
    int    iterations = 200;
    int    seed       = 42;
    std::string output = "results";
};

static Opts parse_args(int argc, char** argv)
{
    Opts o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + a);
            return argv[++i];
        };
        if      (a == "--timeout"    || a == "-t") o.timeout    = std::stod(next());
        else if (a == "--iterations" || a == "-n") o.iterations = std::stoi(next());
        else if (a == "--seed"       || a == "-s") o.seed       = std::stoi(next());
        else if (a == "--output"     || a == "-o") o.output     = next();
        else if (a == "--help"       || a == "-h") {
            std::cout << "Usage: benchmark_all [--timeout sec] [--iterations n] "
                         "[--seed n] [--output folder]\n";
            std::exit(0);
        }
    }
    return o;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
    Opts opts = parse_args(argc, argv);

    // Build instance list
    std::vector<InstanceDesc> instances = {
        // OP
        {"op_s8",       "op",    8,  1, false, true,  []{return std::make_unique<OPProblem>   (make_op_small());   }},
        {"op_m12",      "op",   12,  1, false, true,  []{return std::make_unique<OPProblem>   (make_op_medium());  }},
        {"op_l18",      "op",   18,  1, false, false, []{return std::make_unique<OPProblem>   (make_op_large());   }},
        // OPTW
        {"optw_s8",     "optw",  8,  1, true,  true,  []{return std::make_unique<OPTWProblem> (make_optw_small()); }},
        {"optw_m12",    "optw", 12,  1, true,  true,  []{return std::make_unique<OPTWProblem> (make_optw_medium());}},
        {"optw_l18",    "optw", 18,  1, true,  false, []{return std::make_unique<OPTWProblem> (make_optw_large()); }},
        // TOP
        {"top_s8_2v",   "top",   8,  2, false, true,  []{return std::make_unique<TOPProblem>  (make_top_small());  }},
        {"top_m12_3v",  "top",  12,  3, false, true,  []{return std::make_unique<TOPProblem>  (make_top_medium()); }},
        {"top_l18_4v",  "top",  18,  4, false, false, []{return std::make_unique<TOPProblem>  (make_top_large());  }},
        // TOPTW
        {"toptw_s8_2v", "toptw", 8,  2, true,  true,  []{return std::make_unique<TOPTWProblem>(make_toptw_small());}},
        {"toptw_m12_3v","toptw",12,  3, true,  true,  []{return std::make_unique<TOPTWProblem>(make_toptw_medium());}},
        {"toptw_l18_4v","toptw",18,  4, true,  false, []{return std::make_unique<TOPTWProblem>(make_toptw_large());}},
    };

    int total_runs = static_cast<int>(instances.size())
                   * static_cast<int>(SOLVER_LIST.size());

    std::cout << "=== Orienteering Solver Benchmark ===\n"
              << "Instances  : " << instances.size() << " (4 variants x 3 sizes)\n"
              << "Solvers    : " << SOLVER_LIST.size() << "\n"
              << "Total runs : " << total_runs << "\n"
              << "Timeout    : " << opts.timeout << "s  "
              << "Iterations : " << opts.iterations << "  "
              << "Seed       : " << opts.seed << "\n"
              << "Note: exact solvers (forward_dp/bidir_dp/pulse) skipped on >12-node instances\n\n";

    SolverConfig cfg;
    cfg.seed           = opts.seed;
    cfg.max_cpu_time   = opts.timeout;
    cfg.max_iterations = opts.iterations;
    cfg.verbose        = false;

    // Prepare output folder
    if (!fs_compat::exists(opts.output))
        fs_compat::create_dir(opts.output);

    std::string csv_path = opts.output + "/benchmark_all_" + today_str() + ".csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "[WARN] Cannot open CSV: " << csv_path << "\n";
    } else {
        write_csv_header(csv);
    }

    // Print table header
    const int W_SOLVER = 12;
    const int W_INST   = 14;
    print_header(W_SOLVER, W_INST);

    int ok = 0, skip = 0, err = 0, done = 0;
    std::string prev_variant;

    for (const auto& inst_desc : instances) {
        // Separator between variants
        if (inst_desc.variant != prev_variant && !prev_variant.empty())
            print_separator(W_SOLVER, W_INST);
        prev_variant = inst_desc.variant;

        // Build problem once, reuse across solvers
        auto problem = inst_desc.make();

        for (const auto& [sname, sfactory] : SOLVER_LIST) {
            ++done;
            std::cout << std::flush; // keep terminal responsive

            auto r = run_solver(sname, sfactory, inst_desc, *problem, cfg);

            print_row(r, W_SOLVER, W_INST);
            if (csv.is_open()) write_csv_row(csv, r);

            if      (r.status == "OK")                  ++ok;
            else if (r.status.rfind("SKIP", 0) == 0)   ++skip;
            else                                         ++err;
        }
    }

    print_separator(W_SOLVER, W_INST);
    if (csv.is_open()) {
        csv.close();
        std::cout << "\nCSV saved : " << csv_path << "\n";
    }

    std::cout << "\n=== Summary ===\n"
              << "Total  : " << done << "\n"
              << "OK     : " << ok   << "\n"
              << "Skip   : " << skip << " (exact solvers on large instances)\n"
              << "Error  : " << err  << "\n";

    return (err > 0) ? 1 : 0;
}
