// bench_rr_strategies — rapid RR enhancement evaluation
//
// USAGE
//   ./bench_rr_strategies [--variant toptw] [--instance data/toptw/Cordeau]
//                         [--timeout 10] [--quick 5] [--vehicles 2,3,4]
//
// DESIGN
//   For each (instance, vehicle-count) pair:
//     1. BUILD phase  — run ILS+RR for --timeout seconds.
//                       Captures: best solution, elite pool.
//     2. EVAL phase   — apply every RR strategy config to the SAME pool.
//                       Near-instant: each call costs ~few ms.
//
//   The final score for each strategy is  max(ILS+RR_best, strategy_result),
//   mirroring the post-processing guarantee.
//
// OUTPUT
//   Compact per-row table + per-strategy summary at the bottom.
//   CSV written to  results/<variant>/benchmark_rr_strategies.csv
//
// STRATEGIES TESTED
//   baseline  — original: total-reward sort, no replace(), no pre-makespan
//   E1        — replace() after minimize_makespan()
//   E2        — density sort (reward/customers)
//   E5        — minimize_makespan() before repair()
//   E1+E2
//   E1+E5
//   E1+E2+E5
//   E3(k=5)   — 5 random sub-pool attempts, best kept  (applied on top of E1)

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "bench_utils.h"
#include "solver/metaheuristic/ils_route_recombination.h"
#include "core/random.h"

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::solver::metaheuristic;

// ============================================================================
// Strategy definitions
// ============================================================================

struct Strategy {
    std::string name;
    RRConfig    cfg;
    int         attempts = 1;   // E3: how many sub-pool samples to try
};

static std::vector<Strategy> make_strategies()
{
    return {
        {"baseline",  RRConfig{false, false, false}, 1},
        {"E1",        RRConfig{true,  false, false}, 1},
        {"E2",        RRConfig{false, true,  false}, 1},
        {"E5",        RRConfig{false, false, true},  1},
        {"E1+E2",     RRConfig{true,  true,  false}, 1},
        {"E1+E5",     RRConfig{true,  false, true},  1},
        {"E1+E2+E5",  RRConfig{true,  true,  true},  1},
        {"E1+E3(5)",  RRConfig{true,  false, false}, 5},
    };
}

// ============================================================================
// Sub-pool sampling for E3
// ============================================================================

static std::vector<model::Solution> subsample(
    const std::vector<model::Solution>& pool,
    oplib::utils::Random& rng,
    double keep_ratio = 0.6)
{
    if (pool.empty()) return pool;
    int k = std::max(1, static_cast<int>(pool.size() * keep_ratio));
    std::vector<int> idx(pool.size());
    std::iota(idx.begin(), idx.end(), 0);
    // Fisher-Yates partial shuffle
    for (int i = 0; i < k; ++i) {
        int j = i + rng.next_int(0, static_cast<int>(idx.size()) - 1 - i);
        std::swap(idx[i], idx[j]);
    }
    std::vector<model::Solution> sub;
    sub.reserve(k);
    for (int i = 0; i < k; ++i) sub.push_back(pool[idx[i]]);
    return sub;
}

// ============================================================================
// Evaluate one strategy on a given pool+best
// ============================================================================

static double eval_strategy(
    const model::Problem&               problem,
    const std::vector<model::Solution>& pool,
    double                              ils_rr_best,
    const Strategy&                     strategy,
    local_search::BaseLSUtils&          ls,
    const local_search::LSConfig&       ls_cfg,
    oplib::utils::Random&               rng)
{
    if (pool.empty()) return ils_rr_best;

    model::Solution best_rr;
    bool first = true;

    auto try_pool = [&](const std::vector<model::Solution>& p) {
        auto s = ILSRouteRecombinationSolver::recombine_routes(
            problem, p, ls, ls_cfg, strategy.cfg);
        if (first || s.total_reward > best_rr.total_reward) {
            best_rr = s;
            first   = false;
        }
    };

    try_pool(pool);  // always include a full-pool attempt
    for (int t = 1; t < strategy.attempts; ++t)
        try_pool(subsample(pool, rng));

    return std::max(ils_rr_best, best_rr.total_reward);
}

// ============================================================================
// CSV helpers
// ============================================================================

static void write_csv_header(std::ofstream& f,
                              const std::vector<Strategy>& strategies)
{
    f << "Instance,Nodes,Vehicles,ILS_RR_best";
    for (const auto& s : strategies) f << ',' << s.name;
    f << '\n';
}

static void write_csv_row(std::ofstream& f,
                           const std::string& inst, int nodes, int veh,
                           double ils_rr_best,
                           const std::vector<double>& scores)
{
    f << std::fixed << std::setprecision(2)
      << inst << ',' << nodes << ',' << veh << ',' << ils_rr_best;
    for (double s : scores) f << ',' << s;
    f << '\n';
}

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv)
{
    auto opts = bench::parse_cli(argc, argv, "rr_strategies");

    if (opts.variants.empty() ||
        (opts.variants.size() == bench::ALL_VARIANTS.size()))
        opts.variants = {"toptw"};

    const auto strategies = make_strategies();
    const int  NS = static_cast<int>(strategies.size());

    // ---- Vehicle counts from --vehicles override, otherwise 2,3,4 ----------
    std::vector<int> vehicle_counts;
    if (opts.vehicles > 0) {
        vehicle_counts.push_back(opts.vehicles);
    } else {
        vehicle_counts = {2, 3, 4};
    }

    // ---- Discover instances -------------------------------------------------
    auto specs = bench::discover_instances(opts.instance_path, opts.variants);
    if (specs.empty()) { std::cerr << "No instances found.\n"; return 1; }
    if (opts.quick > 0) specs = bench::quick_filter(std::move(specs), opts.quick);

    std::cout << "Instances: " << specs.size()
              << "  Vehicles: ";
    for (int v : vehicle_counts) std::cout << v << ' ';
    std::cout << " Build-time: " << opts.timeout << "s\n\n";

    // ---- Strategy column header (stdout) ------------------------------------
    const int WI = 10, WV = 3, WB = 9, WS = 9;
    auto sep_line = [&]{
        std::cout << std::string(WI,'-') << '+' << std::string(WV+1,'-') << '+';
        std::cout << std::string(WB+1,'-');
        for (int i = 0; i < NS; ++i) std::cout << '+' << std::string(WS,'-');
        std::cout << '\n';
    };
    sep_line();
    std::cout << std::left << std::setw(WI) << "Instance"
              << '|' << std::right << std::setw(WV) << "Veh"
              << '|' << std::right << std::setw(WB) << "ILS+RR";
    for (const auto& s : strategies)
        std::cout << '|' << std::right << std::setw(WS) << s.name;
    std::cout << '\n';
    sep_line();

    // ---- Open CSV -----------------------------------------------------------
    fs::create_directories("results/" + opts.variants[0]);
    std::ofstream csv("results/" + opts.variants[0] + "/benchmark_rr_strategies.csv",
                      std::ios::out | std::ios::trunc);
    write_csv_header(csv, strategies);

    // ---- Per-instance accumulators ------------------------------------------
    std::vector<double> sum_ils_rr(1, 0.0);
    std::vector<double> sum_strat(NS, 0.0);
    int total_rows = 0;

    // ---- Main loop ----------------------------------------------------------
    ILSRouteRecombinationSolverConfig build_cfg;
    build_cfg.seed               = opts.seed;
    build_cfg.max_cpu_time       = opts.timeout;
    build_cfg.max_iterations     = 0;
    build_cfg.alpha              = 3;
    build_cfg.rcl_size           = 5;
    build_cfg.restart_threshold  = 10;
    build_cfg.pool_size          = 10;
    build_cfg.similarity_threshold = 0.5;

    oplib::utils::Random eval_rng(build_cfg.seed ^ 0xDEADBEEF);

    ILSRouteRecombinationSolver solver;

    for (const auto& spec : specs) {
        const std::string inst_name = fs::path(spec.filepath).stem().string();

        for (int nv : vehicle_counts) {
            auto problem = bench::parse_instance(spec);
            if (!problem) continue;
            bench::Options veh_opts; veh_opts.vehicles = nv;
            bench::apply_overrides(veh_opts, *problem);

            // --- BUILD phase ------------------------------------------------
            build_cfg.seed = opts.seed;  // reset seed per instance
            model::Solution best = solver.solve(*problem, build_cfg);
            double ils_rr_best   = best.total_reward;
            const auto& pool     = solver.get_last_pool();

            oplib::utils::Random local_rng(opts.seed ^ 0xCAFEBABE);
            local_search::BaseLSUtils ls(*problem, local_rng);
            local_search::LSConfig    ls_cfg;
            ls_cfg.alpha    = build_cfg.alpha;
            ls_cfg.rcl_size = build_cfg.rcl_size;

            // --- EVAL phase -------------------------------------------------
            std::vector<double> scores(NS);
            for (int si = 0; si < NS; ++si)
                scores[si] = eval_strategy(*problem, pool, ils_rr_best,
                                           strategies[si], ls, ls_cfg, eval_rng);

            // --- Record -----------------------------------------------------
            write_csv_row(csv, inst_name, problem->get_num_nodes(), nv,
                          ils_rr_best, scores);
            csv.flush();

            sum_ils_rr[0] += ils_rr_best;
            for (int si = 0; si < NS; ++si) sum_strat[si] += scores[si];
            ++total_rows;

            // --- Print row --------------------------------------------------
            std::cout << std::left  << std::setw(WI) << inst_name
                      << '|' << std::right << std::setw(WV) << nv
                      << '|' << std::fixed << std::setprecision(0)
                      << std::right << std::setw(WB) << ils_rr_best;
            for (int si = 0; si < NS; ++si) {
                double delta = scores[si] - ils_rr_best;
                std::ostringstream cell;
                if (delta > 0.5)       cell << std::fixed << std::setprecision(0) << "+" << delta;
                else if (delta < -0.5) cell << std::fixed << std::setprecision(0) << delta;
                else                   cell << "=";
                std::cout << '|' << std::right << std::setw(WS) << cell.str();
            }
            std::cout << '\n';
            std::cout.flush();
        }
    }

    // ---- Summary ------------------------------------------------------------
    sep_line();
    if (total_rows > 0) {
        std::cout << std::left << std::setw(WI) << "AVG_GAIN"
                  << '|' << std::right << std::setw(WV) << "-"
                  << '|' << std::fixed << std::setprecision(1)
                  << std::right << std::setw(WB) << sum_ils_rr[0]/total_rows;
        for (int si = 0; si < NS; ++si) {
            double avg_delta = (sum_strat[si] - sum_ils_rr[0]) / total_rows;
            std::ostringstream cell;
            cell << std::fixed << std::setprecision(1)
                 << (avg_delta >= 0 ? "+" : "") << avg_delta;
            std::cout << '|' << std::right << std::setw(WS) << cell.str();
        }
        std::cout << '\n';
    }
    sep_line();
    std::cout << "\nRows: " << total_rows
              << "  CSV: results/" << opts.variants[0]
              << "/benchmark_rr_strategies.csv\n";

    csv.close();
    return 0;
}
