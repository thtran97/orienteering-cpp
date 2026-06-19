// bench_rr_strategies — rapid RR enhancement evaluation
//
// USAGE
//   ./bench_rr_strategies [--variant toptw] [--instance data/toptw/Cordeau]
//                         [--timeout 10] [--quick 5] [--pool-runs 5]
//
// DESIGN
//   For each (instance, vehicle-count) pair:
//
//   1. POOL-BUILD phase  — run ILS09 --pool-runs times, each for
//                          (--timeout / --pool-runs) seconds, with seeds
//                          seed+0 … seed+(K-1).  Collect distinct best
//                          solutions into an elite pool.
//                          Baseline = best reward across all K runs.
//
//   2. EVAL phase        — apply every RR strategy config to the SAME pool.
//                          Near-instant (~few ms each).
//
//   Final score = max(baseline, strategy_result).
//   Delta       = final_score - baseline  (≥ 0 by construction).
//
//   This separation is the key difference from bench_cordeau_comparison:
//   the pool here has NEVER been used for recombination, so strategies can
//   genuinely improve over the baseline.
//
// POOL DIVERSITY FILTER
//   Jaccard similarity: shared / |A ∪ B|.  A solution is rejected if its
//   Jaccard similarity with any existing pool member exceeds 0.5.
//
// STRATEGIES TESTED
//   baseline  — total-reward sort, no replace(), no pre-makespan
//   E1        — replace() after minimize_makespan()
//   E2        — density sort (reward / num_customers)
//   E5        — minimize_makespan() before repair()
//   E1+E2
//   E1+E5
//   E1+E2+E5
//   E1+E3(5)  — 5 random sub-pool attempts, best kept  (on top of E1)

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "bench_utils.h"
#include "solver/metaheuristic/ils09.h"
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
// Pool management
// ============================================================================

static double jaccard(const model::Solution& a, const model::Solution& b,
                      int num_nodes)
{
    std::vector<bool> in_a(num_nodes, false), in_b(num_nodes, false);
    for (int v = 0; v < a.get_num_vehicles(); ++v)
        for (NodeId n : a.get_route(v)) in_a[n] = true;
    for (int v = 0; v < b.get_num_vehicles(); ++v)
        for (NodeId n : b.get_route(v)) in_b[n] = true;

    int shared = 0, uni = 0;
    for (int i = 0; i < num_nodes; ++i) {
        if (in_a[i] || in_b[i]) ++uni;
        if (in_a[i] && in_b[i]) ++shared;
    }
    return uni == 0 ? 1.0 : static_cast<double>(shared) / uni;
}

static void pool_add(std::vector<model::Solution>& pool,
                     const model::Solution&         sol,
                     int                            pool_size,
                     int                            num_nodes,
                     double                         sim_thresh = 0.5)
{
    for (const auto& m : pool)
        if (jaccard(sol, m, num_nodes) > sim_thresh) return;

    if (static_cast<int>(pool.size()) < pool_size) {
        pool.push_back(sol);
    } else {
        auto worst = std::min_element(pool.begin(), pool.end(),
            [](const model::Solution& x, const model::Solution& y){
                return x.total_reward < y.total_reward; });
        if (sol.total_reward > worst->total_reward) *worst = sol;
    }
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
// Evaluate one strategy on a given pool + baseline
// ============================================================================

static double eval_strategy(
    const model::Problem&               problem,
    const std::vector<model::Solution>& pool,
    double                              baseline,
    const Strategy&                     strategy,
    local_search::BaseLSUtils&          ls,
    const local_search::LSConfig&       ls_cfg,
    oplib::utils::Random&               rng)
{
    if (pool.empty()) return baseline;

    model::Solution best_rr;
    bool first = true;

    auto try_pool = [&](const std::vector<model::Solution>& p) {
        auto s = ILSRouteRecombinationSolver::recombine_routes(
            problem, p, ls, ls_cfg, strategy.cfg);
        if (first || s.total_reward > best_rr.total_reward) {
            best_rr = s; first = false;
        }
    };

    try_pool(pool);
    for (int t = 1; t < strategy.attempts; ++t)
        try_pool(subsample(pool, rng));

    return std::max(baseline, best_rr.total_reward);
}

// ============================================================================
// CSV helpers
// ============================================================================

static void write_csv_header(std::ofstream& f,
                              const std::vector<Strategy>& strategies)
{
    f << "Instance,Nodes,Vehicles,PoolSize,ILS09_best";
    for (const auto& s : strategies) f << ',' << s.name;
    f << '\n';
}

static void write_csv_row(std::ofstream& f,
                           const std::string& inst, int nodes, int veh,
                           int pool_sz, double baseline,
                           const std::vector<double>& scores)
{
    f << std::fixed << std::setprecision(2)
      << inst << ',' << nodes << ',' << veh << ',' << pool_sz << ',' << baseline;
    for (double s : scores) f << ',' << s;
    f << '\n';
}

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv)
{
    // ---- Parse CLI ----------------------------------------------------------
    auto opts = bench::parse_cli(argc, argv, "rr_strategies");
    if (opts.variants.empty() ||
        opts.variants.size() == bench::ALL_VARIANTS.size())
        opts.variants = {"toptw"};

    // --pool-runs N: how many ILS09 seeds to use for pool building
    // (parsed from unknown args — bench::parse_cli ignores unknown flags)
    int pool_runs = 5;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--pool-runs" && i + 1 < argc)
            pool_runs = std::stoi(argv[++i]);
    }

    const double time_per_seed = opts.timeout / pool_runs;

    std::vector<int> vehicle_counts;
    if (opts.vehicles > 0) vehicle_counts.push_back(opts.vehicles);
    else                   vehicle_counts = {2, 3, 4};

    const auto strategies = make_strategies();
    const int  NS = static_cast<int>(strategies.size());
    const int  POOL_SIZE = 10;

    // ---- Discover instances -------------------------------------------------
    auto specs = bench::discover_instances(opts.instance_path, opts.variants);
    if (specs.empty()) { std::cerr << "No instances found.\n"; return 1; }
    if (opts.quick > 0) specs = bench::quick_filter(std::move(specs), opts.quick);

    std::cout << "Instances: " << specs.size()
              << "  Vehicles: ";
    for (int v : vehicle_counts) std::cout << v << ' ';
    std::cout << "\nPool: " << pool_runs << " ILS09 seeds × "
              << std::fixed << std::setprecision(1) << time_per_seed << "s each"
              << "  (total per instance: " << opts.timeout << "s)\n\n";

    // ---- Table header -------------------------------------------------------
    const int WI = 10, WV = 3, WB = 8, WS = 10;
    auto sep_line = [&]{
        std::cout << std::string(WI,'-') << '+' << std::string(WV+1,'-') << '+';
        std::cout << std::string(WB+1,'-');
        for (int i = 0; i < NS; ++i) std::cout << '+' << std::string(WS,'-');
        std::cout << '\n';
    };
    sep_line();
    std::cout << std::left  << std::setw(WI) << "Instance"
              << '|' << std::right << std::setw(WV) << "Veh"
              << '|' << std::right << std::setw(WB) << "ILS09";
    for (const auto& s : strategies)
        std::cout << '|' << std::right << std::setw(WS) << s.name;
    std::cout << '\n';
    sep_line();

    // ---- Open CSV -----------------------------------------------------------
    fs::create_directories("results/" + opts.variants[0]);
    std::ofstream csv("results/" + opts.variants[0] + "/benchmark_rr_strategies.csv",
                      std::ios::out | std::ios::trunc);
    write_csv_header(csv, strategies);

    // ---- Accumulators -------------------------------------------------------
    double        sum_base = 0.0;
    std::vector<double> sum_strat(NS, 0.0);
    int           total_rows = 0;

    // ---- ILS09 config -------------------------------------------------------
    ILS09SolverConfig ils09_cfg;
    ils09_cfg.max_cpu_time      = time_per_seed;
    ils09_cfg.max_iterations    = 0;
    ils09_cfg.alpha             = 3;
    ils09_cfg.rcl_size          = 5;
    ils09_cfg.restart_threshold = 10;

    ILS09Solver ils09;

    // ---- Main loop ----------------------------------------------------------
    for (const auto& spec : specs) {
        const std::string inst_name = fs::path(spec.filepath).stem().string();

        for (int nv : vehicle_counts) {
            auto problem = bench::parse_instance(spec);
            if (!problem) continue;
            bench::Options veh_opts; veh_opts.vehicles = nv;
            bench::apply_overrides(veh_opts, *problem);
            const int nn = problem->get_num_nodes();

            // --- POOL-BUILD phase -------------------------------------------
            std::vector<model::Solution> pool;
            double baseline = 0.0;

            for (int k = 0; k < pool_runs; ++k) {
                ils09_cfg.seed = opts.seed + k;
                model::Solution sol = ils09.solve(*problem, ils09_cfg);
                if (sol.total_reward > baseline) baseline = sol.total_reward;
                pool_add(pool, sol, POOL_SIZE, nn);
            }

            // --- EVAL phase -------------------------------------------------
            oplib::utils::Random eval_rng(opts.seed ^ 0xABCD1234u);
            oplib::utils::Random ls_rng  (opts.seed ^ 0x5A5A5A5Au);
            local_search::BaseLSUtils ls(*problem, ls_rng);
            local_search::LSConfig    ls_cfg;
            ls_cfg.alpha    = ils09_cfg.alpha;
            ls_cfg.rcl_size = ils09_cfg.rcl_size;

            std::vector<double> scores(NS);
            for (int si = 0; si < NS; ++si)
                scores[si] = eval_strategy(*problem, pool, baseline,
                                           strategies[si], ls, ls_cfg, eval_rng);

            // --- Record & print ---------------------------------------------
            write_csv_row(csv, inst_name, nn, nv,
                          static_cast<int>(pool.size()), baseline, scores);
            csv.flush();

            sum_base += baseline;
            for (int si = 0; si < NS; ++si) sum_strat[si] += scores[si];
            ++total_rows;

            std::cout << std::left  << std::setw(WI) << inst_name
                      << '|' << std::right << std::setw(WV) << nv
                      << '|' << std::fixed << std::setprecision(0)
                      << std::right << std::setw(WB) << baseline;
            for (int si = 0; si < NS; ++si) {
                double delta = scores[si] - baseline;
                std::ostringstream cell;
                if (delta > 0.5)
                    cell << std::fixed << std::setprecision(0) << "+" << delta;
                else
                    cell << "=";
                std::cout << '|' << std::right << std::setw(WS) << cell.str();
            }
            std::cout << '\n';
            std::cout.flush();
        }
    }

    // ---- Summary ------------------------------------------------------------
    sep_line();
    if (total_rows > 0) {
        std::cout << std::left  << std::setw(WI) << "AVG_GAIN"
                  << '|' << std::right << std::setw(WV) << "-"
                  << '|' << std::fixed << std::setprecision(1)
                  << std::right << std::setw(WB) << sum_base / total_rows;
        for (int si = 0; si < NS; ++si) {
            double avg_delta = (sum_strat[si] - sum_base) / total_rows;
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
