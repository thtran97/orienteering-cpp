/**
 * @file benchmark.cpp
 * @brief Unified benchmark runner for all orienteering solvers.
 *
 * Usage:
 *   benchmark [options]
 *
 * Options:
 *   --solver   -s  <name,...>   Comma-separated solvers (default: all)
 *   --variant  -v  <name,...>   Problem variants: op,top,toptw,tdop,tdoptw,
 *                               ttdp,mctopmtw,singlesat  (default: all)
 *   --instance -i  <path>       File or folder of instances (default: data/)
 *   --timeout  -t  <seconds>    Time limit per solver/instance (default: 60)
 *   --iterations   <count>      Max iterations per solver (default: 1000)
 *   --seed         <int>        Random seed (default: 42)
 *   --runs         <count>      Runs per solver/instance (default: 1)
 *   --output   -o  <folder>     Output folder for CSV (default: results/)
 *   --verbose                   Pass verbose flag to each solver
 *   --no-progress               Suppress per-instance progress output
 *   --help     -h               Show this help and exit
 *
 * Available solver names:
 *   greedy, rand_greedy, grasp_vns, lns, ils09, ils_rr,
 *   mcts, forward_dp, bidir_dp, pulse
 *
 * Example — run LNS and ILS09 on all TOP instances, 30-second limit:
 *   benchmark -s lns,ils09 -v top -t 30 -i data/
 */

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
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
#include "model/variants/op.h"
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

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver;

// ============================================================================
// Constants
// ============================================================================

static const std::vector<std::string> ALL_VARIANTS = {
    "op", "top", "toptw", "tdop", "tdoptw", "ttdp", "mctopmtw", "singlesat"};

static const std::vector<std::string> ALL_SOLVERS = {
    "greedy",    "rand_greedy", "grasp_vns", "lns",       "ils09",
    "ils_rr",   "mcts",        "forward_dp","bidir_dp",  "pulse"};

// ============================================================================
// CLI options
// ============================================================================

struct BenchmarkOptions {
    std::vector<std::string> solvers    = ALL_SOLVERS;
    std::vector<std::string> variants   = ALL_VARIANTS;
    std::string              instance   = "data";
    double                   timeout    = 60.0;
    int                      iterations = 1000;
    int                      seed       = 42;
    int                      runs       = 1;
    std::string              output     = "results";
    bool                     verbose    = false;
    bool                     no_progress = false;
    bool                     overwrite  = false; ///< if true, ignore existing CSV and re-run everything
    int                      quick      = 0;     ///< >0: run only this many instances per variant
};

// ============================================================================
// Result record
// ============================================================================

struct RunResult {
    std::string solver;
    std::string instance_name;
    std::string variant;
    int         num_nodes        = 0;
    int         num_vehicles     = 1;
    bool        has_time_windows = false;
    bool        is_time_dependent = false;
    double      budget           = 0.0;
    int         run_id           = 1;
    double      reward           = 0.0;
    double      travel_time      = 0.0;
    int         customers_visited = 0;
    double      cpu_ms           = 0.0;
    std::string status           = "OK"; // OK | ERROR
};

// ============================================================================
// Utility helpers
// ============================================================================

static std::vector<std::string> split_csv(const std::string& s)
{
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (!token.empty()) out.push_back(token);
    }
    return out;
}

static std::string today_str()
{
    auto now  = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[16];
    std::strftime(buf, sizeof(buf), "%y%m%d", &tm);
    return buf;
}

static std::string make_csv_name(const std::string& output_folder,
                                  const std::string& instance_path)
{
    std::string base = fs::path(instance_path).filename().string();
    if (base.empty()) base = fs::path(instance_path).parent_path().filename().string();
    std::string fname = "benchmark_" + base + "_" + today_str() + ".csv";
    return (fs::path(output_folder) / fname).string();
}

// ============================================================================
// Instance discovery
// ============================================================================

struct InstanceSpec {
    std::string filepath;
    std::string variant; // detected from directory hierarchy
    // Companion files for multi-file parsers (TDOP, TDOPTW)
    std::string aux1;   // speed_matrix (TDOP) | transition_matrix (TDOPTW)
    std::string aux2;   // arc_category (TDOP)
};

/// Detect variant from file path by matching directory names.
static std::string detect_variant(const fs::path& filepath,
                                   const std::vector<std::string>& known)
{
    fs::path p = filepath.parent_path();
    while (!p.empty() && p != p.parent_path()) {
        std::string name = p.filename().string();
        // lowercase comparison
        std::string lower = name;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        for (const auto& v : known)
            if (lower == v) return v;
        p = p.parent_path();
    }
    return ""; // unknown
}

/// Discover TDOP instances under `tdop_root`.
/// Structure: tdop_root/dataset<N>/OP_instances/*.txt
///            tdop_root/speedmatrix.txt
///            tdop_root/dataset<N>/arc_cat_<N>.txt
static std::vector<InstanceSpec> find_tdop_instances(const fs::path& tdop_root)
{
    std::vector<InstanceSpec> result;
    if (!fs::is_directory(tdop_root)) return result;

    std::string speed_matrix =
        (tdop_root / "speedmatrix.txt").string();
    if (!fs::exists(speed_matrix)) return result;

    for (const auto& ds : fs::directory_iterator(tdop_root)) {
        if (!ds.is_directory()) continue;
        std::string ds_name = ds.path().filename().string();
        // find arc_cat file in dataset dir (arc_cat_<N>.txt)
        std::string arc_cat;
        for (const auto& e : fs::directory_iterator(ds.path())) {
            std::string n = e.path().filename().string();
            if (n.rfind("arc_cat", 0) == 0) { arc_cat = e.path().string(); break; }
        }
        if (arc_cat.empty()) continue;

        fs::path op_dir = ds.path() / "OP_instances";
        if (!fs::is_directory(op_dir)) continue;

        for (const auto& inst : fs::directory_iterator(op_dir)) {
            if (!inst.is_regular_file()) continue;
            result.push_back({inst.path().string(), "tdop",
                              speed_matrix, arc_cat});
        }
    }
    std::sort(result.begin(), result.end(),
              [](const InstanceSpec& a, const InstanceSpec& b){
                  return a.filepath < b.filepath;
              });
    return result;
}

/// Discover TDOPTW instances under `tdoptw_root`.
/// Instance files: <size>.<set>.<n>.TXT  (exclude titt*.TXT, tt*.TXT)
/// Transition matrix: titt<size>.TXT
static std::vector<InstanceSpec> find_tdoptw_instances(const fs::path& tdoptw_root)
{
    std::vector<InstanceSpec> result;
    if (!fs::is_directory(tdoptw_root)) return result;

    // Build map: size_string -> transition_matrix path
    std::map<std::string, std::string> trans_map;
    for (const auto& e : fs::directory_iterator(tdoptw_root)) {
        if (!e.is_regular_file()) continue;
        std::string n = e.path().filename().string();
        if (n.rfind("titt", 0) == 0) {
            // e.g. "titt20.TXT" -> key "20"
            std::string size_str = n.substr(4); // strip "titt"
            // strip extension
            auto dot = size_str.find('.');
            if (dot != std::string::npos) size_str = size_str.substr(0, dot);
            trans_map[size_str] = e.path().string();
        }
    }

    for (const auto& e : fs::directory_iterator(tdoptw_root)) {
        if (!e.is_regular_file()) continue;
        std::string n = e.path().filename().string();
        // Skip transition/speed matrices and format files
        if (n.rfind("titt", 0) == 0) continue;
        if (n.rfind("tt",   0) == 0) continue;
        if (n.rfind("TD_",  0) == 0) continue;
        if (n.rfind("OPTW_",0) == 0) continue;

        // Instance filename: <size>.<set>.<n>.TXT  -> first component is size
        auto dot = n.find('.');
        if (dot == std::string::npos) continue;
        std::string size_str = n.substr(0, dot);

        auto it = trans_map.find(size_str);
        if (it == trans_map.end()) continue;

        result.push_back({e.path().string(), "tdoptw", it->second, ""});
    }
    std::sort(result.begin(), result.end(),
              [](const InstanceSpec& a, const InstanceSpec& b){
                  return a.filepath < b.filepath;
              });
    return result;
}

/// Collect all regular files under `root` whose detected variant is in `variants`.
static std::vector<InstanceSpec> find_instances(
    const std::string& root_path,
    const std::vector<std::string>& variants)
{
    std::vector<InstanceSpec> result;
    fs::path root(root_path);

    if (!fs::exists(root)) {
        std::cerr << "[ERROR] Path does not exist: " << root_path << '\n';
        return result;
    }

    bool want_tdop   = variants.empty()
                       || std::find(variants.begin(), variants.end(), "tdop")   != variants.end();
    bool want_tdoptw = variants.empty()
                       || std::find(variants.begin(), variants.end(), "tdoptw") != variants.end();

    auto add_file = [&](const fs::path& fp) {
        std::string var = detect_variant(fp, ALL_VARIANTS);
        if (var.empty()) return;
        // TDOP/TDOPTW handled separately
        if (var == "tdop" || var == "tdoptw") return;
        if (!variants.empty() &&
            std::find(variants.begin(), variants.end(), var) == variants.end())
            return;
        result.push_back({fp.string(), var});
    };

    if (fs::is_regular_file(root)) {
        // Single file: try all enabled single-file variants
        std::string var = detect_variant(root, ALL_VARIANTS);
        if (!var.empty() && var != "tdop" && var != "tdoptw") {
            if (variants.empty() ||
                std::find(variants.begin(), variants.end(), var) != variants.end())
                result.push_back({root.string(), var});
        }
    } else if (fs::is_directory(root)) {
        // Single-file variants
        for (const auto& entry : fs::recursive_directory_iterator(root)) {
            if (entry.is_regular_file()) add_file(entry.path());
        }
        // TDOP: look for tdop/ sub-directory
        if (want_tdop) {
            fs::path tdop_dir = root / "tdop";
            if (!fs::exists(tdop_dir)) tdop_dir = root; // maybe root IS tdop/
            auto tdop = find_tdop_instances(tdop_dir);
            result.insert(result.end(), tdop.begin(), tdop.end());
        }
        // TDOPTW: look for tdoptw/ sub-directory
        if (want_tdoptw) {
            fs::path tdoptw_dir = root / "tdoptw";
            if (!fs::exists(tdoptw_dir)) tdoptw_dir = root;
            auto tdoptw = find_tdoptw_instances(tdoptw_dir);
            result.insert(result.end(), tdoptw.begin(), tdoptw.end());
        }
        std::sort(result.begin(), result.end(),
                  [](const InstanceSpec& a, const InstanceSpec& b){
                      return a.filepath < b.filepath;
                  });
    }
    return result;
}

// ============================================================================
// Parsing
// ============================================================================

static std::unique_ptr<Problem> parse_instance(const InstanceSpec& spec)
{
    const std::string& path = spec.filepath;
    try {
        if (spec.variant == "op")        { OPParser p;        return p.read(path); }
        if (spec.variant == "top")       { TOPParser p;       return p.read(path); }
        if (spec.variant == "toptw")     { TOPTWParser p;     return p.read(path); }
        if (spec.variant == "ttdp")      { TTDPParser p;      return p.read(path); }
        if (spec.variant == "mctopmtw")  { MCTOPMTWParser p;  return p.read(path); }
        if (spec.variant == "singlesat") { SingleSatParser p; return p.read(path); }
        if (spec.variant == "tdop") {
            if (spec.aux1.empty() || spec.aux2.empty())
                throw std::runtime_error("TDOP requires speed_matrix and arc_cat companion files");
            return TDOPParser::read(path, spec.aux1, spec.aux2);
        }
        if (spec.variant == "tdoptw") {
            if (spec.aux1.empty())
                throw std::runtime_error("TDOPTW requires transition_matrix companion file");
            return TDOPTWParser::read(path, spec.aux1);
        }
    } catch (const std::exception& e) {
        std::cerr << "[WARN] Parse error (" << spec.variant << "): "
                  << spec.filepath << " — " << e.what() << '\n';
    }
    return nullptr;
}

// ============================================================================
// Solver factory
// ============================================================================

using SolverFactory = std::function<std::unique_ptr<Solver>()>;

static const std::map<std::string, SolverFactory>& solver_registry()
{
    static const std::map<std::string, SolverFactory> reg = {
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
    return reg;
}

// ============================================================================
// Single run
// ============================================================================

static RunResult run_once(const std::string& solver_name,
                           const InstanceSpec& spec,
                           const Problem& problem,
                           const BenchmarkOptions& opts,
                           int run_id)
{
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
    r.status          = "OK";

    const auto& reg = solver_registry();
    auto it = reg.find(solver_name);
    if (it == reg.end()) {
        r.status = "ERROR"; return r;
    }

    auto solver = it->second();

    SolverConfig cfg;
    cfg.seed           = opts.seed + run_id - 1; // different seed per run
    cfg.max_cpu_time   = opts.timeout;
    cfg.max_iterations = opts.iterations;
    cfg.verbose        = opts.verbose;

    try {
        auto t0 = std::chrono::high_resolution_clock::now();
        Solution sol = solver->solve(problem, cfg);
        auto t1 = std::chrono::high_resolution_clock::now();

        r.cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        r.reward        = sol.total_reward;
        r.travel_time   = sol.total_travel_time;

        for (const auto& route : sol.get_routes())
            r.customers_visited += std::max(0, static_cast<int>(route.size()) - 2);

    } catch (const std::exception& e) {
        r.status = std::string("ERROR:") + e.what();
    }

    return r;
}

// ============================================================================
// Resume: load already-completed (solver, instance_name, run_id) keys
// ============================================================================

/// Opaque key identifying one completed run inside a CSV file.
using DoneKey = std::tuple<std::string /*solver*/,
                            std::string /*instance_name*/,
                            int         /*run_id*/>;

/// Read an existing CSV and return the set of already-completed run keys.
/// Rows with status starting with "ERROR" are also considered done so they
/// are not retried unless --overwrite is passed.
static std::set<DoneKey> load_done_keys(const std::string& csv_path)
{
    std::set<DoneKey> done;
    std::ifstream in(csv_path);
    if (!in.is_open()) return done;

    std::string line;
    std::getline(in, line); // skip header
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        // Columns: Solver,Instance,Variant,Nodes,Vehicles,HasTW,IsTimeDep,
        //          Budget,Run,Reward,TravelTime,CustomersVisited,CPU_ms,Status
        auto cols = split_csv(line);
        if (cols.size() < 9) continue;
        try {
            std::string solver   = cols[0];
            std::string instance = cols[1];
            int         run_id   = std::stoi(cols[8]);
            done.emplace(solver, instance, run_id);
        } catch (...) {}
    }
    return done;
}

// ============================================================================
// CSV I/O
// ============================================================================

static void write_header(std::ofstream& out)
{
    out << "Solver,Instance,Variant,Nodes,Vehicles,HasTW,IsTimeDep,"
           "Budget,Run,Reward,TravelTime,CustomersVisited,CPU_ms,Status\n";
}

static void write_row(std::ofstream& out, const RunResult& r)
{
    out << r.solver              << ','
        << r.instance_name       << ','
        << r.variant             << ','
        << r.num_nodes           << ','
        << r.num_vehicles        << ','
        << (r.has_time_windows   ? "yes" : "no") << ','
        << (r.is_time_dependent  ? "yes" : "no") << ','
        << std::fixed << std::setprecision(4)
        << r.budget              << ','
        << r.run_id              << ','
        << r.reward              << ','
        << r.travel_time         << ','
        << r.customers_visited   << ','
        << r.cpu_ms              << ','
        << r.status              << '\n';
}

// ============================================================================
// CLI parsing
// ============================================================================

static void print_help()
{
    std::cout <<
R"(Orienteering benchmark runner
Usage: benchmark [options]

Options:
  --solver    -s  <name[,...]>   Solvers to run. Comma-separated list or
                                 multiple -s flags. (default: all)
  --variant   -v  <name[,...]>   Variants to include. (default: all)
  --instance  -i  <path>         File or directory of instances.
                                 (default: data/)
  --timeout   -t  <sec>          CPU time limit per solver/instance run.
                                 (default: 60)
  --iterations    <n>            Max iterations per solver. (default: 1000)
  --seed          <n>            Base random seed. (default: 42)
  --runs          <n>            Runs per solver/instance pair. (default: 1)
  --output    -o  <folder>       Output folder for CSV. (default: results/)
  --overwrite                    Re-run all cases, ignoring any existing CSV.
                                 Without this flag the runner resumes from
                                 where it left off.
  --quick     -q  [<n>]          Run only the first n instances per variant
                                 (default n=3 when flag is present).
                                 Useful for a fast sanity-check of all solvers.
  --verbose                      Pass verbose flag to solvers.
  --no-progress                  Suppress per-instance progress output.
  --help      -h                 Show this help.

Available solvers:
  greedy      rand_greedy  grasp_vns  lns      ils09
  ils_rr      mcts         forward_dp bidir_dp pulse

Available variants:
  op  top  toptw  tdop  tdoptw  ttdp  mctopmtw  singlesat

Notes:
  - forward_dp, bidir_dp, pulse are exact solvers — exponential worst-case.
    Practical only for n <= ~20 nodes. Use --timeout to cap wall-clock time.
  - All solvers share the base Solver interface so can run on any variant,
    though multi-vehicle solvers (grasp_vns, lns, ils09, ils_rr) will use
    all available vehicles while single-vehicle ones use only vehicle 0.

Example:
  benchmark -s lns,ils09 -v top,toptw -t 30 -i data/ -o results/
)";
}

static BenchmarkOptions parse_args(int argc, char** argv)
{
    BenchmarkOptions opts;
    bool solvers_set = false;
    bool variants_set = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];

        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + a);
            return argv[++i];
        };

        if (a == "--help" || a == "-h") { print_help(); std::exit(0); }
        else if (a == "--verbose")       opts.verbose     = true;
        else if (a == "--overwrite")      opts.overwrite   = true;
        else if (a == "--no-progress")   opts.no_progress = true;
        else if (a == "--quick" || a == "-q") {
            // optional numeric argument: -q or -q 5
            if (i + 1 < argc && std::isdigit(static_cast<unsigned char>(argv[i+1][0])))
                opts.quick = std::stoi(argv[++i]);
            else
                opts.quick = 3; // default
        }
        else if (a == "--solver"  || a == "-s") {
            auto names = split_csv(next());
            if (!solvers_set) { opts.solvers.clear(); solvers_set = true; }
            for (auto& n : names) opts.solvers.push_back(n);
        }
        else if (a == "--variant" || a == "-v") {
            auto names = split_csv(next());
            if (!variants_set) { opts.variants.clear(); variants_set = true; }
            for (auto& n : names) opts.variants.push_back(n);
        }
        else if (a == "--instance"   || a == "-i") opts.instance   = next();
        else if (a == "--timeout"    || a == "-t") opts.timeout    = std::stod(next());
        else if (a == "--output"     || a == "-o") opts.output     = next();
        else if (a == "--iterations")              opts.iterations = std::stoi(next());
        else if (a == "--seed")                    opts.seed       = std::stoi(next());
        else if (a == "--runs")                    opts.runs       = std::stoi(next());
        else {
            std::cerr << "[WARN] Unknown argument: " << a << '\n';
        }
    }

    // Validate solver names
    const auto& reg = solver_registry();
    for (const auto& s : opts.solvers) {
        if (reg.find(s) == reg.end()) {
            std::cerr << "[ERROR] Unknown solver: '" << s << "'. Valid names:\n  ";
            for (const auto& kv : reg) std::cerr << kv.first << ' ';
            std::cerr << '\n';
            std::exit(1);
        }
    }

    // Validate variant names
    const std::set<std::string> valid_var(ALL_VARIANTS.begin(), ALL_VARIANTS.end());
    for (const auto& v : opts.variants) {
        if (!valid_var.count(v)) {
            std::cerr << "[ERROR] Unknown variant: '" << v << "'. Valid names:\n  ";
            for (auto& vn : ALL_VARIANTS) std::cerr << vn << ' ';
            std::cerr << '\n';
            std::exit(1);
        }
    }

    return opts;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
    BenchmarkOptions opts = parse_args(argc, argv);

    // --- Print configuration ------------------------------------------------
    std::cout << "=== Orienteering Benchmark ===\n";
    std::cout << "Solvers  : ";
    for (const auto& s : opts.solvers) std::cout << s << ' ';
    std::cout << "\nVariants : ";
    for (const auto& v : opts.variants) std::cout << v << ' ';
    std::cout << "\nInstance : " << opts.instance
              << "\nTimeout  : " << opts.timeout << "s"
              << "  Iterations: " << opts.iterations
              << "  Seed: " << opts.seed
              << "  Runs: " << opts.runs << '\n';

    // --- Discover instances -------------------------------------------------
    auto instances = find_instances(opts.instance, opts.variants);
    if (instances.empty()) {
        std::cerr << "[ERROR] No instances found matching the specified filters.\n";
        return 1;
    }

    // --quick: keep only the first N instances per variant
    if (opts.quick > 0) {
        std::map<std::string, int> variant_count;
        std::vector<InstanceSpec> trimmed;
        for (const auto& spec : instances) {
            if (variant_count[spec.variant] < opts.quick) {
                trimmed.push_back(spec);
                ++variant_count[spec.variant];
            }
        }
        instances = std::move(trimmed);
        std::cout << "Quick    : " << opts.quick << " instance(s) per variant\n";
    }

    std::cout << "Instances: " << instances.size() << '\n';

    // --- Prepare output -----------------------------------------------------
    if (!fs::exists(opts.output)) {
        try { fs::create_directories(opts.output); }
        catch (const std::exception& e) {
            std::cerr << "[ERROR] Cannot create output folder: " << e.what() << '\n';
            return 1;
        }
    }
    std::string csv_path = make_csv_name(opts.output, opts.instance);

    // Load already-completed runs (empty set if overwrite or file absent)
    std::set<DoneKey> done_keys;
    bool csv_exists = fs::exists(csv_path);
    if (!opts.overwrite && csv_exists) {
        done_keys = load_done_keys(csv_path);
        std::cout << "Resume   : " << csv_path
                  << " (" << done_keys.size() << " runs already done)\n";
    } else if (opts.overwrite && csv_exists) {
        std::cout << "Overwrite: " << csv_path << '\n';
    }

    // Open CSV: append when resuming, truncate when starting fresh / overwriting
    bool write_fresh = opts.overwrite || !csv_exists;
    auto open_mode   = write_fresh
                       ? (std::ios::out | std::ios::trunc)
                       : (std::ios::out | std::ios::app);
    std::ofstream csv(csv_path, open_mode);
    if (!csv.is_open()) {
        std::cerr << "[ERROR] Cannot open output file: " << csv_path << '\n';
        return 1;
    }
    if (write_fresh) write_header(csv);
    std::cout << "Output   : " << csv_path << "\n\n";

    // --- Benchmark loop -----------------------------------------------------
    int ok = 0, err = 0;
    int total = static_cast<int>(instances.size())
                * static_cast<int>(opts.solvers.size())
                * opts.runs;
    int done = 0;

    for (const auto& spec : instances) {
        // Parse once per instance, reuse across solvers and runs
        auto problem = parse_instance(spec);
        if (!problem) {
            if (!opts.no_progress)
                std::cerr << "[SKIP] Cannot parse: " << spec.filepath << '\n';
            // Emit error rows for all solver/run combos (skip already-done ones)
            for (const auto& solver_name : opts.solvers) {
                for (int run = 1; run <= opts.runs; ++run) {
                    ++done;
                    DoneKey key{solver_name,
                                fs::path(spec.filepath).filename().string(), run};
                    if (!done_keys.empty() && done_keys.count(key)) continue;
                    RunResult r;
                    r.solver        = solver_name;
                    r.instance_name = fs::path(spec.filepath).filename().string();
                    r.variant       = spec.variant;
                    r.run_id        = run;
                    r.status        = "ERROR:parse";
                    write_row(csv, r);
                    csv.flush();
                    ++err;
                }
            }
            continue;
        }

        for (const auto& solver_name : opts.solvers) {
            for (int run = 1; run <= opts.runs; ++run) {
                ++done;

                // Skip if already completed (resume mode)
                DoneKey key{solver_name,
                            fs::path(spec.filepath).filename().string(),
                            run};
                if (!done_keys.empty() && done_keys.count(key)) {
                    if (!opts.no_progress)
                        std::cout << '[' << done << '/' << total << "] "
                                  << solver_name << " / "
                                  << fs::path(spec.filepath).filename().string()
                                  << " (run " << run << ") [SKIP — already done]\n";
                    continue;
                }

                if (!opts.no_progress) {
                    std::cout << '[' << done << '/' << total << "] "
                              << solver_name << " / "
                              << fs::path(spec.filepath).filename().string()
                              << " (run " << run << ") ... " << std::flush;
                }

                auto r = run_once(solver_name, spec, *problem, opts, run);
                write_row(csv, r);
                csv.flush();

                if (!opts.no_progress) {
                    if (r.status == "OK")
                        std::cout << "reward=" << std::fixed << std::setprecision(2)
                                  << r.reward << "  cpu=" << r.cpu_ms << "ms\n";
                    else
                        std::cout << r.status << '\n';
                }

                (r.status == "OK" ? ok : err)++;
            }
        }
    }

    csv.close();

    // --- Summary ------------------------------------------------------------
    std::cout << "\n=== Summary ===\n"
              << "Total runs : " << done  << '\n'
              << "Success    : " << ok    << '\n'
              << "Error      : " << err   << '\n'
              << "CSV        : " << csv_path << '\n';
    return (err > 0) ? 1 : 0;
}
