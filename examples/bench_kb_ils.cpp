#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "bench_utils.h"
#include "solver/local_search/kb_ls.h"
#include "solver/metaheuristic/ils09.h"
#include "solver/metaheuristic/kb_ils.h"

using namespace oplib::solver;

struct KBVariant {
    std::string id;
    int   learn_iterations;
    int   conflict_max_size;
    float xplain_ms;
    local_search::ScopeHeuristic scope_heuristic;
};

static KBVariant select_variant(const std::string& id)
{
    using SH = local_search::ScopeHeuristic;
    if (id == "A") return {"A", 100, 5,  50.f, SH::NearestTW};
    if (id == "B") return {"B", 300, 5,  50.f, SH::NearestTW};
    if (id == "C") return {"C", 100, 3,  50.f, SH::NearestTW};
    if (id == "D") return {"D", 100, 5, 200.f, SH::NearestTW};
    if (id == "E") return {"E", 100, 5,  50.f, SH::Random};
    std::cerr << "[ERROR] Unknown KB config: '" << id << "' (use A|B|C|D|E or COMPARE)\n";
    std::exit(1);
}

// ---------------------------------------------------------------------------
// Run a single COMPARE sweep: ILS09 vs KB-C back to back on every instance.
// Prints a side-by-side table to stdout and writes a CSV to output_root.
// ---------------------------------------------------------------------------

static void run_compare(const bench::Options& opts)
{
    // KB-C config (the best variant from the ablation study)
    KBVariant kv = select_variant("C");

    metaheuristic::ILS09Solver ils;
    metaheuristic::ILS09SolverConfig ils_cfg;
    ils_cfg.seed              = opts.seed;
    ils_cfg.max_cpu_time      = opts.timeout;
    ils_cfg.max_iterations    = opts.iterations;
    ils_cfg.verbose           = false;
    ils_cfg.alpha             = 3;
    ils_cfg.rcl_size          = 5;
    ils_cfg.restart_threshold = 10;

    metaheuristic::KBILSSolver kbils;
    metaheuristic::KBILSSolverConfig kb_cfg;
    kb_cfg.seed              = opts.seed;
    kb_cfg.max_cpu_time      = opts.timeout;
    kb_cfg.max_iterations    = opts.iterations;
    kb_cfg.verbose           = false;
    kb_cfg.alpha             = 3;
    kb_cfg.rcl_size          = 5;
    kb_cfg.restart_threshold = 10;
    kb_cfg.learn_iterations  = kv.learn_iterations;
    kb_cfg.conflict_max_size = kv.conflict_max_size;
    kb_cfg.xplain_ms         = kv.xplain_ms;
    kb_cfg.scope_heuristic   = kv.scope_heuristic;

    auto instances = bench::discover_instances(opts.instance_path, opts.variants);
    if (instances.empty()) { std::cerr << "No instances found.\n"; return; }
    if (opts.quick > 0) instances = bench::quick_filter(std::move(instances), opts.quick);

    // CSV output
    namespace fs = std::filesystem;
    for (const auto& variant : opts.variants) {
        fs::path dir = fs::path(opts.output_root) / variant;
        fs::create_directories(dir);
        fs::path csv_path = dir / "benchmark_compare_ils09_vs_kbils.csv";
        std::ofstream csv(csv_path.string(), std::ios::out | std::ios::trunc);
        std::cout << "  " << csv_path.string() << '\n';
        csv << "Instance,Nodes,ILS09_Reward,ILS09_Iters,KBILS_Reward,KBILS_Iters,"
               "KB_Size,Delta_Reward,Delta_Iters_Pct\n";

        // Header for stdout table
        std::cout << '\n'
                  << std::left  << std::setw(14) << "Instance"
                  << std::right << std::setw(7)  << "Nodes"
                  << std::setw(10) << "ILS9-R"
                  << std::setw(10) << "ILS9-I"
                  << std::setw(10) << "KBC-R"
                  << std::setw(10) << "KBC-I"
                  << std::setw(10) << "KB-Size"
                  << std::setw(8)  << "ΔR"
                  << std::setw(10) << "ΔIter%"
                  << '\n'
                  << std::string(89, '-') << '\n';

        int ils_wins = 0, kb_wins = 0, ties = 0;
        long long total_ils_iters = 0, total_kb_iters = 0;

        for (const auto& spec : instances) {
            if (spec.variant != variant) continue;
            auto problem = bench::parse_instance(spec);
            if (!problem) continue;
            bench::apply_overrides(opts, *problem);

            ils_cfg.iterations_out = 0;
            kb_cfg.iterations_out  = 0;
            kb_cfg.kb_size_out     = 0;

            auto ils_r = bench::run_and_record(ils,   ils_cfg, *problem, spec, "ils09",  1);
            auto kb_r  = bench::run_and_record(kbils, kb_cfg,  *problem, spec, "kbils_C", 1);

            int    nodes     = problem->get_num_nodes();
            double ils_rew   = ils_r.reward;
            int    ils_iters = ils_cfg.iterations_out;
            double kb_rew    = kb_r.reward;
            int    kb_iters  = kb_cfg.iterations_out;
            int    kb_size   = kb_cfg.kb_size_out;
            double delta_r   = kb_rew - ils_rew;
            double delta_pct = (ils_iters > 0)
                ? 100.0 * (kb_iters - ils_iters) / static_cast<double>(ils_iters)
                : 0.0;

            if (delta_r > 0) ++kb_wins;
            else if (delta_r < 0) ++ils_wins;
            else ++ties;
            total_ils_iters += ils_iters;
            total_kb_iters  += kb_iters;

            // Stdout row
            std::string inst = fs::path(spec.filepath).filename().string();
            std::cout << std::left  << std::setw(14) << inst
                      << std::right << std::setw(7)  << nodes
                      << std::setw(10) << std::fixed << std::setprecision(0) << ils_rew
                      << std::setw(10) << ils_iters
                      << std::setw(10) << kb_rew
                      << std::setw(10) << kb_iters
                      << std::setw(10) << kb_size
                      << std::setw(8)  << std::showpos << delta_r << std::noshowpos
                      << std::setw(9)  << std::fixed << std::setprecision(1)
                      << std::showpos  << delta_pct << "%" << std::noshowpos
                      << '\n';

            // CSV row
            csv << inst << ',' << nodes << ','
                << std::fixed << std::setprecision(0)
                << ils_rew << ',' << ils_iters << ','
                << kb_rew  << ',' << kb_iters  << ','
                << kb_size << ','
                << delta_r << ','
                << std::setprecision(1) << delta_pct << '\n';
        }

        // Summary
        double overall_iter_pct = (total_ils_iters > 0)
            ? 100.0 * (total_kb_iters - total_ils_iters) / static_cast<double>(total_ils_iters)
            : 0.0;
        std::cout << std::string(89, '-') << '\n'
                  << "KB wins=" << kb_wins << "  ILS09 wins=" << ils_wins
                  << "  ties=" << ties << '\n'
                  << "Total iterations: ILS09=" << total_ils_iters
                  << "  KB-C=" << total_kb_iters
                  << "  (" << std::fixed << std::setprecision(1)
                  << std::showpos << overall_iter_pct << "%)\n" << std::noshowpos;
    }
}

// ---------------------------------------------------------------------------
// Standard single-solver benchmark mode
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    // Pre-scan for --kb-config X before delegating to bench::parse_cli
    std::string kb_config_id = "C";
    bool compare_mode = false;
    std::vector<char*> fwd_argv;
    fwd_argv.push_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--kb-config" && i + 1 < argc) {
            kb_config_id = argv[++i];
            if (kb_config_id == "COMPARE") compare_mode = true;
        } else {
            fwd_argv.push_back(argv[i]);
        }
    }
    int fwd_argc = static_cast<int>(fwd_argv.size());

    std::string solver_label = compare_mode ? "compare" : ("kb_ils_" + kb_config_id);
    auto opts = bench::parse_cli(fwd_argc, fwd_argv.data(), solver_label);

    if (compare_mode) {
        run_compare(opts);
        return 0;
    }

    // ---- Standard single-solver mode ----
    KBVariant kv = select_variant(kb_config_id);

    metaheuristic::KBILSSolver solver;
    metaheuristic::KBILSSolverConfig cfg;
    cfg.seed              = opts.seed;
    cfg.max_cpu_time      = opts.timeout;
    cfg.max_iterations    = opts.iterations;
    cfg.verbose           = opts.verbose;
    cfg.alpha             = 3;
    cfg.rcl_size          = 5;
    cfg.restart_threshold = 10;
    cfg.learn_iterations  = kv.learn_iterations;
    cfg.conflict_max_size = kv.conflict_max_size;
    cfg.xplain_ms         = kv.xplain_ms;
    cfg.scope_heuristic   = kv.scope_heuristic;

    auto instances = bench::discover_instances(opts.instance_path, opts.variants);
    if (instances.empty()) { std::cerr << "No instances found.\n"; return 1; }
    if (opts.quick > 0) instances = bench::quick_filter(std::move(instances), opts.quick);
    std::cout << "Benchmarking " << instances.size() << " instance(s)"
              << "  [KB config " << kb_config_id
              << ": learn=" << kv.learn_iterations
              << " max_scope=" << kv.conflict_max_size
              << " xplain_ms=" << kv.xplain_ms << "]\n\n";

    auto csv_map = bench::open_csv_files(
        opts.output_root, solver_label, opts.variants, opts.overwrite);

    int ok = 0, err = 0;
    for (const auto& spec : instances) {
        auto problem = bench::parse_instance(spec);
        if (!problem) { ++err; continue; }
        bench::apply_overrides(opts, *problem);
        for (int run = 1; run <= opts.runs; ++run) {
            cfg.kb_size_out     = 0;
            cfg.iterations_out  = 0;
            auto r = bench::run_and_record(solver, cfg, *problem, spec, solver_label, run);
            std::cout << "  [" << r.instance_name << "] reward=" << r.reward
                      << " iters=" << cfg.iterations_out
                      << " kb_size=" << cfg.kb_size_out
                      << " cpu_ms=" << r.cpu_ms << '\n';
            bench::write_row(csv_map.at(spec.variant), r);
            (r.status == "OK" ? ok : err)++;
        }
    }
    bench::close_csv_files(csv_map);
    std::cout << "\nOK: " << ok << "  Errors: " << err << "\n";
    return err > 0 ? 1 : 0;
}
