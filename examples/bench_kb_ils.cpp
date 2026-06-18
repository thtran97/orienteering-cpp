#include <iostream>
#include <string>
#include <vector>

#include "bench_utils.h"
#include "solver/local_search/kb_ls.h"
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
    if (id == "F") return {"F", 100, 3,  50.f, SH::NearestTW}; // adaptive_learning=true set below
    std::cerr << "[ERROR] Unknown KB config: '" << id << "' (use A|B|C|D|E)\n";
    std::exit(1);
}

int main(int argc, char** argv)
{
    // Pre-scan for --kb-config X before delegating to bench::parse_cli
    std::string kb_config_id = "A";
    std::vector<char*> fwd_argv;
    fwd_argv.push_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--kb-config" && i + 1 < argc) {
            kb_config_id = argv[++i];
        } else {
            fwd_argv.push_back(argv[i]);
        }
    }
    int fwd_argc = static_cast<int>(fwd_argv.size());

    KBVariant kv = select_variant(kb_config_id);
    std::string solver_label = "kb_ils_" + kb_config_id;

    auto opts = bench::parse_cli(fwd_argc, fwd_argv.data(), solver_label);

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
    cfg.adaptive_learning = (kb_config_id == "F");

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
            cfg.kb_size_out = 0;
            auto r = bench::run_and_record(solver, cfg, *problem, spec, solver_label, run);
            if (opts.verbose || cfg.kb_size_out > 0)
                std::cout << "  [" << r.instance_name << "] reward=" << r.reward
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
