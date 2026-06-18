#include "bench_utils.h"
#include "solver/pulse/pulse_solver.h"

int main(int argc, char** argv) {
    auto opts = bench::parse_cli(argc, argv, "pulse");

    // --- Pulse config (adjustable) ---
    pulse::PulseSolver solver;
    pulse::PulseSolverConfig cfg;
    cfg.seed             = opts.seed;
    cfg.max_cpu_time     = opts.timeout;
    cfg.verbose          = opts.verbose;
    cfg.max_labels       = opts.pulse_labels; // 0 = unlimited
    cfg.bound_step       = 1000;  // matches test_pulse_perf_cordeau; fine-grained steps
                                  // (e.g. 10) explode the oracle table on instances with
                                  // large time horizons (Cordeau: tmax ≈ 100 000 units)
                                  // and degrade rather than improve search performance
    cfg.threshold_rate   = 0.2;
    cfg.second_bound_step = 0;

    auto instances = bench::discover_instances(opts.instance_path, opts.variants);
    if (instances.empty()) { std::cerr << "No instances found.\n"; return 1; }
    if (opts.quick > 0) instances = bench::quick_filter(std::move(instances), opts.quick);
    std::cout << "Benchmarking " << instances.size() << " instance(s)...\n\n";

    auto csv_map = bench::open_csv_files(opts.output_root, "pulse", opts.variants, opts.overwrite);
    int ok = 0, err = 0;
    for (const auto& spec : instances) {
        auto problem = bench::parse_instance(spec);
        if (!problem) { err++; continue; }
        bench::apply_overrides(opts, *problem);
        for (int run = 1; run <= opts.runs; ++run) {
            auto r = bench::run_and_record(solver, cfg, *problem, spec, "pulse", run);
            bench::write_row(csv_map.at(spec.variant), r);
            (r.status == "OK" ? ok : err)++;
        }
    }
    bench::close_csv_files(csv_map);
    std::cout << "\nOK: " << ok << "  Errors: " << err << "\n";
    return err > 0 ? 1 : 0;
}
