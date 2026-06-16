#include "bench_utils.h"
#include "solver/constructive/randomized_greedy.h"

int main(int argc, char** argv) {
    auto opts = bench::parse_cli(argc, argv, "rand_greedy");

    // --- Randomized Greedy config (adjustable) ---
    constructive::RandomizedGreedySolver solver;
    constructive::RandomizedGreedySolverConfig cfg;
    cfg.seed           = opts.seed;
    cfg.max_cpu_time   = opts.timeout;
    cfg.max_iterations = opts.iterations;
    cfg.verbose        = opts.verbose;
    cfg.alpha          = 3;
    cfg.rcl_size       = 5;

    auto instances = bench::discover_instances(opts.instance_path, opts.variants);
    if (instances.empty()) { std::cerr << "No instances found.\n"; return 1; }
    if (opts.quick > 0) instances = bench::quick_filter(std::move(instances), opts.quick);
    std::cout << "Benchmarking " << instances.size() << " instance(s)...\n\n";

    auto csv_map = bench::open_csv_files(opts.output_root, "rand_greedy", opts.variants, opts.overwrite);
    int ok = 0, err = 0;
    for (const auto& spec : instances) {
        auto problem = bench::parse_instance(spec);
        if (!problem) { err++; continue; }
        for (int run = 1; run <= opts.runs; ++run) {
            auto r = bench::run_and_record(solver, cfg, *problem, spec, "rand_greedy", run);
            bench::write_row(csv_map.at(spec.variant), r);
            (r.status == "OK" ? ok : err)++;
        }
    }
    bench::close_csv_files(csv_map);
    std::cout << "\nOK: " << ok << "  Errors: " << err << "\n";
    return err > 0 ? 1 : 0;
}
