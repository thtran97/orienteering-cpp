#include "benchmark_utils.h"
#include "solver/constructive/greedy.h"
#include "solver/local_search/LNS.h"

using namespace benchmark_utils;

BenchmarkResult benchmark_instance(const std::string& filepath, 
                                    const std::string& solver_name,
                                    double time_limit,
                                    unsigned int seed) {
    BenchmarkResult result;
    result.instance_name = fs::path(filepath).filename().string();
    result.solver_name = solver_name;
    result.status = "OK";
    result.solver_params = "";
    
    // Parse instance
    std::string problem_type;
    auto problem = parse_instance(filepath, problem_type);
    
    if (!problem) {
        result.status = "ERROR";
        result.problem_type = "UNKNOWN";
        return result;
    }
    
    result.problem_type = problem_type;
    extract_problem_metadata(problem.get(), result);
    
    // Configure solver
    SolverConfig config;
    config.seed = seed;
    config.max_iterations = 5;  // Limited LNS iterations for benchmarking
    config.max_cpu_time = time_limit;
    config.verbose = false;
    
    // Step 1: Generate initial solution with greedy solver
    oplib::solver::constructive::GreedySolver greedy_solver;
    Solution initial_solution = greedy_solver.solve(*problem, config);
    
    // Step 2: Improve with LNS
    oplib::solver::local_search::LNSSolver lns_solver;
    
    auto start = std::chrono::high_resolution_clock::now();
    Solution solution = lns_solver.solve_from(*problem, initial_solution, config);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    result.cpu_time_ms = duration.count();
    
    extract_solution_metrics(solution, result);
    
    return result;
}

int main(int argc, char** argv) {
    std::string dataset_path;
    std::string mode = "full";
    double time_limit = 60.0;
    std::string output_folder = ".";
    unsigned int seed = 42;
    
    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-dataset" && i + 1 < argc) {
            dataset_path = argv[++i];
        } else if (arg == "-mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "-timelimit" && i + 1 < argc) {
            time_limit = std::stod(argv[++i]);
        } else if (arg == "-output" && i + 1 < argc) {
            output_folder = argv[++i];
        } else if (arg == "-seed" && i + 1 < argc) {
            seed = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: ./benchmark_lns -dataset <path> [options]\n"
                      << "  -dataset      Path to dataset folder containing instances (required)\n"
                      << "  -mode         Execution mode: quick (1-3 instances per type) or full (all) (default: full)\n"
                      << "  -timelimit    Time limit per instance in seconds (default: 60.0)\n"
                      << "  -seed         Random seed (default: 42)\n"
                      << "  -output       Output folder for CSV file (default: current directory)\n"
                      << "\nNote: Initial solution generated with GreedySolver, then improved with LNS.\n"
                      << "Output filename format: lns_<dataset>_<yymmdd>.csv\n";
            return 0;
        }
    }
    
    if (dataset_path.empty()) {
        std::cerr << "Error: -dataset argument is required\n";
        std::cerr << "Usage: ./benchmark_lns -dataset <path> [options]\n";
        return 1;
    }
    
    // Create output folder if it doesn't exist
    if (!fs::exists(output_folder)) {
        try {
            fs::create_directories(output_folder);
        } catch (const std::exception& e) {
            std::cerr << "Error: Cannot create output folder: " << e.what() << std::endl;
            return 1;
        }
    }
    
    // Generate output filename
    std::string output_csv = (fs::path(output_folder) / generate_csv_filename("lns", dataset_path)).string();
    
    // Find instances based on mode
    std::cout << "[INFO] Mode: " << mode << std::endl;
    std::cout << "[INFO] Searching for instances in: " << dataset_path << std::endl;
    auto instances = find_instances_mode(dataset_path, mode);
    
    if (instances.empty()) {
        std::cerr << "Error: No instances found in dataset path\n";
        return 1;
    }
    
    std::cout << "[INFO] Found " << instances.size() << " instances\n";
    
    // Open output CSV
    std::ofstream outfile(output_csv);
    if (!outfile.is_open()) {
        std::cerr << "Error: Cannot open output file: " << output_csv << std::endl;
        return 1;
    }
    
    write_csv_header(outfile);
    
    // Benchmark each instance
    int success_count = 0;
    int error_count = 0;
    
    for (size_t i = 0; i < instances.size(); ++i) {
        const auto& instance_path = instances[i];
        
        std::cout << "[" << (i + 1) << "/" << instances.size() << "] Benchmarking: " 
                  << fs::path(instance_path).filename().string() << " ... " << std::flush;
        
        auto result = benchmark_instance(instance_path, "LNSSolver", time_limit, seed);
        
        write_csv_result(outfile, result);
        
        if (result.status == "OK") {
            std::cout << "OK (reward=" << result.total_reward 
                      << ", cpu=" << result.cpu_time_ms << "ms)\n";
            success_count++;
        } else {
            std::cout << "ERROR\n";
            error_count++;
        }
    }
    
    outfile.close();
    
    // Print summary
    std::cout << "\n[INFO] Benchmark complete!\n";
    std::cout << "[INFO] Results written to: " << output_csv << "\n";
    std::cout << "[INFO] Successful: " << success_count << ", Errors: " << error_count << "\n";
    
    return 0;
}
