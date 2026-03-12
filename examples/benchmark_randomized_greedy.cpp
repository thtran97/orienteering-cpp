#include "benchmark_utils.h"
#include "solver/constructive/randomized_greedy.h"

using namespace benchmark_utils;

BenchmarkResult benchmark_instance(const std::string& filepath, 
                                    const std::string& solver_name,
                                    double time_limit,
                                    double alpha,
                                    unsigned int seed) {
    BenchmarkResult result;
    result.instance_name = fs::path(filepath).filename().string();
    result.solver_name = solver_name;
    result.status = "OK";
    
    // Format solver parameters
    std::ostringstream params_stream;
    params_stream << std::fixed << std::setprecision(2) << "alpha=" << alpha;
    result.solver_params = params_stream.str();
    
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
    config.max_cpu_time = time_limit;
    config.verbose = false;
    
    // Create RandomizedGreedy config
    oplib::solver::constructive::RandomizedGreedyConfig rg_config;
    rg_config.seed = seed;
    rg_config.max_cpu_time = time_limit;
    rg_config.max_iterations = 1;  // Single construction pass for benchmarking (alpha controls randomness)
    rg_config.verbose = false;
    rg_config.alpha = alpha;
    
    // Solve with randomized greedy solver
    oplib::solver::constructive::RandomizedGreedySolver rg_solver;
    
    auto start = std::chrono::high_resolution_clock::now();
    Solution solution = rg_solver.solve(*problem, rg_config);
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
    double alpha = 0.3;  // Default RCL parameter
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
        } else if (arg == "-alpha" && i + 1 < argc) {
            alpha = std::stod(argv[++i]);
        } else if (arg == "-seed" && i + 1 < argc) {
            seed = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: ./benchmark_randomized_greedy -dataset <path> [options]\n"
                      << "  -dataset      Path to dataset folder containing instances (required)\n"
                      << "  -mode         Execution mode: quick (1-3 instances per type) or full (all) (default: full)\n"
                      << "  -alpha        RCL parameter [0.0-1.0] (default: 0.3)\n"
                      << "  -timelimit    Time limit per instance in seconds (default: 60.0)\n"
                      << "  -seed         Random seed (default: 42)\n"
                      << "  -output       Output folder for CSV file (default: current directory)\n"
                      << "\nOutput filename format: randomized_greedy_<dataset>_<yymmdd>.csv\n";
            return 0;
        }
    }
    
    if (dataset_path.empty()) {
        std::cerr << "Error: -dataset argument is required\n";
        std::cerr << "Usage: ./benchmark_randomized_greedy -dataset <path> [options]\n";
        return 1;
    }
    
    // Validate alpha
    if (alpha < 0.0 || alpha > 1.0) {
        std::cerr << "Error: alpha must be in [0.0, 1.0]\n";
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
    std::string output_csv = (fs::path(output_folder) / generate_csv_filename("randomized_greedy", dataset_path)).string();
    
    // Find instances based on mode
    std::cout << "[INFO] Mode: " << mode << ", Alpha: " << std::fixed << std::setprecision(2) << alpha << std::endl;
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
        
        auto result = benchmark_instance(instance_path, "RandomizedGreedySolver", time_limit, alpha, seed);
        
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
