#include <iostream>
#include <string>
#include <filesystem>
#include <vector>
#include <fstream>
#include <memory>
#include <chrono>
#include <iomanip>
#include <functional>
#include <ctime>
#include <sstream>
#include <algorithm>

#include "io/op_parser.h"
#include "io/top_parser.h"
#include "io/toptw_parser.h"
#include "io/tdop_parser.h"
#include "io/tdoptw_parser.h"
#include "io/ttdp_parser.h"
#include "io/mctopmtw_parser.h"
#include "io/singlesat_parser.h"
#include "model/problem.h"
#include "model/solution.h"
#include "solver/solver.h"
#include "solver/constructive/greedy.h"

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver;
using namespace oplib::solver::constructive;

struct BenchmarkResult {
    std::string instance_name;
    std::string problem_type;
    int num_nodes;
    int num_vehicles;
    bool has_time_windows;
    bool is_time_dependent;
    double budget_or_tmax;
    double total_time_consumed;
    double total_reward;
    int customers_visited;
    double cpu_time_ms;
    std::string status;  // "OK", "TIMEOUT", "ERROR"
};

// Detect problem type and parse accordingly
std::unique_ptr<Problem> parse_instance(const std::string& filepath, std::string& problem_type) {
    std::string filename = fs::path(filepath).filename().string();
    std::string parent_dir = fs::path(filepath).parent_path().filename().string();
    
    try {
        // Try to determine type by filename/directory patterns
        if (parent_dir.find("op") != std::string::npos || parent_dir.find("Chao") != std::string::npos || 
            parent_dir.find("Tsiligi") != std::string::npos) {
            OPParser parser;
            auto problem = parser.read(filepath);
            if (problem) {
                problem_type = "OP";
                return problem;
            }
        }
        
        if (parent_dir.find("top") != std::string::npos) {
            TOPParser parser;
            auto problem = parser.read(filepath);
            if (problem) {
                problem_type = "TOP";
                return problem;
            }
        }
        
        if (parent_dir.find("toptw") != std::string::npos) {
            TOPTWParser parser;
            auto problem = parser.read(filepath);
            if (problem) {
                problem_type = "TOPTW";
                return problem;
            }
        }
        
        // Try each parser in order
        OPParser op_parser;
        auto problem = op_parser.read(filepath);
        if (problem) {
            problem_type = "OP";
            return problem;
        }
        
        TOPParser top_parser;
        problem = top_parser.read(filepath);
        if (problem) {
            problem_type = "TOP";
            return problem;
        }
        
        TOPTWParser toptw_parser;
        problem = toptw_parser.read(filepath);
        if (problem) {
            problem_type = "TOPTW";
            return problem;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[WARN] Exception parsing " << filepath << ": " << e.what() << std::endl;
    }
    
    return nullptr;
}

BenchmarkResult benchmark_instance(const std::string& filepath, double time_limit) {
    BenchmarkResult result;
    result.instance_name = fs::path(filepath).filename().string();
    result.status = "OK";
    
    // Parse instance
    std::string problem_type;
    auto problem = parse_instance(filepath, problem_type);
    
    if (!problem) {
        result.status = "ERROR";
        result.problem_type = "UNKNOWN";
        return result;
    }
    
    result.problem_type = problem_type;
    result.num_nodes = problem->get_num_nodes();
    result.has_time_windows = problem->has_time_windows();
    result.is_time_dependent = problem->is_time_dependent();
    
    // Try to get num_vehicles
    result.num_vehicles = 1;
    if (problem->is_multi_vehicle()) {
        // Try casting to TOP or TOPTW
        if (auto* top = dynamic_cast<oplib::model::variants::TOPProblem*>(problem.get())) {
            result.num_vehicles = top->get_num_vehicles();
        } else if (auto* toptw = dynamic_cast<oplib::model::variants::TOPTWProblem*>(problem.get())) {
            result.num_vehicles = toptw->get_num_vehicles();
        }
    }
    
    // Get budget/tmax
    if (auto* op = dynamic_cast<oplib::model::variants::OPProblem*>(problem.get())) {
        result.budget_or_tmax = op->get_budget();
    }
    
    // Configure solver
    SolverConfig config;
    config.seed = 42;
    config.max_cpu_time = time_limit;
    config.verbose = false;
    
    // Solve with greedy solver
    GreedySolver greedy_solver;
    
    auto start = std::chrono::high_resolution_clock::now();
    Solution solution = greedy_solver.solve(*problem, config);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    result.cpu_time_ms = duration.count();
    
    result.total_reward = solution.total_reward;
    result.total_time_consumed = solution.total_travel_time;
    
    // Count visited customers
    result.customers_visited = 0;
    for (const auto& route : solution.get_routes()) {
        // Exclude depots from count
        result.customers_visited += static_cast<int>(route.size()) - 2;
    }
    
    return result;
}

void write_csv_header(std::ofstream& out) {
    out << "Instance,Type,Nodes,Vehicles,TimeWindows,TimeDependent,"
        << "Budget_or_Tmax,TimeConsumed,Reward,CustomersVisited,CPU_ms,Status\n";
}

void write_csv_result(std::ofstream& out, const BenchmarkResult& result) {
    out << result.instance_name << ","
        << result.problem_type << ","
        << result.num_nodes << ","
        << result.num_vehicles << ","
        << (result.has_time_windows ? "yes" : "no") << ","
        << (result.is_time_dependent ? "yes" : "no") << ","
        << std::fixed << std::setprecision(2) << result.budget_or_tmax << ","
        << result.total_time_consumed << ","
        << result.total_reward << ","
        << result.customers_visited << ","
        << result.cpu_time_ms << ","
        << result.status << "\n";
}

std::vector<std::string> find_instances(const std::string& dataset_path) {
    std::vector<std::string> instances;
    
    if (!fs::exists(dataset_path)) {
        std::cerr << "Error: Dataset path does not exist: " << dataset_path << std::endl;
        return instances;
    }
    
    // Recursively search for files (limit depth to avoid too many files)
    std::function<void(const std::string&, int)> search_dir = 
        [&](const std::string& path, int current_depth) {
        if (current_depth > 3) return;  // Limit search depth
        
        for (const auto& entry : fs::directory_iterator(path)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                if (ext == ".txt" || ext == ".TXT") {
                    instances.push_back(entry.path().string());
                }
            } else if (entry.is_directory()) {
                // Recursive call
                search_dir(entry.path().string(), current_depth + 1);
            }
        }
    };
    
    search_dir(dataset_path, 0);
    std::sort(instances.begin(), instances.end());
    
    return instances;
}

// Generate CSV filename from solver name, dataset path, and current date
// Format: solvername_datasetname_yymmdd.csv
std::string generate_csv_filename(const std::string& solver_name, const std::string& dataset_path) {
    // Extract dataset name from path (last directory component)
    std::string dataset_name = fs::path(dataset_path).filename().string();
    
    // Convert to lowercase for consistency
    std::transform(dataset_name.begin(), dataset_name.end(), dataset_name.begin(), ::tolower);
    
    // Remove special characters and replace with underscores
    for (char& c : dataset_name) {
        if (!std::isalnum(c)) {
            c = '_';
        }
    }
    
    // Get current date in yymmdd format
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream date_stream;
    date_stream << std::put_time(&tm, "%y%m%d");
    std::string date_str = date_stream.str();
    
    // Construct filename
    std::ostringstream filename_stream;
    filename_stream << solver_name << "_" << dataset_name << "_" << date_str << ".csv";
    
    return filename_stream.str();
}

int main(int argc, char** argv) {
    std::string dataset_path;
    double time_limit = 60.0;
    std::string output_folder = ".";  // Default to current directory
    
    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-dataset" && i + 1 < argc) {
            dataset_path = argv[++i];
        } else if (arg == "-timelimit" && i + 1 < argc) {
            time_limit = std::stod(argv[++i]);
        } else if (arg == "-output" && i + 1 < argc) {
            output_folder = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: ./benchmark_greedy -dataset <path> [-timelimit <seconds>] [-output <folder>]\n"
                      << "  -dataset      Path to dataset folder containing instances (required)\n"
                      << "  -timelimit    Time limit per instance in seconds (default: 60.0)\n"
                      << "  -output       Output folder for CSV file (default: current directory)\n"
                      << "\nOutput filename format: greedy_<dataset>_<yymmdd>.csv\n";
            return 0;
        }
    }
    
    if (dataset_path.empty()) {
        std::cerr << "Error: -dataset argument is required\n";
        std::cerr << "Usage: ./benchmark_greedy -dataset <path> [-timelimit <seconds>] [-output <folder>]\n";
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
    std::string output_csv = fs::path(output_folder) / generate_csv_filename("greedy", dataset_path);
    
    // Find all instances
    std::cout << "[INFO] Searching for instances in: " << dataset_path << std::endl;
    auto instances = find_instances(dataset_path);
    
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
        
        auto result = benchmark_instance(instance_path, time_limit);
        
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
