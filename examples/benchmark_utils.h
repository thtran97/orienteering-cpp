#pragma once

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
#include <map>

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

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver;

namespace benchmark_utils {

// Result structure for benchmark runs
struct BenchmarkResult {
    std::string instance_name;
    std::string problem_type;
    std::string solver_name;
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
    std::string solver_params;  // Solver-specific parameters (e.g., "alpha=0.3")
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

// Find all instances in a directory tree (limiting search depth)
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
                std::string filename = entry.path().filename().string();
                std::string lower_filename = filename;
                std::transform(lower_filename.begin(), lower_filename.end(), 
                              lower_filename.begin(), ::tolower);
                
                // Skip documentation and description files
                if (lower_filename.find("format") != std::string::npos ||
                    lower_filename.find("description") != std::string::npos ||
                    lower_filename.find("readme") != std::string::npos) {
                    continue;
                }
                
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

// Filter instances based on mode (quick or full)
// Quick mode: return first N instances per problem type (to quickly test all variants)
// Full mode: return all instances
std::vector<std::string> find_instances_mode(const std::string& dataset_path, 
                                            const std::string& mode, 
                                            int quick_limit = 3) {
    auto all_instances = find_instances(dataset_path);
    
    if (mode == "full" || mode == "FULL") {
        return all_instances;
    }
    
    if (mode != "quick" && mode != "QUICK") {
        std::cerr << "[WARN] Unknown mode '" << mode << "', using 'full'" << std::endl;
        return all_instances;
    }
    
    // Quick mode: group by problem type, take first N from each group
    std::map<std::string, std::vector<std::string>> grouped;
    
    for (const auto& instance : all_instances) {
        std::string problem_type;
        auto problem = parse_instance(instance, problem_type);
        if (problem) {
            grouped[problem_type].push_back(instance);
        }
    }
    
    // Build result: first quick_limit instances from each type
    std::vector<std::string> result;
    for (auto& [type, instances] : grouped) {
        int count = 0;
        for (const auto& instance : instances) {
            if (count >= quick_limit) break;
            result.push_back(instance);
            count++;
        }
    }
    
    std::sort(result.begin(), result.end());
    return result;
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

// Write CSV header (includes solver_params column)
void write_csv_header(std::ofstream& out) {
    out << "Instance,Solver,Type,Nodes,Vehicles,TimeWindows,TimeDependent,"
        << "Budget_or_Tmax,TimeConsumed,Reward,CustomersVisited,CPU_ms,Status,SolverParams\n";
}

// Write a single result to CSV
void write_csv_result(std::ofstream& out, const BenchmarkResult& result) {
    out << result.instance_name << ","
        << result.solver_name << ","
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
        << result.status << ","
        << result.solver_params << "\n";
}

// Extract problem metadata from a parsed problem
void extract_problem_metadata(const Problem* problem, BenchmarkResult& result) {
    result.num_nodes = problem->get_num_nodes();
    result.has_time_windows = problem->has_time_windows();
    result.is_time_dependent = problem->is_time_dependent();
    
    // Try to get num_vehicles
    result.num_vehicles = 1;
    if (problem->is_multi_vehicle()) {
        // Try casting to TOP or TOPTW
        if (auto* top = dynamic_cast<oplib::model::variants::TOPProblem*>(const_cast<Problem*>(problem))) {
            result.num_vehicles = top->get_num_vehicles();
        } else if (auto* toptw = dynamic_cast<oplib::model::variants::TOPTWProblem*>(const_cast<Problem*>(problem))) {
            result.num_vehicles = toptw->get_num_vehicles();
        }
    }
    
    // Get budget/tmax
    if (auto* op = dynamic_cast<oplib::model::variants::OPProblem*>(const_cast<Problem*>(problem))) {
        result.budget_or_tmax = op->get_budget();
    }
}

// Extract solution metrics from a solution
void extract_solution_metrics(const Solution& solution, BenchmarkResult& result) {
    result.total_reward = solution.total_reward;
    result.total_time_consumed = solution.total_travel_time;
    
    // Count visited customers
    result.customers_visited = 0;
    for (const auto& route : solution.get_routes()) {
        // Exclude depots from count
        result.customers_visited += static_cast<int>(route.size()) - 2;
    }
}

}  // namespace benchmark_utils
