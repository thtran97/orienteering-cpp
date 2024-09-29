#include <iostream>
#include <fstream>
#include "model/orienteering_model.hpp"
#include "solver/iterated_local_search.hpp"
#include "utils/argument_parser.hpp"

#define DEFAULT_TIME_FACTOR 100

using namespace oplib;


// Parse the instance file: instance file format : data/op/OP_format.txt
// Return the number of paths used in the instance

int parse_instance(model::OrienteeringModel & op_model, std::string file, int time_factor=DEFAULT_TIME_FACTOR){

    // parse your instance here
    std::cout << "[INFO::main] Parsing instance : " << file << "\n"; 
    std::ifstream infile(file);
    if (!infile){
        std::cerr << "Error: unable to open file " << file << std::endl;
        exit(1);
    }

    // [optional] set instance_name for summary
    op_model.instance_name = file.substr(file.find_last_of("/\\") + 1);
    
    // See description in data/op/OP_format.txt 
    // read first line
    double original_tmax;
    int nb_paths;
    infile >> original_tmax >> nb_paths;
    if  (nb_paths < 1){
        std::cerr << "Error: number of paths used must be positive" << std::endl;
        exit(1);
    }

    op_model.set_time_budget(original_tmax * time_factor);
    // read point data
    while (!infile.eof()){
        double x, y;
        int score;
        infile >> x >> y >> score;
        op_model.add_node(x, y, score);
    }
    op_model.set_starting_point(0);
    op_model.set_ending_point(1);

    // [IMPORTANT] Compute the travel matrix
    op_model.update_travel_duration_matrix(time_factor);
    infile.close();
    return nb_paths;
}


// Main function
// Usage: ./solve_op -file data/op/OP_format.txt -seed 0 -timeout 0.0

int main(int argc, char** argv){

    // Parse arguments
    utils::ArgParser arg_parser(argv, argv + argc);
    
    // Parse instance from file
    model::OrienteeringModel op_model; 
    char * filename = arg_parser.getCmdOption("-file"); 
    if (!filename) std::cout << "Require instance file !\n";
    int nb_paths = parse_instance(op_model, filename);
    op_model.print_summary();

    // Create solver and set solving parameters
    solver::IteratedLocalSearch ils_solver(op_model);
    int seed = arg_parser.getCmdInt("-seed", 0);
    ils_solver.set_seed(seed);
    double timeout = arg_parser.getCmdDouble("-timeout", 0.0);
    ils_solver.set_timeout(timeout);
    ils_solver.set_nb_paths(nb_paths);
    
    // Solve the problem
    ils_solver.solve();
    // ils_solver._test_construct();
    // ils_solver._test_remove_subseq();
    ils_solver.print_solutions();
    // std::cout << "[INFO:main] Done\n";
    return 0;
}
