#include "../../include/io/tdop_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace oplib::io {

std::unique_ptr<model::variants::TDOPProblem> TDOPParser::read(
    const std::string& instance_path,
    const std::string& speed_matrix_path,
    const std::string& arc_cat_path)
{
    std::ifstream infile(instance_path);
    if (!infile.is_open()) return nullptr;

    std::string tmp;
    int nb_customers, nb_paths;
    double tmax;
    
    // Read header: n N \n m P \n tmax Tmax
    infile >> tmp >> nb_customers;
    infile >> tmp >> nb_paths;
    infile >> tmp >> tmax;

    auto problem = std::make_unique<model::variants::TDOPProblem>(instance_path, tmax);

    for (int i = 0; i < nb_customers; ++i) {
        model::Node node;
        node.id = i;
        if (!(infile >> node.x >> node.y >> node.reward)) break;
        problem->add_node(node);
    }
    infile.close();

    problem->finalize();

    // Load speed matrix
    std::ifstream speed_file(speed_matrix_path);
    if (speed_file.is_open()) {
        std::vector<std::vector<double>> speed_matrix(5, std::vector<double>());
        for (int ic = 0; ic < 5; ++ic) {
            double v1, v2, v3, v4;
            speed_file >> v1 >> v2 >> v3 >> v4;
            // Legacy expands 4 periods into 14 slots
            for (int p=0; p<2; p++) speed_matrix[ic].push_back(v1); // 7-9
            for (int p=0; p<8; p++) speed_matrix[ic].push_back(v2); // 9-17
            for (int p=0; p<2; p++) speed_matrix[ic].push_back(v3); // 17-19
            for (int p=0; p<2; p++) speed_matrix[ic].push_back(v4); // 19-21
        }
        problem->set_speed_matrix(speed_matrix);
        std::vector<double> boundaries;
        for (int i=1; i<=14; ++i) boundaries.push_back(i); 
        problem->set_time_slots(boundaries);
    }

    // Load arc categories
    std::ifstream cat_file(arc_cat_path);
    if (cat_file.is_open()) {
        std::vector<std::vector<int>> arc_categories(nb_customers, std::vector<int>(nb_customers));
        for (int i = 0; i < nb_customers; ++i) {
            for (int j = 0; j < nb_customers; ++j) {
                cat_file >> arc_categories[i][j];
            }
        }
        problem->set_arc_categories(arc_categories);
    }

    return problem;
}

} // namespace oplib::io
