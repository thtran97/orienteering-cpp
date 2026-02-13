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
    problem->set_scaling(ScalingMode::SCALED_INTEGER, 1.0); // TDOP usually scale 1.0 but scaled_int

    for (int i = 0; i < nb_customers; ++i) {
        model::Node node;
        node.id = i;
        if (!(infile >> node.x >> node.y >> node.reward)) break;
        problem->add_node(node);
    }
    infile.close();

    problem->finalize();

    // 1. Load arc categories (Needed for speed lookup)
    std::ifstream cat_file(arc_cat_path);
    std::vector<std::vector<int>> arc_categories;
    if (cat_file.is_open()) {
        arc_categories.assign(nb_customers, std::vector<int>(nb_customers));
        for (int i = 0; i < nb_customers; ++i) {
            for (int j = 0; j < nb_customers; ++j) {
                cat_file >> arc_categories[i][j];
            }
        }
        problem->set_arc_categories(arc_categories);
    }
    cat_file.close();

    // 2. Load speed matrix
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
        for (int i=1; i<=14; ++i) boundaries.push_back(static_cast<double>(i)); 
        problem->set_time_slots(boundaries);
        
        // 3. Compute coefficients (Legacy logic)
        if (!arc_categories.empty()) {
            int n = nb_customers;
            int nb_slots = 14;
            double time_step = 1.0; 

            std::vector<std::vector<std::vector<double>>> coeff_a(n, std::vector<std::vector<double>>(n, std::vector<double>(nb_slots, 0.0)));
            std::vector<std::vector<std::vector<double>>> coeff_b(n, std::vector<std::vector<double>>(n, std::vector<double>(nb_slots, 0.0)));
            std::vector<std::vector<std::vector<double>>> transition_matrix(n, std::vector<std::vector<double>>(n, std::vector<double>(nb_slots, 0.0)));

            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    int cat = arc_categories[i][j];
                    double dist = problem->get_distance(i, j);
                    for (int k = 0; k < nb_slots; ++k) {
                        transition_matrix[i][j][k] = std::ceil(dist / speed_matrix[cat][k]);
                    }
                    
                    for (int k = 0; k < nb_slots - 1; ++k) {
                        coeff_a[i][j][k] = (transition_matrix[i][j][k+1] - transition_matrix[i][j][k]) / time_step;
                        coeff_b[i][j][k] = transition_matrix[i][j][k] - coeff_a[i][j][k] * k * time_step;
                    }
                    coeff_a[i][j][nb_slots-1] = 0;
                    coeff_b[i][j][nb_slots-1] = transition_matrix[i][j][nb_slots-1];
                }
            }
            problem->set_coefficients(coeff_a, coeff_b);
        }
    }
    speed_file.close();

    return problem;
}

} // namespace oplib::io
