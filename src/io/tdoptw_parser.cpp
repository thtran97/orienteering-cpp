#include "../../include/io/tdoptw_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace oplib::io {

std::unique_ptr<model::variants::TDOPTWProblem> TDOPTWParser::read(
    const std::string& instance_path,
    const std::string& transition_matrix_path)
{
    std::ifstream infile(instance_path);
    if (!infile.is_open()) return nullptr;

    int nb_customers;
    double tmax;
    infile >> nb_customers >> tmax;

    auto problem = std::make_unique<model::variants::TDOPTWProblem>(instance_path, tmax);

    double shift_time = 0;
    for (int i = 0; i < nb_customers; ++i) {
        model::Node node;
        node.id = i;
        int ignore;
        char sep;
        // id;s;pen;p;o;c
        infile >> ignore >> sep >> node.reward >> sep >> ignore >> sep >> node.service_time >> sep >> node.tw.opening >> sep >> node.tw.closing;
        if (i == 0) shift_time = node.tw.opening;
        node.tw.opening -= shift_time;
        node.tw.closing -= shift_time;
        problem->add_node(node);
    }
    infile.close();

    // Load transition matrix
    std::ifstream trans_file(transition_matrix_path);
    if (trans_file.is_open()) {
        int nb_slots = 56;
        std::vector<std::vector<std::vector<double>>> matrix(nb_customers, std::vector<std::vector<double>>(nb_customers, std::vector<double>(nb_slots)));
        char sep;
        for (int i = 0; i < nb_customers; ++i) {
            for (int j = 0; j < nb_customers; ++j) {
                for (int k = 0; k < nb_slots; ++k) {
                    trans_file >> matrix[i][j][k] >> sep;
                }
            }
        }
        problem->set_transition_matrix(matrix);
        problem->set_slot_duration(900000.0); // 15 mins in ms
        problem->set_start_time(0.0);
    }

    return problem;
}

} // namespace oplib::io
