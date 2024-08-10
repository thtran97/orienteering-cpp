#include <iostream>
#include <fstream>
#include <cmath>

#include "model/orienteering_model.hpp"

using namespace std;
using namespace oplib::model;

OrienteeringModel::OrienteeringModel()
{
    instance_name = "default";
}

void OrienteeringModel::update_travel_duration_matrix(int time_factor)
{
    int n = nodes.size();
    t_travel.resize(n, std::vector<int>(n, 0));
    for (int i = 0; i < n; ++i){
        for (int j = 0; j < n; ++j){
            if (i == j)
                t_travel[i][j] = 0;
            else
                t_travel[i][j] = compute_travel_duration(i, j, time_factor);
        }
    }
}

int OrienteeringModel::compute_travel_duration(int i, int j, int time_factor)
{
    double distance = std::sqrt(
                        std::pow(nodes[i].x_coord - nodes[j].x_coord, 2) +
                        std::pow(nodes[i].y_coord - nodes[j].y_coord, 2)
                        );
    distance = std::round(distance * time_factor);
    return static_cast<int>(distance);
}

void OrienteeringModel::print_summary(){
    std::cout << "[INFO::model] Model summary\n";
    std::cout << "---------------------------------------------" << "\n";
    // std::cout << "|     inst_id : " << instance_name << "\n";
    std::cout << "|     #nodes  : " << nodes.size() 
            << " (start_id = " << STARTING_ID << ", end_id = " << ENDING_ID << ")\n";
    std::cout << "|     T_max   : " << T_MAX << "\n";
    std::cout << "---------------------------------------------" << "\n";
}

void OrienteeringModel::add_node(double x, double y, int score){
    model::Node new_node;
    new_node.id = nodes.size();
    new_node.x_coord = x;
    new_node.y_coord = y;
    new_node.score = score;
    nodes.emplace_back(new_node);
}
