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


int OrienteeringModel::get_travel_duration(int from_node_i, int to_node_j)
{
    return t_travel[from_node_i][to_node_j];
}

Node OrienteeringModel::get_node(int i)
{
    return nodes[i];
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

void OrienteeringModel::parse_instance(std::string file, int time_factor)
{
    ifstream infile(file);
    if (!infile){
        cerr << "Error: unable to open file " << file << endl;
        exit(1);
    }

    infile >> instance_name;
    int n;
    infile >> n;
    nodes.resize(n);
    for (int i = 0; i < n; ++i){
        infile >> nodes[i].id >> nodes[i].x_coord >> nodes[i].y_coord >> nodes[i].profit >> nodes[i].t_opening >> nodes[i].t_closing >> nodes[i].t_service;
    }
    infile.close();

    update_travel_duration_matrix(time_factor);
}

