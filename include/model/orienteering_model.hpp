#pragma once
#include <string>
#include <vector>
#include "model/node.hpp"

namespace oplib::model{

    class OrienteeringModel
    {
    public:
        /* data */
        std::string instance_name;
        /* constructors / destructors */
        OrienteeringModel();
        ~OrienteeringModel()=default;
        /* method */
        void parse_instance(std::string file, int time_factor);
        int get_travel_duration(int from_node_i, int to_node_j);
        Node get_node(int i);

    protected:
        /* data */
        std::vector<Node> nodes;
        std::vector<std::vector<int>> t_travel;
        /* method */
        void update_travel_duration_matrix(int time_factor);
        int compute_travel_duration(int i, int j, int time_factor);
    };
    
}
