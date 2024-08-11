#pragma once
#include <string>
#include <vector>
#include "model/node.hpp"

#define DEFAULT_TIME_FACTOR 100

namespace oplib::model{

    class OrienteeringModel
    {
    public:
        /* data */
        std::string instance_name = "unknown";

        /* constructors / destructors */
        OrienteeringModel(){};
        ~OrienteeringModel()=default;
        
        /* method */

        void add_node(double x, double y, int score);
        void print_summary();
        void update_travel_duration_matrix(int time_factor);
        void preprocessing_precedence_graph();
        
        inline Node & get_node(int id){return nodes[id];};
        inline int get_nb_nodes(){return nodes.size();};
        inline int get_starting_id(){return STARTING_ID;};
        inline int get_ending_id(){return ENDING_ID;};
        inline int get_travel_duration(int from_node_i, int to_node_j){return t_travel[from_node_i][to_node_j];};

        inline void set_time_budget(int t_max){T_MAX=t_max;};
        inline void set_starting_point(int id){STARTING_ID=id;};
        inline void set_ending_point(int id){ENDING_ID=id;};
        inline bool is_depot(int id){return id == STARTING_ID || id == ENDING_ID;};
    
    protected:
        /* data */
        int T_MAX;
        int STARTING_ID, ENDING_ID;
        std::vector<Node> nodes;
        std::vector<std::vector<int>> t_travel;
        /* method */
        int compute_travel_duration(int i, int j, int time_factor);
    };
    
}
