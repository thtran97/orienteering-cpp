#pragma once
#include <random>
#include <chrono>
#include <queue>
#include <vector>
#include "model/orienteering_model.hpp"

#define DEFAULT_RND_SEED 0


namespace oplib::solver{

    namespace custom_types{
        // Move's info, including heuristic value and other necessary data to excecute this move
        // < heuristic_value, {c, p, v, shift} > for insertion, 
        // < heuristic_value, {vehicle, position, length}>  for destroy
        using heuristic_move = std::pair<double,std::vector<int>>;   
        
        struct decreasing_cmp{
            bool operator()(heuristic_move const & a, heuristic_move const & b) const{
                return a.first > b.first;
            }
        };
        struct increasing_cmp{
            bool operator()(heuristic_move const & a, heuristic_move const & b) const{
                return a.first < b.first;
            }
        };

        // Queue of heuristic moves <heuristic_value, move_info>, where move_info is a vector<int>
        // Whenn invoking pop(), the element having the smallest heuristic value is discared from the queue
        using heuristic_queue = std::priority_queue<heuristic_move, std::vector<heuristic_move>, decreasing_cmp>;     
    }

    class IteratedLocalSearch
    {
    public:
        IteratedLocalSearch(model::OrienteeringModel& model): model(model), rnd_gen(rd())
        {
            rnd_gen.seed(DEFAULT_RND_SEED);
            NB_NODES = model.get_nb_nodes();
        }
        ~IteratedLocalSearch()=default;

        void solve();

        inline void set_seed(int random_seed){rnd_gen.seed(random_seed);};
        inline void set_timeout(double timeout){TIMEOUT_DURATION=timeout* 1000.;};
        inline void set_nb_paths(int nb_paths){NB_PATHS=nb_paths;};

    private:
        model::OrienteeringModel& model;
        std::random_device rd;
        std::mt19937 rnd_gen;
        
        double TIMEOUT_DURATION; // MAx CPU time used for solving problem
        std::chrono::duration<double, std::milli> solving_duration; // solving duration until now
        std::chrono::high_resolution_clock::time_point start_ts; // at which timestamp the solver starts

        int NB_PATHS = 1; // nb of paths used
        int NB_NODES = 0; // nb of nodes in the graph (including depots)

        std::vector<std::vector<int>> visit_sequences; // node ordering in each path
        std::vector<int> t_arrival; // used for recording the arrival time at a node if being visited
        std::vector<int> max_shift; // used for incremental checking temporal constraints
        custom_types::heuristic_queue restricted_feasible_moves; // restricted list of *best k* feasible moves 

        // --------SOLVING PARAMETERS------------
        int RCL_SIZE = 5; // size of the Restricted Candidate List (RCL)

        // --------------------------------------


        bool is_timeout_reached();
        void single_pass_construct();
        void multi_pass_construct();
        void perturb();
        void check_acceptance_criterion();

        void reset_visit_sequences();
        // Try the insertion of `node_id` into the `position` in `path_id`, and return the time_shift if possible, otherwise `INT_MAX`
        int eval_insertion(int node_id, int path_id, int position); 
        void push_move_insert(int node_id, int path_id, int position, int time_shift);
        // Insert `node_id` into into the `position` in `path_id`, but requires invoking `eval_insertion` before applying insertion. 
        void insert_node(int node_id, int path_id, int position, int shift_time);
        
        // Select a move from `restricted_feasible_moves` using Random Wheel Selection (RWS)
        custom_types::heuristic_move select_next_move();


        // --------------------------------------

        inline double heuristic(int score, double shift){
            return pow(static_cast<double>(score), 2) / (shift + 1e-5);
        } 
    };

    
}

