#include <iostream>
#include <climits>
#include <algorithm>
#include "solver/iterated_local_search.hpp"
#include "utils/rnd_generator.hpp"

using namespace oplib;
using namespace oplib::solver;

void IteratedLocalSearch::solve(){
    std::cout << "[INFO::solver] Solving model...\n";
    start_ts = std::chrono::high_resolution_clock::now();
    reset_visit_sequences();
    single_pass_construct();
    do {
        perturb();
        multi_pass_construct();
        check_acceptance_criterion();
    } while (!is_timeout_reached());
    std::cout << "[INFO::solver] Solving finished.\n";
}

bool IteratedLocalSearch::is_timeout_reached(){
    solving_duration = std::chrono::high_resolution_clock::now() - start_ts;
    return solving_duration.count() >= TIMEOUT_DURATION;    
}

void IteratedLocalSearch::single_pass_construct(){
    // O(N*M*M)
    std::cout << "single-pass-construction \n";
    // Generate a random order of nodes
    std::vector<int> selectable(NB_NODES);
    for (int i=0; i<NB_NODES; i++)
        selectable[i] = i;
    std::random_shuffle(selectable.begin(), selectable.end(), rnd_gen);

    // Process each node, except depots,in a random order
    // Greedily insert node into a path, if possible, otherwise process next node 
    for (int node_id: selectable){
        if (model.is_depot(node_id)) continue;
        // for each path, evaluate all possible insertions
        for (int path_id=0; path_id<NB_PATHS; path_id++){
            // check insertion into every possible position between two depots
            int min_shift = INT_MAX;
            int best_position = 0;
            for (int pos=1; pos<visit_sequences[path_id].size(); pos++){
                int shift = eval_insertion(node_id, path_id, pos);
                if (shift == INT_MAX) continue;
                if (shift < min_shift){
                    min_shift = shift;
                    best_position = pos; 
                }
            }
            // record the *best* move if exists and if possible
            if (best_position > 0) 
                push_move_insert(node_id, path_id, best_position, min_shift);        
        }
        
        // cannot insert node_id, pass
        if (restricted_feasible_moves.empty()) continue;

        // selection & apply the *best* insertion (in a randomized greedy way)
        std::vector<int> best_insertion = select_next_move().second;
        insert_node(node_id, best_insertion[0], best_insertion[1], best_insertion[2]);
    }
}

void IteratedLocalSearch::multi_pass_construct(){
    // O(N*N*M*M)
    std::cout << "multi-pass-construction\n";
    bool found_move;
    do{
        found_move = false;
        // evaluate insertion for each customer, except depots 
        for (int node_id=0; node_id<NB_NODES; node_id++){
            if (model.is_depot(node_id)) continue;
            // for each path, evaluate all possible insertions
            for (int path_id=0; path_id<NB_PATHS; path_id++){
                // check insertion into every possible position between two depots
                int min_shift = INT_MAX;
                int best_position = 0;
                for (int pos=1; pos<visit_sequences[path_id].size(); pos++){
                    int shift = eval_insertion(node_id, path_id, pos);
                    if (shift == INT_MAX) continue;
                    if (shift < min_shift){
                        min_shift = shift;
                        best_position = pos; 
                    }
                }
                // record the best move if exists and if possible
                if (best_position > 0) 
                    push_move_insert(node_id, path_id, best_position, min_shift);        
            }
        }

        if (! restricted_feasible_moves.empty()){
            // selection & apply the *best* insertion (in a randomized greedy way)
            std::vector<int> best_insertion = select_next_move().second;
            insert_node(best_insertion[0], best_insertion[1], best_insertion[2], best_insertion[3]);
            found_move = true;
        }
    } while (found_move);
}


void IteratedLocalSearch::perturb(){
    std::cout << "destroy sol\n";
    // TODO
}

void IteratedLocalSearch::check_acceptance_criterion(){
    std::cout << "Check acceptance \n";
    // TODO
}

void IteratedLocalSearch::reset_visit_sequences(){
    // * reset visit ordering info: seq = {start -> end}
    if (visit_sequences.empty())
        visit_sequences.resize(NB_PATHS);
    for (int path_id=0; path_id<NB_PATHS; path_id++){
        if (!visit_sequences[path_id].empty()) 
            visit_sequences[path_id].clear();
        visit_sequences[path_id] = {model.get_starting_id(), model.get_ending_id()};
    }
}

void IteratedLocalSearch::push_move_insert(int node_id, int path_id, int position, int time_shift){
    if (restricted_feasible_moves.size() > RCL_SIZE){
        restricted_feasible_moves.pop();
    }
    restricted_feasible_moves.emplace(std::make_pair(
        heuristic(model.get_node(node_id).score, time_shift),       // heuristic value of an insertion move
        std::vector<int>{node_id, path_id, position, time_shift}    // insertion move's info
    ));
}

int IteratedLocalSearch::eval_insertion(int node_id, int path_id, int position){
    // [classical OP]
    // shift = d(k-1,k) + d(k, k+1) - d(k-1, k+1)
    int prev_node_id = visit_sequences[path_id][position-1];
    int next_node_id = visit_sequences[path_id][position];
    int shift = model.get_travel_duration(prev_node_id, node_id) 
                + model.get_travel_duration(node_id, next_node_id) 
                - model.get_travel_duration(prev_node_id, next_node_id);
    // TODO: compare with T_max ?
    return shift;
}

custom_types::heuristic_move IteratedLocalSearch::select_next_move(){
    if (restricted_feasible_moves.empty()){
        std::cerr << "Error: Cannot select an element from an empty list\n";
        exit(1);
    }
    std::vector<custom_types::heuristic_move> candidates;
    double sum_f = 0.;

    while (!restricted_feasible_moves.empty()){
        auto move = restricted_feasible_moves.top();
        sum_f += move.first;
        candidates.emplace_back(move);
        restricted_feasible_moves.pop();
    }
    
    double accum_proba = 0.;
    double rnd_theshold = utils::get_rnd_double(0., 1., rnd_gen);
    
    for (const auto & move: candidates){
        accum_proba += move.first / sum_f;
        if (accum_proba > rnd_theshold)
            return move;
    }
    return candidates.back();
}

void IteratedLocalSearch::insert_node(int node_id, int path_id, int position, int shift_time){
    // TODO
}