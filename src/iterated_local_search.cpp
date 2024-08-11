#include <iostream>
#include <climits>

#include "solver/iterated_local_search.hpp"
#include "utils/rnd_generator.hpp"

using namespace oplib;
using namespace oplib::solver;

void IteratedLocalSearch::solve(){
    std::cout << "[INFO::solver] Solving model...\n";
    start_ts = std::chrono::high_resolution_clock::now();
    reset_visit_sequences();
    construct();
    do {
        perturb();
        construct();
        check_acceptance_criterion();
    } while (!is_timeout_reached());
    std::cout << "[INFO::solver] Solving finished.\n";
}

bool IteratedLocalSearch::is_timeout_reached(){
    solving_duration = std::chrono::high_resolution_clock::now() - start_ts;
    return solving_duration.count() >= TIMEOUT_DURATION;    
}

void IteratedLocalSearch::construct(){
    std::cout << "construct sol \n";
    // for each customer, except depots 
    for (int node_id=0; node_id<NB_NODES; node_id++){
        if (model.is_depot(node_id)) continue;
        // for each path
        for (int path_id=0; path_id<NB_PATHS; path_id++){
            // check insertion into every possible position between two depots
            int min_shift = INT_MAX;
            int best_position = 0;
            for (int pos=1; pos<visit_sequences[path_id].size(); pos++){
                int shift = try_insertion(node_id, path_id, pos);
                if (shift == INT_MAX) continue;
                if (shift < min_shift){
                    min_shift = shift;
                    best_position = pos; 
                }
            }
            // record the best move if exists and if possible
            if (best_position > 0) 
                push_move_insert(node_id, path_id, best_position, min_shift);
                
            // TODO : selection & apply insertin (greedy or random ?)    
        }
    }
}

void IteratedLocalSearch::perturb(){
    std::cout << "destroy sol\n";
}

void IteratedLocalSearch::check_acceptance_criterion(){
    std::cout << "Check acceptance \n";
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

int IteratedLocalSearch::try_insertion(int node_id, int path_id, int position){
    // TODO
    return 0;
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
