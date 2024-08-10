#include <iostream>
#include "solver/iterated_local_search.hpp"

using namespace oplib;
using namespace oplib::solver;

IteratedLocalSearch::IteratedLocalSearch(model::OrienteeringModel& model): model(model), rnd_gen(rd())
{
    rnd_gen.seed(DEFAULT_RND_SEED);
}

void IteratedLocalSearch::solve(){
    std::cout << "[INFO::solver] Solving model...\n";
    start_ts = std::chrono::high_resolution_clock::now();
    construct_solution();
    do {
        destroy_solution();
        construct_solution();
        check_acceptance_criterion();
    } while (!is_timeout_reached());
    std::cout << "[INFO::solver] Solving finished.\n";
}

bool IteratedLocalSearch::is_timeout_reached(){
    solving_duration = std::chrono::high_resolution_clock::now() - start_ts;
    return solving_duration.count() >= TIMEOUT_DURATION;    
}

void IteratedLocalSearch::construct_solution(){
    std::cout << "construct sol \n";
}

void IteratedLocalSearch::destroy_solution(){
    std::cout << "destroy sol\n";
}

void IteratedLocalSearch::check_acceptance_criterion(){
    std::cout << "Check acceptance \n";
}