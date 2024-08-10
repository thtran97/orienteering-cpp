#pragma once
#include <random>
#include <chrono>
#include "model/orienteering_model.hpp"

#define DEFAULT_RND_SEED 0


namespace oplib::solver{

    class IteratedLocalSearch
    {
    public:
        /* data */
        IteratedLocalSearch(model::OrienteeringModel& model);
        ~IteratedLocalSearch()=default;

        inline void set_seed(int random_seed){rnd_gen.seed(random_seed);};
        inline void set_timeout(double timeout){TIMEOUT_DURATION=timeout* 1000.;};
        
        void solve();
    
    private:
        model::OrienteeringModel& model;
        std::random_device rd;
        std::mt19937 rnd_gen;
        
        double TIMEOUT_DURATION;
        std::chrono::duration<double, std::milli> solving_duration;
        std::chrono::high_resolution_clock::time_point start_ts; // at which timestamp the solver starts

        bool is_timeout_reached();
        void construct_solution();
        void destroy_solution();
        void check_acceptance_criterion();


        
    };
    
}

