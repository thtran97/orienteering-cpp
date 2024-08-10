#pragma once
#include <random>
#include "model/orienteering_model.hpp"

#define DEFAULT_RND_SEED 0

namespace oplib::solver{

    class IteratedLocalSearch
    {
    public:
        /* data */
        model::OrienteeringModel& model;
        std::random_device rd;
        std::mt19937 rnd_gen;
        
        IteratedLocalSearch(model::OrienteeringModel& model);
        ~IteratedLocalSearch()=default;

        void set_seed(int random_seed);
        void solve();
        
    };
    
}

