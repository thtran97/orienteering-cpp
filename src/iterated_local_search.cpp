#include "solver/iterated_local_search.hpp"

using namespace oplib;
using namespace oplib::solver;

IteratedLocalSearch::IteratedLocalSearch(model::OrienteeringModel& model): model(model), rnd_gen(rd())
{
    rnd_gen.seed(DEFAULT_RND_SEED);
}

void IteratedLocalSearch::set_seed(int random_seed){
    rnd_gen.seed(random_seed);
}

void IteratedLocalSearch::solve(){
    
}
