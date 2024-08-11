#pragma once
#include <random>

namespace oplib::utils{

    double get_rnd_int(int lb, int ub, std::mt19937 rnd_gen){
        auto int_dis = std::uniform_int_distribution<>(lb, ub);
        return int_dis(rnd_gen);
    }

    double get_rnd_double(double lb, double ub, std::mt19937 rnd_gen){
        auto real_dis = std::uniform_real_distribution<>(lb, ub);
        return real_dis(rnd_gen);
    }

}