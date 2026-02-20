#pragma once

#include <random>
#include <algorithm>

namespace oplib::utils {

class Random {
public:
    explicit Random(uint32_t seed = 0) : engine(seed) {}

    void set_seed(uint32_t seed) {
        engine.seed(seed);
    }

    int next_int(int lb, int ub) {
        std::uniform_int_distribution<int> dist(lb, ub);
        return dist(engine);
    }

    double next_double(double lb = 0.0, double ub = 1.0) {
        std::uniform_real_distribution<double> dist(lb, ub);
        return dist(engine);
    }

    template<typename T>
    void shuffle(std::vector<T>& v) {
        std::shuffle(v.begin(), v.end(), engine);
    }

    template<typename T>
    const T& pick_random(const std::vector<T>& v) {
        return v[next_int(0, static_cast<int>(v.size()) - 1)];
    }

private:
    std::mt19937 engine;
};

} // namespace oplib::utils
