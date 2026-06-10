#pragma once

#include <algorithm>

#include "model/problem.h"
#include "model/solution.h"

namespace oplib::solver::local_search {

/**
 * @brief Base class for a Local Search move.
 */
struct Move {
    virtual ~Move() = default;
    virtual bool apply(model::Solution& sol) const = 0;
    double delta_reward = 0.0;
    double delta_cost = 0.0;
};

struct InsertionMove : public Move {
    NodeId customer;
    int vehicle;
    int position;

    bool apply(model::Solution& sol) const override {
        auto& route = sol.get_route(vehicle);
        route.insert(route.begin() + position, customer);
        return true;
    }
};

struct SwapMove : public Move {
    int vehicle;
    int pos1;
    int pos2;

    bool apply(model::Solution& sol) const override {
        auto& route = sol.get_route(vehicle);
        std::swap(route[pos1], route[pos2]);
        return true;
    }
};

struct TwoOptMove : public Move {
    int vehicle;
    int pos1;
    int pos2;

    bool apply(model::Solution& sol) const override {
        auto& route = sol.get_route(vehicle);
        std::reverse(route.begin() + pos1, route.begin() + pos2 + 1);
        return true;
    }
};

} // namespace oplib::solver::local_search
