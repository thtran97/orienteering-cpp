#pragma once

#include "op.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of the Team Orienteering Problem (TOP).
 * Multiple vehicles, each limited by the same time budget.
 */
class TOPProblem : public OPProblem {
public:
    TOPProblem(std::string name, int num_vehicles, Time budget)
        : OPProblem(std::move(name), budget), num_vehicles(num_vehicles) {}

    bool is_multi_vehicle() const override { return num_vehicles > 1; }
    int get_num_vehicles() const override { return num_vehicles; }

private:
    int num_vehicles;
};

} // namespace oplib::model::variants
