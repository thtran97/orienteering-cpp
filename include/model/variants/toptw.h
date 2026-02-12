#pragma once

#include "optw.h"

namespace oplib::model::variants {

/**
 * @brief Concrete implementation of the Team Orienteering Problem with Time Windows (TOPTW).
 */
class TOPTWProblem : public OPTWProblem {
public:
    TOPTWProblem(std::string name, int num_vehicles, Time tmax)
        : OPTWProblem(std::move(name), tmax), num_vehicles(num_vehicles) {}

    bool is_multi_vehicle() const override { return num_vehicles > 1; }
    int get_num_vehicles() const { return num_vehicles; }

private:
    int num_vehicles;
};

} // namespace oplib::model::variants
