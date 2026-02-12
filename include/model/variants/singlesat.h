#pragma once

#include "optw.h"

namespace oplib::model::variants {

/**
 * @brief Single Satellite Scheduling Problem.
 * Typically modeled as an OPTW with single vehicle.
 */
class SingleSatProblem : public OPTWProblem {
public:
    SingleSatProblem(std::string name, Time tmax)
        : OPTWProblem(std::move(name), tmax) {}
    
    // Can add specific satellite constraints (e.g., transition times, memory) here.
};

} // namespace oplib::model::variants
