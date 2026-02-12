#pragma once

#include "toptw.h"

namespace oplib::model::variants {

/**
 * @brief Tourist Trip Design Problem (TTDP).
 * Similar to TOPTW but often spanning multiple days with different time windows.
 */
class TTDPProblem : public TOPTWProblem {
public:
    TTDPProblem(std::string name, int num_days, Time tmax_per_day)
        : TOPTWProblem(std::move(name), num_days, tmax_per_day) {}
    
    // Legacy implementation shifts time windows based on start day.
    // New architecture can handle this by storing all day windows in the Node.
};

} // namespace oplib::model::variants
