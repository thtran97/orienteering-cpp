#pragma once

#include "toptw.h"

namespace oplib::model::variants {

/**
 * @brief Multi-Constraint TOP with Multi-Time Windows (MCTOPMTW).
 */
class MCTOPMTWProblem : public TOPTWProblem {
public:
    MCTOPMTWProblem(std::string name, int num_vehicles, Time tmax)
        : TOPTWProblem(std::move(name), num_vehicles, tmax) {}

    void add_knapsack_constraint(double rhs, std::vector<double> coeffs) {
        knapsack_rhs.push_back(rhs);
        knapsack_coeffs.push_back(std::move(coeffs));
    }

    // New variants for multi-TW could be added as a list of TWindows per node
    // For now, let's just hold the knapsack constraints.

private:
    std::vector<double> knapsack_rhs;
    std::vector<std::vector<double>> knapsack_coeffs; // ConstraintIdx x NodeIdx
};

} // namespace oplib::model::variants
