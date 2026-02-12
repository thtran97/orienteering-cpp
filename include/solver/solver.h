#pragma once

#include <memory>
#include <string>
#include "../model/problem.h"
#include "../model/solution.h"

namespace oplib::solver {

/**
 * @brief Base class for all solver parameters.
 */
struct SolverConfig {
    int seed = 0;
    double max_cpu_time = 60.0;
    int max_iterations = 1000;
    bool verbose = false;
};

/**
 * @brief Abstract interface for solvers.
 */
class Solver {
public:
    virtual ~Solver() = default;

    virtual std::string get_name() const = 0;
    
    /**
     * @brief Solves the given problem instance.
     * 
     * @param problem The problem to solve.
     * @param config Configuration parameters.
     * @return Solution The best solution found.
     */
    virtual model::Solution solve(const model::Problem& problem, const SolverConfig& config) = 0;
};

} // namespace oplib::solver
