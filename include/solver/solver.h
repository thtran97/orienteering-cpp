#pragma once

#include <memory>
#include <string>

#include "core/constants.h"
#include "model/problem.h"
#include "model/solution.h"

namespace oplib::solver {

/**
 * @brief Base class for all solver parameters.
 */
struct SolverConfig {
    int seed = oplib::constants::DEFAULT_SEED;
    double max_cpu_time = oplib::constants::DEFAULT_TIMELIMIT_SECONDS;
    int max_iterations = oplib::constants::DEFAULT_MAX_ITERATIONS;
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
