#pragma once

#include "solver/solver.h"
#include "solver/local_search/base_ls.h"

namespace oplib::solver::constructive {

/**
 * @brief Configuration for the randomized greedy solver.
 *
 * Uses the restricted candidate list (RCL) heuristic: at each step a
 * candidate is drawn randomly from the top-rcl_size insertions weighted
 * by their heuristic score = reward^alpha / (time_shift + eps).
 */
struct RandomizedGreedySolverConfig : public SolverConfig {
    int    alpha    = oplib::constants::DEFAULT_ALPHA;    // reward exponent
    int    rcl_size = oplib::constants::DEFAULT_RCL_SIZE; // RCL capacity
};

/**
 * @brief Multi-start randomized greedy constructive solver.
 *
 * On each of max_iterations restarts:
 *  1. Reset the solution to empty depot routes.
 *  2. Call BaseLSUtils::repair() to greedily fill routes using the RCL.
 * Returns the best solution found across all restarts.
 *
 * This solver unblocks benchmark_randomized_greedy.cpp and serves as a
 * stand-alone baseline as well as the construction phase of GRASP+VNS.
 */
class RandomizedGreedySolver : public Solver {
public:
    std::string get_name() const override { return "RandomizedGreedy"; }

    model::Solution solve(const model::Problem& problem,
                          const SolverConfig&   config) override;

    model::Solution solve(const model::Problem&              problem,
                          const RandomizedGreedySolverConfig& config);
};

} // namespace oplib::solver::constructive
