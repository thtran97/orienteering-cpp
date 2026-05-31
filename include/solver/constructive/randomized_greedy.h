#pragma once

#include <vector>
#include <memory>

#include "solver/solver.h"
#include "solver/constructive/greedy.h"
#include "core/random.h"

namespace oplib::solver::constructive {

/**
 * @brief Configuration for RandomizedGreedy solver.
 * 
 * Extends SolverConfig with RandomizedGreedy-specific parameters.
 */
struct RandomizedGreedyConfig : public SolverConfig {
    double alpha = 0.3;  // RCL parameter: controls randomness [0, 1]
                         // 0 = greedy (only best moves), 1 = random (all feasible moves)
};

/**
 * @brief Randomized Greedy Solver using Restricted Candidate List (RCL).
 * 
 * Extends GreedySolver to reuse insertion infrastructure (move_evaluator,
 * infeasibility_cache, apply_insertion, find_all_feasible_insertions).
 *
 * At each construction step, all feasible insertions are gathered and an RCL
 * is formed from those scoring within alpha of the best. A move is then
 * randomly selected from the RCL.
 *
 * - alpha = 0.0: pure greedy (only best move is in RCL)
 * - alpha = 1.0: fully random (all feasible moves are in RCL)
 */
class RandomizedGreedySolver : public GreedySolver {
public:
    std::string get_name() const override { return "Randomized Greedy"; }

    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override;

    /**
     * @brief Solve with RandomizedGreedy-specific configuration.
     */
    model::Solution solve(const model::Problem& problem, const RandomizedGreedyConfig& config);

private:
    model::Solution construct_randomized_greedy(
        const model::Problem& problem,
        const SolverConfig& config,
        utils::Random& rng,
        double alpha);
};

} // namespace oplib::solver::constructive
