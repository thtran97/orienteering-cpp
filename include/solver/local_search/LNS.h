#pragma once

#include <vector>
#include <memory>

#include "solver/solver.h"
#include "core/random.h"

namespace oplib::solver::local_search {

/**
 * @brief Large Neighborhood Search (LNS) Solver.
 */
class LNSSolver : public Solver {
public:
    std::string get_name() const override { return "LNS"; }

    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override;

    /**
     * @brief Run LNS starting from an externally-provided initial solution.
     * 
     * Used by GRASP to improve solutions from the construction phase.
     */
    model::Solution solve_from(
        const model::Problem& problem,
        const model::Solution& initial_solution,
        const SolverConfig& config
    );

    // Specific LNS parameters
    int min_destroy = 2;
    int max_destroy = 10;

private:
    /**
     * @brief Randomly destroy a contiguous subsequence from the solution.
     * 
     * Selects a random starting position and removal length, then removes that many
     * customer nodes from the route. If removal extends past the end of the route,
     * it wraps around to the beginning (skipping depots at both ends).
     * Example: route [0,1,3,2,5,4,6], start_pos=4, remove_len=3 → [0,3,2,6]
     * (removes nodes at indices 4,5,1, skipping depot 0 and 6).
     */
    void destroy_random(
        const model::Problem& problem,
        model::Solution& solution,
        int count,
        utils::Random& rng);
    void repair_greedy(const model::Problem& problem, model::Solution& solution, const SolverConfig& config);
};

} // namespace oplib::solver::local_search
