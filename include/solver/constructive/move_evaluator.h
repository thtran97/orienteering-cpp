#pragma once

#include <memory>
#include "model/problem.h"
#include "core/types.h"

namespace oplib::solver::constructive {

using oplib::model::Problem;

/**
 * @brief Abstract base class for scoring/evaluating insertion moves.
 * 
 * Different heuristic strategies can prioritize moves differently:
 * - Reward-only: greedily pick highest reward customer
 * - Reward per distance: balance profit with cost efficiency
 * - Reward per time: optimize for time-window constrained variants
 * 
 * This abstraction allows the greedy construction algorithm to be independent
 * of the evaluation strategy used to select the best move.
 */
class MoveEvaluator {
public:
    virtual ~MoveEvaluator() = default;

    /**
     * @brief Compute a score for an insertion move.
     * 
     * Higher scores indicate better moves. The scoring function can incorporate:
     * - Customer reward (always increases with customer value)
     * - Travel time cost (penalizes moves that add travel time)
     * - Other factors (proximity to time windows, etc.)
     * 
     * @param problem The problem instance
     * @param customer The customer being inserted
     * @param travel_time_cost The additional travel time incurred by insertion
     * @return Score value (higher is better)
     */
    virtual double evaluate(
        const Problem& problem,
        NodeId customer,
        Time travel_time_cost
    ) const = 0;

    /**
     * @brief Get a human-readable name for this evaluator (for logging/debugging).
     */
    virtual std::string get_name() const = 0;
};

/**
 * @brief Evaluator that scores moves purely by customer reward.
 * 
 * This is the simplest strategy: always pick the customer with highest reward,
 * regardless of insertion cost. Useful as a baseline or for problems where
 * feasible insertions are scarce.
 */
class RewardOnlyEvaluator : public MoveEvaluator {
public:
    double evaluate(
        const Problem& problem,
        NodeId customer,
        Time travel_time_cost
    ) const override;

    std::string get_name() const override { return "RewardOnlyEvaluator"; }
};

/**
 * @brief Evaluator that scores moves by reward-to-time ratio.
 * 
 * Score = reward / (travel_time_cost + epsilon)
 * Similar to travel time-based evaluation but uses total time as the cost metric.
 * More appropriate for time-window constrained problems (OPTW, TOPTW).
 */
class RewardPerTimeEvaluator : public MoveEvaluator {
public:
    explicit RewardPerTimeEvaluator(Time epsilon = 0.001)
        : epsilon(epsilon) {}

    double evaluate(
        const Problem& problem,
        NodeId customer,
        Time travel_time_cost
    ) const override;

    std::string get_name() const override { return "RewardPerTimeEvaluator"; }

private:
    Time epsilon;  // Small value to prevent division by zero
};

/**
 * @brief Factory function to create a move evaluator by name.
 * 
 * @param evaluator_type Enum identifier for the desired evaluator type
 * @return Unique pointer to created evaluator, or nullptr if type is unrecognized
 */
std::unique_ptr<MoveEvaluator> create_move_evaluator(const MoveEvaluatorType& evaluator_type);

} // namespace oplib::solver::constructive
