#include "solver/constructive/move_evaluator.h"
#include "model/problem.h"

namespace oplib::solver::constructive {

double RewardOnlyEvaluator::evaluate(
    const Problem& problem,
    NodeId customer,
    Time travel_time_cost
) const {
    // Score is simply the reward of the customer
    return problem.get_reward(customer);
}


double RewardPerTimeEvaluator::evaluate(
    const Problem& problem,
    NodeId customer,
    Time travel_time_cost
) const {
    // Score = reward / (travel_time_cost + epsilon)
    // For time-window constrained problems, minimize time spent
    double reward = problem.get_reward(customer);
    return reward / (travel_time_cost + epsilon);
}

std::unique_ptr<MoveEvaluator> create_move_evaluator(const MoveEvaluatorType& evaluator_type) {
    if (evaluator_type == MoveEvaluatorType::REWARD) {
        return std::make_unique<RewardOnlyEvaluator>();
    } else if (evaluator_type == MoveEvaluatorType::REWARD_PER_TIME) {
        return std::make_unique<RewardPerTimeEvaluator>();
    }
    return nullptr; // Unrecognized type
}

} // namespace oplib::solver::constructive
