#pragma once

#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "model/problem.h"
#include "model/solution.h"
#include "model/variants/mctopmtw.h"

namespace oplib::solver {

/**
 * @brief Result of a solution validity check.
 */
struct CheckResult {
    bool valid = true;
    std::vector<std::string> violations;

    void add_violation(std::string msg) {
        valid = false;
        violations.push_back(std::move(msg));
    }
};

/**
 * @brief Verifies that a Solution satisfies all constraints of a Problem.
 *
 * Handles all problem variants via the base Problem API (has_time_windows(),
 * is_time_dependent(), get_budget(), get_travel_time(), etc.).
 * Override check_variant_specific() to add extra checks for derived variants.
 */
class SolutionChecker {
public:
    virtual ~SolutionChecker() = default;

    CheckResult check(const model::Problem& problem,
                      const model::Solution& solution) const {
        CheckResult result;
        check_vehicle_count(problem, solution, result);
        check_route_structure(problem, solution, result);
        check_no_duplicates(problem, solution, result);
        check_reward_consistency(problem, solution, result);
        check_budgets_and_time_windows(problem, solution, result);
        check_variant_specific(problem, solution, result);
        return result;
    }

protected:
    void check_vehicle_count(const model::Problem& problem,
                             const model::Solution& solution,
                             CheckResult& result) const {
        if (solution.get_num_vehicles() != problem.get_num_vehicles()) {
            std::ostringstream oss;
            oss << "Vehicle count mismatch: solution has "
                << solution.get_num_vehicles() << ", problem expects "
                << problem.get_num_vehicles();
            result.add_violation(oss.str());
        }
    }

    void check_route_structure(const model::Problem& problem,
                               const model::Solution& solution,
                               CheckResult& result) const {
        const NodeId src = problem.get_source_depot();
        const NodeId snk = problem.get_sink_depot();
        for (int v = 0; v < solution.get_num_vehicles(); ++v) {
            const auto& route = solution.get_route(v);
            if (route.size() < 2) {
                std::ostringstream oss;
                oss << "Vehicle " << v << ": route has fewer than 2 nodes";
                result.add_violation(oss.str());
                continue;
            }
            if (route.front() != src) {
                std::ostringstream oss;
                oss << "Vehicle " << v << ": route does not start at source depot ("
                    << src << "), starts at " << route.front();
                result.add_violation(oss.str());
            }
            if (route.back() != snk) {
                std::ostringstream oss;
                oss << "Vehicle " << v << ": route does not end at sink depot ("
                    << snk << "), ends at " << route.back();
                result.add_violation(oss.str());
            }
        }
    }

    void check_no_duplicates(const model::Problem& problem,
                             const model::Solution& solution,
                             CheckResult& result) const {
        const NodeId src = problem.get_source_depot();
        const NodeId snk = problem.get_sink_depot();
        std::set<NodeId> visited;
        for (int v = 0; v < solution.get_num_vehicles(); ++v) {
            const auto& route = solution.get_route(v);
            for (size_t i = 1; i + 1 < route.size(); ++i) {
                NodeId node = route[i];
                if (node == src || node == snk) {
                    std::ostringstream oss;
                    oss << "Vehicle " << v << " position " << i
                        << ": depot node " << node << " appears in route interior";
                    result.add_violation(oss.str());
                    continue;
                }
                if (!visited.insert(node).second) {
                    std::ostringstream oss;
                    oss << "Node " << node << " is visited more than once across all vehicles";
                    result.add_violation(oss.str());
                }
            }
        }
    }

    void check_reward_consistency(const model::Problem& problem,
                                  const model::Solution& solution,
                                  CheckResult& result) const {
        const NodeId src = problem.get_source_depot();
        const NodeId snk = problem.get_sink_depot();
        Reward expected = 0.0;
        for (int v = 0; v < solution.get_num_vehicles(); ++v) {
            const auto& route = solution.get_route(v);
            for (size_t i = 1; i + 1 < route.size(); ++i) {
                NodeId node = route[i];
                if (node != src && node != snk)
                    expected += problem.get_reward(node);
            }
        }
        if (std::abs(solution.total_reward - expected) > 1e-6) {
            std::ostringstream oss;
            oss << "total_reward " << solution.total_reward
                << " does not match sum of visited node rewards " << expected;
            result.add_violation(oss.str());
        }
    }

    // Simulates each route to check both the travel-time budget and time windows
    // (when applicable). Uses get_travel_time(i, j, departure_time) for
    // time-dependent correctness.
    void check_budgets_and_time_windows(const model::Problem& problem,
                                        const model::Solution& solution,
                                        CheckResult& result) const {
        const bool check_tw = problem.has_time_windows();
        const Time budget = problem.get_budget();

        for (int v = 0; v < solution.get_num_vehicles(); ++v) {
            const auto& route = solution.get_route(v);
            if (route.size() < 2) continue;

            Time current_time = problem.get_time_window(route.front()).opening;
            current_time += problem.get_service_time(route.front());
            Time travel_sum = 0.0;

            for (size_t i = 0; i + 1 < route.size(); ++i) {
                NodeId from = route[i];
                NodeId to   = route[i + 1];

                Time arc_time = problem.get_travel_time(from, to, current_time);
                travel_sum += arc_time;
                Time arrival = current_time + arc_time;

                if (check_tw) {
                    const auto& tw = problem.get_time_window(to);
                    if (arrival > tw.closing + 1e-9) {
                        std::ostringstream oss;
                        oss << "Vehicle " << v << ": arrives at node " << to
                            << " at time " << arrival
                            << " which exceeds its closing time " << tw.closing;
                        result.add_violation(oss.str());
                    }
                    current_time = std::max(arrival, tw.opening)
                                   + problem.get_service_time(to);
                } else {
                    current_time = arrival + problem.get_service_time(to);
                }
            }

            if (travel_sum > budget + 1e-9) {
                std::ostringstream oss;
                oss << "Vehicle " << v << ": total arc travel time " << travel_sum
                    << " exceeds budget " << budget;
                result.add_violation(oss.str());
            }
        }
    }

    virtual void check_variant_specific(const model::Problem& /*problem*/,
                                        const model::Solution& /*solution*/,
                                        CheckResult& /*result*/) const {}
};

/**
 * @brief Extends SolutionChecker with knapsack constraint validation for MCTOPMTW.
 */
class MCTOPMTWChecker : public SolutionChecker {
protected:
    void check_variant_specific(const model::Problem& problem,
                                const model::Solution& solution,
                                CheckResult& result) const override {
        const auto* p = dynamic_cast<const model::variants::MCTOPMTWProblem*>(&problem);
        if (!p) return;

        const auto& rhs    = p->get_knapsack_rhs();
        const auto& coeffs = p->get_knapsack_coeffs();
        const NodeId src   = problem.get_source_depot();
        const NodeId snk   = problem.get_sink_depot();

        for (int k = 0; k < p->get_num_knapsack_constraints(); ++k) {
            double lhs = 0.0;
            for (int v = 0; v < solution.get_num_vehicles(); ++v) {
                for (NodeId node : solution.get_route(v)) {
                    if (node == src || node == snk) continue;
                    if (node < static_cast<NodeId>(coeffs[k].size()))
                        lhs += coeffs[k][node];
                }
            }
            if (lhs > rhs[k] + 1e-9) {
                std::ostringstream oss;
                oss << "Knapsack constraint " << k << " violated: lhs=" << lhs
                    << " > rhs=" << rhs[k];
                result.add_violation(oss.str());
            }
        }
    }
};

/**
 * @brief Returns the appropriate checker for the given problem.
 */
inline std::unique_ptr<SolutionChecker> create_checker(const model::Problem& problem) {
    if (dynamic_cast<const model::variants::MCTOPMTWProblem*>(&problem))
        return std::make_unique<MCTOPMTWChecker>();
    return std::make_unique<SolutionChecker>();
}

} // namespace oplib::solver
