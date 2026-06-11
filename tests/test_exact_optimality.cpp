// ============================================================================
// test_exact_optimality.cpp
//
// Phase 1 (exact / bounding layer): verify that the exact single-vehicle
// solvers (ForwardDP, BidirectionalDP, Pulse) return the *true optimum*.
//
// Strategy: an independent brute-force oracle enumerates every feasible simple
// path src -> ... -> sink and records the maximum collectable reward.  We then
// assert each exact solver matches the oracle on many randomized OP and OPTW
// instances (small enough for brute force, varied enough to exercise budget /
// time-window pruning).  BackwardDP bounds are checked for validity (upper
// bound >= optimum reachable from each node).
// ============================================================================

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/pulse/pulse_solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;

namespace {

// ---------------------------------------------------------------------------
// Brute-force oracle: maximum reward over all feasible simple paths src->sink.
// Independent of the DP/Pulse code so it can act as ground truth.
// ---------------------------------------------------------------------------
class BruteForce {
public:
    explicit BruteForce(const model::Problem& p)
        : p_(p),
          n_(p.get_num_nodes()),
          src_(p.get_source_depot()),
          sink_(p.get_sink_depot()) {}

    Reward optimum() {
        std::vector<bool> visited(n_, false);
        visited[src_] = true;
        Time t0 = p_.get_time_window(src_).opening + p_.get_service_time(src_);
        best_ = 0.0;
        recurse(src_, t0, 0.0, visited);
        return best_;
    }

private:
    void recurse(NodeId node, Time time, Reward reward, std::vector<bool>& visited) {
        // Option: close the route now by going to the sink.
        if (can_reach_sink(node, time)) best_ = std::max(best_, reward);

        for (NodeId j = 1; j < n_; ++j) {
            if (j == sink_ || visited[j]) continue;
            Time arr = time + p_.get_travel_time(node, j, time);
            const auto& tw = p_.get_time_window(j);
            if (arr > tw.closing + kEps) continue;
            Time dep = std::max(arr, tw.opening) + p_.get_service_time(j);
            if (!can_reach_sink(j, dep)) continue;  // dead-end prune (still exact)
            visited[j] = true;
            recurse(j, dep, reward + p_.get_reward(j), visited);
            visited[j] = false;
        }
    }

    bool can_reach_sink(NodeId node, Time time) const {
        Time arr_sink = time + p_.get_travel_time(node, sink_, time);
        if (arr_sink > p_.get_time_window(sink_).closing + kEps) return false;
        Time elapsed = arr_sink - p_.get_time_window(src_).opening;
        return elapsed <= p_.get_budget() + kEps;
    }

    static constexpr double kEps = 1e-9;
    const model::Problem& p_;
    NodeId n_, src_, sink_;
    Reward best_ = 0.0;
};

// ---------------------------------------------------------------------------
// Randomized instance generators (depot at index 0, sink at index n-1).
// ---------------------------------------------------------------------------
OPProblem make_random_op(unsigned seed, int n_customers, Time budget) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> coord(0.0, 20.0);
    std::uniform_real_distribution<double> reward(1.0, 20.0);

    OPProblem p("rand_op_" + std::to_string(seed), budget);
    p.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    for (int i = 1; i <= n_customers; ++i)
        p.add_node(Node{i, coord(rng), coord(rng), reward(rng), 0.0});
    p.add_node(Node{n_customers + 1, 0.0, 0.0, 0.0, 0.0});
    p.finalize();
    p.preprocessing();
    return p;
}

OPTWProblem make_random_optw(unsigned seed, int n_customers, Time budget) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> coord(0.0, 20.0);
    std::uniform_real_distribution<double> reward(1.0, 20.0);
    std::uniform_real_distribution<double> open(0.0, 25.0);
    std::uniform_real_distribution<double> width(25.0, 70.0);

    OPTWProblem p("rand_optw_" + std::to_string(seed), budget);
    p.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, budget}});
    for (int i = 1; i <= n_customers; ++i) {
        double o = open(rng);
        p.add_node(Node{i, coord(rng), coord(rng), reward(rng), 1.0, {o, o + width(rng)}});
    }
    p.add_node(Node{n_customers + 1, 0.0, 0.0, 0.0, 0.0, {0.0, budget}});
    p.finalize();
    p.preprocessing();
    return p;
}

solver::dp::DPSolverConfig dp_cfg() {
    solver::dp::DPSolverConfig c;
    c.seed = 1; c.verbose = false; c.max_labels = 0;  // unlimited for exactness
    return c;
}
solver::pulse::PulseSolverConfig pulse_cfg() {
    solver::pulse::PulseSolverConfig c;
    c.seed = 1; c.verbose = false; c.max_labels = 0;  // unlimited for exactness
    return c;
}

// Assert that `solve_fn` returns the brute-force optimum across `count`
// randomized instances built by `make` (seeds 1..count).
template <class ProblemFactory, class SolveFn>
void expect_optimal(const char* label, ProblemFactory make, SolveFn solve_fn,
                    unsigned count = 25) {
    for (unsigned seed = 1; seed <= count; ++seed) {
        auto problem = make(seed);
        Reward opt = BruteForce(problem).optimum();
        EXPECT_NEAR(solve_fn(problem), opt, 1e-6) << label << " seed=" << seed;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Optimality: every exact single-vehicle solver must match the brute-force
// optimum on randomized OP and OPTW instances.
// ---------------------------------------------------------------------------
TEST(ExactOptimality, ForwardDP_MatchesBruteForce_OP) {
    expect_optimal("ForwardDP OP",
        [](unsigned s) { return make_random_op(s, 6, 45.0); },
        [](const model::Problem& p) { return solver::dp::ForwardDPSolver().solve(p, dp_cfg()).total_reward; });
}
TEST(ExactOptimality, ForwardDP_MatchesBruteForce_OPTW) {
    expect_optimal("ForwardDP OPTW",
        [](unsigned s) { return make_random_optw(s, 6, 60.0); },
        [](const model::Problem& p) { return solver::dp::ForwardDPSolver().solve(p, dp_cfg()).total_reward; });
}
TEST(ExactOptimality, BidirectionalDP_MatchesBruteForce_OP) {
    expect_optimal("BidirectionalDP OP",
        [](unsigned s) { return make_random_op(s, 6, 45.0); },
        [](const model::Problem& p) { return solver::dp::BidirectionalDPSolver().solve(p, dp_cfg()).total_reward; });
}
TEST(ExactOptimality, BidirectionalDP_MatchesBruteForce_OPTW) {
    expect_optimal("BidirectionalDP OPTW",
        [](unsigned s) { return make_random_optw(s, 6, 60.0); },
        [](const model::Problem& p) { return solver::dp::BidirectionalDPSolver().solve(p, dp_cfg()).total_reward; });
}
TEST(ExactOptimality, Pulse_MatchesBruteForce_OP) {
    expect_optimal("Pulse OP",
        [](unsigned s) { return make_random_op(s, 6, 45.0); },
        [](const model::Problem& p) { return solver::pulse::PulseSolver().solve(p, pulse_cfg()).total_reward; });
}
TEST(ExactOptimality, Pulse_MatchesBruteForce_OPTW) {
    expect_optimal("Pulse OPTW",
        [](unsigned s) { return make_random_optw(s, 6, 60.0); },
        [](const model::Problem& p) { return solver::pulse::PulseSolver().solve(p, pulse_cfg()).total_reward; });
}

// ---------------------------------------------------------------------------
// BackwardDP bounds must be valid upper bounds: ub[src] >= optimum.
// ---------------------------------------------------------------------------
TEST(ExactOptimality, BackwardDP_BoundDominatesOptimum) {
    for (unsigned seed = 1; seed <= 25; ++seed) {
        auto p = make_random_op(seed, 6, 45.0);
        Reward opt = BruteForce(p).optimum();
        solver::dp::BackwardDPSolver bw;
        auto ub = bw.compute_bounds(p);
        // ub[src] is a relaxation, so it must be >= the true optimum.
        EXPECT_GE(ub[p.get_source_depot()], opt - 1e-6) << "Backward bound OP seed=" << seed;
        EXPECT_DOUBLE_EQ(ub[p.get_sink_depot()], 0.0);
    }
}
