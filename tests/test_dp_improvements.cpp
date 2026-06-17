// ============================================================================
// test_dp_improvements.cpp
//
// Tests for the DP solver improvements based on Righini & Salani (2006):
//   Phase 1 — Label dominance fix (strict inequality)
//   Phase 2 — TrueBackwardDPSolver
//   Phase 3 — TrueBidirectionalDPSolver
//   Phase 4 — DSSRSolver
// ============================================================================

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/dynamic_programming/label.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;

namespace {

// ---------------------------------------------------------------------------
// Brute-force oracle (same as test_exact_optimality.cpp)
// ---------------------------------------------------------------------------
class BruteForce {
public:
    explicit BruteForce(const model::Problem& p)
        : p_(p), n_(p.get_num_nodes()),
          src_(p.get_source_depot()), sink_(p.get_sink_depot()) {}

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
        if (can_reach_sink(node, time)) best_ = std::max(best_, reward);
        for (NodeId j = 1; j < n_; ++j) {
            if (j == sink_ || visited[j]) continue;
            Time arr = time + p_.get_travel_time(node, j, time);
            const auto& tw = p_.get_time_window(j);
            if (arr > tw.closing + kEps) continue;
            Time dep = std::max(arr, tw.opening) + p_.get_service_time(j);
            if (!can_reach_sink(j, dep)) continue;
            visited[j] = true;
            recurse(j, dep, reward + p_.get_reward(j), visited);
            visited[j] = false;
        }
    }

    bool can_reach_sink(NodeId node, Time time) const {
        Time arr_sink = time + p_.get_travel_time(node, sink_, time);
        if (arr_sink > p_.get_time_window(sink_).closing + kEps) return false;
        return arr_sink - p_.get_time_window(src_).opening <= p_.get_budget() + kEps;
    }

    static constexpr double kEps = 1e-9;
    const model::Problem& p_;
    NodeId n_, src_, sink_;
    Reward best_ = 0.0;
};

OPProblem make_random_op(unsigned seed, int n_customers = 6, Time budget = 45.0) {
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

OPTWProblem make_random_optw(unsigned seed, int n_customers = 6, Time budget = 60.0) {
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

solver::dp::DPSolverConfig unlimited_cfg() {
    solver::dp::DPSolverConfig c;
    c.seed = 1; c.verbose = false; c.max_labels = 0;
    return c;
}

template <class ProblemFactory, class SolveFn>
void expect_optimal(const char* tag, ProblemFactory make, SolveFn solve_fn,
                    unsigned count = 25) {
    for (unsigned s = 1; s <= count; ++s) {
        auto prob = make(s);
        Reward opt = BruteForce(prob).optimum();
        EXPECT_NEAR(solve_fn(prob), opt, 1e-6) << tag << " seed=" << s;
    }
}

// Helper to build a minimal Label for unit testing
solver::dp::Label make_label(Time t, Reward p, std::vector<bool> vis) {
    return solver::dp::Label(0, t, p, nullptr, std::move(vis));
}

}  // namespace

// ============================================================================
// Phase 1: Label::dominates() — strict inequality required
// ============================================================================

TEST(DominanceFix, IdenticalLabelsDoNotDominate) {
    auto L = make_label(5.0, 10.0, {true, false, true});
    EXPECT_FALSE(L.dominates(L))
        << "A label must not dominate itself (no strict dimension)";

    auto L2 = make_label(5.0, 10.0, {true, false, true});
    EXPECT_FALSE(L.dominates(L2))
        << "Identical labels must not dominate each other";
    EXPECT_FALSE(L2.dominates(L));
}

TEST(DominanceFix, WeakerLabelStillDominated) {
    // L1 is strictly better on time
    auto L1 = make_label(4.0, 10.0, {true, false, true});
    auto L2 = make_label(5.0, 10.0, {true, false, true});
    EXPECT_TRUE(L1.dominates(L2));
    EXPECT_FALSE(L2.dominates(L1));

    // L1 is strictly better on profit
    auto L3 = make_label(5.0, 11.0, {true, false, true});
    auto L4 = make_label(5.0, 10.0, {true, false, true});
    EXPECT_TRUE(L3.dominates(L4));
    EXPECT_FALSE(L4.dominates(L3));

    // L1 has strictly fewer visits (subset)
    auto L5 = make_label(5.0, 10.0, {true, false, false});
    auto L6 = make_label(5.0, 10.0, {true, false, true});
    EXPECT_TRUE(L5.dominates(L6));
    EXPECT_FALSE(L6.dominates(L5));
}

TEST(DominanceFix, IncomparableLabelsNeitherDominates) {
    // Better time but worse profit
    auto L1 = make_label(4.0,  9.0, {true, false, false});
    auto L2 = make_label(5.0, 10.0, {true, false, false});
    EXPECT_FALSE(L1.dominates(L2));
    EXPECT_FALSE(L2.dominates(L1));

    // L3 has more profit but a superset of visited nodes vs L4 — incomparable:
    // L4.visited ⊄ L3.visited (L3.visited[1]=true, L4.visited[1]=false fails ⊆ check)
    // L4 lacks the profit advantage needed to dominate either.
    auto L3 = make_label(5.0, 12.0, {true, true,  false});  // high profit, more visited
    auto L4 = make_label(5.0, 10.0, {true, false, false});  // low profit, fewer visited
    EXPECT_FALSE(L3.dominates(L4));  // visited(L3)={0,1} ⊄ visited(L4)={0}
    EXPECT_FALSE(L4.dominates(L3)); // profit(L4) < profit(L3)
}

TEST(DominanceFix, ForwardDP_StillOptimal_OP) {
    // Regression: dominance fix must not break ForwardDP correctness
    expect_optimal("ForwardDP OP (post-fix)",
        [](unsigned s) { return make_random_op(s); },
        [](const model::Problem& p) {
            return solver::dp::ForwardDPSolver().solve(p, unlimited_cfg()).total_reward;
        });
}

TEST(DominanceFix, ForwardDP_StillOptimal_OPTW) {
    expect_optimal("ForwardDP OPTW (post-fix)",
        [](unsigned s) { return make_random_optw(s); },
        [](const model::Problem& p) {
            return solver::dp::ForwardDPSolver().solve(p, unlimited_cfg()).total_reward;
        });
}

// ============================================================================
// Phase 2: TrueBackwardDPSolver
// ============================================================================

TEST(TrueBackwardDP, MatchesBruteForce_OP) {
    expect_optimal("TrueBackwardDP OP",
        [](unsigned s) { return make_random_op(s); },
        [](const model::Problem& p) {
            return solver::dp::TrueBackwardDPSolver().solve(p, unlimited_cfg()).total_reward;
        });
}

TEST(TrueBackwardDP, MatchesBruteForce_OPTW) {
    expect_optimal("TrueBackwardDP OPTW",
        [](unsigned s) { return make_random_optw(s); },
        [](const model::Problem& p) {
            return solver::dp::TrueBackwardDPSolver().solve(p, unlimited_cfg()).total_reward;
        });
}

TEST(TrueBackwardDP, RouteIsValid) {
    auto prob = make_random_optw(42);
    solver::dp::TrueBackwardDPSolver bw;
    auto sol = bw.solve(prob, unlimited_cfg());
    const auto& route = sol.get_route(0);
    ASSERT_GE(route.size(), 2u);
    EXPECT_EQ(route.front(), prob.get_source_depot());
    EXPECT_EQ(route.back(),  prob.get_sink_depot());
    // No duplicates
    std::vector<NodeId> sorted(route.begin(), route.end());
    std::sort(sorted.begin(), sorted.end());
    EXPECT_EQ(std::unique(sorted.begin(), sorted.end()), sorted.end());
}

// ============================================================================
// Phase 3: TrueBidirectionalDPSolver
// ============================================================================

TEST(TrueBidirectionalDP, MatchesBruteForce_OP) {
    expect_optimal("TrueBidirectionalDP OP",
        [](unsigned s) { return make_random_op(s); },
        [](const model::Problem& p) {
            return solver::dp::TrueBidirectionalDPSolver().solve(p, unlimited_cfg()).total_reward;
        });
}

TEST(TrueBidirectionalDP, MatchesBruteForce_OPTW) {
    expect_optimal("TrueBidirectionalDP OPTW",
        [](unsigned s) { return make_random_optw(s); },
        [](const model::Problem& p) {
            return solver::dp::TrueBidirectionalDPSolver().solve(p, unlimited_cfg()).total_reward;
        });
}

TEST(TrueBidirectionalDP, AtLeastAsGoodAsForward) {
    for (unsigned s = 1; s <= 25; ++s) {
        auto prob = make_random_op(s);
        Reward fw  = solver::dp::ForwardDPSolver().solve(prob, unlimited_cfg()).total_reward;
        Reward bfw = solver::dp::TrueBidirectionalDPSolver().solve(prob, unlimited_cfg()).total_reward;
        EXPECT_NEAR(bfw, fw, 1e-6) << "TrueBidir should match ForwardDP seed=" << s;
    }
}

// ============================================================================
// Phase 4: DSSRSolver
// ============================================================================

namespace {
solver::dp::DSSRSolverConfig dssr_cfg(solver::dp::DSSRExpansionStrategy s) {
    solver::dp::DSSRSolverConfig c;
    c.max_labels = 0;
    c.verbose = false;
    c.strategy = s;
    c.use_ms_init = false;
    return c;
}
}  // namespace

TEST(DSSR, MO_ALL_MatchesBruteForce_OP) {
    expect_optimal("DSSR MO_ALL OP",
        [](unsigned s) { return make_random_op(s); },
        [](const model::Problem& p) {
            return solver::dp::DSSRSolver().solve(
                p, dssr_cfg(solver::dp::DSSRExpansionStrategy::MO_ALL)).total_reward;
        });
}

TEST(DSSR, HMO_MatchesBruteForce_OP) {
    expect_optimal("DSSR HMO OP",
        [](unsigned s) { return make_random_op(s); },
        [](const model::Problem& p) {
            return solver::dp::DSSRSolver().solve(
                p, dssr_cfg(solver::dp::DSSRExpansionStrategy::HMO)).total_reward;
        });
}

TEST(DSSR, HMO_ALL_MatchesBruteForce_OP) {
    expect_optimal("DSSR HMO_ALL OP",
        [](unsigned s) { return make_random_op(s); },
        [](const model::Problem& p) {
            return solver::dp::DSSRSolver().solve(
                p, dssr_cfg(solver::dp::DSSRExpansionStrategy::HMO_ALL)).total_reward;
        });
}

TEST(DSSR, MO_ALL_MatchesBruteForce_OPTW) {
    expect_optimal("DSSR MO_ALL OPTW",
        [](unsigned s) { return make_random_optw(s); },
        [](const model::Problem& p) {
            return solver::dp::DSSRSolver().solve(
                p, dssr_cfg(solver::dp::DSSRExpansionStrategy::MO_ALL)).total_reward;
        });
}

TEST(DSSR, OutputIsElementary) {
    for (unsigned s = 1; s <= 25; ++s) {
        auto prob = make_random_optw(s);
        auto sol = solver::dp::DSSRSolver().solve(
            prob, dssr_cfg(solver::dp::DSSRExpansionStrategy::MO_ALL));
        const auto& route = sol.get_route(0);
        std::vector<NodeId> sorted(route.begin(), route.end());
        std::sort(sorted.begin(), sorted.end());
        EXPECT_EQ(std::unique(sorted.begin(), sorted.end()), sorted.end())
            << "DSSR route has repeated nodes, seed=" << s;
    }
}

TEST(DSSR, MSm_DoesNotHurtQuality) {
    // MS_m seeded init must not produce worse solutions than empty init
    for (unsigned s = 1; s <= 25; ++s) {
        auto prob = make_random_optw(s);
        auto cfg_no_init  = dssr_cfg(solver::dp::DSSRExpansionStrategy::MO_ALL);
        auto cfg_ms_init  = dssr_cfg(solver::dp::DSSRExpansionStrategy::MO_ALL);
        cfg_ms_init.use_ms_init = true;
        cfg_ms_init.ms_m = 3;

        Reward r_no_init = solver::dp::DSSRSolver().solve(prob, cfg_no_init).total_reward;
        Reward r_ms_init = solver::dp::DSSRSolver().solve(prob, cfg_ms_init).total_reward;
        EXPECT_NEAR(r_ms_init, r_no_init, 1e-6)
            << "MS_m init hurt quality at seed=" << s;
    }
}
