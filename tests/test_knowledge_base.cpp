// ============================================================================
// test_knowledge_base.cpp
//
// Phase 3 (knowledge base / conflict explanation): verify the pseudo-Boolean
// SelectionConstraintManager — the dependency-free successor of kb_ls_cpp's KB
// and the engine a conflict explainer would feed. It had zero coverage before.
//
// Variables are 1-indexed. Constraint encodings:
//   conflict({S})     -> sum(S) <= |S|-1   (the full combination is a no-good)
//   disjunctive({S})  -> sum(S) <= 1        (at most one)
//   knapsack(S,c,r)   -> c·S    <= r        (capacity)
//   objective(r)      -> profit·x >= r      (reward lower bound)
// assign() applies unit propagation and returns false on infeasibility.
// ============================================================================

#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "knowledge_base/sel_manager.h"

using namespace oplib::knowledge_base;

namespace {

// Drain the propagation queue: +id = forced true, -id = forced false.
std::set<int> drain_propagations(SelectionConstraintManager& kb) {
    std::set<int> out;
    while (kb.has_new_propagation()) out.insert(kb.get_next_propagation());
    return out;
}

bool is_selected(SelectionConstraintManager& kb, int k) {
    return !kb.is_allowed(k, false);  // value fixed to 1
}
bool is_excluded(SelectionConstraintManager& kb, int k) {
    return !kb.is_allowed(k, true);   // value fixed to 0
}

}  // namespace

// A learned no-good {1,2} must exclude its complement once one side is chosen.
TEST(KnowledgeBase, ConflictConstraintForcesComplement) {
    SelectionConstraintManager kb(2);
    std::vector<int> scope{1, 2};
    kb.add_conflict_constraint(scope);          // x1 + x2 <= 1

    ASSERT_TRUE(kb.assign(1, true));
    EXPECT_TRUE(drain_propagations(kb).count(-2)) << "selecting 1 must force 2 false";
    EXPECT_TRUE(is_selected(kb, 1));
    EXPECT_TRUE(is_excluded(kb, 2));
}

// "At most one" excludes every other member of the scope.
TEST(KnowledgeBase, DisjunctiveAllowsAtMostOne) {
    SelectionConstraintManager kb(3);
    std::vector<int> scope{1, 2, 3};
    kb.add_disjunctive_constraint(scope);       // x1 + x2 + x3 <= 1

    ASSERT_TRUE(kb.assign(2, true));
    auto prop = drain_propagations(kb);
    EXPECT_TRUE(prop.count(-1));
    EXPECT_TRUE(prop.count(-3));
    EXPECT_TRUE(is_excluded(kb, 1));
    EXPECT_TRUE(is_excluded(kb, 3));
}

// A capacity that admits only one unit-of-2 item behaves like "at most one".
TEST(KnowledgeBase, KnapsackCapacityPropagates) {
    SelectionConstraintManager kb(3);
    std::vector<int> scope{1, 2, 3}, coeff{2, 2, 2};
    kb.add_knapsack_constraint(scope, coeff, 3);  // 2x1 + 2x2 + 2x3 <= 3

    ASSERT_TRUE(kb.assign(1, true));
    auto prop = drain_propagations(kb);
    EXPECT_TRUE(prop.count(-2));
    EXPECT_TRUE(prop.count(-3));
}

// The objective lower bound detects infeasibility: dropping any item makes the
// remaining 20 fall short of the required 25.
TEST(KnowledgeBase, ObjectiveLowerBoundDetectsInfeasibility) {
    SelectionConstraintManager kb(3);
    std::vector<int> profits{10, 10, 10};
    kb.set_profit(profits);
    kb.add_objective_constraint(25);            // profit·x >= 25

    EXPECT_FALSE(kb.assign(1, false));
}

// Backtracking: unassign restores variables to a free state and the symmetric
// decision then works.
TEST(KnowledgeBase, UnassignRestoresFreedom) {
    SelectionConstraintManager kb(2);
    std::vector<int> scope{1, 2};
    kb.add_conflict_constraint(scope);

    ASSERT_TRUE(kb.assign(1, true));
    EXPECT_TRUE(is_excluded(kb, 2));

    std::vector<int> both{1, 2};
    kb.unassign(both);
    EXPECT_TRUE(kb.is_allowed(1, true));
    EXPECT_TRUE(kb.is_allowed(2, true));

    ASSERT_TRUE(kb.assign(2, true));
    EXPECT_TRUE(drain_propagations(kb).count(-1));
}

// Following the KB's own decision heuristic to a complete, constraint-respecting
// selection (the construction loop a KB-guided solver would run).
TEST(KnowledgeBase, GuidedSelectionRespectsConstraints) {
    SelectionConstraintManager kb(3);
    std::vector<int> profits{5, 10, 8};
    kb.set_profit(profits);
    std::vector<int> exclusive{1, 2};
    kb.add_disjunctive_constraint(exclusive);   // can't take both 1 and 2

    int guard = 0;
    while (kb.has_next_decision() && guard++ < 100) {
        int k = kb.get_next_decision(1.0);      // greedy: highest profit first
        if (k == 0) break;
        ASSERT_TRUE(kb.assign(k, true));
        drain_propagations(kb);
    }

    // Disjunction respected, and the greedy heuristic kept the better of {1,2}.
    EXPECT_FALSE(is_selected(kb, 1) && is_selected(kb, 2));
    EXPECT_TRUE(is_selected(kb, 2));
    EXPECT_TRUE(is_selected(kb, 3));
    EXPECT_TRUE(is_excluded(kb, 1));
}

// Sanity: get_size reflects the number of registered constraints.
TEST(KnowledgeBase, TracksConstraintCount) {
    SelectionConstraintManager kb(3);
    EXPECT_EQ(kb.get_size(), 0);
    std::vector<int> s1{1, 2};
    kb.add_conflict_constraint(s1);
    std::vector<int> s2{2, 3};
    kb.add_disjunctive_constraint(s2);
    EXPECT_EQ(kb.get_size(), 2);
}
