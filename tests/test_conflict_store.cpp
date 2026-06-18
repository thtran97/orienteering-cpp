#include <gtest/gtest.h>

#include <vector>

#include "knowledge_base/conflict_store.h"

using namespace oplib::knowledge_base;

// Assign two clients, then check the third is blocked by the conflict.
TEST(ConflictStore, BasicConflictBlocks) {
    ConflictStore kb(5, 1);
    std::vector<int> scope{1, 2, 3};
    kb.add_conflict(scope); // {1,2,3} cannot all be in vehicle 0

    EXPECT_TRUE(kb.check_assign(1, 0));
    EXPECT_TRUE(kb.check_assign(2, 0));
    EXPECT_TRUE(kb.check_assign(3, 0));

    ASSERT_TRUE(kb.assign(1, 0));
    ASSERT_TRUE(kb.assign(2, 0));

    // Conflict is now active: 3 is blocked
    EXPECT_FALSE(kb.check_assign(3, 0));
    // Other clients not in the conflict remain free
    EXPECT_TRUE(kb.check_assign(4, 0));
}

// Unassigning restores feasibility.
TEST(ConflictStore, UnassignRestoresFeasibility) {
    ConflictStore kb(4, 1);
    std::vector<int> scope{1, 2, 3};
    kb.add_conflict(scope);

    ASSERT_TRUE(kb.assign(1, 0));
    ASSERT_TRUE(kb.assign(2, 0));
    EXPECT_FALSE(kb.check_assign(3, 0));

    kb.unassign(1);

    // One watcher freed → conflict deactivated → 3 is free again
    EXPECT_TRUE(kb.check_assign(3, 0));
}

// Conflicts on vehicle 0 must not affect vehicle 1.
TEST(ConflictStore, MultiVehicleIsolation) {
    ConflictStore kb(5, 3);
    std::vector<int> scope{1, 2, 3};
    kb.add_conflict(scope); // encoded once per vehicle

    // Saturate vehicle 0's conflict
    ASSERT_TRUE(kb.assign(1, 0));
    ASSERT_TRUE(kb.assign(2, 0));
    EXPECT_FALSE(kb.check_assign(3, 0)); // blocked in vehicle 0

    // Vehicle 1 is independent
    EXPECT_TRUE(kb.check_assign(1, 1));
    EXPECT_TRUE(kb.check_assign(2, 1));
    EXPECT_TRUE(kb.check_assign(3, 1));
}

// Size reflects the number of added conflicts (once per vehicle).
TEST(ConflictStore, SizeCountsConflicts) {
    ConflictStore kb(4, 2);
    EXPECT_EQ(kb.size(), 0);

    std::vector<int> s1{1, 2};
    kb.add_conflict(s1); // adds 2 (one per vehicle)
    EXPECT_EQ(kb.size(), 2);

    std::vector<int> s2{2, 3, 4};
    kb.add_conflict(s2); // adds 2 more
    EXPECT_EQ(kb.size(), 4);
}

// reset() clears all assignments and deactivates all conflicts.
TEST(ConflictStore, ResetRestoresAll) {
    ConflictStore kb(3, 1);
    std::vector<int> scope{1, 2};
    kb.add_conflict(scope);

    ASSERT_TRUE(kb.assign(1, 0));
    EXPECT_FALSE(kb.check_assign(2, 0));

    kb.reset();

    EXPECT_TRUE(kb.check_assign(1, 0));
    EXPECT_TRUE(kb.check_assign(2, 0));
}

// Binary conflict {1, 2}: assigning 1 blocks 2, assigning 2 blocks 1.
TEST(ConflictStore, BinaryConflictSymmetric) {
    {
        ConflictStore kb(2, 1);
        std::vector<int> scope{1, 2};
        kb.add_conflict(scope);
        ASSERT_TRUE(kb.assign(1, 0));
        EXPECT_FALSE(kb.check_assign(2, 0));
    }
    {
        ConflictStore kb(2, 1);
        std::vector<int> scope{1, 2};
        kb.add_conflict(scope);
        ASSERT_TRUE(kb.assign(2, 0));
        EXPECT_FALSE(kb.check_assign(1, 0));
    }
}

// Assign already-assigned variable is idempotent.
TEST(ConflictStore, DoubleAssignIdempotent) {
    ConflictStore kb(3, 1);
    std::vector<int> scope{1, 2, 3};
    kb.add_conflict(scope);

    ASSERT_TRUE(kb.assign(1, 0));
    ASSERT_TRUE(kb.assign(1, 0)); // second assign same client/vehicle
    ASSERT_TRUE(kb.assign(2, 0));
    EXPECT_FALSE(kb.check_assign(3, 0));
}
