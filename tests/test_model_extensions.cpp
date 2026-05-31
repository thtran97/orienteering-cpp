#include <gtest/gtest.h>
#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "model/variants/tdop.h"
#include "core/routing_utils.h"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;

class ModelExtensionTest : public ::testing::Test {};

// Helper class to access protected members
class TestOPProblem : public OPProblem {
public:
    using OPProblem::OPProblem;
    const std::vector<Node>& get_nodes() const { return nodes; }
    void public_finalize() { finalize(); preprocessing(); }
};

class TestOPTWProblem : public OPTWProblem {
public:
    using OPTWProblem::OPTWProblem;
    bool is_arc_allowed(NodeId i, NodeId j) const { return exists_arc(i, j); }
    void public_finalize() { finalize(); preprocessing(); }
};

TEST_F(ModelExtensionTest, ScalingTest) {
    TestOPProblem problem("test_scaling", 1000.0);
    problem.set_scaling(ScalingMode::SCALED_INTEGER, 10.0);

    // Node 0 at (0,0)
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    // Node 1 at (10, 10)
    problem.add_node(Node{1, 10.0, 10.0, 0.0, 0.0});

    problem.public_finalize();

    // Raw distance = sqrt(10^2 + 10^2) = sqrt(200) ~= 14.1421356
    // Scaled distance = floor(14.1421356 * 10) = floor(141.421356) = 141
    EXPECT_EQ(problem.get_distance(0, 1), 141.0);
    EXPECT_EQ(problem.get_distance(1, 0), 141.0);
}

TEST_F(ModelExtensionTest, PreprocessingTest) {
    // Budget 100
    TestOPTWProblem problem("test_preprocessing", 100.0);
    
    // Node 0: Depot (0,0), TW [0, 100], Service 0
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    // Node 1: (10,0), TW [0, 5], Service 5
    // Travel time 0->1 is 10. Arrival at 10. 10 > 5 => Invalid.
    problem.add_node(Node{1, 10.0, 0.0, 5.0, 10.0, {0.0, 5.0}});
    
    // Node 2: (20,0), TW [15, 60], Service 5
    // Travel time 0->2 is 20. Arrival at 20. 20 >= 15. Valid.
    // Depart 25. Return to 0 (dist 20). Arrive 45 <= 100. Valid.
    problem.add_node(Node{2, 20.0, 0.0, 5.0, 10.0, {15.0, 60.0}});
    
    // Duplicate depot as sink (Node 3)
    problem.add_node(Node{3, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});

    problem.public_finalize();

    EXPECT_FALSE(problem.is_arc_allowed(0, 1)); // Should be pruned due to TW
    EXPECT_TRUE(problem.is_arc_allowed(0, 2));  // Should exist
}

TEST_F(ModelExtensionTest, NeighborSortCheck) {
    TestOPProblem problem("test_neighbors_check", 100.0);
    // Node 0: Depot
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0});
    
    // Node 1: Dist 10, Reward 20. Ratio = 20 / (10 + 0 + 1e-6) ~= 2.0
    problem.add_node(Node{1, 10.0, 0.0, 20.0, 0.0});
    
    // Node 2: Dist 5, Reward 5. Ratio = 5 / (5 + 0 + 1e-6) ~= 1.0
    problem.add_node(Node{2, 5.0, 0.0, 5.0, 0.0});
    
    // Node 3: Dist 2, Reward 100. Ratio = 100 / (2 + 0 + 1e-6) ~= 50.0
    problem.add_node(Node{3, 2.0, 0.0, 100.0, 0.0});
    
    // Node 4: Sink
    problem.add_node(Node{4, 0.0, 0.0, 0.0, 0.0});

    problem.public_finalize();

    const auto& nodes = problem.get_nodes();
    const auto& neighbors = nodes[0].neighbors;
    
    ASSERT_GE(neighbors.size(), 3);
    EXPECT_EQ(neighbors[0].first, 3); // Highest ratio
    EXPECT_EQ(neighbors[1].first, 1); // Middle ratio
    EXPECT_EQ(neighbors[2].first, 2); // Lowest ratio
}

TEST_F(ModelExtensionTest, DISABLED_RoutingUtilsTest) {
    // 1. Test compute_travel_time_td
    std::vector<double> ca = {0.5};
    std::vector<double> cb = {10.0};
    
    // Time slot 0: a=0.5, b=10. departure=10 -> travel = ceil(10*0.5)+10 = 5+10 = 15
    Time travel = oplib::utils::RoutingUtils::compute_travel_time_td(10.0, 0, ca, cb);
    EXPECT_EQ(travel, 15.0);
    
    // 2. Test compute_departure_time_td
    // We need a Problem instance with TimeWindows
    TestOPProblem problem("test_td", 100.0);
    
    // Node 0: Depot, Open 0, Close 100, Service 0
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    // Node 1: Dest, Open 0, Close 100, Service 0
    problem.add_node(Node{1, 10.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    
    // Finalize to compute distances and init internal structures
    problem.public_finalize();
    
    // We don't finalize, so distances are 0 by default if uninitialized, 
    // but TDOP overrides get_travel_time.
    // However, compute_departure_time_td calls problem.get_service_time(i) and get_time_window(i).
    
    // arrival_time_at_j = 25.0
    // slot duration = 100.0 (one large slot)
    // equation: depart = (arrival - b) / (1 + a) = (25 - 10) / 1.5 = 15 / 1.5 = 10.0
    
    // We need to pass coeff_a and coeff_b to the static function directly.
    // The function signature is static Time compute_departure_time_td(Problem&, i, j, arrival, ca, cb, slot_dur)
    
    Time depart = oplib::utils::RoutingUtils::compute_departure_time_td(
        problem, 0, 1, 25.0, ca, cb, 100.0
    );
    
    EXPECT_NEAR(depart, 10.0, 1e-5);
}
