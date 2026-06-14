#include "gtest/gtest.h"
#include <memory>
#include <filesystem>

#include "io/op_parser.h"
#include "io/top_parser.h"
#include "io/toptw_parser.h"
#include "io/tdop_parser.h"
#include "io/tdoptw_parser.h"
#include "io/ttdp_parser.h"
#include "io/mctopmtw_parser.h"
#include "io/singlesat_parser.h"

namespace fs = std::filesystem;
using namespace oplib;
using namespace oplib::io;
using namespace oplib::model;

// Helper function to construct data file path
std::string get_data_path(const std::string& relative_path) {
    return (fs::path(__FILE__).parent_path().parent_path() / "data" / relative_path).string();
}

// ============================================================================
// OPParserTest
// ============================================================================
class OPParserTest : public ::testing::Test {
protected:
    OPParser parser;
};

TEST_F(OPParserTest, ChaoSetTest) {
    std::string filepath = get_data_path("op/ChaoSet_64/set_64_1_15.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // Verify rewards are non-negative
    for (NodeId i = 1; i < problem->get_num_nodes(); ++i) {
        EXPECT_GE(problem->get_reward(i), 0);
    }
    
    // Verify travel times are non-negative and symmetric
    for (NodeId i = 0; i < problem->get_num_nodes(); ++i) {
        for (NodeId j = 0; j < problem->get_num_nodes(); ++j) {
            Time t_ij = problem->get_distance(i, j);
            Time t_ji = problem->get_distance(j, i);
            EXPECT_GE(t_ij, 0) << "Travel time from " << i << " to " << j << " is negative";
            EXPECT_EQ(t_ij, t_ji) << "Travel time matrix is not symmetric";
        }
    }
}

TEST_F(OPParserTest, TsiligiridesSetTest) {
    std::string filepath = get_data_path("op/Tsiligirides_1/tsiligirides_problem_1_budget_05.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    
    // Verify basic structure
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    EXPECT_FALSE(problem->has_time_windows());
    EXPECT_FALSE(problem->is_time_dependent());
    EXPECT_FALSE(problem->is_multi_vehicle());
}

// ============================================================================
// TOPParserTest
// ============================================================================
class TOPParserTest : public ::testing::Test {
protected:
    TOPParser parser;
};

TEST_F(TOPParserTest, Set1Test) {
    std::string filepath = get_data_path("top/Set1/p1.2.a.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // Verify that rewards exist for at least some nodes
    bool has_reward = false;
    for (NodeId i = 1; i < problem->get_num_nodes(); ++i) {
        if (problem->get_reward(i) > 0) {
            has_reward = true;
            break;
        }
    }
    EXPECT_TRUE(has_reward);
}

// ============================================================================
// TOPTWParserTest
// ============================================================================
class TOPTWParserTest : public ::testing::Test {
protected:
    TOPTWParser parser;
};

TEST_F(TOPTWParserTest, SolomonTest) {
    std::string filepath = get_data_path("toptw/c101.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // TOPTW should have time windows and be multi-vehicle (Team OP)
    EXPECT_TRUE(problem->has_time_windows());
    EXPECT_FALSE(problem->is_time_dependent());
    EXPECT_TRUE(problem->is_multi_vehicle());  // TOPTW is Team OP => multi-vehicle
    EXPECT_GT(problem->get_num_vehicles(), 1); // Should have multiple vehicles
}

TEST_F(TOPTWParserTest, CordeauTest) {
    std::string filepath = get_data_path("toptw/pr01.txt");
    auto problem = parser.read(filepath);

    ASSERT_NE(problem, nullptr);
    EXPECT_TRUE(problem->has_time_windows());

    // pr01: 48 customers + 1 depot + 1 virtual sink depot = 50 nodes total
    EXPECT_EQ(problem->get_num_nodes(), 50);

    // num_vehicles from header col 0 = 5
    EXPECT_EQ(problem->get_num_vehicles(), 5);

    // Budget = depot tw_close = 1000 (not 48, which is num_customers)
    EXPECT_DOUBLE_EQ(problem->get_budget(), 1000.0);

    // Depot (node 0): tw_open=0, tw_close=1000
    EXPECT_DOUBLE_EQ(problem->get_time_window(0).opening, 0.0);
    EXPECT_DOUBLE_EQ(problem->get_time_window(0).closing, 1000.0);

    // Node 1 (group 1–12, 1 combo code): tw=[354, 509], reward=12, service=2
    EXPECT_DOUBLE_EQ(problem->get_time_window(1).opening, 354.0);
    EXPECT_DOUBLE_EQ(problem->get_time_window(1).closing, 509.0);
    EXPECT_EQ(problem->get_reward(1), 12);

    // Node 13 (group 13–24, 2 combo codes): tw=[368, 528], reward=9, service=2
    // Bug: the old parser read 3 fixed dummies and got tw=[10, 368] here.
    EXPECT_DOUBLE_EQ(problem->get_time_window(13).opening, 368.0);
    EXPECT_DOUBLE_EQ(problem->get_time_window(13).closing, 528.0);
    EXPECT_EQ(problem->get_reward(13), 9);

    // Node 25 (group 25–48, 4 combo codes): tw=[360, 505], reward=14, service=4
    // Bug: the old parser got tw=[2, 4] here, making all nodes 25-48 infeasible.
    EXPECT_DOUBLE_EQ(problem->get_time_window(25).opening, 360.0);
    EXPECT_DOUBLE_EQ(problem->get_time_window(25).closing, 505.0);
    EXPECT_EQ(problem->get_reward(25), 14);
}

// ============================================================================
// TDOPParserTest
// ============================================================================
class TDOPParserTest : public ::testing::Test {
protected:
    // No setup needed for static parser
};

TEST_F(TDOPParserTest, Set1Test) {
    std::string instance_path = get_data_path("tdop/dataset1/OP_instances/p1.1.a.txt");
    std::string arc_cat_path = get_data_path("tdop/dataset1/arc_cat_1.txt");
    std::string speed_matrix_path = get_data_path("tdop/speedmatrix.txt");
    
    auto problem = TDOPParser::read(instance_path, speed_matrix_path, arc_cat_path);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // TDOP should be time-dependent
    EXPECT_TRUE(problem->is_time_dependent());
}

// ============================================================================
// TDOPTWParserTest
// ============================================================================
class TDOPTWParserTest : public ::testing::Test {
protected:
    // No setup needed for static parser
};

TEST_F(TDOPTWParserTest, tdoptwInstanceTest) {
    std::string instance_path = get_data_path("tdoptw/20.1.1.TXT");
    std::string transition_matrix_path = get_data_path("tdoptw/titt20.TXT");
    
    auto problem = TDOPTWParser::read(instance_path, transition_matrix_path);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // TDOPTW should have both time windows and time-dependency
    EXPECT_TRUE(problem->has_time_windows());
    EXPECT_TRUE(problem->is_time_dependent());
}

// ============================================================================
// TTDPParserTest
// ============================================================================
class TTDPParserTest : public ::testing::Test {
protected:
    TTDPParser parser;
};

TEST_F(TTDPParserTest, ttdpInstanceTest) {
    std::string filepath = get_data_path("ttdp/t101.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // TTDP is multi-day TOPTW variant
    EXPECT_TRUE(problem->has_time_windows());
}

// ============================================================================
// MCTOPMTWParserTest
// ============================================================================
class MCTOPMTWParserTest : public ::testing::Test {
protected:
    MCTOPMTWParser parser;
};

TEST_F(MCTOPMTWParserTest, CordeauSetTest) {
    std::string filepath = get_data_path("mctopmtw/Cordeau/MCTOPMTW-1-pr01.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // MCTOPMTW variant 1 has 1 vehicle, so not multi-vehicle
    EXPECT_FALSE(problem->is_multi_vehicle());
    EXPECT_TRUE(problem->has_time_windows());
}

TEST_F(MCTOPMTWParserTest, SolomonSetTest) {
    std::string filepath = get_data_path("mctopmtw/Solomon/MCTOPMTW-1-c101.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_FALSE(problem->is_multi_vehicle());
    EXPECT_TRUE(problem->has_time_windows());
}

// ============================================================================
// SingleSatParserTest
// ============================================================================
class SingleSatParserTest : public ::testing::Test {
protected:
    SingleSatParser parser;
};

TEST_F(SingleSatParserTest, s500InstanceTest) {
    std::string filepath = get_data_path("singlesat/s500-01");
    // SingleSat instances (~1 MB each) are intentionally excluded from the
    // committed data set to keep the repository small. Skip when absent so the
    // suite stays green; drop the file at the path above to enable this test.
    if (!fs::exists(filepath)) {
        GTEST_SKIP() << "singlesat data not bundled (excluded by size): " << filepath;
    }
    auto problem = parser.read(filepath);

    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // SingleSat is modeled as OPTW variant
    EXPECT_TRUE(problem->has_time_windows());
}
