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
    return fs::path(__FILE__).parent_path().parent_path() / "data" / relative_path;
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
    
    // Verify distances are non-negative and symmetric
    for (NodeId i = 0; i < problem->get_num_nodes(); ++i) {
        for (NodeId j = 0; j < problem->get_num_nodes(); ++j) {
            Distance d_ij = problem->get_distance(i, j);
            Distance d_ji = problem->get_distance(j, i);
            EXPECT_GE(d_ij, 0) << "Distance from " << i << " to " << j << " is negative";
            EXPECT_EQ(d_ij, d_ji) << "Distance matrix is not symmetric";
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
    
    // TOPTW should have time windows
    EXPECT_TRUE(problem->has_time_windows());
    EXPECT_FALSE(problem->is_time_dependent());
    EXPECT_FALSE(problem->is_multi_vehicle());
}

TEST_F(TOPTWParserTest, CordeauTest) {
    std::string filepath = get_data_path("toptw/pr01.txt");
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_TRUE(problem->has_time_windows());
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
    auto problem = parser.read(filepath);
    
    ASSERT_NE(problem, nullptr);
    EXPECT_GT(problem->get_num_nodes(), 0);
    EXPECT_GE(problem->get_source_depot(), 0);
    EXPECT_GE(problem->get_sink_depot(), 0);
    
    // SingleSat is modeled as OPTW variant
    EXPECT_TRUE(problem->has_time_windows());
}
