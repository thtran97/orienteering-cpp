// ============================================================================
// test_dssr_perf_cordeau.cpp
//
// Verifies that DSSRSolver — the scalable exact algorithm of Righini & Salani
// (2006, Section 4) — finds the literature-verified optimum on a real-world
// Cordeau OPTW instance (pr01: 48 customers), not just on small synthetic
// instances cross-checked against brute force in test_dp_improvements.cpp.
// ============================================================================

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>

#include "io/toptw_parser.h"
#include "model/solution.h"
#include "solver/dynamic_programming/dp_solvers.h"

namespace fs = std::filesystem;

using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver::dp;

namespace {

static std::string get_perf_data_path(const std::string& relative_path) {
    return (fs::path(__FILE__).parent_path().parent_path() / "data" / relative_path).string();
}

}  // namespace

TEST(DSSRCordeauPerf, pr01_MatchesLiteratureOptimum) {
    TOPTWParser parser;
    auto problem = parser.read(get_perf_data_path("toptw/Cordeau/pr01.txt"));
    ASSERT_NE(problem, nullptr);

    DSSRSolverConfig cfg;
    cfg.seed         = 1;
    cfg.verbose      = false;
    cfg.max_cpu_time = 90.0;
    cfg.use_ms_init  = false;
    cfg.strategy     = DSSRExpansionStrategy::HMO;

    auto t0 = std::chrono::high_resolution_clock::now();
    Solution sol = DSSRSolver().solve(*problem, cfg);
    double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::high_resolution_clock::now() - t0).count();

    // Righini & Salani (2006), Table 3: pr01 optimum = 308 (21-vertex path).
    EXPECT_EQ(static_cast<int>(sol.total_reward), 308);

    if (ms >= 90000.0) {
        std::cout << "[ WARN  ] pr01 DSSR solving time " << static_cast<int>(ms)
                  << "ms approached the 90000ms budget. Consider building with "
                  << "-DCMAKE_BUILD_TYPE=Release for better performance.\n";
    }
}
