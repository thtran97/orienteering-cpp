#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>

#include "io/toptw_parser.h"
#include "model/solution.h"
#include "solver/pulse/pulse_solver.h"

namespace fs = std::filesystem;

using namespace oplib;
using namespace oplib::model;
using namespace oplib::io;
using namespace oplib::solver::pulse;

namespace {

static std::string get_perf_data_path(const std::string& relative_path) {
    return (fs::path(__FILE__).parent_path().parent_path() / "data" / relative_path).string();
}

static Solution run_pulse(Problem& problem, double max_cpu_seconds) {
    PulseSolverConfig cfg;
    cfg.max_labels   = 0;
    cfg.max_cpu_time = max_cpu_seconds;
    cfg.verbose      = false;
    cfg.bound_step   = 1000;
    return PulseSolver{}.solve(problem, cfg);
}

static void check_perf(Problem& problem, int expected_reward,
                       const char* instance, double max_cpu_ms) {
    auto t0 = std::chrono::high_resolution_clock::now();
    Solution sol = run_pulse(problem, 120.0);
    double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::high_resolution_clock::now() - t0).count();

    EXPECT_EQ(static_cast<int>(sol.total_reward), expected_reward);

    if (ms >= max_cpu_ms) {
        std::cout << "[ WARN  ] " << instance
                  << " solving time " << static_cast<int>(ms) << "ms exceeded "
                  << static_cast<int>(max_cpu_ms) << "ms limit. "
                  << "Consider building with -DCMAKE_BUILD_TYPE=Release "
                  << "for better performance.\n";
    }
}

}  // namespace

class PulseCordeauPerf : public ::testing::Test {
protected:
    TOPTWParser parser;
};

TEST_F(PulseCordeauPerf, pr01) {
    auto problem = parser.read(get_perf_data_path("toptw/pr01.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 308, "pr01", 500.0);
}

TEST_F(PulseCordeauPerf, pr02) {
    auto problem = parser.read(get_perf_data_path("toptw/pr02.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 404, "pr02", 2000.0);
}

TEST_F(PulseCordeauPerf, pr03) {
    auto problem = parser.read(get_perf_data_path("toptw/pr03.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 394, "pr03", 3000.0);
}

TEST_F(PulseCordeauPerf, pr04) {
    auto problem = parser.read(get_perf_data_path("toptw/pr04.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 489, "pr04", 5000.0);
}

TEST_F(PulseCordeauPerf, pr05) {
    auto problem = parser.read(get_perf_data_path("toptw/pr05.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 595, "pr05", 60000.0);
}

TEST_F(PulseCordeauPerf, pr07) {
    auto problem = parser.read(get_perf_data_path("toptw/pr07.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 298, "pr07", 500.0);
}

TEST_F(PulseCordeauPerf, pr08) {
    auto problem = parser.read(get_perf_data_path("toptw/pr08.txt"));
    ASSERT_NE(problem, nullptr);
    check_perf(*problem, 463, "pr08", 2000.0);
}
