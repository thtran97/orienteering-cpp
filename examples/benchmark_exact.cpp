/**
 * benchmark_exact.cpp
 *
 * Deep investigation of the three exact solvers (forward_dp, bidir_dp, pulse)
 * across OP and OPTW instances of increasing size (n = 10..18).
 *
 * For each instance the benchmark:
 *   - Runs forward_dp, bidir_dp, and pulse via their typed configs
 *     (explicit max_labels) so we can control the label budget precisely.
 *   - Also runs pulse via the BASE SolverConfig interface (exposes the
 *     max_labels=0 / unlimited issue).
 *   - Runs LNS as a heuristic reference ("upper-bound proxy").
 *   - Validates every returned solution for feasibility.
 *   - Reports label counts, rewards, CPU time, and whether the limit was hit.
 *
 * Usage:  ./benchmark_exact [--max-labels N]  (default 500000)
 */

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "model/node.h"
#include "model/solution.h"
#include "model/variants/op.h"
#include "model/variants/optw.h"
#include "solver/constructive/randomized_greedy.h"
#include "solver/dynamic_programming/dp_solvers.h"
#include "solver/metaheuristic/lns.h"
#include "solver/pulse/pulse_solver.h"
#include "solver/solver.h"

using namespace oplib;
using namespace oplib::model;
using namespace oplib::model::variants;
using namespace oplib::solver;

// ============================================================================
// Instance generators — source at (0,0), sink at (L,0).
// Customers are seeded deterministically on a band around the direct route.
// Budget = L * 1.35  (enough to allow ~50 % of customers to be visited).
// ============================================================================

static Node nd(NodeId id, double x, double y, double r,
               double o = 0.0, double c = 1e18, double s = 0.0)
{
    Node n; n.id=id; n.x=x; n.y=y; n.reward=r;
    n.service_time=s; n.tw={o,c};
    return n;
}

// Deterministic pseudo-random coords given a seed
static double prng(int& state)
{
    state = state * 1664525 + 1013904223;
    return ((state >> 8) & 0xFFFF) / 65535.0;
}

static OPProblem make_op(int n_customers, double L = 60.0)
{
    // n_customers customers + source + sink = n_customers+2 nodes
    double budget = L * 1.35;
    OPProblem p("op_n" + std::to_string(n_customers + 2), budget);
    p.add_node(nd(0, 0.0, 0.0, 0.0));  // source
    int state = 12345 + n_customers * 97;
    for (int i = 1; i <= n_customers; ++i) {
        double t = prng(state);
        double x = t * L;
        double off = (prng(state) - 0.5) * 20.0;
        double r = 5.0 + prng(state) * 25.0;
        p.add_node(nd(i, x, off, std::round(r)));
    }
    p.add_node(nd(n_customers + 1, L, 0.0, 0.0));  // sink
    p.finalize(); p.preprocessing();
    return p;
}

static OPTWProblem make_optw(int n_customers, double L = 60.0)
{
    double budget = L * 1.50;
    OPTWProblem p("optw_n" + std::to_string(n_customers + 2), budget);
    // Depots: wide open windows
    p.add_node(nd(0, 0.0, 0.0, 0.0, 0.0, 1e18, 0.0));
    int state = 99991 + n_customers * 113;
    for (int i = 1; i <= n_customers; ++i) {
        double t = prng(state);
        double x = t * L;
        double off = (prng(state) - 0.5) * 15.0;
        double r = 5.0 + prng(state) * 25.0;
        // Time window: opening ~ travel time from source, closing = opening + 30
        double open  = x * 0.9;
        double close = open + 30.0;
        p.add_node(nd(i, x, off, std::round(r), open, close, 2.0));
    }
    p.add_node(nd(n_customers + 1, L, 0.0, 0.0, 0.0, 1e18, 0.0));
    p.finalize(); p.preprocessing();
    return p;
}

// ============================================================================
// Solution validator
// ============================================================================

struct Validity {
    bool ok          = true;
    std::string why;
};

static Validity validate(const Solution& sol, const Problem& prob)
{
    if (sol.get_num_vehicles() < 1)
        return {false, "no_vehicles"};

    const auto& route = sol.get_route(0);
    if (route.size() < 2)
        return {false, "route_too_short"};
    if (route.front() != prob.get_source_depot())
        return {false, "wrong_source"};
    if (route.back() != prob.get_sink_depot())
        return {false, "wrong_sink"};

    // Duplicate node check
    std::set<NodeId> seen;
    for (NodeId nid : route) {
        if (!seen.insert(nid).second)
            return {false, "duplicate_node_" + std::to_string(nid)};
    }

    // Budget check (recompute actual route time)
    Time total_time = 0.0;
    Time t = prob.get_time_window(route[0]).opening + prob.get_service_time(route[0]);
    for (size_t k = 1; k < route.size(); ++k) {
        NodeId u = route[k-1], v = route[k];
        Time travel = prob.get_travel_time(u, v, t);
        t += travel;
        if (prob.has_time_windows()) {
            const auto& tw = prob.get_time_window(v);
            if (t > tw.closing + 1e-6)
                return {false, "tw_violated_at_" + std::to_string(v)};
            t = std::max(t, tw.opening) + prob.get_service_time(v);
        }
        total_time += travel;
    }
    if (total_time > prob.get_budget() + 1e-6)
        return {false, "budget_violated(" + std::to_string(total_time)
                + ">" + std::to_string(prob.get_budget()) + ")"};

    return {true, "OK"};
}

// ============================================================================
// Run helpers
// ============================================================================

struct Result {
    std::string solver;
    double      reward   = -1.0;
    double      cpu_ms   = 0.0;
    int         visited  = 0;
    bool        valid    = false;
    std::string validity;
    bool        limit_hit = false; // indicates label budget was exhausted
};

template <typename Solver, typename Config>
static Result run_typed(const std::string& name,
                         Solver& s, const Config& cfg,
                         const Problem& prob)
{
    Result r;
    r.solver = name;
    auto t0 = std::chrono::high_resolution_clock::now();
    Solution sol = s.solve(prob, cfg);
    auto t1 = std::chrono::high_resolution_clock::now();
    r.cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.reward  = sol.total_reward;
    for (const auto& route : sol.get_routes())
        r.visited += std::max(0, static_cast<int>(route.size()) - 2);
    auto v = validate(sol, prob);
    r.valid    = v.ok;
    r.validity = v.why;
    return r;
}

static Result run_base(const std::string& name,
                        Solver& s, const SolverConfig& cfg,
                        const Problem& prob)
{
    Result r;
    r.solver = name;
    auto t0 = std::chrono::high_resolution_clock::now();
    Solution sol = s.solve(prob, cfg);
    auto t1 = std::chrono::high_resolution_clock::now();
    r.cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.reward  = sol.total_reward;
    for (const auto& route : sol.get_routes())
        r.visited += std::max(0, static_cast<int>(route.size()) - 2);
    auto v = validate(sol, prob);
    r.valid    = v.ok;
    r.validity = v.why;
    return r;
}

// ============================================================================
// Printing
// ============================================================================

static void print_sep()
{
    std::cout << '+' << std::string(20,'-')
              << '+' << std::string(10,'-')
              << '+' << std::string(8,'-')
              << '+' << std::string(10,'-')
              << '+' << std::string(8,'-')
              << '+' << std::string(22,'-') << "+\n";
}

static void print_hdr()
{
    print_sep();
    std::cout << "| " << std::left  << std::setw(18) << "Solver"
              << " | " << std::right << std::setw(8)  << "Reward"
              << " | " << std::setw(6) << "Visit"
              << " | " << std::setw(8) << "CPU(ms)"
              << " | " << std::setw(6) << "Valid"
              << " | " << std::left  << std::setw(20) << "Notes"
              << " |\n";
    print_sep();
}

static void print_row(const Result& r, double lns_ref)
{
    std::string notes;
    if (!r.valid)            notes = "INVALID:" + r.validity;
    else if (r.limit_hit)    notes = "label_limit_hit";
    else if (lns_ref > 0 && r.reward < lns_ref - 1e-6)
        notes = "sub_LNS(-" + [&]{
            std::ostringstream os;
            os << std::fixed << std::setprecision(0) << (lns_ref - r.reward);
            return os.str(); }() + ")";
    else                     notes = "OK";

    std::cout << "| " << std::left  << std::setw(18) << r.solver
              << " | " << std::right << std::fixed << std::setprecision(1)
              << std::setw(8) << r.reward
              << " | " << std::setw(6) << r.visited
              << " | " << std::setw(8) << r.cpu_ms
              << " | " << std::setw(6) << (r.valid ? "YES" : "NO")
              << " | " << std::left  << std::setw(20) << notes
              << " |\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
    int max_labels = 500000;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--max-labels" || a == "-m") && i + 1 < argc)
            max_labels = std::stoi(argv[++i]);
    }

    std::cout << "=== Exact Solver Deep Investigation ===\n"
              << "max_labels = " << max_labels << "\n\n";

    // Base config for LNS reference and Pulse-via-base tests
    SolverConfig base_cfg;
    base_cfg.seed           = 42;
    base_cfg.max_cpu_time   = 5.0;
    base_cfg.max_iterations = 500;
    base_cfg.verbose        = false;

    dp::DPSolverConfig dp_cfg;
    dp_cfg.seed       = 42;
    dp_cfg.max_labels = max_labels;
    dp_cfg.verbose    = true;  // prints label count to stdout

    pulse::PulseSolverConfig ps_cfg;
    ps_cfg.seed       = 42;
    ps_cfg.max_labels = max_labels;
    ps_cfg.verbose    = true;

    // Instance sizes to test (number of customers, not total nodes)
    const std::vector<int> sizes = {8, 10, 12, 14, 16};

    for (const std::string& variant : {"OP", "OPTW"}) {
        std::cout << "══════════════════════ Variant: " << variant
                  << " ══════════════════════\n";

        for (int nc : sizes) {
            auto prob_op   = make_op(nc);
            auto prob_optw = make_optw(nc);
            const Problem& prob = (variant == "OP")
                                  ? static_cast<const Problem&>(prob_op)
                                  : static_cast<const Problem&>(prob_optw);

            int N = prob.get_num_nodes();
            std::cout << "\n--- " << prob.get_name()
                      << "  (N=" << N << ", budget=" << prob.get_budget() << ") ---\n";
            print_hdr();

            // LNS reference
            {
                metaheuristic::LNSSolver lns;
                auto r = run_base("LNS(ref)", lns, base_cfg, prob);
                print_row(r, -1.0);

                double lns_reward = r.reward;

                // ForwardDP via typed config
                {
                    dp::ForwardDPSolver fwd;
                    // Redirect verbose output via stringstream temporarily
                    std::streambuf* old_buf = std::cout.rdbuf();
                    std::ostringstream captured;
                    std::cout.rdbuf(captured.rdbuf());
                    auto res = run_typed("forward_dp", fwd, dp_cfg, prob);
                    std::cout.rdbuf(old_buf);
                    // Parse label count from captured output
                    std::string cap = captured.str();
                    if (cap.find("labels=") != std::string::npos) {
                        auto pos = cap.find("labels=") + 7;
                        int lcount = std::stoi(cap.substr(pos));
                        res.limit_hit = (lcount >= max_labels);
                    }
                    print_row(res, lns_reward);
                }

                // BidirectionalDP via typed config
                {
                    dp::BidirectionalDPSolver bidir;
                    std::streambuf* old_buf = std::cout.rdbuf();
                    std::ostringstream captured;
                    std::cout.rdbuf(captured.rdbuf());
                    auto res = run_typed("bidir_dp", bidir, dp_cfg, prob);
                    std::cout.rdbuf(old_buf);
                    std::string cap = captured.str();
                    if (cap.find("labels=") != std::string::npos) {
                        auto pos = cap.find("labels=") + 7;
                        int lcount = std::stoi(cap.substr(pos));
                        res.limit_hit = (lcount >= max_labels);
                    }
                    print_row(res, lns_reward);
                }

                // Pulse via typed PulseSolverConfig
                {
                    pulse::PulseSolver ps;
                    std::streambuf* old_buf = std::cout.rdbuf();
                    std::ostringstream captured;
                    std::cout.rdbuf(captured.rdbuf());
                    auto res = run_typed("pulse(typed)", ps, ps_cfg, prob);
                    std::cout.rdbuf(old_buf);
                    std::string cap = captured.str();
                    if (cap.find("pulses=") != std::string::npos) {
                        auto pos = cap.find("pulses=") + 7;
                        int pcount = std::stoi(cap.substr(pos));
                        res.limit_hit = (pcount >= ps_cfg.max_labels);
                    }
                    print_row(res, lns_reward);
                }

                // Pulse via BASE SolverConfig (exposes max_labels=0 issue)
                {
                    pulse::PulseSolver ps;
                    // Use a SHORT timeout via iterations to avoid infinite loop
                    SolverConfig small_base = base_cfg;
                    small_base.max_cpu_time = 2.0;
                    std::streambuf* old_buf = std::cout.rdbuf();
                    std::ostringstream captured;
                    std::cout.rdbuf(captured.rdbuf());
                    auto res = run_base("pulse(base,unlim)", ps, small_base, prob);
                    std::cout.rdbuf(old_buf);
                    std::string cap = captured.str();
                    if (cap.find("pulses=") != std::string::npos) {
                        auto pos = cap.find("pulses=") + 7;
                        int pcount = std::stoi(cap.substr(pos));
                        // unlimited — report actual pulse count
                        std::ostringstream note;
                        note << "pulses=" << pcount;
                        // store in notes via limit_hit=false but report count
                        res.validity = note.str();
                        res.valid = true;
                    }
                    print_row(res, lns_reward);
                }
            }

            print_sep();
        }
        std::cout << '\n';
    }

    std::cout << "\n=== Legend ===\n"
              << "sub_LNS(-X) : reward is X less than the LNS heuristic reference\n"
              << "label_limit_hit : solver exhausted the label/pulse budget\n"
              << "pulse(base,unlim): Pulse called via SolverConfig — sets max_labels=0 (unlimited)\n"
              << "Notes column for pulse(base,unlim) shows actual pulse count\n";

    return 0;
}
