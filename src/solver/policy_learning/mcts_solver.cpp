#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include "solver/policy_learning/mcts_solver.h"
#include "core/random.h"

namespace oplib::solver::policy_learning {

// ---------------------------------------------------------------------------
// Tree helpers
// ---------------------------------------------------------------------------

void MCTSSolver::free_tree(MCTSNode* node)
{
    if (!node) return;
    for (auto& [action, child] : node->children)
        free_tree(child);
    delete node;
}

MCTSNode* MCTSSolver::select(MCTSNode* root) const
{
    MCTSNode* cur = root;
    while (!cur->children.empty()) {
        MCTSNode* best   = nullptr;
        double    best_s = -std::numeric_limits<double>::infinity();
        for (auto& [action, child] : cur->children) {
            double s = child->ucb_score();
            if (s > best_s) { best_s = s; best = child; }
        }
        cur = best;
    }
    return cur;
}

MCTSNode* MCTSSolver::expand(MCTSNode* node,
                               const model::Problem& problem,
                               oplib::utils::Random& rng) const
{
    const NodeId src  = problem.get_source_depot();
    const NodeId sink = problem.get_sink_depot();
    const int    nn   = static_cast<int>(problem.get_num_nodes());

    // Collect unvisited customers that are time-feasible from this node's state
    std::vector<NodeId> candidates;
    for (NodeId c = 0; c < nn; ++c) {
        if (c == src || c == sink) continue;
        if (node->visited[c]) continue;

        const auto& tw_c = problem.get_time_window(c);
        Time travel = problem.get_travel_time(node->state, c, node->time_consumed);
        Time arr    = node->time_consumed + travel;
        if (arr > tw_c.closing) continue;

        // Check we can still reach sink after visiting c
        Time dep_c  = std::max(arr, tw_c.opening) + problem.get_service_time(c);
        Time t_sink = problem.get_travel_time(c, sink, dep_c);
        if (dep_c + t_sink > problem.get_time_window(sink).closing) continue;

        // Budget check
        if (dep_c + t_sink - problem.get_time_window(src).opening > problem.get_budget()) continue;

        candidates.push_back(c);
    }

    if (candidates.empty()) return node; // leaf — cannot expand

    // Pick a random candidate (exploration)
    NodeId action = candidates[rng.next_int(0, static_cast<int>(candidates.size()) - 1)];

    // Create child node
    const auto& tw_a = problem.get_time_window(action);
    Time travel = problem.get_travel_time(node->state, action, node->time_consumed);
    Time arr    = node->time_consumed + travel;
    Time dep    = std::max(arr, tw_a.opening) + problem.get_service_time(action);

    std::vector<bool> child_vis = node->visited;
    child_vis[action] = true;

    auto* child = new MCTSNode(action, node, std::move(child_vis));
    child->reward_collected = node->reward_collected + problem.get_reward(action);
    child->time_consumed    = dep;

    node->add_child(action, child);
    return child;
}

double MCTSSolver::simulate(MCTSNode* node,
                             const model::Problem& problem,
                             local_search::BaseLSUtils& ls,
                             const local_search::LSConfig& ls_cfg) const
{
    // Build a partial solution that includes the path from root to this node,
    // then run repair to complete it.
    model::Solution                         sol;
    std::vector<bool>                       visited;
    std::vector<local_search::RouteContext> ctx;
    ls.init(sol, visited, ctx);

    // Mark all nodes visited along the path as visited in the partial solution
    // by inserting them at position 1 into vehicle 0's route
    MCTSNode* cur = node;
    std::vector<NodeId> path;
    while (cur->parent != nullptr) {
        path.push_back(cur->state);
        cur = cur->parent;
    }
    std::reverse(path.begin(), path.end());

    for (NodeId c : path) {
        if (visited[c]) continue;
        auto& route = sol.get_route(0);
        const int insert_pos = static_cast<int>(route.size()) - 1;
        double shift = ls.check_insertion(sol, ctx, 0, c, insert_pos);
        if (shift < local_search::BaseLSUtils::INF) {
            ls.apply_insertion_public(sol, ctx, visited, 0, c, insert_pos, shift);
        }
    }

    // Complete the solution with repair
    ls.repair(sol, visited, ctx, ls_cfg);
    return sol.total_reward;
}

void MCTSSolver::backpropagate(MCTSNode* node, double reward) const
{
    MCTSNode* cur = node;
    while (cur != nullptr) {
        ++cur->nb_simulations;
        cur->reward_estimated = std::max(cur->reward_estimated, reward);
        cur = cur->parent;
    }
}

model::Solution MCTSSolver::extract_best(MCTSNode* root,
                                          const model::Problem& problem,
                                          local_search::BaseLSUtils& ls,
                                          const local_search::LSConfig& ls_cfg) const
{
    // Follow the path of highest average rewards to build the greedy solution
    model::Solution                         sol;
    std::vector<bool>                       visited;
    std::vector<local_search::RouteContext> ctx;
    ls.init(sol, visited, ctx);

    MCTSNode* cur = root;
    while (!cur->children.empty()) {
        MCTSNode* best   = nullptr;
        double    best_v = -1.0;
        for (auto& [action, child] : cur->children) {
            constexpr double EPS = 1e-5;
            double avg = (child->reward_collected + child->reward_estimated)
                         / (child->nb_simulations + EPS);
            if (avg > best_v) { best_v = avg; best = child; }
        }
        if (!best) break;

        NodeId c = best->state;
        if (!visited[c] && c != problem.get_source_depot() && c != problem.get_sink_depot()) {
            auto& route = sol.get_route(0);
            int   pos   = static_cast<int>(route.size()) - 1;
            double shift = ls.check_insertion(sol, ctx, 0, c, pos);
            if (shift < local_search::BaseLSUtils::INF)
                ls.apply_insertion_public(sol, ctx, visited, 0, c, pos, shift);
        }
        cur = best;
    }

    ls.repair(sol, visited, ctx, ls_cfg);
    return sol;
}

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

model::Solution MCTSSolver::solve(const model::Problem& problem,
                                   const SolverConfig&   config)
{
    MCTSSolverConfig cfg;
    cfg.seed           = config.seed;
    cfg.max_cpu_time   = config.max_cpu_time;
    cfg.max_iterations = config.max_iterations;
    cfg.verbose        = config.verbose;
    return solve(problem, cfg);
}

model::Solution MCTSSolver::solve(const model::Problem&   problem,
                                   const MCTSSolverConfig& config)
{
    using Clock = std::chrono::high_resolution_clock;

    oplib::utils::Random      rng(static_cast<uint32_t>(config.seed));
    local_search::BaseLSUtils ls(problem, rng);
    local_search::LSConfig    ls_cfg;
    ls_cfg.alpha    = config.alpha;
    ls_cfg.rcl_size = config.rcl_size;

    const int nn = static_cast<int>(problem.get_num_nodes());

    // Build root node (at source depot, nothing visited except depots)
    std::vector<bool> root_vis(nn, false);
    root_vis[problem.get_source_depot()] = true;
    root_vis[problem.get_sink_depot()]   = true;

    auto* root = new MCTSNode(problem.get_source_depot(), nullptr, root_vis);
    root->nb_simulations = 1;

    // Initialise best with a valid empty solution (correct number of routes)
    // so callers can safely call get_route(v) even when MCTS finds nothing.
    model::Solution best;
    {
        std::vector<bool> init_vis;
        std::vector<local_search::RouteContext> init_ctx;
        ls.init(best, init_vis, init_ctx);
    }
    auto t_start = Clock::now();

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        double elapsed = std::chrono::duration<double>(Clock::now() - t_start).count();
        if (elapsed >= config.max_cpu_time) break;

        // 1. Selection
        MCTSNode* leaf = select(root);

        // 2. Expansion
        MCTSNode* child = expand(leaf, problem, rng);

        // 3. Simulation (rollout)
        double reward = simulate(child, problem, ls, ls_cfg);

        // Track best solution found during rollout
        if (reward > best.total_reward) {
            // Re-run repair from child state to reconstruct the full solution
            model::Solution                         sol;
            std::vector<bool>                       vis;
            std::vector<local_search::RouteContext> ctx;
            ls.init(sol, vis, ctx);

            MCTSNode* c2 = child;
            std::vector<NodeId> path;
            while (c2->parent != nullptr) { path.push_back(c2->state); c2 = c2->parent; }
            std::reverse(path.begin(), path.end());
            for (NodeId nc : path) {
                if (vis[nc]) continue;
                auto& route = sol.get_route(0);
                int   pos   = static_cast<int>(route.size()) - 1;
                double shift = ls.check_insertion(sol, ctx, 0, nc, pos);
                if (shift < local_search::BaseLSUtils::INF)
                    ls.apply_insertion_public(sol, ctx, vis, 0, nc, pos, shift);
            }
            ls.repair(sol, vis, ctx, ls_cfg);
            if (sol.total_reward > best.total_reward) {
                best = sol;
                if (config.verbose)
                    std::cout << "[MCTS] iter=" << iter << " reward=" << best.total_reward << '\n';
            }
        }

        // 4. Backpropagation
        backpropagate(child, reward);
    }

    free_tree(root);
    return best;
}

} // namespace oplib::solver::policy_learning
