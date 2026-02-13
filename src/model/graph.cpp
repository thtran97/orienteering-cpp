#include <iostream>
#include <sstream>
#include <algorithm>

#include "model/graph.h"

namespace oplib::model {

std::string OPArc::to_string() const {
    std::stringstream ss;
    ss << "Arc(" << source_id << "->" << dest_id << ") dist=" << dist << " detours={";
    for (const auto& d : detours) {
        ss << "(" << d.first << "," << d.second << ") ";
    }
    ss << "}";
    return ss.str();
}

std::string OPNode::to_string() const {
    std::stringstream ss;
    ss << "Node(" << id << ") r=" << reward << " tw=[" << opening << "," << closing << "] service=" << service_time;
    return ss.str();
}

void OPGraph::build_graph(const Problem& problem, int log_level) {
    int n = static_cast<int>(problem.get_num_nodes());
    source_id = problem.get_source_depot();
    sink_id = problem.get_sink_depot();
    
    const auto& source_tw = problem.get_time_window(source_id);
    tmax = source_tw.closing;

    nodes.clear();
    arc_pool.clear();
    adjacency_matrix.assign(n, std::vector<bool>(n, false));

    // Create nodes
    for (int i = 0; i < n; ++i) {
        const auto& tw = problem.get_time_window(i);
        nodes.emplace_back(i, tw.opening, tw.closing, problem.get_service_time(i), problem.get_reward(i));
    }

    // Create arcs
    for (int i = 0; i < n; ++i) {
        if (i == sink_id) continue;

        for (int j = 0; j < n; ++j) {
            if (i == j || j == source_id) continue;

            // Feasibility check
            Time earliest_arrival_j = nodes[i].opening + nodes[i].service_time + problem.get_distance(i, j);
            if (earliest_arrival_j > nodes[j].closing) continue;

            Time earliest_return = std::max(earliest_arrival_j, nodes[j].opening) + nodes[j].service_time + problem.get_distance(j, sink_id);
            if (earliest_return > tmax) continue;

            // Arcs are feasible, create OPArc
            auto arc = std::make_unique<OPArc>(i, j, problem.get_distance(i, j));
            bool is_pruned = false;

            // Preprocessing by detour pruning (Legacy logic)
            for (int k = 0; k < n; ++k) {
                if (k == i || k == j || k == source_id || k == sink_id) continue;

                // Check if latest starting at i can detour at k
                Time arrival_k = nodes[i].closing + nodes[i].service_time + problem.get_distance(i, k);
                if (arrival_k <= nodes[k].closing) {
                    Time depart_k = std::max(arrival_k, nodes[k].opening);
                    Time arrival_j_from_k = depart_k + nodes[k].service_time + problem.get_distance(k, j);
                    
                    if (arrival_j_from_k <= nodes[j].opening) {
                        is_pruned = true;
                        break; // Detour pruning
                    }
                }

                // Compute latest detour time at k for arc (i, j)
                Time latest_kj = std::min(nodes[j].opening - problem.get_distance(k, j) - nodes[k].service_time,
                                          nodes[k].closing);
                if (latest_kj < nodes[k].opening) continue;

                Time latest_ik = std::min(latest_kj - problem.get_distance(i, k) - nodes[i].service_time,
                                          nodes[i].closing);
                if (latest_ik < nodes[i].opening) continue;

                arc->detours.push_back({k, latest_ik});
            }

            if (!is_pruned) {
                arc->sort_detours();
                OPArc* arc_ptr = arc.get();
                arc_pool.push_back(std::move(arc));
                nodes[i].add_out_arc(arc_ptr);
                nodes[j].add_in_arc(arc_ptr);
                adjacency_matrix[i][j] = true;

                if (log_level >= 2) {
                    std::cout << "[Graph] Created arc " << i << "->" << j << "\n";
                }
            }
        }
    }

    if (log_level >= 1) {
        print_graph();
    }
}

void OPGraph::print_graph() const {
    std::cout << "====== GRAPH INFO =====\n";
    std::cout << "[Graph] Nb nodes = " << nodes.size() << "\n";
    std::cout << "[Graph] Nb arcs = " << arc_pool.size() << "\n";
    std::cout << "[Graph] Tmax = " << tmax << "\n";
    std::cout << "========================\n";
}

bool OPGraph::arc_exists(NodeId i, NodeId j) const {
    if (i < 0 || i >= static_cast<NodeId>(adjacency_matrix.size())) return false;
    return adjacency_matrix[i][j];
}

bool OPGraph::arc_feasible(NodeId i, NodeId j, Time start_i) const {
    if (!adjacency_matrix[i][j]) return false;
    
    const OPArc* arc = get_arc(i, j);
    Time arrival_j = start_i + nodes[i].service_time + arc->dist;
    return arrival_j <= nodes[j].closing;
}

Time OPGraph::get_earliest_start(NodeId i, NodeId j, Time start_i) const {
    if (!adjacency_matrix[i][j]) return -1.0;
    
    const OPArc* arc = get_arc(i, j);
    Time arrival_j = start_i + nodes[i].service_time + arc->dist;
    if (arrival_j > nodes[j].closing) return -1.0;
    
    return std::max(arrival_j, nodes[j].opening);
}

const OPArc* OPGraph::get_arc(NodeId i, NodeId j) const {
    auto it = nodes[i].arc_to.find(j);
    if (it != nodes[i].arc_to.end()) return it->second;
    return nullptr;
}

} // namespace oplib::model
