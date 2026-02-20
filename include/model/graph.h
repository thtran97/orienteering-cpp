#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <algorithm>

#include "core/types.h"
#include "model/problem.h"

namespace oplib::model {

struct OPArc {
    NodeId source_id;
    NodeId dest_id;
    Time travel_time;  // travel time from source to dest
    
    // (detour_node, latest_departure_from_source)
    std::vector<std::pair<NodeId, Time>> detours;

    OPArc(NodeId s, NodeId d, Time travel_time) : source_id(s), dest_id(d), travel_time(travel_time) {}
    
    void sort_detours() {
        std::sort(detours.begin(), detours.end(), [](const auto& a, const auto& b) {
            return a.second > b.second; // Default legacy sort by latest_date desc
        });
    }

    std::string to_string() const;
};

struct OPNode {
    NodeId id;
    Time opening;
    Time closing;
    Time service_time;
    Reward reward;

    std::vector<OPArc*> outgoing_arcs;
    std::vector<OPArc*> incoming_arcs;
    
    std::unordered_map<NodeId, OPArc*> arc_to;
    std::unordered_map<NodeId, OPArc*> arc_from;

    OPNode(NodeId id, Time opening, Time closing, Time service_time, Reward reward)
        : id(id), opening(opening), closing(closing), service_time(service_time), reward(reward) {}

    void add_out_arc(OPArc* arc) {
        outgoing_arcs.push_back(arc);
        arc_to[arc->dest_id] = arc;
    }

    void add_in_arc(OPArc* arc) {
        incoming_arcs.push_back(arc);
        arc_from[arc->source_id] = arc;
    }

    std::string to_string() const;
};

/**
 * @brief Graph representation of an Orienteering Problem.
 * Built from an abstract Problem instance.
 */
class OPGraph {
public:
    OPGraph() = default;
    ~OPGraph() = default;

    void build_graph(const Problem& problem, int log_level = 0);
    
    void print_graph() const;
    
    bool arc_exists(NodeId i, NodeId j) const;
    bool arc_feasible(NodeId i, NodeId j, Time start_i) const;
    
    Time get_earliest_start(NodeId i, NodeId j, Time start_i) const;
    Time get_required_time(NodeId i, NodeId j, Time start_j) const;
    
    Reward get_reward(NodeId i) const { return nodes[i].reward; }
    int get_num_nodes() const { return static_cast<int>(nodes.size()); }
    Time get_tmax() const { return tmax; }
    
    const OPNode& get_node(NodeId i) const { return nodes[i]; }
    const OPArc* get_arc(NodeId i, NodeId j) const;

private:
    NodeId source_id = -1;
    NodeId sink_id = -1;
    Time tmax = 0.0;

    std::vector<OPNode> nodes;
    std::vector<std::unique_ptr<OPArc>> arc_pool;
    std::vector<std::vector<bool>> adjacency_matrix;
};

} // namespace oplib::model
