#include <iostream>
#include "io/toptw_parser.h"

using namespace oplib;

int main() {
    io::TOPTWParser parser;
    auto problem = parser.read("data/toptw/pr01.txt");
    
    if (!problem) {
        std::cout << "Failed to parse\n";
        return 1;
    }
    
    std::cout << "Problem: " << problem->get_name() << "\n";
    std::cout << "Num nodes: " << problem->get_num_nodes() << "\n";
    std::cout << "Num vehicles: " << problem->get_num_vehicles() << "\n";
    std::cout << "Budget: " << problem->get_budget() << "\n";
    std::cout << "\nFirst 10 node rewards:\n";
    
    for (int i = 0; i < std::min(10, problem->get_num_nodes()); ++i) {
        double reward = problem->get_reward(i);
        std::cout << "Node " << i << ": reward=" << reward << "\n";
    }
    
    return 0;
}
