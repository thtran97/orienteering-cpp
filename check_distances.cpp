#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>

int main() {
    std::ifstream file("data/toptw/c101.txt");
    std::string line;
    
    int num_veh;
    double budget;
    std::getline(file, line);
    std::istringstream ss1(line);
    double dummy1, dummy3;
    ss1 >> num_veh >> dummy1 >> budget >> dummy3;
    
    std::getline(file, line); // Skip line 2
    
    struct Point { double x, y; };
    std::vector<Point> nodes;
    
    int count = 0;
    while (std::getline(file, line) && count < 5) {
        std::istringstream ss(line);
        double id, x, y;
        double rest[7];
        if (ss >> id >> x >> y && ss >> rest[0] >> rest[1] >> rest[2] >> rest[3] >> rest[4] >> rest[5] >> rest[6]) {
            nodes.push_back({x, y});
        }
        count++;
    }
    
    std::cout << "Budget from header: " << budget << "\n";
    std::cout << "Nodes parsed: " << nodes.size() << "\n\n";
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        std::cout << "Node " << i << ": (" << nodes[i].x << ", " << nodes[i].y << ")\n";
    }
    
    std::cout << "\nDistances (Euclidean, unscaled):\n";
    for (size_t i = 0; i < std::min(nodes.size(), size_t(3)); ++i) {
        for (size_t j = i+1; j < std::min(nodes.size(), size_t(3)); ++j) {
            double dx = nodes[i].x - nodes[j].x;
            double dy = nodes[i].y - nodes[j].y;
            double dist = std::sqrt(dx*dx + dy*dy);
            std::cout << "  " << i << "->" << j << ": " << dist << " (scaled x10: " << dist*10 << ")\n";
        }
    }
    
    return 0;
}
