#include "../../include/io/ttdp_parser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

namespace oplib::io {

std::unique_ptr<model::Problem> TTDPParser::read(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return nullptr;

    // Read header line
    int k = 0, M = 0, SD = 0, N = 0;
    if (!(file >> k >> M >> SD >> N)) return nullptr;

    // Read depot line
    int depot_i;
    double depot_x, depot_y;
    double depot_d, depot_S;
    double depot_open, depot_close;
    if (!(file >> depot_i >> depot_x >> depot_y >> depot_d >> depot_S)) return nullptr;
    // consume remaining tokens of depot line if any
    file >> std::ws;
    // For header remark: Tmax is closing - opening of starting point
    // Next token on depot line may include 0 and C; attempt to read two more
    // We try to read two values; if not present, default tmax=0
    if (!(file >> std::ws)) {}

    // For robustness, rewind to start of nodes parsing by using getline for remaining
    std::string rest_of_line;
    std::getline(file, rest_of_line);

    // Now create TTDPProblem with 7 days and tmax equal to depot closing-opening.
    // We'll assume 7 days always present in format.
    double tmax_per_day = 0.0;
    // Try to parse opening/closing from the depot's rest_of_line
    {
        std::stringstream ss(rest_of_line);
        double maybe0, maybeC;
        if (ss >> maybe0 >> maybeC) {
            tmax_per_day = maybeC - maybe0;
            depot_open = maybe0;
            depot_close = maybeC;
        }
    }

    auto problem = std::make_unique<model::variants::TTDPProblem>(filepath, 7, tmax_per_day);

    // Read N-1 remaining nodes (including depot already read partially)
    // Re-open the file to parse lines cleanly from the top
    file.clear();
    file.seekg(0);
    std::string line;
    // skip first header line
    std::getline(file, line);
    // read depot full line
    std::getline(file, line);

    int node_count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        model::Node node;
        int i;
        double x, y, d, S, t;
        if (!(ss >> i >> x >> y >> d >> S >> t)) continue;
        node.id = i;
        node.x = x;
        node.y = y;
        node.service_time = d;
        node.reward = S;
        // read 7 pairs of open/close
        std::vector<double> opens(7), closes(7);
        for (int day = 0; day < 7; ++day) {
            if (!(ss >> opens[day] >> closes[day])) {
                opens[day] = 0.0;
                closes[day] = 0.0;
            }
        }
        // set time window to the start day windows
        int start_day = SD % 7;
        node.tw.opening = opens[start_day];
        node.tw.closing = closes[start_day];
        problem->add_node(node);
        ++node_count;
    }

    problem->finalize();
    return problem;
}

} // namespace oplib::io
