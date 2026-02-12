#pragma once

#include "../model/variants/tdoptw.h"
#include <memory>
#include <string>

namespace oplib::io {

/**
 * @brief Parser for Time-Dependent OPTW variant instances.
 *
 * The TD-OPTW requires an instance file and a transition matrix file.
 */
class TDOPTWParser {
public:
    static std::unique_ptr<model::variants::TDOPTWProblem> read(
        const std::string& instance_path,           // path to adapted TOP instance file (with time windows and service times)
        const std::string& transition_matrix_path   // path to transition matrix file (n lines of n lists of time-dependent travel times)
    );
};

} // namespace oplib::io
