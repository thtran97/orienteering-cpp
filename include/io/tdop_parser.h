#pragma once

#include <memory>
#include <string>

#include "model/variants/tdop.h"

namespace oplib::io {

/**
 * @brief Parser for Time-Dependent OP variant instances.
 *
 * The TD-OP requires three input files: the instance (adapted TOP), the
 * speed matrix and the arc category matrix. Use `read` to load all three.
 */
class TDOPParser {
public:
    static std::unique_ptr<model::variants::TDOPProblem> read(
        const std::string& instance_path,       // path to adapted TOP instance file
        const std::string& speed_matrix_path,   // path to speed matrix file
        const std::string& arc_cat_path         // path to arc category matrix file (n lines of n integers each)
    );
};

} // namespace oplib::io
