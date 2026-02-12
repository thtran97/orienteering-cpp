#pragma once

#include <string>
#include <memory>
#include "../model/problem.h"

namespace oplib::io {

/**
 * @brief Abstract interface for instance parsers.
 */
class InstanceParser {
public:
    virtual ~InstanceParser() = default;
    virtual std::unique_ptr<model::Problem> read(const std::string& filepath) = 0;
};

} // namespace oplib::io
