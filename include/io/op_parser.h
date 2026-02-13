#pragma once

#include <memory>
#include <string>

#include "io/instance_parser.h"
#include "model/variants/op.h"

namespace oplib::io {

/**
 * @brief Parser for OP variant instances (supports Chao/Tsiligirides format).
 */
class OPParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
