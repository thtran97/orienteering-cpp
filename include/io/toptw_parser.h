#pragma once

#include "instance_parser.h"
#include "../model/variants/toptw.h"

namespace oplib::io {

/**
 * @brief Parser for TOPTW variant instances (supports Solomon/Cordeau format).
 */
class TOPTWParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
