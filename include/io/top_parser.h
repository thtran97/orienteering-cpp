#pragma once

#include "instance_parser.h"
#include "../model/variants/top.h"

namespace oplib::io {

/**
 * @brief Parser for TOP variant instances.
 */
class TOPParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
