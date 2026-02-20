#pragma once

#include <memory>
#include <string>

#include "io/instance_parser.h"
#include "model/variants/mctopmtw.h"

namespace oplib::io {

/**
 * @brief Parser for MC-TOP-MTW instances.
 */
class MCTOPMTWParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
