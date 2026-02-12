#pragma once

#include "instance_parser.h"
#include "../model/variants/singlesat.h"
#include <memory>
#include <string>

namespace oplib::io {

/**
 * @brief Parser for SingleSat instances (modeled as OPTW, dataset on demand). 
 */
class SingleSatParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
