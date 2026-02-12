#pragma once

#include "instance_parser.h"
#include "../model/variants/ttdp.h"
#include <memory>
#include <string>

namespace oplib::io {

/**
 * @brief Parser for TTDP instances (multi-day TOPTW-like instances).
 * TTDP for Tourist Trip Design Problem
 */
class TTDPParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
