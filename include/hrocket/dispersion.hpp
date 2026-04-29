#pragma once

#include "hrocket/io.hpp"
#include "hrocket/models.hpp"

#include <vector>

namespace hrocket {

DispersionPoint summarize_result(int run_index, const SimulationResult& result);
std::vector<DispersionPoint> run_dispersion(const SimulationInputs& inputs, const SimulationResult& nominal, const DispersionConfig& config);

} // namespace hrocket

