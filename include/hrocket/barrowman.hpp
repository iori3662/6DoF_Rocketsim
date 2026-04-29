#pragma once

#include "hrocket/models.hpp"

namespace hrocket {

struct BarrowmanResult {
    double cp_from_nose_m{};
    double cn_alpha_per_rad{};
};

BarrowmanResult compute_barrowman(const VehicleModel& vehicle);

} // namespace hrocket

