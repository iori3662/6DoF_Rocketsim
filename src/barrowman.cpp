#include "hrocket/barrowman.hpp"

#include <algorithm>
#include <cmath>

namespace hrocket {

BarrowmanResult compute_barrowman(const VehicleModel& v) {
    constexpr double pi = 3.14159265358979323846;
    const double d = std::max(v.diameter_m, 1.0e-6);
    const double nose_cn = 2.0;
    const double nose_cp = 0.466 * v.nose_length_m;

    const double semi_span = v.fin_span_m;
    const double root = v.fin_root_chord_m;
    const double tip = v.fin_tip_chord_m;
    const double mid = 0.5 * (root + tip);
    const double body_radius = 0.5 * d;
    const double fin_area = 0.5 * (root + tip) * semi_span;
    const double fins_cn = (1.0 + body_radius / (semi_span + body_radius)) *
        (4.0 * v.fin_count * std::pow(semi_span / d, 2.0)) /
        (1.0 + std::sqrt(1.0 + std::pow(2.0 * mid / (root + tip), 2.0)));

    const double fin_cp_local = v.fin_sweep_m * (root + 2.0 * tip) / (3.0 * (root + tip)) +
        (root * root + root * tip + tip * tip) / (6.0 * (root + tip));
    const double fins_cp = v.fin_distance_from_nose_m + fin_cp_local;

    const double total_cn = std::max(nose_cn + fins_cn, 1.0e-6);
    const double cp = (nose_cn * nose_cp + fins_cn * fins_cp) / total_cn;
    return {std::clamp(cp, 0.0, v.length_m), total_cn};
}

} // namespace hrocket
