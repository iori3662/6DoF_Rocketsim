#include "hrocket/models.hpp"

#include <algorithm>

namespace hrocket {

double ThrustCurve::value_at(double t) const {
    if (samples.empty()) {
        return 0.0;
    }
    if (t <= samples.front().t) {
        return samples.front().value;
    }
    for (size_t i = 1; i < samples.size(); ++i) {
        if (t <= samples[i].t) {
            const auto a = samples[i - 1];
            const auto b = samples[i];
            const double u = (t - a.t) / (b.t - a.t);
            return a.value + (b.value - a.value) * u;
        }
    }
    return samples.back().value;
}

Vec3 WindProfile::value_at_altitude(double altitude_m) const {
    if (samples.empty()) {
        return {};
    }
    if (altitude_m <= samples.front().altitude_m) {
        return samples.front().wind_ned_mps;
    }
    for (size_t i = 1; i < samples.size(); ++i) {
        if (altitude_m <= samples[i].altitude_m) {
            const auto& a = samples[i - 1];
            const auto& b = samples[i];
            const double u = (altitude_m - a.altitude_m) / (b.altitude_m - a.altitude_m);
            return a.wind_ned_mps + (b.wind_ned_mps - a.wind_ned_mps) * u;
        }
    }
    return samples.back().wind_ned_mps;
}

} // namespace hrocket

