#include "hrocket/dispersion.hpp"

#include "hrocket/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace hrocket {
namespace {

WindProfile constant_horizontal_wind(double speed_mps, double azimuth_rad) {
    const Vec3 wind{speed_mps * std::cos(azimuth_rad), speed_mps * std::sin(azimuth_rad), 0.0};
    return {{{0.0, wind}, {10000.0, wind}}};
}

} // namespace

DispersionPoint summarize_result(int run_index, const SimulationResult& result) {
    DispersionPoint point;
    point.run_index = run_index;
    point.impacted = result.impacted;
    for (const auto& p : result.points) {
        point.apogee_m = std::max(point.apogee_m, p.altitude_m);
    }
    if (!result.points.empty()) {
        const auto& last = result.points.back();
        point.impact_north_m = last.state.position_ned_m.x;
        point.impact_east_m = last.state.position_ned_m.y;
        point.flight_time_s = last.state.t_s;
    }
    return point;
}

std::vector<DispersionPoint> run_dispersion(const SimulationInputs& inputs, const SimulationResult& nominal, const DispersionConfig& config) {
    std::vector<DispersionPoint> dispersion;
    dispersion.push_back(summarize_result(0, nominal));

    if (config.mode == DispersionMode::MonteCarlo) {
        const int runs = std::max(1, config.runs);
        std::mt19937 rng(config.seed);
        std::normal_distribution<double> wind_error(0.0, config.wind_sigma_mps);
        for (int i = 1; i < runs; ++i) {
            auto varied = inputs;
            const double dn = wind_error(rng);
            const double de = wind_error(rng);
            for (auto& sample : varied.wind.samples) {
                sample.wind_ned_mps.x += dn;
                sample.wind_ned_mps.y += de;
            }
            auto point = summarize_result(i, run_simulation(varied));
            point.wind_delta_north_mps = dn;
            point.wind_delta_east_mps = de;
            point.wind_speed_mps = std::sqrt(dn * dn + de * de);
            point.wind_direction_deg = std::atan2(de, dn) * 57.29577951308232;
            dispersion.push_back(point);
        }
        return dispersion;
    }

    int run_index = 1;
    const int directions = std::max(1, config.sweep_directions);
    const double step = std::max(0.1, config.sweep_step_mps);
    for (double speed = step; speed <= config.sweep_max_wind_mps + 1.0e-9; speed += step) {
        for (int dir = 0; dir < directions; ++dir) {
            auto varied = inputs;
            const double azimuth = (2.0 * 3.14159265358979323846 * dir) / directions;
            varied.wind = constant_horizontal_wind(speed, azimuth);
            varied.config.wind_mode = WindMode::Nominal;
            auto point = summarize_result(run_index++, run_simulation(varied));
            point.wind_speed_mps = speed;
            point.wind_direction_deg = azimuth * 57.29577951308232;
            dispersion.push_back(point);
        }
    }
    return dispersion;
}

} // namespace hrocket
