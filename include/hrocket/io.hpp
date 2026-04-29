#pragma once

#include "hrocket/models.hpp"

#include <filesystem>

namespace hrocket {

VehicleModel load_vehicle_csv(const std::filesystem::path& path);
ThrustCurve load_thrust_csv(const std::filesystem::path& path);
WindProfile load_wind_csv(const std::filesystem::path& path);

void write_trajectory_csv(const std::filesystem::path& path, const SimulationResult& result, const VehicleModel& vehicle);
void write_kml(const std::filesystem::path& path, const SimulationResult& result, const VehicleModel& vehicle);
void write_summary_csv(const std::filesystem::path& path, const SimulationResult& result);

struct DispersionPoint {
    int run_index{};
    double impact_north_m{};
    double impact_east_m{};
    double apogee_m{};
    double flight_time_s{};
    bool impacted{};
};

void write_dispersion_csv(const std::filesystem::path& path, const std::vector<DispersionPoint>& points);
void write_graph_svgs(const std::filesystem::path& directory, const SimulationResult& result, const std::vector<DispersionPoint>& dispersion);

} // namespace hrocket
