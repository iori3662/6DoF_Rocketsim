#include "hrocket/io.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace hrocket {
namespace {

std::string trim(std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        out.push_back(trim(cell));
    }
    return out;
}

double as_double(const std::string& value, double fallback = 0.0) {
    if (value.empty()) {
        return fallback;
    }
    return std::stod(value);
}

double get(const std::map<std::string, double>& values, const std::string& key, double fallback) {
    const auto it = values.find(key);
    return it == values.end() ? fallback : it->second;
}

std::vector<std::vector<std::string>> read_table(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open " + path.string());
    }
    std::vector<std::vector<std::string>> rows;
    std::string line;
    while (std::getline(in, line)) {
        const auto s = trim(line);
        if (s.empty() || s[0] == '#') {
            continue;
        }
        rows.push_back(split_csv_line(s));
    }
    return rows;
}

std::pair<double, double> ned_to_lat_lon(double lat0_deg, double lon0_deg, double north_m, double east_m) {
    constexpr double earth_radius_m = 6378137.0;
    constexpr double deg_per_rad = 57.29577951308232;
    const double lat0_rad = lat0_deg / deg_per_rad;
    const double lat = lat0_deg + (north_m / earth_radius_m) * deg_per_rad;
    const double lon = lon0_deg + (east_m / (earth_radius_m * std::cos(lat0_rad))) * deg_per_rad;
    return {lat, lon};
}

} // namespace

VehicleModel load_vehicle_csv(const std::filesystem::path& path) {
    std::map<std::string, double> values;
    for (const auto& row : read_table(path)) {
        if (row.size() >= 2 && row[0] != "key") {
            values[row[0]] = as_double(row[1]);
        }
    }

    VehicleModel v;
    v.mass_kg = get(values, "mass_kg", v.mass_kg);
    v.inertia_kg_m2 = {get(values, "ixx_kg_m2", v.inertia_kg_m2.x), get(values, "iyy_kg_m2", v.inertia_kg_m2.y), get(values, "izz_kg_m2", v.inertia_kg_m2.z)};
    v.reference_area_m2 = get(values, "reference_area_m2", v.reference_area_m2);
    v.diameter_m = get(values, "diameter_m", v.diameter_m);
    v.length_m = get(values, "length_m", v.length_m);
    v.cg_from_nose_m = get(values, "cg_from_nose_m", v.cg_from_nose_m);
    v.cn_alpha_per_rad = get(values, "cn_alpha_per_rad", v.cn_alpha_per_rad);
    v.cd = get(values, "cd", v.cd);
    v.launch_lat_deg = get(values, "launch_lat_deg", v.launch_lat_deg);
    v.launch_lon_deg = get(values, "launch_lon_deg", v.launch_lon_deg);
    v.launch_alt_m = get(values, "launch_alt_m", v.launch_alt_m);
    v.rail_elevation_deg = get(values, "rail_elevation_deg", v.rail_elevation_deg);
    v.rail_azimuth_deg = get(values, "rail_azimuth_deg", v.rail_azimuth_deg);
    v.nose_length_m = get(values, "nose_length_m", v.nose_length_m);
    v.fin_count = get(values, "fin_count", v.fin_count);
    v.fin_root_chord_m = get(values, "fin_root_chord_m", v.fin_root_chord_m);
    v.fin_tip_chord_m = get(values, "fin_tip_chord_m", v.fin_tip_chord_m);
    v.fin_span_m = get(values, "fin_span_m", v.fin_span_m);
    v.fin_sweep_m = get(values, "fin_sweep_m", v.fin_sweep_m);
    v.fin_distance_from_nose_m = get(values, "fin_distance_from_nose_m", v.fin_distance_from_nose_m);
    v.parachute_area_m2 = get(values, "parachute_area_m2", v.parachute_area_m2);
    v.parachute_cd = get(values, "parachute_cd", v.parachute_cd);
    v.parachute_deploy_altitude_m = get(values, "parachute_deploy_altitude_m", v.parachute_deploy_altitude_m);
    return v;
}

ThrustCurve load_thrust_csv(const std::filesystem::path& path) {
    ThrustCurve curve;
    for (const auto& row : read_table(path)) {
        if (row.size() >= 2 && row[0] != "time_s") {
            curve.samples.push_back({as_double(row[0]), as_double(row[1])});
        }
    }
    std::sort(curve.samples.begin(), curve.samples.end(), [](auto a, auto b) { return a.t < b.t; });
    return curve;
}

WindProfile load_wind_csv(const std::filesystem::path& path) {
    WindProfile profile;
    for (const auto& row : read_table(path)) {
        if (row.size() >= 4 && row[0] != "altitude_m") {
            profile.samples.push_back({as_double(row[0]), {as_double(row[1]), as_double(row[2]), as_double(row[3])}});
        }
    }
    std::sort(profile.samples.begin(), profile.samples.end(), [](auto a, auto b) { return a.altitude_m < b.altitude_m; });
    return profile;
}

void write_trajectory_csv(const std::filesystem::path& path, const SimulationResult& result, const VehicleModel& vehicle) {
    std::ofstream out(path);
    out << "time_s,north_m,east_m,down_m,altitude_m,lat_deg,lon_deg,alt_m,vn_mps,ve_mps,vd_mps,thrust_n,wind_n_mps,wind_e_mps,wind_d_mps,cp_from_nose_m\n";
    out << std::setprecision(10);
    for (const auto& p : result.points) {
        const auto [lat, lon] = ned_to_lat_lon(vehicle.launch_lat_deg, vehicle.launch_lon_deg, p.state.position_ned_m.x, p.state.position_ned_m.y);
        out << p.state.t_s << ',' << p.state.position_ned_m.x << ',' << p.state.position_ned_m.y << ',' << p.state.position_ned_m.z << ','
            << p.altitude_m << ',' << lat << ',' << lon << ',' << vehicle.launch_alt_m + p.altitude_m << ','
            << p.state.velocity_ned_mps.x << ',' << p.state.velocity_ned_mps.y << ',' << p.state.velocity_ned_mps.z << ','
            << p.thrust_n << ',' << p.wind_ned_mps.x << ',' << p.wind_ned_mps.y << ',' << p.wind_ned_mps.z << ',' << p.cp_from_nose_m << '\n';
    }
}

void write_kml(const std::filesystem::path& path, const SimulationResult& result, const VehicleModel& vehicle) {
    std::ofstream out(path);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>Hybrid Rocket Trajectory</name><Placemark><LineString><altitudeMode>absolute</altitudeMode><coordinates>\n";
    out << std::setprecision(10);
    for (const auto& p : result.points) {
        const auto [lat, lon] = ned_to_lat_lon(vehicle.launch_lat_deg, vehicle.launch_lon_deg, p.state.position_ned_m.x, p.state.position_ned_m.y);
        out << lon << ',' << lat << ',' << vehicle.launch_alt_m + p.altitude_m << '\n';
    }
    out << "</coordinates></LineString></Placemark></Document></kml>\n";
}

void write_summary_csv(const std::filesystem::path& path, const SimulationResult& result) {
    std::ofstream out(path);
    out << "metric,value\n";
    if (result.points.empty()) {
        out << "points,0\n";
        return;
    }
    const auto apogee = std::max_element(result.points.begin(), result.points.end(), [](const auto& a, const auto& b) { return a.altitude_m < b.altitude_m; });
    const auto& last = result.points.back();
    out << "points," << result.points.size() << '\n';
    out << "apogee_m," << apogee->altitude_m << '\n';
    out << "flight_time_s," << last.state.t_s << '\n';
    out << "impact_north_m," << last.state.position_ned_m.x << '\n';
    out << "impact_east_m," << last.state.position_ned_m.y << '\n';
    out << "impacted," << (result.impacted ? 1 : 0) << '\n';
}

void write_dispersion_csv(const std::filesystem::path& path, const std::vector<DispersionPoint>& points) {
    std::ofstream out(path);
    out << "run_index,impact_north_m,impact_east_m,apogee_m,flight_time_s,impacted\n";
    for (const auto& p : points) {
        out << p.run_index << ',' << p.impact_north_m << ',' << p.impact_east_m << ','
            << p.apogee_m << ',' << p.flight_time_s << ',' << (p.impacted ? 1 : 0) << '\n';
    }
}

} // namespace hrocket
