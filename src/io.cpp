#include "hrocket/io.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

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

double speed(const TrajectoryPoint& p) {
    return norm(p.state.velocity_ned_mps);
}

double roll_deg(const TrajectoryPoint& p) {
    const auto q = normalize(p.state.attitude_body_to_ned);
    return std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * 57.29577951308232;
}

double pitch_deg(const TrajectoryPoint& p) {
    const auto q = normalize(p.state.attitude_body_to_ned);
    const double s = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
    return std::asin(s) * 57.29577951308232;
}

double yaw_deg(const TrajectoryPoint& p) {
    const auto q = normalize(p.state.attitude_body_to_ned);
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 57.29577951308232;
}

struct Series {
    std::string name;
    std::string color;
    std::vector<std::pair<double, double>> points;
};

std::pair<double, double> range_for(const std::vector<Series>& series, bool x_axis) {
    bool has_value = false;
    double lo = 0.0;
    double hi = 1.0;
    for (const auto& s : series) {
        for (const auto& p : s.points) {
            const double v = x_axis ? p.first : p.second;
            if (!has_value) {
                lo = hi = v;
                has_value = true;
            } else {
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
        }
    }
    if (std::abs(hi - lo) < 1.0e-9) {
        hi = lo + 1.0;
    }
    const double pad = 0.05 * (hi - lo);
    return {lo - pad, hi + pad};
}

std::string fmt(double value) {
    std::ostringstream ss;
    const double a = std::abs(value);
    if ((a >= 10000.0 || (a > 0.0 && a < 0.01))) {
        ss << std::scientific << std::setprecision(2) << value;
    } else if (a >= 100.0) {
        ss << std::fixed << std::setprecision(0) << value;
    } else if (a >= 10.0) {
        ss << std::fixed << std::setprecision(1) << value;
    } else {
        ss << std::fixed << std::setprecision(2) << value;
    }
    return ss.str();
}

void write_line_svg(const std::filesystem::path& path, const std::string& title, const std::string& x_label, const std::string& y_label, const std::vector<Series>& series) {
    constexpr double width = 1200.0;
    constexpr double height = 760.0;
    constexpr double left = 92.0;
    constexpr double right = 40.0;
    constexpr double top = 76.0;
    constexpr double bottom = 92.0;
    const auto [xmin, xmax] = range_for(series, true);
    const auto [ymin, ymax] = range_for(series, false);
    const auto sx = [&](double x) { return left + (x - xmin) / (xmax - xmin) * (width - left - right); };
    const auto sy = [&](double y) { return height - bottom - (y - ymin) / (ymax - ymin) * (height - top - bottom); };

    std::ofstream out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#101418\"/>\n";
    out << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\"" << width - left - right << "\" height=\"" << height - top - bottom << "\" rx=\"10\" fill=\"#151b22\" stroke=\"#2b3744\"/>\n";
    out << "<text x=\"40\" y=\"42\" fill=\"#e8eef5\" font-family=\"Segoe UI,Arial\" font-size=\"26\" font-weight=\"700\">" << title << "</text>\n";
    out << "<text x=\"" << width * 0.5 << "\" y=\"" << height - 28 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"16\" text-anchor=\"middle\">" << x_label << "</text>\n";
    out << "<text x=\"28\" y=\"" << height * 0.5 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"16\" transform=\"rotate(-90 28 " << height * 0.5 << ")\" text-anchor=\"middle\">" << y_label << "</text>\n";

    for (int i = 0; i <= 6; ++i) {
        const double x = left + i * (width - left - right) / 6.0;
        const double y = top + i * (height - top - bottom) / 6.0;
        const double xv = xmin + i * (xmax - xmin) / 6.0;
        const double yv = ymax - i * (ymax - ymin) / 6.0;
        out << "<line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" << x << "\" y2=\"" << height - bottom << "\" stroke=\"#26313c\"/>\n";
        out << "<line x1=\"" << left << "\" y1=\"" << y << "\" x2=\"" << width - right << "\" y2=\"" << y << "\" stroke=\"#26313c\"/>\n";
        out << "<text x=\"" << x << "\" y=\"" << height - bottom + 24 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"13\" text-anchor=\"middle\">" << fmt(xv) << "</text>\n";
        out << "<text x=\"" << left - 12 << "\" y=\"" << y + 5 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"13\" text-anchor=\"end\">" << fmt(yv) << "</text>\n";
    }

    double legend_x = left + 16.0;
    for (const auto& s : series) {
        out << "<polyline fill=\"none\" stroke=\"" << s.color << "\" stroke-width=\"3\" points=\"";
        for (const auto& p : s.points) {
            out << sx(p.first) << ',' << sy(p.second) << ' ';
        }
        out << "\"/>\n";
        out << "<rect x=\"" << legend_x << "\" y=\"92\" width=\"18\" height=\"4\" fill=\"" << s.color << "\"/>\n";
        out << "<text x=\"" << legend_x + 26.0 << "\" y=\"99\" fill=\"#cbd6e2\" font-family=\"Segoe UI,Arial\" font-size=\"14\">" << s.name << "</text>\n";
        legend_x += 150.0;
    }
    out << "</svg>\n";
}

void write_scatter_svg(const std::filesystem::path& path, const std::vector<DispersionPoint>& points) {
    std::vector<Series> series{{"impact", "#4fc3f7", {}}};
    for (const auto& p : points) {
        series.front().points.push_back({p.impact_east_m, p.impact_north_m});
    }
    constexpr double width = 900.0;
    constexpr double height = 760.0;
    constexpr double left = 92.0;
    constexpr double right = 40.0;
    constexpr double top = 76.0;
    constexpr double bottom = 92.0;
    const auto [xmin, xmax] = range_for(series, true);
    const auto [ymin, ymax] = range_for(series, false);
    const auto sx = [&](double x) { return left + (x - xmin) / (xmax - xmin) * (width - left - right); };
    const auto sy = [&](double y) { return height - bottom - (y - ymin) / (ymax - ymin) * (height - top - bottom); };

    std::ofstream out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#101418\"/>\n";
    out << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\"" << width - left - right << "\" height=\"" << height - top - bottom << "\" rx=\"10\" fill=\"#151b22\" stroke=\"#2b3744\"/>\n";
    out << "<text x=\"40\" y=\"42\" fill=\"#e8eef5\" font-family=\"Segoe UI,Arial\" font-size=\"26\" font-weight=\"700\">Impact Dispersion</text>\n";
    out << "<text x=\"" << width * 0.5 << "\" y=\"" << height - 28 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"16\" text-anchor=\"middle\">East [m]</text>\n";
    out << "<text x=\"28\" y=\"" << height * 0.5 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"16\" transform=\"rotate(-90 28 " << height * 0.5 << ")\" text-anchor=\"middle\">North [m]</text>\n";
    for (int i = 0; i <= 6; ++i) {
        const double x = left + i * (width - left - right) / 6.0;
        const double y = top + i * (height - top - bottom) / 6.0;
        const double xv = xmin + i * (xmax - xmin) / 6.0;
        const double yv = ymax - i * (ymax - ymin) / 6.0;
        out << "<line x1=\"" << x << "\" y1=\"" << top << "\" x2=\"" << x << "\" y2=\"" << height - bottom << "\" stroke=\"#26313c\"/>\n";
        out << "<line x1=\"" << left << "\" y1=\"" << y << "\" x2=\"" << width - right << "\" y2=\"" << y << "\" stroke=\"#26313c\"/>\n";
        out << "<text x=\"" << x << "\" y=\"" << height - bottom + 24 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"13\" text-anchor=\"middle\">" << fmt(xv) << "</text>\n";
        out << "<text x=\"" << left - 12 << "\" y=\"" << y + 5 << "\" fill=\"#9fb0c0\" font-family=\"Segoe UI,Arial\" font-size=\"13\" text-anchor=\"end\">" << fmt(yv) << "</text>\n";
    }
    for (const auto& p : points) {
        out << "<circle cx=\"" << sx(p.impact_east_m) << "\" cy=\"" << sy(p.impact_north_m) << "\" r=\"5\" fill=\"#4fc3f7\" fill-opacity=\"0.75\"/>\n";
    }
    out << "</svg>\n";
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
    v.control_enabled = get(values, "control_enabled", v.control_enabled);
    v.control_axis = get(values, "control_axis", v.control_axis);
    v.control_tail_area_m2 = get(values, "control_tail_area_m2", v.control_tail_area_m2);
    v.control_tail_lift_slope_per_rad = get(values, "control_tail_lift_slope_per_rad", v.control_tail_lift_slope_per_rad);
    v.control_tail_distance_from_nose_m = get(values, "control_tail_distance_from_nose_m", v.control_tail_distance_from_nose_m);
    v.control_max_deflection_deg = get(values, "control_max_deflection_deg", v.control_max_deflection_deg);
    v.control_min_speed_mps = get(values, "control_min_speed_mps", v.control_min_speed_mps);
    v.control_target_angle_deg = get(values, "control_target_angle_deg", v.control_target_angle_deg);
    v.control_kp = get(values, "control_kp", v.control_kp);
    v.control_kd = get(values, "control_kd", v.control_kd);
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
    out << "time_s,north_m,east_m,down_m,altitude_m,lat_deg,lon_deg,alt_m,vn_mps,ve_mps,vd_mps,thrust_n,wind_n_mps,wind_e_mps,wind_d_mps,cp_from_nose_m,control_deflection_deg,control_moment_x_nm,control_moment_y_nm,control_moment_z_nm\n";
    out << std::setprecision(10);
    for (const auto& p : result.points) {
        const auto [lat, lon] = ned_to_lat_lon(vehicle.launch_lat_deg, vehicle.launch_lon_deg, p.state.position_ned_m.x, p.state.position_ned_m.y);
        out << p.state.t_s << ',' << p.state.position_ned_m.x << ',' << p.state.position_ned_m.y << ',' << p.state.position_ned_m.z << ','
            << p.altitude_m << ',' << lat << ',' << lon << ',' << vehicle.launch_alt_m + p.altitude_m << ','
            << p.state.velocity_ned_mps.x << ',' << p.state.velocity_ned_mps.y << ',' << p.state.velocity_ned_mps.z << ','
            << p.thrust_n << ',' << p.wind_ned_mps.x << ',' << p.wind_ned_mps.y << ',' << p.wind_ned_mps.z << ',' << p.cp_from_nose_m << ','
            << p.control_deflection_deg << ','
            << p.control_moment_body_nm.x << ',' << p.control_moment_body_nm.y << ',' << p.control_moment_body_nm.z << '\n';
    }
}

void write_kml(const std::filesystem::path& path, const SimulationResult& result, const VehicleModel& vehicle, const std::vector<DispersionPoint>& dispersion) {
    std::ofstream out(path);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document><name>Hybrid Rocket Simulation</name>\n";
    out << "<Style id=\"trajectoryStyle\"><LineStyle><color>ff00c8ff</color><width>5</width></LineStyle></Style>\n";
    out << "<Style id=\"dispersionStyle\"><IconStyle><scale>1.1</scale><color>ff4fc3f7</color><Icon><href>http://maps.google.com/mapfiles/kml/shapes/target.png</href></Icon></IconStyle><LabelStyle><scale>0.65</scale></LabelStyle></Style>\n";
    out << "<Placemark><name>Trajectory</name><styleUrl>#trajectoryStyle</styleUrl><LineString><tessellate>1</tessellate><altitudeMode>absolute</altitudeMode><coordinates>\n";
    out << std::setprecision(10);
    for (const auto& p : result.points) {
        const auto [lat, lon] = ned_to_lat_lon(vehicle.launch_lat_deg, vehicle.launch_lon_deg, p.state.position_ned_m.x, p.state.position_ned_m.y);
        out << lon << ',' << lat << ',' << vehicle.launch_alt_m + p.altitude_m << '\n';
    }
    out << "</coordinates></LineString></Placemark>\n";
    out << "<Folder><name>Impact dispersion</name>\n";
    for (const auto& p : dispersion) {
        const auto [lat, lon] = ned_to_lat_lon(vehicle.launch_lat_deg, vehicle.launch_lon_deg, p.impact_north_m, p.impact_east_m);
        out << "<Placemark><name>Run " << p.run_index << "</name><styleUrl>#dispersionStyle</styleUrl>";
        out << "<description><![CDATA[wind_speed_mps=" << p.wind_speed_mps
            << "<br/>wind_direction_deg=" << p.wind_direction_deg
            << "<br/>apogee_m=" << p.apogee_m
            << "<br/>flight_time_s=" << p.flight_time_s << "]]></description>";
        out << "<Point><altitudeMode>absolute</altitudeMode><coordinates>" << lon << ',' << lat << ',' << vehicle.launch_alt_m << "</coordinates></Point></Placemark>\n";
    }
    out << "</Folder></Document></kml>\n";
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
    out << "run_index,wind_speed_mps,wind_direction_deg,wind_delta_north_mps,wind_delta_east_mps,impact_north_m,impact_east_m,apogee_m,flight_time_s,impacted\n";
    for (const auto& p : points) {
        out << p.run_index << ',' << p.wind_speed_mps << ',' << p.wind_direction_deg << ','
            << p.wind_delta_north_mps << ',' << p.wind_delta_east_mps << ','
            << p.impact_north_m << ',' << p.impact_east_m << ','
            << p.apogee_m << ',' << p.flight_time_s << ',' << (p.impacted ? 1 : 0) << '\n';
    }
}

void write_graph_svgs(const std::filesystem::path& directory, const SimulationResult& result, const std::vector<DispersionPoint>& dispersion) {
    std::filesystem::create_directories(directory);
    std::vector<Series> trajectory{{"Altitude vs downrange", "#4fc3f7", {}}};
    std::vector<Series> profile{
        {"Altitude [m]", "#4fc3f7", {}},
        {"Speed [m/s]", "#ffca28", {}},
        {"Thrust [N]", "#ef5350", {}},
    };
    std::vector<Series> attitude{
        {"Roll [deg]", "#ffca28", {}},
        {"Pitch [deg]", "#4fc3f7", {}},
        {"Yaw [deg]", "#ef5350", {}},
    };
    std::vector<Series> velocity{
        {"North [m/s]", "#4fc3f7", {}},
        {"East [m/s]", "#ffca28", {}},
        {"Down [m/s]", "#ef5350", {}},
    };
    std::vector<Series> control{
        {"Twin fins [deg]", "#4fc3f7", {}},
        {"Control moment [Nm]", "#ef5350", {}},
    };
    for (const auto& p : result.points) {
        const double downrange = std::sqrt(p.state.position_ned_m.x * p.state.position_ned_m.x + p.state.position_ned_m.y * p.state.position_ned_m.y);
        trajectory[0].points.push_back({downrange, p.altitude_m});
        profile[0].points.push_back({p.state.t_s, p.altitude_m});
        profile[1].points.push_back({p.state.t_s, speed(p)});
        profile[2].points.push_back({p.state.t_s, p.thrust_n});
        attitude[0].points.push_back({p.state.t_s, roll_deg(p)});
        attitude[1].points.push_back({p.state.t_s, pitch_deg(p)});
        attitude[2].points.push_back({p.state.t_s, yaw_deg(p)});
        velocity[0].points.push_back({p.state.t_s, p.state.velocity_ned_mps.x});
        velocity[1].points.push_back({p.state.t_s, p.state.velocity_ned_mps.y});
        velocity[2].points.push_back({p.state.t_s, p.state.velocity_ned_mps.z});
        control[0].points.push_back({p.state.t_s, p.control_deflection_deg});
        control[1].points.push_back({p.state.t_s, norm(p.control_moment_body_nm)});
    }
    write_line_svg(directory / "graph_trajectory.svg", "Trajectory", "Downrange [m]", "Altitude [m]", trajectory);
    write_line_svg(directory / "graph_profile.svg", "Altitude / Speed / Thrust", "Time [s]", "Value", profile);
    write_line_svg(directory / "graph_attitude.svg", "Attitude", "Time [s]", "Angle [deg]", attitude);
    write_line_svg(directory / "graph_velocity.svg", "Velocity", "Time [s]", "Velocity [m/s]", velocity);
    write_line_svg(directory / "graph_control.svg", "Twin Tail Control", "Time [s]", "Command / Moment", control);
    write_scatter_svg(directory / "graph_dispersion.svg", dispersion);
}

} // namespace hrocket
