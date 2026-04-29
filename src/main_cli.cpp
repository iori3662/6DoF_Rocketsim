#include "hrocket/io.hpp"
#include "hrocket/simulator.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace {

std::string arg_value(int argc, char** argv, const std::string& name, const std::string& fallback = {}) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (argv[i] == name) {
            return argv[i + 1];
        }
    }
    return fallback;
}

void usage() {
    std::cout << "Usage: hrocket_cli --vehicle vehicle.csv --thrust thrust.csv --wind wind.csv --out output_dir [--dt 0.01] [--max-time 180] [--descent freefall|parachute] [--wind-mode nominal|calm] [--dispersion 100] [--wind-sigma 2.0] [--seed 1]\n";
}

hrocket::DescentMode parse_descent_mode(const std::string& value) {
    if (value == "parachute") {
        return hrocket::DescentMode::Parachute;
    }
    if (value == "freefall") {
        return hrocket::DescentMode::FreeFall;
    }
    throw std::runtime_error("invalid --descent value: " + value);
}

hrocket::WindMode parse_wind_mode(const std::string& value) {
    if (value == "nominal") {
        return hrocket::WindMode::Nominal;
    }
    if (value == "calm") {
        return hrocket::WindMode::Calm;
    }
    throw std::runtime_error("invalid --wind-mode value: " + value);
}

hrocket::DispersionPoint summarize(int run_index, const hrocket::SimulationResult& result) {
    hrocket::DispersionPoint point;
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

} // namespace

int main(int argc, char** argv) {
    try {
        const auto vehicle_path = arg_value(argc, argv, "--vehicle");
        const auto thrust_path = arg_value(argc, argv, "--thrust");
        const auto wind_path = arg_value(argc, argv, "--wind");
        const auto out_dir = std::filesystem::path(arg_value(argc, argv, "--out", "out"));
        if (vehicle_path.empty() || thrust_path.empty() || wind_path.empty()) {
            usage();
            return 2;
        }

        hrocket::SimulationInputs inputs;
        inputs.vehicle = hrocket::load_vehicle_csv(vehicle_path);
        inputs.thrust = hrocket::load_thrust_csv(thrust_path);
        inputs.wind = hrocket::load_wind_csv(wind_path);
        if (const auto dt = arg_value(argc, argv, "--dt"); !dt.empty()) {
            inputs.config.dt_s = std::stod(dt);
        }
        if (const auto max_time = arg_value(argc, argv, "--max-time"); !max_time.empty()) {
            inputs.config.max_time_s = std::stod(max_time);
        }
        inputs.config.descent_mode = parse_descent_mode(arg_value(argc, argv, "--descent", "freefall"));
        inputs.config.wind_mode = parse_wind_mode(arg_value(argc, argv, "--wind-mode", "nominal"));
        const int dispersion_runs = std::max(1, std::stoi(arg_value(argc, argv, "--dispersion", "1")));
        const double wind_sigma = std::stod(arg_value(argc, argv, "--wind-sigma", "0.0"));
        const unsigned seed = static_cast<unsigned>(std::stoul(arg_value(argc, argv, "--seed", "1")));

        std::filesystem::create_directories(out_dir);
        const auto result = hrocket::run_simulation(inputs);
        hrocket::write_trajectory_csv(out_dir / "trajectory.csv", result, inputs.vehicle);
        hrocket::write_kml(out_dir / "trajectory.kml", result, inputs.vehicle);
        hrocket::write_summary_csv(out_dir / "summary.csv", result);
        std::vector<hrocket::DispersionPoint> dispersion;
        dispersion.push_back(summarize(0, result));
        if (dispersion_runs > 1) {
            std::mt19937 rng(seed);
            std::normal_distribution<double> wind_error(0.0, wind_sigma);
            for (int i = 1; i < dispersion_runs; ++i) {
                auto varied = inputs;
                const double dn = wind_error(rng);
                const double de = wind_error(rng);
                for (auto& sample : varied.wind.samples) {
                    sample.wind_ned_mps.x += dn;
                    sample.wind_ned_mps.y += de;
                }
                dispersion.push_back(summarize(i, hrocket::run_simulation(varied)));
            }
        }
        hrocket::write_dispersion_csv(out_dir / "dispersion.csv", dispersion);

        std::cout << "Wrote " << result.points.size() << " trajectory points to " << out_dir.string() << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
