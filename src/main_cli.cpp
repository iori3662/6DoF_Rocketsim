#include "hrocket/dispersion.hpp"
#include "hrocket/io.hpp"
#include "hrocket/simulator.hpp"

#include <filesystem>
#include <iostream>
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
    std::cout << "Usage: hrocket_cli --vehicle vehicle.csv --thrust thrust.csv --wind wind.csv --out output_dir [--dt 0.01] [--max-time 180] [--descent freefall|parachute] [--wind-mode nominal|calm] [--dispersion-mode montecarlo|wind-sweep] [--dispersion 100] [--wind-sigma 2.0] [--seed 1] [--sweep-max-wind 7] [--sweep-step 1] [--sweep-directions 16]\n";
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

hrocket::DispersionMode parse_dispersion_mode(const std::string& value) {
    if (value == "montecarlo") {
        return hrocket::DispersionMode::MonteCarlo;
    }
    if (value == "wind-sweep") {
        return hrocket::DispersionMode::WindSweep;
    }
    throw std::runtime_error("invalid --dispersion-mode value: " + value);
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
        hrocket::DispersionConfig dispersion_config;
        dispersion_config.mode = parse_dispersion_mode(arg_value(argc, argv, "--dispersion-mode", "montecarlo"));
        dispersion_config.runs = std::max(1, std::stoi(arg_value(argc, argv, "--dispersion", "1")));
        dispersion_config.wind_sigma_mps = std::stod(arg_value(argc, argv, "--wind-sigma", "0.0"));
        dispersion_config.seed = static_cast<unsigned>(std::stoul(arg_value(argc, argv, "--seed", "1")));
        dispersion_config.sweep_max_wind_mps = std::stod(arg_value(argc, argv, "--sweep-max-wind", "7.0"));
        dispersion_config.sweep_step_mps = std::stod(arg_value(argc, argv, "--sweep-step", "1.0"));
        dispersion_config.sweep_directions = std::max(1, std::stoi(arg_value(argc, argv, "--sweep-directions", "16")));

        std::filesystem::create_directories(out_dir);
        const auto result = hrocket::run_simulation(inputs);
        const auto dispersion = hrocket::run_dispersion(inputs, result, dispersion_config);
        hrocket::write_trajectory_csv(out_dir / "trajectory.csv", result, inputs.vehicle);
        hrocket::write_kml(out_dir / "trajectory.kml", result, inputs.vehicle, dispersion);
        hrocket::write_summary_csv(out_dir / "summary.csv", result);
        hrocket::write_dispersion_csv(out_dir / "dispersion.csv", dispersion);
        hrocket::write_graph_svgs(out_dir, result, dispersion);

        std::cout << "Wrote " << result.points.size() << " trajectory points to " << out_dir.string() << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
