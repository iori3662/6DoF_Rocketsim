#include "hrocket/barrowman.hpp"
#include "hrocket/simulator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    hrocket::VehicleModel vehicle;
    const auto barrow = hrocket::compute_barrowman(vehicle);
    assert(barrow.cp_from_nose_m > 0.0);
    assert(barrow.cp_from_nose_m <= vehicle.length_m);

    hrocket::SimulationInputs inputs;
    inputs.vehicle = vehicle;
    inputs.thrust.samples = {{0.0, 0.0}, {0.1, 500.0}, {1.0, 500.0}, {1.2, 0.0}, {10.0, 0.0}};
    inputs.wind.samples = {{0.0, {0.0, 0.0, 0.0}}, {1000.0, {2.0, 0.0, 0.0}}};
    inputs.config.dt_s = 0.02;
    inputs.config.max_time_s = 10.0;
    const auto result = hrocket::run_simulation(inputs);
    assert(!result.points.empty());
    double apogee = 0.0;
    for (const auto& p : result.points) {
        apogee = std::max(apogee, p.altitude_m);
    }
    assert(apogee > 1.0);

    inputs.config.descent_mode = hrocket::DescentMode::Parachute;
    inputs.config.wind_mode = hrocket::WindMode::Calm;
    inputs.vehicle.parachute_deploy_altitude_m = 1000.0;
    const auto parachute_result = hrocket::run_simulation(inputs);
    assert(!parachute_result.points.empty());
    assert(parachute_result.points.back().state.t_s >= result.points.back().state.t_s);

    inputs.vehicle.control_enabled = 1.0;
    inputs.vehicle.control_target_pitch_deg = 0.0;
    inputs.vehicle.control_target_yaw_deg = 0.0;
    const auto controlled_result = hrocket::run_simulation(inputs);
    bool saw_control = false;
    for (const auto& p : controlled_result.points) {
        saw_control = saw_control || std::abs(p.control_pitch_deflection_deg) > 0.0 || std::abs(p.control_yaw_deflection_deg) > 0.0;
        assert(std::abs(p.control_pitch_deflection_deg) <= inputs.vehicle.control_max_deflection_deg + 1.0e-9);
        assert(std::abs(p.control_yaw_deflection_deg) <= inputs.vehicle.control_max_deflection_deg + 1.0e-9);
    }
    assert(saw_control);
    std::cout << "ok\n";
    return 0;
}
