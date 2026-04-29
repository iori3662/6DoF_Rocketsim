#include "hrocket/barrowman.hpp"
#include "hrocket/simulator.hpp"

#include <algorithm>
#include <cassert>
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
    std::cout << "ok\n";
    return 0;
}
