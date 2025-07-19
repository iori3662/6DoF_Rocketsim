#include "parameters.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

BodyParameters loadBodyParameters(const std::string& filename) {
    BodyParameters params = {};
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        getline(file, line); // Skip header
        if (getline(file, line)) {
            std::stringstream ss(line);
            std::string value;

            std::getline(ss, value, ','); params.mass = std::stod(value);
            std::getline(ss, value, ','); params.radius = std::stod(value);
            std::getline(ss, value, ','); params.inertia = std::stod(value);
            std::getline(ss, value, ','); params.drag_coefficient = std::stod(value);
            std::getline(ss, value, ','); params.drag_area = std::stod(value);
            std::getline(ss, value, ','); params.thrust = std::stod(value);
            std::getline(ss, value, ','); params.burn_time = std::stod(value);
            std::getline(ss, value, ','); params.fuel_mass = std::stod(value);
            std::getline(ss, value, ','); params.fuel_density = std::stod(value);
            std::getline(ss, value, ','); params.nozzle_area = std::stod(value);
            std::getline(ss, value, ','); params.specific_impulse = std::stod(value);
            std::getline(ss, value, ','); params.exhaust_velocity = std::stod(value);
            std::getline(ss, value, ','); params.max_thrust = std::stod(value);
            std::getline(ss, value, ','); params.min_thrust = std::stod(value);
            std::getline(ss, value, ','); params.average_thrust = std::stod(value);
        }
        file.close();
    } else {
        std::cerr << "Failed to open parameter file." << std::endl;
    }

    return params;
}
