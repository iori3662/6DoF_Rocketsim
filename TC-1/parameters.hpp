#include <string>

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

struct BodyParameters {
    double mass;
    double radius;
    double inertia;
    double drag_coefficient;
    double drag_area;
    double thrust;
    double burn_time;
    double fuel_mass;
    double fuel_density;
    double nozzle_area;
    double specific_impulse;
    double exhaust_velocity;
    double max_thrust;
    double min_thrust;
    double average_thrust;
    double Isp;  // 比推力 [s]
};

BodyParameters loadBodyParameters(const std::string& filename);

#endif
