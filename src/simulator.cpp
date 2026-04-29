#include "hrocket/simulator.hpp"

#include "hrocket/barrowman.hpp"

#include <cmath>

namespace hrocket {
namespace {

constexpr double g_mps2 = 9.80665;
constexpr double rho0_kgpm3 = 1.225;
constexpr double deg_to_rad = 0.017453292519943295;

struct Derivative {
    Vec3 pos_dot;
    Vec3 vel_dot;
    Quat quat_dot;
    Vec3 omega_dot;
};

State add_scaled(State s, const Derivative& d, double h) {
    s.position_ned_m += d.pos_dot * h;
    s.velocity_ned_mps += d.vel_dot * h;
    s.attitude_body_to_ned = normalize({
        s.attitude_body_to_ned.w + d.quat_dot.w * h,
        s.attitude_body_to_ned.x + d.quat_dot.x * h,
        s.attitude_body_to_ned.y + d.quat_dot.y * h,
        s.attitude_body_to_ned.z + d.quat_dot.z * h,
    });
    s.omega_body_radps += d.omega_dot * h;
    s.t_s += h;
    return s;
}

Quat initial_attitude(double elevation_deg, double azimuth_deg) {
    const double el = elevation_deg * deg_to_rad;
    const double az = azimuth_deg * deg_to_rad;
    const Vec3 forward_ned{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), -std::sin(el)};
    const Vec3 body_x{1.0, 0.0, 0.0};
    const Vec3 axis = normalized(cross(body_x, forward_ned));
    const double c = clamp(dot(body_x, forward_ned), -1.0, 1.0);
    const double angle = std::acos(c);
    return normalize({std::cos(angle * 0.5), axis.x * std::sin(angle * 0.5), axis.y * std::sin(angle * 0.5), axis.z * std::sin(angle * 0.5)});
}

Derivative eval(const SimulationInputs& in, const BarrowmanResult& barrow, const State& s) {
    const double altitude = -s.position_ned_m.z;
    const Vec3 wind = in.config.wind_mode == WindMode::Calm ? Vec3{} : in.wind.value_at_altitude(altitude);
    const Vec3 air_rel_ned = s.velocity_ned_mps - wind;
    const Vec3 air_rel_body = rotate_ned_to_body(s.attitude_body_to_ned, air_rel_ned);
    const double speed = norm(air_rel_body);
    const double speed_ned = norm(air_rel_ned);
    const double q = 0.5 * rho0_kgpm3 * speed * speed;
    const double q_ned = 0.5 * rho0_kgpm3 * speed_ned * speed_ned;
    const double alpha = speed > 1.0e-6 ? std::atan2(std::sqrt(air_rel_body.y * air_rel_body.y + air_rel_body.z * air_rel_body.z), std::abs(air_rel_body.x)) : 0.0;

    const Vec3 drag_body = speed > 1.0e-6 ? normalized(air_rel_body) * (-q * in.vehicle.reference_area_m2 * in.vehicle.cd) : Vec3{};
    const double normal_mag = -q * in.vehicle.reference_area_m2 * in.vehicle.cn_alpha_per_rad * alpha;
    const Vec3 lateral = normalized({0.0, air_rel_body.y, air_rel_body.z});
    const Vec3 normal_body = lateral * normal_mag;
    const Vec3 thrust_body{in.thrust.value_at(s.t_s), 0.0, 0.0};
    const Vec3 gravity_ned{0.0, 0.0, g_mps2};
    const bool parachute_active = in.config.descent_mode == DescentMode::Parachute &&
        s.velocity_ned_mps.z > 0.0 &&
        altitude <= in.vehicle.parachute_deploy_altitude_m;
    const Vec3 parachute_drag_ned = parachute_active && speed_ned > 1.0e-6
        ? normalized(air_rel_ned) * (-q_ned * in.vehicle.parachute_area_m2 * in.vehicle.parachute_cd)
        : Vec3{};
    const Vec3 force_ned = rotate_body_to_ned(s.attitude_body_to_ned, thrust_body + drag_body + normal_body) +
        parachute_drag_ned + gravity_ned * in.vehicle.mass_kg;

    const Vec3 moment_arm{in.vehicle.cg_from_nose_m - barrow.cp_from_nose_m, 0.0, 0.0};
    const Vec3 moment_body = cross(moment_arm, normal_body);
    const Vec3 iw{in.vehicle.inertia_kg_m2.x * s.omega_body_radps.x, in.vehicle.inertia_kg_m2.y * s.omega_body_radps.y, in.vehicle.inertia_kg_m2.z * s.omega_body_radps.z};
    const Vec3 omega_cross_iw = cross(s.omega_body_radps, iw);
    const Vec3 omega_dot{
        (moment_body.x - omega_cross_iw.x) / in.vehicle.inertia_kg_m2.x,
        (moment_body.y - omega_cross_iw.y) / in.vehicle.inertia_kg_m2.y,
        (moment_body.z - omega_cross_iw.z) / in.vehicle.inertia_kg_m2.z,
    };
    const Quat omega_q{0.0, s.omega_body_radps.x, s.omega_body_radps.y, s.omega_body_radps.z};
    const Quat qdot_raw = multiply(s.attitude_body_to_ned, omega_q);

    return {
        s.velocity_ned_mps,
        force_ned / in.vehicle.mass_kg,
        {0.5 * qdot_raw.w, 0.5 * qdot_raw.x, 0.5 * qdot_raw.y, 0.5 * qdot_raw.z},
        omega_dot,
    };
}

State rk4_step(const SimulationInputs& in, const BarrowmanResult& barrow, State s, double dt) {
    const auto k1 = eval(in, barrow, s);
    const auto k2 = eval(in, barrow, add_scaled(s, k1, dt * 0.5));
    const auto k3 = eval(in, barrow, add_scaled(s, k2, dt * 0.5));
    const auto k4 = eval(in, barrow, add_scaled(s, k3, dt));

    s.position_ned_m += (k1.pos_dot + k2.pos_dot * 2.0 + k3.pos_dot * 2.0 + k4.pos_dot) * (dt / 6.0);
    s.velocity_ned_mps += (k1.vel_dot + k2.vel_dot * 2.0 + k3.vel_dot * 2.0 + k4.vel_dot) * (dt / 6.0);
    s.omega_body_radps += (k1.omega_dot + k2.omega_dot * 2.0 + k3.omega_dot * 2.0 + k4.omega_dot) * (dt / 6.0);
    s.attitude_body_to_ned = normalize({
        s.attitude_body_to_ned.w + (k1.quat_dot.w + 2.0 * k2.quat_dot.w + 2.0 * k3.quat_dot.w + k4.quat_dot.w) * (dt / 6.0),
        s.attitude_body_to_ned.x + (k1.quat_dot.x + 2.0 * k2.quat_dot.x + 2.0 * k3.quat_dot.x + k4.quat_dot.x) * (dt / 6.0),
        s.attitude_body_to_ned.y + (k1.quat_dot.y + 2.0 * k2.quat_dot.y + 2.0 * k3.quat_dot.y + k4.quat_dot.y) * (dt / 6.0),
        s.attitude_body_to_ned.z + (k1.quat_dot.z + 2.0 * k2.quat_dot.z + 2.0 * k3.quat_dot.z + k4.quat_dot.z) * (dt / 6.0),
    });
    s.t_s += dt;
    return s;
}

} // namespace

SimulationResult run_simulation(const SimulationInputs& inputs) {
    SimulationResult result;
    const BarrowmanResult barrow = compute_barrowman(inputs.vehicle);

    State state;
    state.attitude_body_to_ned = initial_attitude(inputs.vehicle.rail_elevation_deg, inputs.vehicle.rail_azimuth_deg);

    for (double t = 0.0; t <= inputs.config.max_time_s; t += inputs.config.dt_s) {
        const double altitude = -state.position_ned_m.z;
        const Vec3 wind = inputs.config.wind_mode == WindMode::Calm ? Vec3{} : inputs.wind.value_at_altitude(altitude);
        result.points.push_back({state, altitude, inputs.thrust.value_at(state.t_s), wind, barrow.cp_from_nose_m});
        if (state.t_s > 0.5 && state.position_ned_m.z >= 0.0 && state.velocity_ned_mps.z > 0.0) {
            result.impacted = true;
            break;
        }
        state = rk4_step(inputs, barrow, state, inputs.config.dt_s);
    }
    return result;
}

} // namespace hrocket
