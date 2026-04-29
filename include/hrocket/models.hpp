#pragma once

#include "hrocket/math.hpp"

#include <string>
#include <vector>

namespace hrocket {

struct VehicleModel {
    double mass_kg{10.0};
    Vec3 inertia_kg_m2{1.0, 1.0, 0.1};
    double reference_area_m2{0.01};
    double diameter_m{0.1};
    double length_m{1.5};
    double cg_from_nose_m{0.75};
    double cn_alpha_per_rad{12.0};
    double cd{0.6};
    double launch_lat_deg{35.0};
    double launch_lon_deg{139.0};
    double launch_alt_m{0.0};
    double rail_elevation_deg{85.0};
    double rail_azimuth_deg{0.0};
    double nose_length_m{0.25};
    double fin_count{4.0};
    double fin_root_chord_m{0.18};
    double fin_tip_chord_m{0.08};
    double fin_span_m{0.08};
    double fin_sweep_m{0.04};
    double fin_distance_from_nose_m{1.15};
    double parachute_area_m2{1.0};
    double parachute_cd{1.5};
    double parachute_deploy_altitude_m{300.0};
    double control_enabled{0.0};
    double control_fin_count{2.0};
    double control_tail_area_m2{0.003};
    double control_tail_lift_slope_per_rad{6.0};
    double control_tail_distance_from_nose_m{1.35};
    double control_max_deflection_deg{15.0};
    double control_min_speed_mps{20.0};
    double control_roll_target_deg{0.0};
    double control_pitch_target_deg{0.0};
    double control_yaw_target_deg{0.0};
    double control_roll_kp{0.4};
    double control_roll_kd{0.08};
    double control_pitch_kp{0.4};
    double control_pitch_kd{0.08};
    double control_yaw_kp{0.4};
    double control_yaw_kd{0.08};
};

struct TimeSample {
    double t{};
    double value{};
};

struct WindSample {
    double altitude_m{};
    Vec3 wind_ned_mps{};
};

struct ThrustCurve {
    std::vector<TimeSample> samples;
    double value_at(double t) const;
};

struct WindProfile {
    std::vector<WindSample> samples;
    Vec3 value_at_altitude(double altitude_m) const;
};

enum class DescentMode {
    FreeFall,
    Parachute,
};

enum class WindMode {
    Nominal,
    Calm,
};

struct SimulationConfig {
    double dt_s{0.01};
    double max_time_s{180.0};
    double rail_length_m{3.0};
    DescentMode descent_mode{DescentMode::FreeFall};
    WindMode wind_mode{WindMode::Nominal};
};

struct State {
    double t_s{};
    Vec3 position_ned_m{};
    Vec3 velocity_ned_mps{};
    Quat attitude_body_to_ned{};
    Vec3 omega_body_radps{};
};

struct TrajectoryPoint {
    State state;
    double altitude_m{};
    double thrust_n{};
    Vec3 wind_ned_mps{};
    double cp_from_nose_m{};
    double control_roll_deflection_deg{};
    double control_pitch_deflection_deg{};
    double control_yaw_deflection_deg{};
    Vec3 control_moment_body_nm{};
};

struct SimulationResult {
    std::vector<TrajectoryPoint> points;
    bool impacted{false};
};

struct SimulationInputs {
    VehicleModel vehicle;
    ThrustCurve thrust;
    WindProfile wind;
    SimulationConfig config;
};

enum class DispersionMode {
    MonteCarlo,
    WindSweep,
};

struct DispersionConfig {
    DispersionMode mode{DispersionMode::MonteCarlo};
    int runs{1};
    double wind_sigma_mps{0.0};
    unsigned seed{1};
    double sweep_max_wind_mps{7.0};
    double sweep_step_mps{1.0};
    int sweep_directions{16};
};

class IActuator {
public:
    virtual ~IActuator() = default;
    virtual Vec3 force_body_n(const State& state) = 0;
    virtual Vec3 moment_body_nm(const State& state) = 0;
};

class IController {
public:
    virtual ~IController() = default;
    virtual void update(double dt_s, const State& state) = 0;
};

} // namespace hrocket
