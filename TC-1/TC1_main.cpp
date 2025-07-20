#include "parameters.hpp"
#include "thrust.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <iomanip>

using namespace std;
namespace fs = std::filesystem;

//=== 定数 ===//
const double R0_m = 6371000.0;        // 地球の半径 [m]
const double G0_mss = 9.80665;        // 重力加速度 [m/s^2]

//=== ODE 関数 ===//
double ode_x(double x, double z, double v, double path_angle_rad) {
    return R0_m / (R0_m + z) * v * cos(path_angle_rad);
}

double ode_z(double x, double z, double v, double path_angle_rad) {
    return v * sin(path_angle_rad);
}

double ode_v(double x, double z, double v, double path_angle_rad, double mass, double thrust, double rho, double Cd, double A) {
    double drag = 0.5 * rho * v * v * Cd * A;
    return (thrust - drag) / mass - G0_mss * sin(path_angle_rad);
}

double ode_path_angle_rad(double x, double z, double v, double path_angle_rad) {
    return -G0_mss * cos(path_angle_rad) / v + v * cos(path_angle_rad) / (R0_m + z);
}

double ode_mass(double thrust, double Isp) {
    return -thrust / (Isp * G0_mss);
}

int main() {
    //=== Load Parameters ===//
    BodyParameters params = loadBodyParameters("parameters.csv");
    ThrustProfile thrust_data = loadThrustProfile("Hypertek_440CC125J-J250.csv");

    //=== Create output directory ===//
    string dir_name = "output";
    if (!fs::exists(dir_name)) {
        fs::create_directory(dir_name);
    }
    ofstream fout(dir_name + "/flight_simulation.csv");
    fout << fixed << setprecision(5);
    fout << "time,x,z,v,angle,mass,thrust\n";

    //=== Initial conditions ===//
    double t = 0.0;
    double h = 0.1;
    double t_max = 100.0;

    double x = 0.0;
    double z = 0.0;
    double v = 100.0;
    double path_angle_deg = 80.0; // 初期のパス角度 [度]
    if (path_angle_deg < 0.0 || path_angle_deg > 90.0) {
        cerr << "Error: Path angle must be between 0 and 90 degrees." << endl;
        return 1;
    }
    double path_angle_rad = path_angle_deg * M_PI / 180.0; // パス角度をラジアンに変換

    //=== Initial mass parameters ===// 

    double m_structure = params.mass - params.fuel_mass;
    double m_fuel = params.fuel_mass;
    double m0 = params.mass;
    double mass = m0;
    double burn_time = params.burn_time;
    double fuel_burn_rate = m_fuel / burn_time;

    const double Cd = params.drag_coefficient;
    const double A = params.drag_area;
    double rho = 1.225; // constant air density

    //=== Simulation loop ===//
    while (t <= t_max) {
        // Update mass
        if (t < burn_time) {
            mass = m0 - fuel_burn_rate * t;
        } else {
            mass = m_structure;
        }

        // Get thrust from profile
        double thrust = getThrustAtTime(thrust_data, t);

        // Output
        fout << t << "," << x << "," << z << "," << v << "," << path_angle_rad << "," << mass << "," << thrust << "\n";

        // Runge-Kutta method
        double k1_x = h * ode_x(x, z, v, path_angle_rad);
        double k1_z = h * ode_z(x, z, v, path_angle_rad);
        double k1_v = h * ode_v(x, z, v, path_angle_rad, mass, thrust, rho, Cd, A);
        double k1_a = h * ode_path_angle_rad(x, z, v, path_angle_rad);
        double k1_m = h * ode_mass(thrust, params.Isp);

        double k2_x = h * ode_x(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle_rad + 0.5 * k1_a);
        double k2_z = h * ode_z(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle_rad + 0.5 * k1_a);
        double k2_v = h * ode_v(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle_rad + 0.5 * k1_a, mass + 0.5 * k1_m, thrust, rho, Cd, A);
        double k2_a = h * ode_path_angle_rad(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle_rad + 0.5 * k1_a);
        double k2_m = h * ode_mass(thrust, params.Isp);

        double k3_x = h * ode_x(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle_rad + 0.5 * k2_a);
        double k3_z = h * ode_z(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle_rad + 0.5 * k2_a);
        double k3_v = h * ode_v(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle_rad + 0.5 * k2_a, mass + 0.5 * k2_m, thrust, rho, Cd, A);
        double k3_a = h * ode_path_angle_rad(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle_rad + 0.5 * k2_a);
        double k3_m = h * ode_mass(thrust, params.Isp);

        double k4_x = h * ode_x(x + k3_x, z + k3_z, v + k3_v, path_angle_rad + k3_a);
        double k4_z = h * ode_z(x + k3_x, z + k3_z, v + k3_v, path_angle_rad + k3_a);
        double k4_v = h * ode_v(x + k3_x, z + k3_z, v + k3_v, path_angle_rad + k3_a, mass + k3_m, thrust, rho, Cd, A);
        double k4_a = h * ode_path_angle_rad(x + k3_x, z + k3_z, v + k3_v, path_angle_rad + k3_a);
        double k4_m = h * ode_mass(thrust, params.Isp);

        // 状態更新
        x += (k1_x + 2*k2_x + 2*k3_x + k4_x) / 6.0;
        z += (k1_z + 2*k2_z + 2*k3_z + k4_z) / 6.0;
        v += (k1_v + 2*k2_v + 2*k3_v + k4_v) / 6.0;
        path_angle_rad += (k1_a + 2*k2_a + 2*k3_a + k4_a) / 6.0;
        mass += (k1_m + 2*k2_m + 2*k3_m + k4_m) / 6.0;

        // 着地判定
        if (z < 0.0) {
            cout << "Rocket has landed." << endl;
            break;
        }

        t += h;
    }

    fout.close();
    cout << "Simulation complete. Results written to output/flight_simulation.csv" << endl;
    return 0;
}