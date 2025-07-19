#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>

using namespace std;

// 定数定義
const double R0_m = 6371000.0;        // 地球の半径 [m]
const double G0_mss = 9.80665;        // 重力加速度 [m/s^2]
const double Cd = 0.5;                // 空気抵抗係数（仮定）
const double A_m2 = 0.01;             // 断面積 [m^2]（仮定）
const double g_mss = G0_mss;

// ODEの関数定義
double ode_x(double x, double z, double v, double path_angle) {
    return R0_m / (R0_m + z) * v * cos(path_angle);
}

double ode_z(double x, double z, double v, double path_angle) {
    return v * sin(path_angle);
}

double ode_v(double x, double z, double v, double path_angle, double mass, double thrust, double Isp, double rho) {
    double drag = 0.5 * rho * v * v * Cd * A_m2;
    return (thrust - drag) / mass - g_mss * sin(path_angle);
}

double ode_path_angle(double x, double z, double v, double path_angle) {
    return -g_mss * cos(path_angle) / v + v * cos(path_angle) / (R0_m + z);
}

double ode_mass(double thrust, double Isp) {
    return -thrust / (Isp * G0_mss);
}

int main() {
    // 初期値設定
    double x = 0.0;
    double z = 0.0;
    double v = 100.0;                  // 初速度 [m/s]
    double path_angle = M_PI / 4.0;   // 発射角 [rad]
    double mass = 50.0;               // 初期質量 [kg]
    double thrust = 1000.0;           // 推力 [N]
    double Isp = 250.0;               // 比推力 [s]
    double rho = 1.225;               // 空気密度 [kg/m^3]（定数と仮定）

    double h = 0.1;                   // 時間刻み [s]
    double t_max = 100.0;              // シミュレーション時間 [s]

    // 出力フォルダとファイル作成
    string dir_name = "output";
    if (!filesystem::exists(dir_name)) {
        filesystem::create_directory(dir_name);
    }
    ofstream file(dir_name + "/result.csv");

    if (!file.is_open()) {
        cerr << "ファイルを開けませんでした。" << endl;
        return 1;
    }

    // ヘッダ出力
    file << "time,x,z,v,angle,mass\n";

    // シミュレーションループ
    for (double t = 0.0; t <= t_max; t += h) {
        // 出力
        file << t << "," << x << "," << z << "," << v << "," << path_angle << "," << mass << "\n";

        // ルンゲクッタ4次法
        double k1_x = h * ode_x(x, z, v, path_angle);
        double k1_z = h * ode_z(x, z, v, path_angle);
        double k1_v = h * ode_v(x, z, v, path_angle, mass, thrust, Isp, rho);
        double k1_a = h * ode_path_angle(x, z, v, path_angle);
        double k1_m = h * ode_mass(thrust, Isp);

        double k2_x = h * ode_x(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle + 0.5 * k1_a);
        double k2_z = h * ode_z(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle + 0.5 * k1_a);
        double k2_v = h * ode_v(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle + 0.5 * k1_a, mass + 0.5 * k1_m, thrust, Isp, rho);
        double k2_a = h * ode_path_angle(x + 0.5 * k1_x, z + 0.5 * k1_z, v + 0.5 * k1_v, path_angle + 0.5 * k1_a);
        double k2_m = h * ode_mass(thrust, Isp);

        double k3_x = h * ode_x(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle + 0.5 * k2_a);
        double k3_z = h * ode_z(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle + 0.5 * k2_a);
        double k3_v = h * ode_v(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle + 0.5 * k2_a, mass + 0.5 * k2_m, thrust, Isp, rho);
        double k3_a = h * ode_path_angle(x + 0.5 * k2_x, z + 0.5 * k2_z, v + 0.5 * k2_v, path_angle + 0.5 * k2_a);
        double k3_m = h * ode_mass(thrust, Isp);

        double k4_x = h * ode_x(x + k3_x, z + k3_z, v + k3_v, path_angle + k3_a);
        double k4_z = h * ode_z(x + k3_x, z + k3_z, v + k3_v, path_angle + k3_a);
        double k4_v = h * ode_v(x + k3_x, z + k3_z, v + k3_v, path_angle + k3_a, mass + k3_m, thrust, Isp, rho);
        double k4_a = h * ode_path_angle(x + k3_x, z + k3_z, v + k3_v, path_angle + k3_a);
        double k4_m = h * ode_mass(thrust, Isp);

        // 状態更新
        x += (k1_x + 2*k2_x + 2*k3_x + k4_x) / 6.0;
        z += (k1_z + 2*k2_z + 2*k3_z + k4_z) / 6.0;
        v += (k1_v + 2*k2_v + 2*k3_v + k4_v) / 6.0;
        path_angle += (k1_a + 2*k2_a + 2*k3_a + k4_a) / 6.0;
        mass += (k1_m + 2*k2_m + 2*k3_m + k4_m) / 6.0;

        // 着地判定
        if (z < 0) {
            break; // 地面に到達したらループを抜ける
            cout << "Rocket has landed." << endl;
        }
    }

    file.close();
    cout << "Simulation complete. Results written to output/flight_simulation.csv" << endl;
    return 0;
}
