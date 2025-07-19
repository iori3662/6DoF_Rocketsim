#include "thrust.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

ThrustProfile loadThrustProfile(const std::string& filename) {
    ThrustProfile profile;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open thrust profile file." << std::endl;
        return profile;
    }

    getline(file, line); // skip header
    while (getline(file, line)) {
        std::stringstream ss(line);
        std::string t_str, thrust_str;
        getline(ss, t_str, ',');
        getline(ss, thrust_str, ',');
        profile.emplace_back(std::stod(t_str), std::stod(thrust_str));
    }

    return profile;
}

double getThrustAtTime(const ThrustProfile& profile, double time) {
    if (profile.empty()) return 0.0;

    for (size_t i = 1; i < profile.size(); ++i) {
        if (time < profile[i].first) {
            double t1 = profile[i - 1].first;
            double t2 = profile[i].first;
            double f1 = profile[i - 1].second;
            double f2 = profile[i].second;
            return f1 + (f2 - f1) * (time - t1) / (t2 - t1); // 線形補間
        }
    }

    return profile.back().second; // 時間が最後より後なら末尾値を返す
}
