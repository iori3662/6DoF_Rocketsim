#include "parameters.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>

BodyParameters loadBodyParameters(const std::string& filename) {
    BodyParameters params;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open parameter file: " << filename << std::endl;
        return params;
    }

    std::unordered_map<std::string, double> param_map;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string key, value_str;
        if (std::getline(ss, key, ',') && std::getline(ss, value_str)) {
            try {
                double value = std::stod(value_str);
                param_map[key] = value;
            } catch (const std::exception& e) {
                std::cerr << "Invalid value for " << key << ": " << value_str << std::endl;
            }
        }
    }

    // 各パラメータに代入
    params.mass = param_map["mass"];
    params.fuel_mass = param_map["fuel_mass"];
    params.burn_time = param_map["burn_time"];
    params.drag_coefficient = param_map["drag_coefficient"];
    params.drag_area = param_map["drag_area"];
    params.Isp = param_map["Isp"];

    return params;
}
