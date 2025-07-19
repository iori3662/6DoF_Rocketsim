#ifndef THRUST_HPP
#define THRUST_HPP

#include <vector>
#include <utility>
#include <string>

using ThrustProfile = std::vector<std::pair<double, double>>; // {time, thrust}

ThrustProfile loadThrustProfile(const std::string& filename);
double getThrustAtTime(const ThrustProfile& profile, double time);

#endif
