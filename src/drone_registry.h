#ifndef DRONE_REGISTRY_H
#define DRONE_REGISTRY_H

#include <string>
#include <unordered_map>

namespace DroneRegistry {

constexpr auto drone_one = "cf_0";
constexpr auto drone_two = "cf_1";

std::unordered_map<std::string, unsigned int> DroneIdToPort = {
    {drone_one, 3995}, {drone_two, 3996}};

} // namespace DroneRegistry

#endif