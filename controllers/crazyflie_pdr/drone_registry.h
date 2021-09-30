#ifndef DRONE_REGISTRY_H
#define DRONE_REGISTRY_H

#include <unordered_map>
#include <string>

namespace DroneRegistry {

    std::unordered_map<std::string, unsigned int> DroneIdToPort = { {"cf_0", 3995}, {"cf_1", 3996}};

}

#endif