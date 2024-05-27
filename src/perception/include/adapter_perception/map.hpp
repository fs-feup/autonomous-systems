#pragma once

#include <map>
#include <string>
#include <functional>

#include "adapter_perception/fsds.hpp"
#include "adapter_perception/eufs.hpp"
#include "adapter_perception/vehicle.hpp"

/**
 * @brief Map of adapter names to functions creating instances of corresponding adapters.
 * 
 * This map allows dynamically selecting and creating instances of adapter classes based on
 * their names. Each adapter class has an associated creation function, which is stored in
 * the map.
 */
std::map<std::string, std::function<Adapter*(Perception*)>> adapter_map = {
    // Mapping adapter names to functions creating instances of corresponding adapters.
    {"fsds", [](Perception* perception) -> Adapter* { return new FsdsAdapter(perception); }},
    {"vehicle", [](Perception* perception) -> Adapter* { return new VehicleAdapter(perception); }},
    {"eufs", [](Perception* perception) -> Adapter* { return new EufsAdapter(perception); }},
};

