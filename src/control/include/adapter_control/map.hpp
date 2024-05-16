#pragma once

#include <map>
#include <string>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/pac_sim.hpp"
#include "adapter_control/vehicle.hpp"

std::map<std::string, std::function<Adapter*(Control*)>> adapter_map = {
    {"fsds", [](Control* control) -> Adapter* { return new FsdsAdapter(control); }},
    {"pacsim", [](Control* control) -> Adapter* { return new PacSimAdapter(control); }},
    {"eufs", [](Control* control) -> Adapter* { return new EufsAdapter(control); }},
    {"vehicle", [](Control* control) -> Adapter* { return new VehicleAdapter(control); }}};
