#pragma once

#include <map>
#include <string>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/pac_sim.hpp"
#include "adapter_control/vehicle.hpp"

const std::map<std::string, std::function<std::shared_ptr<Adapter>(Control*)>, std::less<>>
    adapter_map = {
        {"fsds", [](Control* control) { return std::make_shared<FsdsAdapter>(control); }},
        {"pacsim", [](Control* control) { return std::make_shared<PacSimAdapter>(control); }},
        {"eufs", [](Control* control) { return std::make_shared<EufsAdapter>(control); }},
        {"vehicle", [](Control* control) { return std::make_shared<VehicleAdapter>(control); }}};

