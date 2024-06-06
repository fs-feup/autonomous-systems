#pragma once

#include <map>
#include <string>

#include "adapter_control/eufs.hpp"
#include "adapter_control/fsds.hpp"
#include "adapter_control/pac_sim.hpp"
#include "adapter_control/vehicle.hpp"

const std::map<std::string, std::function<std::shared_ptr<Control>()>, std::less<>> adapter_map = {
    {"fsds", []() { return std::make_shared<FsdsAdapter>(); }},
    {"pacsim", []() { return std::make_shared<PacSimAdapter>(); }},
    {"eufs", []() { return std::make_shared<EufsAdapter>(); }},
    {"vehicle", []() { return std::make_shared<VehicleAdapter>(); }}};
