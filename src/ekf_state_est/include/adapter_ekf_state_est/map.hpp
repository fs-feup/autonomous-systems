#pragma once

#include <map>
#include <string>

#include "adapter_ekf_state_est/eufs.hpp"
#include "adapter_ekf_state_est/fsds.hpp"

std::map<std::string, std::function<Adapter*(SENode*)>> adapter_map = {
    {"fsds", [](SENode* speed_est) -> Adapter* { return new FsdsAdapter(speed_est); }},
    {"eufs", [](SENode* speed_est) -> Adapter* { return new EufsAdapter(speed_est); }},
};
