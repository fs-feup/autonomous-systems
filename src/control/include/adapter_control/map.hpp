#ifndef SRC_CONTROL_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_CONTROL_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "adapter_control/fsds.hpp"

std::map<std::string, std::function<Adapter*(Control*)>> adapter_map = {
    {"fsds", [](Control* control) -> Adapter* { return new FsdsAdapter(control); }},
};

#endif