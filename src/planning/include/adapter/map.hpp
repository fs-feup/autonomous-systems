#ifndef SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "adapter/eufs.hpp"
#include "adapter/fsds.hpp"

std::map<std::string, std::function<Adapter*(Planning*)>> adapter_map = {
    {"fsds", [](Planning* planning) -> Adapter* { return new FsdsAdapter(planning); }},
    {"eufs", [](Planning* planning) -> Adapter* { return new EufsAdapter(planning); }}
};

#endif