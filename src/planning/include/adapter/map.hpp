#ifndef SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_

#include <string>
#include <map>
#include "adapter/fsds.hpp"
#include "adapter/eufs.hpp"
#include "adapter/adsdv.hpp"

std::map<std::string, std::function<Adapter*(Planning*)>> adapter_map = {
    {"fsds", [](Planning* planning) -> Adapter* { return new FsdsAdapter(planning); } },
    {"eufs", [](Planning* planning) -> Adapter* { return new EufsAdapter(planning); } },
    {"adsdv", [](Planning* planning) -> Adapter* { return new AdsdvAdapter(planning); } },
};

#endif