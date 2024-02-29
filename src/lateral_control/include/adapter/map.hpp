#ifndef SRC_LATERAL_CONTROL_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_LATERAL_CONTROL_INCLUDE_ADAPTER_MAP_HPP_

#include <string>
#include <map>
#include "adapter/fsds.hpp"

std::map<std::string, std::function<Adapter*(LateralControl*)>> adapter_map = {
    {"fsds", [](LateralControl* lat_control) -> Adapter* { return new FsdsAdapter(lat_control); } },
};

#endif