#ifndef SRC_LONG_CONTROL_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_LONG_CONTROL_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "src/long_control/include/adapter/fsds.hpp"

std::map<std::string, std::function<Adapter*(LongitudinalControl*)>> adapter_map = {
    {"fsds",
     [](LongitudinalControl* long_control) -> Adapter* { return new FsdsAdapter(long_control); }},
};

#endif