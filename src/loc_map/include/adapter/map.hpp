#ifndef SRC_LOC_MAP_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_LOC_MAP_INCLUDE_ADAPTER_MAP_HPP_

#include <string>
#include <map>
#include "adapter/fsds.hpp"
#include "adapter/eufs.hpp"

std::map<std::string, std::function<Adapter*(LMNode*)>> adapter_map = {
    {"fsds", [](LMNode* loc_map) -> Adapter* { return new FsdsAdapter(loc_map); } },
    {"eufs", [](LMNode* loc_map) -> Adapter* { return new EufsAdapter(loc_map); } },
};

#endif