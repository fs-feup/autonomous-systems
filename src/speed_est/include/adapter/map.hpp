#ifndef SRC_SPEED_EST_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_SPEED_EST_INCLUDE_ADAPTER_MAP_HPP_

#include <string>
#include <map>
#include "adapter/fsds.hpp"
#include "adapter/eufs.hpp"

std::map<std::string, std::function<Adapter*(SENode*)>> adapter_map = {
    {"fsds", [](SENode* speed_est) -> Adapter* { return new FsdsAdapter(speed_est); } },
    {"eufs", [](SENode* speed_est) -> Adapter* { return new EufsAdapter(speed_est); } },
};

#endif