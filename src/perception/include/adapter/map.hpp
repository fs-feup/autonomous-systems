#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_MAP_HPP_

#include <string>
#include <map>
#include "adapter/fsds.hpp"
#include "adapter/testlidar.hpp"

std::map<std::string, std::function<Adapter*(Perception*)>> adapter_map = {
    {"fsds", [](Perception* perception) -> Adapter* { return new FsdsAdapter(perception); } },
    {"test", [](Perception* perception) -> Adapter* { return new TestAdapter(perception); } },
};

#endif