#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "adapter_perception/fsds.hpp"
#include "adapter_perception/testlidar.hpp"

std::map<std::string, std::function<Adapter*(Perception*)>> adapter_map = {
    {"fsds", [](Perception* perception) -> Adapter* { return new FsdsAdapter(perception); }},
    {"test", [](Perception* perception) -> Adapter* { return new TestAdapter(perception); }},
};

#endif