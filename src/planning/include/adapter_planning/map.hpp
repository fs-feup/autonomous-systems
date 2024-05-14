#ifndef SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/pacsim.hpp"

std::map<std::string, std::function<Adapter*(Planning*)>> adapter_map = {
    {"fsds", [](Planning* planning) -> Adapter* { return new FsdsAdapter(planning); }},
    {"eufs", [](Planning* planning) -> Adapter* { return new EufsAdapter(planning); }},
    {"pacsim", [](Planning* planning) -> Adapter* { return new PacSimAdapter(planning); }}};

#endif