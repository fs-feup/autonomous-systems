#ifndef SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "adapter/eufs.hpp"
#include "adapter/fsds.hpp"
#include "adapter/pacsim.hpp"

std::map<std::string, std::function<Adapter*(Planning*)>> adapter_map = {
    {"fsds", [](Planning* planning) -> Adapter* { return new FsdsAdapter(planning); }},
    {"eufs", [](Planning* planning) -> Adapter* { return new EufsAdapter(planning); }},
    {"pacsim", [](Planning* planning) -> Adapter* { return new PacSimAdapter(planning); }}};

#endif