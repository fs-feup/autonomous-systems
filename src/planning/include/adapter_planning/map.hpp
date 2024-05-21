#ifndef SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_
#define SRC_PLANNING_INCLUDE_ADAPTER_MAP_HPP_

#include <map>
#include <string>

#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"

const std::map<std::string, std::function<std::shared_ptr<Adapter>(std::shared_ptr<Planning>)>,
               std::less<>>
    adapter_map = {{"fsds",
                    [](std::shared_ptr<Planning> planning) -> std::shared_ptr<Adapter> {
                      return std::make_shared<FsdsAdapter>(planning);
                    }},
                   {"eufs",
                    [](std::shared_ptr<Planning> planning) -> std::shared_ptr<Adapter> {
                      return std::make_shared<EufsAdapter>(planning);
                    }},
                   {"pacsim",
                    [](std::shared_ptr<Planning> planning) -> std::shared_ptr<Adapter> {
                      return std::make_shared<PacSimAdapter>(planning);
                    }},
                   {"vehicle", [](std::shared_ptr<Planning> planning) -> std::shared_ptr<Adapter> {
                      return std::make_shared<VehicleAdapter>(planning);
                    }}};

#endif