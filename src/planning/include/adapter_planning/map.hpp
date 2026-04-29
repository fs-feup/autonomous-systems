#pragma once

#include <map>
#include <memory>
#include <string>

#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/invictasim.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"

/*
 * Map of adapters, with the key being the type of the adapter and the value being a lambda function
 * that creates the adapter
 */
const std::map<std::string, std::function<std::shared_ptr<Planning>(const PlanningParameters&)>,
               std::less<>>
    adapter_map = {{"pacsim",
                    [](const PlanningParameters& params) -> std::shared_ptr<Planning> {
                      return std::make_shared<PacSimAdapter>(params);
                    }},
                   {"vehicle",
                    [](const PlanningParameters& params) -> std::shared_ptr<Planning> {
                      return std::make_shared<VehicleAdapter>(params);
                    }},
                   {"invictasim",
                    [](const PlanningParameters& params) -> std::shared_ptr<Planning> {
                      return std::make_shared<InvictaSimAdapter>(params);
                    }},
                   {"eufs",
                    [](const PlanningParameters& params) -> std::shared_ptr<Planning> {
                      return std::make_shared<EufsAdapter>(params);
                    }},
                   {"fsds", [](const PlanningParameters& params) -> std::shared_ptr<Planning> {
                      return std::make_shared<FsdsAdapter>(params);
                    }}};
