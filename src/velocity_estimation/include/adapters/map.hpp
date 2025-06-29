#pragma once

#include <map>
#include <memory>
#include <string>

#include "adapters/pacsim_adapter.hpp"
#include "adapters/vehicle_adapter.hpp"

/*
 * Map of adapters, with the key being the type of the adapter and the value being a lambda function
 * that creates the adapter
 */
const std::map<std::string, std::function<std::shared_ptr<VENode>(const VEParameters&)>,
               std::less<>>
    adapter_map = {{"pacsim",
                    [](const VEParameters& params) -> std::shared_ptr<VENode> {
                      return std::make_shared<PacsimAdapter>(params);
                    }},
                   {"vehicle", [](const VEParameters& params) -> std::shared_ptr<VENode> {
                      return std::make_shared<VehicleAdapter>(params);
                    }}};
