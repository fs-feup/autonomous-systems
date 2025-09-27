#pragma once

#include <map>
#include <memory>
#include <string>

#include "adapter/eufs.hpp"
#include "adapter/pacsim.hpp"
#include "adapter/vehicle.hpp"
#include "ros_node/ros_node.hpp"

/*
 * Map of adapters, with the key being the type of the adapter and the value being a lambda function
 * that creates the adapter
 */
const std::map<std::string, std::function<std::shared_ptr<ControlNode>(const ControlParameters&)>,
               std::less<>>
    adapter_map = {{"pacsim",
                    [](const ControlParameters& params) -> std::shared_ptr<ControlNode> {
                      return std::make_shared<PacSimAdapter>(params);
                    }},
                   {"vehicle",
                    [](const ControlParameters& params) -> std::shared_ptr<ControlNode> {
                      return std::make_shared<VehicleAdapter>(params);
                    }},
                   {"eufs", [](const ControlParameters& params) -> std::shared_ptr<ControlNode> {
                      return std::make_shared<EufsAdapter>(params);
                    }}};
