#pragma once

#include <map>
#include <memory>
#include <string>

#include "adapter_slam/pacsim.hpp"
#include "adapter_slam/vehicle.hpp"
#include "ros_node/slam_node.hpp"

/*
 * Map of adapters, with the key being the type of the adapter and the value being a lambda function
 * that creates the adapter
 */
const std::map<std::string, std::function<std::shared_ptr<SLAMNode>(const SLAMParameters&)>,
               std::less<>>
    adapter_map = {{"pacsim",
                    [](const SLAMParameters& params) -> std::shared_ptr<SLAMNode> {
                      return std::make_shared<PacsimAdapter>(params);
                    }},
                   {"vehicle", [](const SLAMParameters& params) -> std::shared_ptr<SLAMNode> {
                      return std::make_shared<VehicleAdapter>(params);
                    }}};
