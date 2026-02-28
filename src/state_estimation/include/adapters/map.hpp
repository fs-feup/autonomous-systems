#pragma once

#include <map>
#include <memory>
#include <string>

#include "adapters/pacsim.hpp"
#include "adapters/vehicle.hpp"

/*
 * Map of state estimators, with the key being the type of the state estimator and the value being a
 * lambda function that creates the state estimator
 */
const std::map<std::string, std::function<std::shared_ptr<SENode>(SEParameters)>, std::less<>>
    adapter_map = {{"pacsim", [](const SEParameters& params) -> std::shared_ptr<SENode> {
                      return std::make_shared<PacsimAdapter>(params);
                    }}};