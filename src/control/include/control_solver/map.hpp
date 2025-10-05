#pragma once

#include <map>
#include <memory>
#include <string>

#include "base_decoupled_controller.hpp"

/*
 * Map of control solvers, with the key being the type of the solver and the value being a lambda
 * function that creates the solver
 */
const std::map<std::string, std::function<std::shared_ptr<ControlSolver>(const ControlParameters&)>,
               std::less<>>
    controller_map = {{"decoupled",
                    [](const ControlParameters& params) -> std::shared_ptr<ControlSolver> {
                      return std::make_shared<DecoupledController>(params);
                    }}};
