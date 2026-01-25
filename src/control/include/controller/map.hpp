#pragma once

#include <map>
#include <memory>
#include <string>

#include "base_decoupled_controller.hpp"
#include "mpc.hpp"

/*
 * Map of control solvers, with the key being the type of the solver and the value being a lambda
 * function that creates the solver
 */
const std::map<std::string, std::function<std::shared_ptr<Controller>(const ControlParameters&)>,
               std::less<>>
    controller_map = {{"decoupled",
                    [](const ControlParameters& params) -> std::shared_ptr<Controller> {
                      return std::make_shared<DecoupledController>(params);
                    }},
                    {"mpc",
                    [](const ControlParameters& params) -> std::shared_ptr<Controller> {
                      return std::make_shared<MPC>(params);
                    }}};
