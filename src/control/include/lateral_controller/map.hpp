#pragma once

#include <map>
#include <memory>
#include <string>

#include "pure_pursuit.hpp"

/*
 * Map of lateral control solvers, with the key being the type of the solver and the value being a lambda
 * function that creates the solver
 */
const std::map<std::string, std::function<std::shared_ptr<LateralController>(const ControlParameters&)>,
               std::less<>>
    lateral_controller_map = {
        {"pure_pursuit",
         [](const ControlParameters& params) -> std::shared_ptr<LateralController> {
           return std::make_shared<PurePursuit>(params);
         }},
    };
