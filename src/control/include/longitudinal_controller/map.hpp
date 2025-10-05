#pragma once

#include <map>
#include <memory>
#include <string>

#include "base_longitudinal_controller.hpp"

/*
 * Map of longitudinal control solvers, with the key being the type of the solver and the value being a lambda
 * function that creates the solver
 */
const std::map<std::string, std::function<std::shared_ptr<LongitudinalController>(const ControlParameters&)>,
               std::less<>>
    longitudinal_controller_map = {};
