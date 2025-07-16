#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "filters/filter.hpp"
#include "lateral_controller/lateral_controller.hpp"
#include "node_/control_parameters.hpp"
#include "pure_pursuit/pp.hpp"
#include "stanley/stanley.hpp"

/*
 * Map of lateral controllers, with the key being the controller type and the value being a lambda
 * function that creates the controller
 */
const std::map<std::string,
               std::function<std::shared_ptr<LateralController>(std::shared_ptr<Filter>,
                                                                const ControlParameters&)>,
               std::less<>>
    lateral_controller_map = {
        {"pure_pursuit",
         [](std::shared_ptr<Filter> lpf,
            const ControlParameters& params) -> std::shared_ptr<LateralController> {
           return std::make_shared<PurePursuit>(lpf, params);
         }},
        {"stanley",
         [](std::shared_ptr<Filter> lpf,
            const ControlParameters& params) -> std::shared_ptr<LateralController> {
           return std::make_shared<Stanley>(lpf, params);
         }}};