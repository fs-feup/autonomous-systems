#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "default_brake.hpp"

/*
 * Map of brake models, with the key being the model name and the value
 * being a lambda function that returns a shared pointer to the corresponding model.
 */
const std::map<std::string,
               std::function<std::shared_ptr<BrakeModel>(
                   const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
               std::less<>>
    brake_models_map = {
        {"default",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<BrakeModel> { return std::make_shared<DefaultBrake>(*params); }},
};
