#pragma once

#include <map>
#include <memory>
#include <string>

#include "default_aero_model.hpp"

/*
 * Map of aero models, with the key being the name of the aero model and the value being a lambda
 * function that returns a shared pointer to the corresponding aero model
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<AeroModel>(const common_lib::car_parameters::CarParameters&)>,
    std::less<>>
    aero_models_map = {
        {"default_aero",
         [](const common_lib::car_parameters::CarParameters& params) -> std::shared_ptr<AeroModel> {
           return std::make_shared<DefaultAeroModel>(params);
         }},
};
