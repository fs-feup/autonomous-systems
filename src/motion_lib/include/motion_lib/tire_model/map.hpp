#pragma once

#include <map>
#include <memory>
#include <string>

#include "pacejka_combined_slip.hpp"

/*
 * Map of tire models, with the key being the name of the tire model and the value being a lambda
 * function that returns a shared pointer to the corresponding tire model
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<TireModel>(const common_lib::car_parameters::CarParameters&)>,
    std::less<>>
    tire_models_map = {
        {"pacejka_combined_slip",
         [](const common_lib::car_parameters::CarParameters& params) -> std::shared_ptr<TireModel> {
           return std::make_shared<PacejkaCombinedSlip>(params);
         }},
};
