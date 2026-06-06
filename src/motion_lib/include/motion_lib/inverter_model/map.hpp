#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "linear_inverter.hpp"
#include "quadratic_inverter.hpp"

/*
 * Map of inverter models, with the key being the model name and the value
 * being a lambda function that returns a shared pointer to the corresponding model.
 */
const std::map<std::string,
               std::function<std::shared_ptr<InverterModel>(
                   const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
               std::less<>>
    inverter_models_map = {
        {"linear",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<InverterModel> { return std::make_shared<LinearInverter>(*params); }},
        {"quadratic",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<InverterModel> {
           return std::make_shared<QuadraticInverter>(*params);
         }},
};
