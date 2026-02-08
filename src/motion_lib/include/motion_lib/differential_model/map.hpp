#pragma once

#include <map>
#include <memory>
#include <string>

#include "viscous_lsd.hpp"

/*
 * Map of differential models, with the key being the name of the differential model and the value
 * being a lambda function that returns a shared pointer to the corresponding differential model
 */
const std::map<std::string,
               std::function<std::shared_ptr<DifferentialModel>(
                   const common_lib::car_parameters::CarParameters&)>,
               std::less<>>
    differential_models_map = {
        {"viscous_lsd",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<DifferentialModel> {
           return std::make_shared<ViscousLSD>(params);
         }},
};