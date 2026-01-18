#pragma once

#include <map>
#include <memory>
#include <string>

#include "no_slip_bicycle_model.hpp"

/*
 * Map of Velocity estimation observation models, with the key being the name of the model and the
 * value being a lambda function that returns a shared pointer to the corresponding observation
 * model.
 */
const std::map<std::string,
               std::function<std::shared_ptr<VEObservationModel>(
                   const common_lib::car_parameters::CarParameters&)>,
               std::less<>>
    ve_observation_models_map = {
        {"no_slip_bicycle_model",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<VEObservationModel> {
           return std::make_shared<NoSlipBicycleModel>(params);
         }},
};