#pragma once

#include <map>
#include <memory>
#include <string>

#include "parallel_front_steering.hpp"

/*
 * Map of steering models, with the key being the name of the steering model and the value being a
 * lambda function that returns a shared pointer to the corresponding steering model
 */
const std::map<std::string,
               std::function<std::shared_ptr<SteeringModel>(
                   const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
               std::less<>>
    steering_models_map = {
        {"parallel_front_steering",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<SteeringModel> {
           return std::make_shared<ParallelFrontSteering>(*params);
         }},
};
