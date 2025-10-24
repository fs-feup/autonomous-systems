#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_lib/steering_motor_model/pid_steering_motor.hpp"

/*
 * Map of steering motor models, with the key being the name of the steering motor model and the
 * value being a lambda function that returns a shared pointer to the corresponding steering motor
 * model
 */
const std::map<std::string,
               std::function<std::shared_ptr<SteeringMotorModel>(
                   const common_lib::car_parameters::CarParameters&)>,
               std::less<>>
    steering_motor_models_map = {
        {"pid",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<SteeringMotorModel> {
           return std::make_shared<PIDSteeringMotor>(params);
         }},
};
