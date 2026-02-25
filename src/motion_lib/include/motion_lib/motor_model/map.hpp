#pragma once

#include <map>
#include <memory>
#include <string>

#include "map_based_motor.hpp"

/*
 * Map of motor models, with the key being the name of the motor model and the value being
 * a lambda function that returns a shared pointer to the corresponding motor model
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<MotorModel>(const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
    std::less<>>
    motor_models_map = {
        {"map_based_motor",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<MotorModel> { return std::make_shared<MapBasedMotor>(params); }},
};