#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_lib/v2p_models/base_v2p_motion_model.hpp"
#include "motion_lib/v2p_models/constant_acceleration_turnrate_model.hpp"
#include "motion_lib/v2p_models/constant_velocity_model.hpp"
#include "motion_lib/v2p_models/constant_velocity_turnrate_model.hpp"

/*
 * Map of motion models, with the key being the type of the motion model and the value being a
 * lambda function that returns a shared pointer to the corresponding motion model
 */
const std::map<std::string, std::function<std::shared_ptr<V2PMotionModel>()>, std::less<>>
    v2p_models_map = {
        {"constant_velocity",
         []() -> std::shared_ptr<V2PMotionModel> {
           return std::make_shared<ConstantVelocityModel>();
         }},
        {"constant_velocity_turnrate",
         []() -> std::shared_ptr<V2PMotionModel> {
           return std::make_shared<ConstantVelocityTurnrateModel>();
         }},
        {"constant_acceleration_turnrate",
         []() -> std::shared_ptr<V2PMotionModel> {
           return std::make_shared<ConstantAccelerationTurnrateModel>();
         }},
};
