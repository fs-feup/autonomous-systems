#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_lib/vel_process_model/base_vel_process_model.hpp"
#include "motion_lib/vel_process_model/kinematic_bicycle.hpp"
#include "motion_lib/vel_process_model/particle_model.hpp"

/*
 * Map velocity process models, with the key being the type of the process model and the value being
 * a lambda function that returns a shared pointer to the corresponding process model
 */
const std::map<std::string,
               std::function<std::shared_ptr<BaseVelocityProcessModel>(
                   const common_lib::car_parameters::CarParameters&)>,
               std::less<>>
    vel_process_models_map = {
        {"ca_particle_model",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<BaseVelocityProcessModel> {
           return std::make_shared<CAParticleModel>(params);
         }},
        {"kinematic_bicycle",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<BaseVelocityProcessModel> {
           return std::make_shared<KinematicBicycle>(params);
         }},
};
