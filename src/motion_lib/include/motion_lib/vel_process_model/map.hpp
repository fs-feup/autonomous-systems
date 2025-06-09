#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_lib/vel_process_model/base_vel_process_model.hpp"
#include "motion_lib/vel_process_model/particle_model.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string, std::function<std::shared_ptr<BaseVelocityProcessModel>()>, std::less<>>
    vel_process_models_map = {
        {"ca_particle_model",
         []() -> std::shared_ptr<BaseVelocityProcessModel> {
           return std::make_shared<CAParticleModel>();
         }},
};
