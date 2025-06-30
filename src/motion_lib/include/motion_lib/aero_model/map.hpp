#pragma once

#include <map>
#include <memory>
#include <string>

#include "default_aero_model.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string,
               std::function<std::shared_ptr<AeroModel>(common_lib::car_parameters::CarParameters)>,
               std::less<>>
    tire_models_map = {
        {"pacejka_combined_slip",
         [](const common_lib::car_parameters::CarParameters& params) -> std::shared_ptr<AeroModel> {
           return std::make_shared<DefaultAeroModel>(params);
         }},
};
