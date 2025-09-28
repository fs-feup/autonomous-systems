#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_lib/s2v_model/bicycle_model.hpp"
#include "motion_lib/s2v_model/no_rear_wss_bicycle_model.hpp"
#include "motion_lib/s2v_model/no_wss_bicycle_model.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<S2VModel>(const common_lib::car_parameters::CarParameters&)>,
    std::less<>>
    s2v_models_map = {
        {"bicycle_model",
         [](const common_lib::car_parameters::CarParameters& params) -> std::shared_ptr<S2VModel> {
           return std::make_shared<BicycleModel>(params);
         }},
        {"no_rear_wss_bicycle_model",
         [](const common_lib::car_parameters::CarParameters& params) -> std::shared_ptr<S2VModel> {
           return std::make_shared<NoRearWSSBicycleModel>(params);
         }},
};
