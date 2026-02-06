#pragma once

#include <map>
#include <memory>
#include <string>

#include "FSFEUP02.hpp"
#include "vehicle_model.hpp"

/*
 * Map of vehicle models, with the key being the name of the vehicle model and the value being
 * a lambda function that returns a shared pointer to the corresponding vehicle model
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<VehicleModel>(const common_lib::car_parameters::CarParameters&)>,
    std::less<>>
    vehicle_models_map = {
        {"FSFEUP02",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<VehicleModel> { return std::make_shared<FSFEUP02Model>(params); }},
};