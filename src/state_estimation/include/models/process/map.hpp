#pragma once

#include <math.h>

#include <map>
#include <memory>
#include <string>

#include "models/process/component_based_vehicle_model.hpp"

const std::map<std::string,
               std::function<std::shared_ptr<ProcessModel>(std::shared_ptr<SEParameters>)>,
               std::less<>>
    process_models_map = {
        {"component_based_vehicle_model",
         [](std::shared_ptr<SEParameters> parameters) -> std::shared_ptr<ProcessModel> {
           return std::make_shared<ComponentBasedVehicleModel>(parameters);
         }},
};