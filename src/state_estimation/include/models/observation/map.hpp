#pragma once

#include <math.h>

#include <map>
#include <memory>
#include <string>

#include "models/observation/02_sensors.hpp"
#include "models/observation/pacsim_sensors.hpp"

const std::map<std::string, std::function<std::shared_ptr<ObservationModel>(void)>, std::less<>>
    observation_models_map = {
        {"02_sensors",
         []() -> std::shared_ptr<ObservationModel> {
           return std::make_shared<ObservationModel02>();
         }},
        {"pacsim_sensors",
         []() -> std::shared_ptr<ObservationModel> {
           return std::make_shared<ObservationModelPacsim>();
         }},
};