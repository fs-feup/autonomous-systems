#pragma once

#include <math.h>

#include <map>
#include <memory>
#include <string>

#include "models/observation/02_sensors.hpp"
#include "models/observation/pacsim_sensors.hpp"

const std::map<std::string,
               std::function<std::shared_ptr<ObservationModel>(std::shared_ptr<SEParameters>)>,
               std::less<>>
    observation_models_map = {
        {"02_sensors",
         [](const std::shared_ptr<SEParameters>& parameters) -> std::shared_ptr<ObservationModel> {
           return std::make_shared<ObservationModel02>(parameters);
         }},
        {"pacsim_sensors",
         [](const std::shared_ptr<SEParameters>& parameters) -> std::shared_ptr<ObservationModel> {
           return std::make_shared<ObservationModelPacsim>(parameters);
         }},
};