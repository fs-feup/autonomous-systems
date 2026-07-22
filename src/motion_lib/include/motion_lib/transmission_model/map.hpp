#pragma once

#include <map>
#include <memory>
#include <string>

#include "viscous_differential.hpp"
#include "salisbury_differential.hpp"

/*
 * Map of transmission models, with the key being the model name and the value
 * being a lambda function that returns a shared pointer to the corresponding model.
 */
const std::map<std::string,
               std::function<std::shared_ptr<TransmissionModel>(
                   const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
               std::less<>>
    transmission_models_map = {
         {"diff_viscous",
          [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
              -> std::shared_ptr<TransmissionModel> {
            return std::make_shared<ViscousDifferential>(*params);
          }},
         {"diff_salisbury",
          [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
              -> std::shared_ptr<TransmissionModel> {
            return std::make_shared<SalisburyDifferential>(*params);
          }},
};
