#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "single_stage_drive.hpp"

/*
 * Map of independent drive models, with the key being the model name and the value
 * being a lambda function that returns a shared pointer to the corresponding model.
 */
const std::map<std::string,
               std::function<std::shared_ptr<IndependentDriveModel>(
                   const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
               std::less<>>
    independent_drive_models_map = {
        {"single_stage",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<IndependentDriveModel> {
           return std::make_shared<SingleStageDrive>(*params);
         }},
};
