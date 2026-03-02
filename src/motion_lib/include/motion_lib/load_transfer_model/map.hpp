#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_lib/load_transfer_model/rigid_body_model.hpp"
#include "motion_lib/load_transfer_model/vd_load_transfer_model.hpp"

/*
 * Map of load transfer models, with the key being the name of the model and the value being a
 * lambda function that returns a shared pointer to the corresponding load transfer model.
 */
const std::map<std::string,
               std::function<std::shared_ptr<LoadTransferModel>(
                   const common_lib::car_parameters::CarParameters&)>,
               std::less<>>
    load_transfer_models_map = {
        {"rigid_body",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<LoadTransferModel> {
           return std::make_shared<RigidBodyLoadTransferModel>(params);
         }},
        {"vd_model",
         [](const common_lib::car_parameters::CarParameters& params)
             -> std::shared_ptr<LoadTransferModel> {
           return std::make_shared<VDLoadTransferModel>(params);
         }},
};
