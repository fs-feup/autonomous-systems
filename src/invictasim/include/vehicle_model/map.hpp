#pragma once

#include <map>
#include <memory>
#include <string>

#include "FSFEUP02.hpp"
#include "rk4_state_est_model.hpp"
#include "state_est_model.hpp"
#include "FSFEUP03.hpp"
#include "vehicle_model.hpp"

/*
 * Map of vehicle models, with the key being the name of the vehicle model and the value being
 * a lambda function that returns a shared pointer to the corresponding vehicle model
 */
const std::map<std::string,
               std::function<std::shared_ptr<VehicleModel>(const InvictaSimParameters&)>,
               std::less<>>
    vehicle_models_map = {
        {"02",
         [](const InvictaSimParameters& params) -> std::shared_ptr<VehicleModel> {
           return std::make_shared<FSFEUP02Model>(params);
         }},
        {"StateEstModel",
         [](const InvictaSimParameters& params) -> std::shared_ptr<VehicleModel> {
         }},
        {"rk4StateEstModel",
         [](const InvictaSimParameters& params) -> std::shared_ptr<VehicleModel> {
           return std::make_shared<RK4StateEstModel>(params);
         }},
         {"03",
         [](const InvictaSimParameters& params) -> std::shared_ptr<VehicleModel> {
         }}
};
