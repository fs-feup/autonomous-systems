#pragma once

#include <map>
#include <memory>
#include <string>

#include "equivalent_circuit_battery.hpp"

/*
 * Map of battery models, with the key being the name of the battery model and the value being
 * a lambda function that returns a shared pointer to the corresponding battery model
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<BatteryModel>(const std::shared_ptr<common_lib::car_parameters::CarParameters>)>,
    std::less<>>
    battery_models_map = {
        {"equivalent_circuit_battery",
         [](const std::shared_ptr<common_lib::car_parameters::CarParameters> params)
             -> std::shared_ptr<BatteryModel> {
           return std::make_shared<EquivalentCircuitBattery>(params);
         }},
};