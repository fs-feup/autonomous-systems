#pragma once

#include <map>
#include <memory>
#include <string>

#include "perception_sensor_lib/data_association/base_data_association.hpp"
#include "perception_sensor_lib/data_association/maximum_likelihood.hpp"
/**
 * @brief Map of data association models, with the key being the name of the data association model
 * and the value being a lambda function that returns a shared pointer to the corresponding
 * DataAssociationModel object.
 */
const std::map<std::string, std::function<std::shared_ptr<DataAssociationModel>()>, std::less<>>
    data_association_models_map = {
        {"maximum_likelihood",
         []() -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<MaximumLikelihood>();
         }},
};