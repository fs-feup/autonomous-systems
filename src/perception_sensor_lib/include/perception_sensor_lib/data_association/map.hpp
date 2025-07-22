#pragma once

#include <map>
#include <memory>
#include <string>

#include "perception_sensor_lib/data_association/jcbb.hpp"
#include "perception_sensor_lib/data_association/maximum_likelihood_md.hpp"
#include "perception_sensor_lib/data_association/maximum_likelihood_nll.hpp"
#include "perception_sensor_lib/data_association/nearest_neighbor.hpp"
#include "perception_sensor_lib/data_association/nearest_neighbour.hpp"

/**
 * @brief Map of data association models, with the key being the name of the data association model
 * and the value being a lambda function that returns a shared pointer to the corresponding
 * DataAssociationModel object.
 */
const std::map<
    std::string,
    std::function<std::shared_ptr<DataAssociationModel>(const DataAssociationParameters&)>,
    std::less<>>
    data_association_models_map = {
        {"maximum_likelihood_md",
         [](const DataAssociationParameters& params) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<MaximumLikelihoodMD>(params);
         }},
        {"maximum_likelihood_nll",
         [](const DataAssociationParameters& params) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<MaximumLikelihoodNLL>(params);
         }},
        {"nearest_neighbour",
         [](const DataAssociationParameters& params) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<NearestNeighbour>(params);
         }},
        {"nearest_neighbor",
         [](const DataAssociationParameters& params) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<NearestNeighbor>(params);
         }},
        {"jcbb",
         [](const DataAssociationParameters& params) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<JCBB>(params);
         }},
};