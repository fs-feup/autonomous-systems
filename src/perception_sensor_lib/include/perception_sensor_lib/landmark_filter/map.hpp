#pragma once

#include <map>
#include <memory>
#include <string>

#include "perception_sensor_lib/data_association/base_data_association.hpp"
#include "perception_sensor_lib/landmark_filter/consecutive_counter_filter.hpp"

/**
 * @brief Map of landmark filters, with the key being the name of the landmark filter
 * and the value being a lambda function that returns a shared pointer to the corresponding
 * LandmarkFilter object.
 */
const std::map<std::string,
               std::function<std::shared_ptr<LandmarkFilter>(
                   const LandmarkFilterParameters&, std::shared_ptr<DataAssociationModel>)>,
               std::less<>>
    landmark_filters_map = {
        {"consecutive_count",
         [](const LandmarkFilterParameters& params,
            std::shared_ptr<DataAssociationModel> data_association)
             -> std::shared_ptr<LandmarkFilter> {
           return std::make_shared<ConsecutiveCounterFilter>(params, data_association);
         }},
};