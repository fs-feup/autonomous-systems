#pragma once

#include <map>
#include <memory>
#include <string>

#include "perception_sensor_lib/data_association/base_data_association.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string, std::function<std::shared_ptr<DataAssociationModel>()>, std::less<>>
    data_association_models_map = {
        // {"graph_slam",
        //  []() -> std::shared_ptr<DataAssociationModel> {
        //    return std::make_shared<GraphSLAMSolver>();
        //  }},
};
