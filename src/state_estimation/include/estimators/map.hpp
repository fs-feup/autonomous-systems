#pragma once

#include <math.h>

#include <map>
#include <memory>
#include <string>

#include "estimators/ukf.hpp"

const std::map<std::string, std::function<std::shared_ptr<StateEstimator>(SEParameters)>,
               std::less<>>
    slam_solver_constructors_map = {
        {"ukf",
         [](SEParameters parameters) -> std::shared_ptr<StateEstimator> {
           return std::make_shared<UKF>(parameters);
         }},
};