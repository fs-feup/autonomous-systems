#pragma once

#include <map>
#include <memory>
#include <string>

#include "estimators/ekf.hpp"
#include "estimators/kinematic_bicycle_ekf.hpp"
#include "estimators/no_rear_wss_ekf.hpp"
#include "estimators/no_wss_ekf.hpp"

/*
 * Map of velocity estimators, with the key being the type of the estimator and the value being a
 * lambda function that returns a shared pointer to the corresponding estimator
 */
const std::map<std::string, std::function<std::shared_ptr<VelocityEstimator>(const VEParameters&)>,
               std::less<>>
    estimators_map_ = {
        {"ekf",
         [](const VEParameters& params) -> std::shared_ptr<VelocityEstimator> {
           return std::make_shared<EKF>(params);
         }},
        {"no_rear_wss_ekf",
         [](const VEParameters& params) -> std::shared_ptr<VelocityEstimator> {
           return std::make_shared<NoRearWSSEKF>(params);
         }},
        {"no_wss_ekf",
         [](const VEParameters& params) -> std::shared_ptr<VelocityEstimator> {
           return std::make_shared<NoWSSEKF>(params);
         }},
        {"kinematic_bicycle", [](const VEParameters& params) -> std::shared_ptr<VelocityEstimator> {
           return std::make_shared<KinematicEKF>(params);
         }}};
