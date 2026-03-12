#pragma once

#include <math.h>

#include <map>
#include <memory>
#include <string>

#include "estimators/ukf.hpp"

const std::map<std::string,
               std::function<std::shared_ptr<StateEstimator>(std::shared_ptr<SEParameters>,
                                                             std::shared_ptr<ProcessModel>,
                                                             std::shared_ptr<ObservationModel>)>,
               std::less<>>
    estimators_map = {
        {"ukf",
         [](std::shared_ptr<SEParameters> parameters, std::shared_ptr<ProcessModel> process_model,
            std::shared_ptr<ObservationModel> observation_model)
             -> std::shared_ptr<StateEstimator> {
           return std::make_shared<UKF>(parameters, process_model, observation_model);
         }},
};