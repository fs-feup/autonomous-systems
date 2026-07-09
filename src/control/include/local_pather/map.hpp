#pragma once

#include <map>
#include <memory>
#include <string>

#include "interpolator.hpp"
#include "alpha_beta.hpp"

/*
 * Map of local pathers, with the key being the type of the pather and the value being a lambda
 * function that creates the pather
 */
const std::map<std::string, std::function<std::shared_ptr<LocalPather>(const ControlParameters&)>,
               std::less<>>
    local_pather_map = {
        {"interpolator",
         [](const ControlParameters& params) -> std::shared_ptr<LocalPather> {
           return std::make_shared<Interpolator>(params);
         }},
        {"alpha_beta",
         [](const ControlParameters& params) -> std::shared_ptr<LocalPather> {
           return std::make_shared<AlphaBeta>(params);
         }},
    };
