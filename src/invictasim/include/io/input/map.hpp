#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "io/input/input_adapter.hpp"
#include "io/input/keyboard.hpp"
#include "io/input/ros.hpp"
#include "simulator/invictasim.hpp"

const std::map<
    std::string,
    std::function<std::shared_ptr<InvictaSimInputAdapter>(const std::shared_ptr<InvictaSim>&)>,
    std::less<>>
    input_adapters_map = {
        {"ros",
         [](const std::shared_ptr<InvictaSim>& sim) -> std::shared_ptr<InvictaSimInputAdapter> {
           return std::make_shared<RosInputAdapter>(sim);
         }},
        {"keyboard",
         [](const std::shared_ptr<InvictaSim>& sim) -> std::shared_ptr<InvictaSimInputAdapter> {
           return std::make_shared<KeyboardInputAdapter>(sim);
         }},
};
