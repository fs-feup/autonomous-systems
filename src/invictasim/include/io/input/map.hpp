#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "io/input/input_adapter.hpp"
#include "io/input/keyboard.hpp"
#include "io/input/ros.hpp"

const std::map<std::string, std::function<std::shared_ptr<InvictaSimInputAdapter>()>, std::less<>>
    input_adapters_map = {
        {"ros",
         []() -> std::shared_ptr<InvictaSimInputAdapter> {
           return std::make_shared<RosInputAdapter>();
         }},
        {"keyboard",
         []() -> std::shared_ptr<InvictaSimInputAdapter> {
           return std::make_shared<KeyboardInputAdapter>();
         }},
};
