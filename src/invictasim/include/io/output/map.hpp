#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "io/output/output_adapter.hpp"
#include "io/output/ros.hpp"

const std::map<std::string, std::function<std::shared_ptr<InvictaSimOutputAdapter>()>, std::less<>>
    output_adapters_map = {
        {"ros",
         []() -> std::shared_ptr<InvictaSimOutputAdapter> {
           return std::make_shared<RosOutputAdapter>();
         }},
};
