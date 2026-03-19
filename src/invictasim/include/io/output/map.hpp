#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "io/output/output_adapter.hpp"
#include "io/output/ros.hpp"
#include "simulator/invictasim.hpp"

const std::map<
    std::string,
    std::function<std::shared_ptr<InvictaSimOutputAdapter>(const std::shared_ptr<InvictaSim>&)>,
    std::less<>>
    output_adapters_map = {
        {"ros",
         [](const std::shared_ptr<InvictaSim>& sim) -> std::shared_ptr<InvictaSimOutputAdapter> {
           return std::make_shared<RosOutputAdapter>(sim);
         }},
};
