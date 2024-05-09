#pragma once

#include <map>
#include <memory>
#include <string>

#include "adapter_ekf_state_est/eufs.hpp"
#include "adapter_ekf_state_est/fsds.hpp"

const std::map<std::string, std::function<std::shared_ptr<Adapter>(std::shared_ptr<SENode>)>,
               std::less<>>
    adapter_map = {
        {"fsds",
         [](std::shared_ptr<SENode> se_node) -> std::shared_ptr<Adapter> {
           return std::make_shared<FsdsAdapter>(se_node);
         }},
        {"eufs",
         [](std::shared_ptr<SENode> se_node) -> std::shared_ptr<Adapter> {
           return std::make_shared<EufsAdapter>(se_node);
         }},
};
