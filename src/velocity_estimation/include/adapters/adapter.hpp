#pragma once

#include "rclcpp/rclcpp.hpp"
#include "node/node.hpp"

class VENode;

class Adapter {
    public:
        std::shared_ptr<VENode> node_;
        explicit Adapter(std::shared_ptr<VENode> node): node_(node) {}
}