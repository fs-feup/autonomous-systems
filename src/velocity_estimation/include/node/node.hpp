#pragma once

#include "rclcpp/rclcpp.hpp"
#include "adapters/adapter.hpp"

class Adapter;

class VENode : public rclcpp::Node {
    std::shared_ptr<Adapter> adapter_;
    public: 
        VENode();
        friend class Adapter;
}