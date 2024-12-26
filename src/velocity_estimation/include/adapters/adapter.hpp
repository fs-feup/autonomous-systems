#pragma once

#include "node/node.hpp"
#include "rclcpp/rclcpp.hpp"

class VENode;

class Adapter : public VENode {
public:
  explicit Adapter(const VEParameters& parameters) : VENode(parameters){};
};