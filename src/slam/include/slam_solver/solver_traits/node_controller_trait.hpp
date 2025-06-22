#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

class NodeControllerTrait {
public:
  /**
   * @brief Initialize the SLAM solver
   * @description This method is used to initialize the SLAM solver's
   * aspects that require the node e.g. timer callbacks
   *
   * @param node ROS2 node
   */
  virtual void init(std::weak_ptr<rclcpp::Node> node) = 0;

  virtual ~NodeControllerTrait() = default;
};